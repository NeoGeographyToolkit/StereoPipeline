// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file ortho2pinhole.cc
///

// Given a raw image and a map-projected version of it (an ortho
// image), use that to find the camera position and orientation from
// which the image was acquired. If no DEM is provided via
// opt.reference_dem, we assume for now that the image is mapprojected
// onto the datum. Save on output a gcp file, that may be used to further
// refine the camera using bundle_adjust.
#include <asp/Core/Macros.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/InterestPointMatching.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Cartography/GeoTransform.h>
#include <boost/core/null_deleter.hpp>


// Turn off warnings from eigen
#if defined(__GNUC__) || defined(__GNUG__)
#define LOCAL_GCC_VERSION (__GNUC__ * 10000                    \
                           + __GNUC_MINOR__ * 100              \
                           + __GNUC_PATCHLEVEL__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic push
#endif
#if LOCAL_GCC_VERSION >= 40202
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif
#endif

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#if defined(__GNUC__) || defined(__GNUG__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic pop
#endif
#undef LOCAL_GCC_VERSION
#endif


namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;

typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;


struct Options : public vw::cartography::GdalWriteOptions {
  std::string raw_image, ortho_image, input_cam, output_cam, reference_dem, camera_estimate;
  double camera_height, orthoimage_height, ip_inlier_factor, max_translation;
  int    ip_per_tile, ip_detect_method, min_ip;
  bool   individually_normalize, keep_match_file, write_gcp_file, skip_image_normalization, 
    show_error, short_circuit, crop_reference_dem;

  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): camera_height(-1), orthoimage_height(0), ip_per_tile(0),
             ip_detect_method(0), individually_normalize(false), keep_match_file(false){}
};

/// Record a set of IP results as ground control points
void write_gcp_file(Options const& opt, 
                    std::vector<Vector3> const& llh_pts,
                    std::vector<Vector2> const& pixels) {

  // Save a gcp file, later bundle_adjust can use it to improve upon this camera model
  std::string gcp_file = opt.output_cam + ".gcp";
  vw_out() << "Writing: " << gcp_file << std::endl;
  std::ofstream output_handle(gcp_file.c_str());
  for (size_t pt_iter = 0; pt_iter < pixels.size(); pt_iter++) { // Loop through IPs

    Vector3 llh = llh_pts[pt_iter];
    Vector2 pix = pixels[pt_iter];

    // The ground control point ID
    output_handle << pt_iter;
    
    // Lat, lon, height
    output_handle << ", " << llh[1] << ", " << llh[0] << ", " << llh[2];

    // Sigma values
    output_handle << ", " << 1 << ", " << 1 << ", " << 1;

    // Pixel value
    output_handle << ", " <<  opt.raw_image;
    output_handle << ", " << pix.x() << ", " << pix.y(); // IP location in image
    output_handle << ", " << 1 << ", " << 1; // Sigma values
    output_handle << std::endl; // Finish the line
    
  } // End loop through IPs
  output_handle.close();
  
} // End write_gcp_file
  


/// Read the camera height above ground from the ortho file, unless it was user-specified.
double get_cam_height_estimate(Options const& opt) {
  double cam_height = opt.camera_height;
  if (cam_height < 0){
    const std::string alt_key = "Altitude";
    std::string alt_str;
    boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(opt.ortho_image));
    vw::cartography::read_header_string(*rsrc.get(), alt_key, alt_str);
    if (alt_str == "") 
      vw_throw( ArgumentErr() << "Cannot find the 'Altitude' metadata in the image: " 
                              << opt.ortho_image
                  << ". It should then be specified on the command line as --camera-height.\n");
    
    cam_height = atof(alt_str.c_str());
  }
  return cam_height;
}



// Unpack six parameters into a translation and rotation
void unpack_parameters(Vector<double> const &C,
                       Vector3   &translation,
                       Matrix3x3 &rotation) {
  translation = Vector3(C[3], C[4], C[5]);
  Vector3 angles(C[0], C[1], C[2]);
  rotation = axis_angle_to_matrix(angles);
}

/// Find the camera model that best explains the DEM pixel observations
class OtpSolveLMA : public vw::math::LeastSquaresModelBase<OtpSolveLMA> {
  
  // TODO: Normalize!
  /// The normalized values are in the -1 to 1 range.
  std::vector<vw::Vector3> const& m_gcc_coords;
  std::vector<vw::Vector2> const& m_pixel_coords;
  boost::shared_ptr<CameraModel> m_camera_model;
  mutable size_t m_iter_count;
  
public:
 

  typedef vw::Vector<double> result_type;   // normalized pixels
  typedef result_type        domain_type;   // Camera parameters: x,y,z,
  typedef vw::Matrix<double> jacobian_type;

  /// Instantiate the solver with a set of GCC <--> Pixel pairs.
  OtpSolveLMA(std::vector<vw::Vector3> const& gcc_coords,
              std::vector<vw::Vector2> const& pixel_coords,
              boost::shared_ptr<CameraModel> camera_model):
    m_gcc_coords(gcc_coords),
    m_pixel_coords(pixel_coords),
    m_camera_model(camera_model), m_iter_count(0){
    std::cout << "Init model with " << m_gcc_coords.size() << " points.\n";
  }

  /// Given a set of RPC coefficients, compute the projected pixels.
  inline result_type operator()( domain_type const& C ) const {


    // Set up the camera with the proposed rotation/translation
    Vector3 translation;
    Quat    rotation;
    //unpack_parameters(C, translation, rotation);
    translation = Vector3(C[3], C[4], C[5]);
    Vector3 angles(C[0], C[1], C[2]);
    rotation = axis_angle_to_quaternion(angles);
    
    //std::cout << "Testing translation: " << translation << std::endl;
    //std::cout << "Testing rotation: " << rotation << std::endl;
    
    AdjustedCameraModel adj_cam(m_camera_model, translation, rotation);
    
    // Compute the error scores
    const double NO_VIEW_ERROR = 999;
    const size_t result_size = m_gcc_coords.size() * 2;
    result_type result;
    result.set_size(result_size);
    double mean_error = 0;
    int missedPixelCount = 0;
    for (size_t i=0; i<m_gcc_coords.size(); ++i) {
      try{
        Vector2 pixel = adj_cam.point_to_pixel(m_gcc_coords[i]);
        //std::cout << pixel << " vs " << m_pixel_coords[i] << std::endl;
        result[2*i  ] = pixel[0];
        result[2*i+1] = pixel[1];
        mean_error += (fabs(pixel[0] - m_pixel_coords[i][0]) + fabs(pixel[1] - m_pixel_coords[i][1]))/2.0;
      } catch(vw::camera::PointToPixelErr) {
        result[2*i  ] = NO_VIEW_ERROR;
        result[2*i+1] = NO_VIEW_ERROR;
        ++missedPixelCount;
      }
    }
    
    //std::cout << "Iter " << m_iter_count << " -> mean error = " << mean_error/m_gcc_coords.size() << std::endl;
    //std::cout << "Missed pixel count = " << missedPixelCount << std::endl;
    ++m_iter_count;
    
    return result;
  }

}; // End class OtpSolveLMA


/// Solve for a camera translation/rotation based on pixel/gcc coordinate pairs.
int solve_for_cam_adjust(boost::shared_ptr<PinholeModel> camera_model,
                         std::vector<Vector2> const& pixel_coords,
                         std::vector<Vector3> const& gcc_coords,
                         vw::Vector3         & translation,
                         vw::Matrix3x3       & rotation,
                         double              & norm_error) {

  // Set up the optimizer model
  OtpSolveLMA lma_model(gcc_coords, pixel_coords, camera_model);

  int status;

  // Use the L-M solver to optimize the camera position.
  const double abs_tolerance  = 1e-24;
  const double rel_tolerance  = 1e-24;
  const int    max_iterations = 2000;

  // Set up with identity transform  
  const size_t NUM_PARAMS = 6;
  Vector<double> seed_params(NUM_PARAMS);
  for (size_t i=0; i<NUM_PARAMS; ++i)
    seed_params[i] = 0.0;

  // Pack the target pixel observations
  const size_t num_observations = pixel_coords.size()*2;
  Vector<double> packed_observations(num_observations);
  for(size_t i=0; i<pixel_coords.size(); ++i) {
    packed_observations[2*i  ] = pixel_coords[i][0];
    packed_observations[2*i+1] = pixel_coords[i][1];
  }

  // Run the observation
  Vector<double> final_params;
  final_params = math::levenberg_marquardt( lma_model, seed_params, packed_observations,
                                            status, abs_tolerance, rel_tolerance,
                                            max_iterations );

  if (status < 1) { // This means the solver failed to converge!
    VW_OUT(DebugMessage, "asp") << "ortho2pinhole: WARNING --> Levenberg-Marquardt solver status = " 
                                << status << std::endl;
  }

  // Otherwise the solver converged, compute the final error number.
  Vector<double> final_projected = lma_model(final_params);
  Vector<double> final_error     = lma_model.difference(final_projected, packed_observations);
  norm_error = norm_2(final_error);

  unpack_parameters(final_params, translation, rotation);
  
  return status;
}
 



   
/// Load the DEM and adjust some options depending on DEM statistics.
void load_reference_dem(Options &opt, boost::shared_ptr<DiskImageResource> const& rsrc_ortho,
                        vw::cartography::GeoReference const& ortho_georef,
                        ImageViewRef< PixelMask<float> > &dem,
                        vw::cartography::GeoReference &dem_georef,
                        bool &elevation_change_present) {

  // Set up the DEM if it was provided.
  float dem_nodata = -std::numeric_limits<float>::max();

  bool is_good = vw::cartography::read_georeference(dem_georef, opt.reference_dem);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read georeference from: "
                           << opt.reference_dem << ".\n");
  }

  {
    // Read the no-data
    DiskImageResourceGDAL rsrc(opt.reference_dem);
    if (rsrc.has_nodata_read()) dem_nodata = rsrc.nodata_read();
  }


  bool crop_is_success = false;
  if (opt.crop_reference_dem){
    
    // Crop the DEM to a small area and read fully into memory
    
    DiskImageView<float> tmp_ortho(rsrc_ortho);
    BBox2 ortho_bbox = bounding_box(tmp_ortho);

    DiskImageView<float> tmp_dem(opt.reference_dem);
    BBox2 dem_bbox = bounding_box(tmp_dem);
    
    // The GeoTransform will hide the messy details of conversions
    vw::cartography::GeoTransform geotrans(dem_georef, ortho_georef, dem_bbox, ortho_bbox);
    
    // Get the ortho bbox in the DEM pixel domain
    BBox2 crop_box = geotrans.reverse_bbox(ortho_bbox);
    if (crop_box.empty()) {
      // This should not happen, but go figure, as the DEM could be very coarse
      ortho_bbox.expand(10000);
      crop_box = geotrans.reverse_bbox(ortho_bbox);
    }
    
    if (!crop_box.empty() && crop_box.width() < 5000 && crop_box.height() < 5000) {
      // It may be empty for a very coarse DEM maybe
      
      crop_box.expand(200); // TODO: Need to think more here
      crop_box.crop(dem_bbox);
      
      if (!crop_box.empty()) {
        ImageView<float> cropped_dem = crop(DiskImageView<float>(opt.reference_dem), crop_box);
        dem = create_mask(cropped_dem, dem_nodata);
        dem_georef = crop(dem_georef, crop_box);
        crop_is_success = true;
      }
    }

    vw_out() << "DEM bounding box: " << bounding_box(dem) << std::endl;
  }
  
  // Default behavior  
  if (!crop_is_success)
    dem = create_mask(DiskImageView<float>(opt.reference_dem), dem_nodata);

  
  // Get an estimate of the elevation range in the input image
  const int HEIGHT_SKIP = 500; // The DEM is very low resolution
  
  InterpolationView<EdgeExtensionView< ImageViewRef< PixelMask<float> >, 
                                       ConstantEdgeExtension >, 
                    BilinearInterpolation> interp_dem = 
      interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());
  double max_height = -9999, min_height = 9999999, mean_height=0.0, num_heights = 0;
  for (int r=0; r<rsrc_ortho->rows(); r+=HEIGHT_SKIP) {
    for (int c=0; c<rsrc_ortho->cols(); c+=HEIGHT_SKIP) {
      // Ortho pixel to DEM pixel
      Vector2 ortho_pix(c, r);
      Vector2 ll      = ortho_georef.pixel_to_lonlat(ortho_pix);
      Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
      double x = dem_pix.x(), y = dem_pix.y();
      if (0 <= x && x <= dem.cols() - 1 && 0 <= y && y <= dem.rows() - 1 ) {
        PixelMask<float> dem_val = interp_dem(x, y);
        if (is_valid(dem_val)) {
          // Accumulate the valid height range
          double height = dem_val.child();
          mean_height += height;
          num_heights += 1.0;
          if (height < min_height) min_height = height;
          if (height > max_height) max_height = height;
        }
      } // End boundary check
    } // End col loop
  } // End row loop
  
  if (num_heights < 1.0) {
    vw_out() << "No intersection found with the reference DEM!\n";
    return; // No information can be gained in this case.
  }
  
  mean_height = mean_height / num_heights;
  
  // Set a flag if we detect there is a significant amount of elevation change in the image
  const double ELEVATION_CHANGE_THRESHOLD = 40; // Meters
  double elevation_change = max_height - min_height;
  vw_out() << "Estimated elevation change of " << elevation_change << std::endl;
  if (elevation_change > ELEVATION_CHANGE_THRESHOLD)
    elevation_change_present = true;

  // If the user did not specify an orthoimage height, use the statistics we computed
  //  to get an estimate.
  if (opt.orthoimage_height == 0.0) {
    opt.orthoimage_height = mean_height;
    vw_out() << "Estimated orthoimage height to be " << opt.orthoimage_height << std::endl;
  }
  
} // End load_reference_dem

void load_camera_and_find_ip(Options const& opt, 
                             boost::shared_ptr<DiskImageResource> const& rsrc_raw,
                             boost::shared_ptr<DiskImageResource> const& rsrc_ortho,
                             std::string const& match_filename,
                             boost::shared_ptr<CameraModel> &cam) {

  std::string out_prefix = "tmp-prefix";
  std::string stereo_session_string = "pinhole";
  float nodata1, nodata2;
  SessionPtr session(asp::StereoSessionFactory::create(stereo_session_string, opt,
                                                       opt.raw_image, opt.ortho_image,
                                                       opt.input_cam, opt.input_cam,
                                                       out_prefix));
  session->get_nodata_values(rsrc_raw, rsrc_ortho, nodata1, nodata2);
  
  cam = session->camera_model(opt.raw_image, opt.input_cam);
  
  // Skip IP finding if the match file exists since the code will re-use it anyways.
  if (boost::filesystem::exists(match_filename)) {
    vw_out() << "Using existing match filename " << match_filename << std::endl;
    return;
  }
  
  try{
    // IP matching may not succeed for all pairs
    
    // Get masked views of the images to get statistics from
    DiskImageView<float> image1_view(rsrc_raw), image2_view(rsrc_ortho);
    ImageViewRef< PixelMask<float> > masked_image1
      = create_mask_less_or_equal(image1_view,  nodata1);
    ImageViewRef< PixelMask<float> > masked_image2
      = create_mask_less_or_equal(image2_view, nodata2);
    vw::Vector<vw::float32,6> image1_stats = asp::gather_stats(masked_image1, opt.raw_image  );
    vw::Vector<vw::float32,6> image2_stats = asp::gather_stats(masked_image2, opt.ortho_image);
    
    session->ip_matching(opt.raw_image, opt.ortho_image,
                         Vector2(masked_image1.cols(), masked_image1.rows()),
                         image1_stats,
                         image2_stats,
                         opt.ip_per_tile,
                         nodata1, nodata2, match_filename, cam.get(), cam.get()
                        );
  } catch ( const std::exception& e ){
    vw_throw( ArgumentErr()
              << "Could not find interest points between images "
              << opt.raw_image << " and " << opt.ortho_image << "\n" << e.what() << "\n");
  } //End try/catch
 
} // End load_camera_and_find_ip


/// Copy pertinent information from the nav camera estimate to the 
///  pinhole camera model so that we keep the intrinsic parameters.
void update_pinhole_from_nav_estimate(vw::camera::PinholeModel *pcam,
                                      vw::camera::PinholeModel const& nav_camera) {
  // Copy the camera position
  pcam->set_camera_center(nav_camera.camera_center());
  
  // Copy the camera orientation, being careful to avoid undiagnosed quaternion/matrix issue.
  pcam->set_camera_pose(nav_camera.get_rotation_matrix());
  
  // Copy the camera<->image transform which nav2cam uses to flip the vertical axis.
  // -> This is not currently happening because our camera model is not properly
  //    handling those parameters!  For now we are messing with the pose instead.
  Vector3 u_vec, v_vec, w_vec;
  nav_camera.coordinate_frame(u_vec, v_vec, w_vec);
  pcam->set_coordinate_frame(u_vec, v_vec, w_vec);
}

/// Generate the estimated camera using a 3D affine transform or load in from a file.
/// - pcam is the input camera model.
void get_estimated_camera_position(Options const& opt,
                                   bool have_dem_points,
                                   vw::camera::PinholeModel *pcam,
                                   std::vector<Vector3> const& raw_flat_xyz,
                                   std::vector<Vector3> const& ortho_flat_xyz) {

  // Use a simple solution to get an initial estimate.

  // Pack the in/out point pairs into two matrices.
  int num_pts = raw_flat_xyz.size();
  vw_out() << "Using " << num_pts << " points to create the camera model.\n";
  vw::Matrix<double> points_in(3, num_pts), points_out(3, num_pts);
  typedef vw::math::MatrixCol<vw::Matrix<double> > ColView;
  for (int pt_iter = 0; pt_iter < num_pts; pt_iter++){
    ColView colIn (points_in,  pt_iter);
    ColView colOut(points_out, pt_iter);
    colIn  = raw_flat_xyz  [pt_iter];
    colOut = ortho_flat_xyz[pt_iter];
  }

  // Call function to compute a 3D affine transform between the two point sets
  // - This is always between the nadir facing camera over flat terrain and the
  //   ortho at the estimated constant elevation.
  // - The ortho elevation needs to be flat so that the "plane" of the camera points
  //   does not tilt away from the correct position trying to align to 3D terrain.
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale;
  asp::find_3D_affine_transform(points_in, points_out, rotation, translation, scale);

  vw_out() << "Determined camera extrinsics from orthoimage: " << std::endl;
  vw_out() << "scale: " << scale << std::endl;

  // Apply the transform to the camera.
  pcam->apply_transform(rotation, translation, scale);

  std::cout << "Est affine camera:\n" << *pcam << std::endl;

  if (opt.camera_estimate != "") {
    // Load the estimated camera
    PinholeModel est_cam(opt.camera_estimate);
    std::cout << "est_cam = \n" << est_cam << std::endl;
      
    // If our solution moved too far from the estimated camera, use
    // the estimated position/pose instead.
    double dist = norm_2(pcam->camera_center() - est_cam.camera_center());
    vw_out() << "Flat affine estimate is " << dist 
             << " meters from the input camera estimate.\n";
    if (dist > opt.max_translation) {
      update_pinhole_from_nav_estimate(pcam, est_cam);
      vw_out() << "Flat affine estimate difference limit is " << opt.max_translation 
               << ", using input estimate instead\n";
    }
  } // End have camera estimate case


} // End load_estimated_camera



/// Use IPs from the ortho image and DEM to optimize the camera with a Lev-Mar solver.
/// - Returns norm_error of the chosen solution.
double refine_camera_with_dem_pts(Options const& opt,
                                  vw::cartography::GeoReference const& dem_georef,
                                  bool use_extra_caution,
                                  std::vector<Vector3> const& ortho_dem_xyz,
                                  std::vector<Vector2> const& raw_pixels2,
                                  vw::camera::PinholeModel *pcam) {

  int num_pts = ortho_dem_xyz.size();

  // Try multiple solver attempts with varying camera heights
  const double TARGET_ERROR = 1000; // Error can be quite low but incorrect results tend to be very large
  const double HEIGHT_STEP = 50;
  const int    NUM_SEARCH_STEPS = 6;
  
  // Set up the search elevation offsets
  // - Only use additional offsets if we don't have an estimated camera file.
  std::vector<double> height_offsets;
  height_offsets.push_back(0);
  if (opt.camera_estimate == "") {
    for (int i=1; i<=NUM_SEARCH_STEPS; ++i) {
      height_offsets.push_back(   i*HEIGHT_STEP);
      height_offsets.push_back(-1*i*HEIGHT_STEP);
    }
  }

  const size_t max_attempts = height_offsets.size();
 
  size_t attempt_count = 0;
  vw::Matrix3x3 rotation, best_rotation;
  vw::Vector3   translation, best_translation, best_gcc;
  double norm_error = 99999999999;
  double best_error = norm_error;
  
  // Record starting position
  Vector3 initial_center_gcc = pcam->camera_center(Vector2(0,0));
  Vector3 initial_center_llh = dem_georef.datum().cartesian_to_geodetic(initial_center_gcc);
  
  while (attempt_count < max_attempts) {
  
    // Set the camera position
    double new_height = initial_center_llh[2] + height_offsets[attempt_count];
    Vector3 new_llh = initial_center_llh;
    new_llh[2] = new_height;
    Vector3 new_gcc = dem_georef.datum().geodetic_to_cartesian(new_llh);
    pcam->set_camera_center(new_gcc);    
  
    vw_out() << "Running LM solver attempt " << attempt_count << " with starting elevation: " << new_height << std::endl;
  
    // Refine our initial estimate of the camera position using an LM solver with the camera model.
    int status = solve_for_cam_adjust(boost::shared_ptr<PinholeModel>(pcam, boost::null_deleter()),
                                      raw_pixels2, ortho_dem_xyz,
                                      translation, rotation, norm_error);

    attempt_count += 1;

    vw_out() << "LM solver status: " << status << std::endl;
    if (status < 1) {
      vw_out() << "LM solver failed!\n";
      continue; // Move on to the next attempt
    }

    // Success case
    vw_out() << "Success: LM solver error = " << norm_error << std::endl;
    
    // Keep track of our best solution
    if (norm_error < best_error) {
        best_rotation    = rotation;
        best_translation = translation;
        best_error       = norm_error;
        best_gcc         = new_gcc;
    }
    
    // If the error was very good, stop trying new solutions.
    if ((norm_error < TARGET_ERROR) && !use_extra_caution)
      break;      

  } // End solver attempt loop
  
  vw_out() << "Best LM solver error  = " << best_error << std::endl;

  // TODO: Make a function for this?
  // Apply the transform to the camera.     
  Quat rot_q(best_rotation);
  pcam->set_camera_center(best_gcc); // Make sure the adjustment applies to the right center location
  AdjustedCameraModel adj_cam(boost::shared_ptr<PinholeModel>(pcam, boost::null_deleter()),
                              best_translation, rot_q);
  Quat    pose   = adj_cam.camera_pose(Vector2());
  Vector3 center = adj_cam.camera_center(Vector2());
  pcam->set_camera_pose(pose); // TODO: There may be a problem here!
  pcam->set_camera_center(center);
  
  if (opt.show_error) { // Print error in pixels for each IP
    for (int i=0; i<num_pts; ++i) {
      Vector2 pixel = pcam->point_to_pixel(ortho_dem_xyz[i]);
      double diff = norm_2(pixel - raw_pixels2[i]);
      std::cout << "Error " << i << " = " << diff << std::endl;
    }
  }

  return best_error;

} // End function refine_camera_with_dem_pts


// Primary task-solving function.
void ortho2pinhole(Options & opt){

  // Input image handles
  boost::shared_ptr<DiskImageResource>
    rsrc_raw(vw::DiskImageResourcePtr(opt.raw_image)),
    rsrc_ortho(vw::DiskImageResourcePtr(opt.ortho_image));
  if ( (rsrc_raw->channels() > 1) || (rsrc_ortho->channels() > 1) )
    vw_throw(ArgumentErr() << "Error: Input images can only have a single channel!\n\n");

  // Load GeoRef from the ortho image
  vw::cartography::GeoReference ortho_georef;
  bool is_good = vw::cartography::read_georeference(ortho_georef, opt.ortho_image);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read georeference from: "
                           << opt.ortho_image << ".\n");
  }

  // Set up the DEM if it was provided.
  ImageViewRef< PixelMask<float> > dem;
  vw::cartography::GeoReference dem_georef;
  bool has_ref_dem = (opt.reference_dem != "");
  bool elevation_change_present = false;
  if (has_ref_dem) {
    load_reference_dem(opt, rsrc_ortho, ortho_georef, dem, dem_georef, elevation_change_present);
  }
  

  // When significant elevation change is present, the homography IP filter is not
  //  accurate and we need to compensate by relaxing our inlier threshold.
  const double ELEVATION_INLIER_SCALE = 10;
  if (elevation_change_present) {
    // TODO: Decouple threshold from other params!
    asp::stereo_settings().epipolar_threshold = 150*asp::stereo_settings().ip_inlier_factor * ELEVATION_INLIER_SCALE;
    //asp::stereo_settings().ip_inlier_factor *= ELEVATION_INLIER_SCALE;
    
    vw_out() << "Due to elevation change, increasing ip_inlier_factor to " 
             << asp::stereo_settings().ip_inlier_factor << std::endl;
  }

  // Load camera and find IP
  std::string match_filename = opt.output_cam + ".match";
  boost::shared_ptr<CameraModel> cam;
  load_camera_and_find_ip(opt, rsrc_raw, rsrc_ortho, match_filename, cam);

  // The ortho image file must have the height of the camera above the ground.
  // This can be over-written from the command line.
  double cam_height = get_cam_height_estimate(opt);
  vw_out() << "Using estimated cam height: " << cam_height << std::endl;

  std::vector<vw::ip::InterestPoint> raw_ip, ortho_ip;
  ip::read_binary_match_file(match_filename, raw_ip, ortho_ip);
  vw::camera::PinholeModel *pcam = dynamic_cast<vw::camera::PinholeModel*>(cam.get());
  if (pcam == NULL) {
    vw_throw(ArgumentErr() << "Expecting a pinhole camera model.\n");
  }

  if ( !(pcam->camera_pose() == Quaternion<double>(1, 0, 0, 0)) ) {
    // We like to start with a camera pointing along the z axis. That makes life easy.
    vw_throw(ArgumentErr() << "Expecting the input camera to have identity rotation.\n");
  }

  /*
  // There is a vertical flip in the camera/image relationship not included in the camera files!
  TODO: The camera model is not handling this parameter!!!!!!!
  Vector3 u_vec, v_vec, w_vec;
  pcam->coordinate_frame(u_vec, v_vec, w_vec);
  pcam->set_coordinate_frame(u_vec, -1.0*v_vec, w_vec);
  */


  // Find pairs of points. First point is in the camera coordinate system,
  // second in the ground coordinate system. This will allow us to transform
  // the camera to the latter system.
  // We use two versions of ground points. In one we assume constant height,
  // in the second we pull the heights from a DEM. If there are enough
  // of the latter kind of points, we use those.
  // - The "2" variables go with the "ortho_dem" variables.
  std::vector<Vector3> raw_flat_xyz,  ortho_flat_xyz, ortho_flat_llh;
  std::vector<Vector3> raw_flat_xyz2, ortho_dem_xyz,  ortho_dem_llh;
  std::vector<Vector2> raw_pixels, raw_pixels2;
  
  // Estimate the relative camera height from the terrain
  // - Make sure the camera always has some room to intersect rays!
  bool use_extra_caution = false;
  const double MIN_CAM_HEIGHT = 100;
  double relative_cam_height = cam_height - opt.orthoimage_height;
  if (relative_cam_height < MIN_CAM_HEIGHT) {
    vw_out() << "WARNING: Estimated camera height is too close to the estimated ortho height!\n"
             << "Forcing the relative camera height to be " << MIN_CAM_HEIGHT << std::endl;
    relative_cam_height = MIN_CAM_HEIGHT;
    use_extra_caution   = true;
  }
  else
    vw_out() << "Relative cam height = " << relative_cam_height << std::endl;
  
  for (size_t ip_iter = 0; ip_iter < raw_ip.size(); ip_iter++){
    try {
      // Get ray from the raw image pixel
      Vector2 raw_pix(raw_ip[ip_iter].x, raw_ip[ip_iter].y);
      Vector3 ctr = pcam->camera_center  (raw_pix);
      Vector3 dir = pcam->pixel_to_vector(raw_pix);
      
      // We assume the ground is flat. Intersect rays going from camera
      // with the plane z = cam_height - orthoimage_height.
      Vector3 point_in = ctr + (relative_cam_height/dir[2])*dir;
      
      // Get lonlat location for this IP IP.
      Vector2 ortho_pix(ortho_ip[ip_iter].x, ortho_ip[ip_iter].y);
      Vector2 ll = ortho_georef.pixel_to_lonlat(ortho_pix);
      
      // Use given assumed height of orthoimage above the datum.
      Vector3 llh(ll[0], ll[1], opt.orthoimage_height);
      Vector3 point_out = ortho_georef.datum().geodetic_to_cartesian(llh);

      raw_flat_xyz.push_back(point_in);
      ortho_flat_xyz.push_back(point_out);
      ortho_flat_llh.push_back(llh);
      raw_pixels.push_back(raw_pix);
      
      if (!has_ref_dem)
        continue;
      
      // Now let's see if the DEM is any good
        
      PixelMask<float> dem_val; 
      dem_val.invalidate();
      Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
      double x = dem_pix.x(), y = dem_pix.y();
      // Is this pixel contained in the DEM?
      if (0 <= x && x <= dem.cols() - 1 && 0 <= y && y <= dem.rows() - 1 ) {
        InterpolationView<EdgeExtensionView< ImageViewRef< PixelMask<float> >, 
                                             ConstantEdgeExtension >, 
                          BilinearInterpolation> interp_dem = 
            interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());
        dem_val = interp_dem(x, y);
        if (is_valid(dem_val)) {
          Vector3 llh(ll[0], ll[1], dem_val.child());
          Vector3 point_out = dem_georef.datum().geodetic_to_cartesian(llh);
          raw_flat_xyz2.push_back(point_in);
          ortho_dem_xyz.push_back(point_out);
          ortho_dem_llh.push_back(llh);
          raw_pixels2.push_back(raw_pix);
        }
      } // end considering the reference DEM

    }
    catch(...){} // Ignore any points which trigger a camera exception
  }

  // See if enough points were found
  const int num_ip_found     = static_cast<int>(raw_pixels.size());
  const int num_dem_ip_found = static_cast<int>(ortho_dem_xyz.size());
  if (num_ip_found < opt.min_ip) {
    if (!opt.keep_match_file) {
      vw_out() << "Removing: " << match_filename << std::endl;
      boost::filesystem::remove(match_filename);
    }
    
    // If we have the estimated model use that, better than nothing.
    if (opt.camera_estimate != "") {
      PinholeModel est_cam(opt.camera_estimate);
      update_pinhole_from_nav_estimate(pcam, est_cam);
    }
    vw_throw(ArgumentErr() << "Error: Only found " << num_ip_found << " interest points, quitting.\n");
  }

  // See if enough points on the DEM were found
  bool use_dem_points = false;
  if (has_ref_dem) {
    const int MIN_NUM_DEM_POINTS = 8;
    
    if ((num_ip_found > num_dem_ip_found) && (num_dem_ip_found < MIN_NUM_DEM_POINTS)) {
      vw_out() << "Less than " << MIN_NUM_DEM_POINTS 
               << " interest points were on the DEM. Igorning the DEM "
               << "and using the constant height: " << opt.orthoimage_height << "\n";
    }else{ // Use the DEM points
      use_dem_points = true;
    }
  }
  
  // Get our best estimate of the camera position
  // - This will be from an affine transform between points or
  //   just be loaded from an input file.  
  get_estimated_camera_position(opt, use_dem_points, pcam, raw_flat_xyz, ortho_flat_xyz);

  Vector3 gcc_cam_est = pcam->camera_center();
  Vector3 llh_cam_est = ortho_georef.datum().cartesian_to_geodetic(gcc_cam_est);
  vw_out() << "Non-DEM camera center located at " << llh_cam_est << std::endl;

  if (use_dem_points) {
  
    // Use a Lev-Mar solver to refine the camera points using the DEM.
    double refine_error = refine_camera_with_dem_pts(opt, dem_georef, use_extra_caution,
                                                     ortho_dem_xyz, raw_pixels2, pcam);

    Vector3 llh_cam_dem = ortho_georef.datum().cartesian_to_geodetic(pcam->camera_center());
    vw_out() << "DEM enhanced camera center located at " << llh_cam_dem << std::endl;
    
    // If an estimate was provided and the DEM optimization moved us too far away from that estimate,
    //  revert back to the input estimate.  Also revert if the error was too high.
    const double MAX_REFINE_ERROR = 4000;
    if (opt.camera_estimate != "") {
      bool revert_camera = false;
      if (refine_error > MAX_REFINE_ERROR) {
        vw_out() << "Refinement error is too high, reverting to input camera estimate.\n";
        revert_camera = true;
      }
      else {
        double dist = norm_2(pcam->camera_center() - gcc_cam_est);
        vw_out() << "Camera distance from estimate is " << dist << std::endl;
        if (dist > opt.max_translation) {
          vw_out() << "Camera solution is too far from the input estimate, reverting to estimate.\n";
          revert_camera = true;
        }
      }
      if (revert_camera) { // Use estimated camera with input camera intrinsics
        PinholeModel est_cam(opt.camera_estimate);
        update_pinhole_from_nav_estimate(pcam, est_cam);
      }
    }
    
  } // End of second pass DEM handling case

  vw_out() << "Writing: " << opt.output_cam << std::endl;
  pcam->write(opt.output_cam);

  // Save a gcp file in case we want to use it for something later.
  if (opt.write_gcp_file) {
    if (use_dem_points) {
      write_gcp_file(opt, ortho_dem_llh, raw_pixels2);
    }else {
      write_gcp_file(opt, ortho_flat_llh, raw_pixels);
    }
  }
    
  if (!opt.keep_match_file) {
    vw_out() << "Removing: " << match_filename << std::endl;
    boost::filesystem::remove(match_filename);
  }
}  

/// If an rgb input image was passed in, convert to a temporary grayscale
///  image and work on that instead.
/// - This is useful for the Icebridge case.
std::string handle_rgb_input(std::string const& input_path, Options const& opt) {

  boost::shared_ptr<DiskImageResource> rsrc(vw::DiskImageResourcePtr(input_path));
  if (rsrc->channels() != 3)
    return input_path; // Nothing to do in this case

  vw_out() << "Assuming input image is RGB format...\n";
  std::string gray_path = input_path + "_gray.tif";

  // Read the misc header strings from the file so we can propogate them
  // to the output file.
  std::map<std::string, std::string> keywords;
  vw::cartography::read_header_strings(*rsrc.get(), keywords);

  vw::cartography::GeoReference georef;
  bool   has_georef = vw::cartography::read_georeference(georef, input_path);
  bool   has_nodata = true;
  double nodata     = 0.0;
  block_write_gdal_image(gray_path,
                         weighted_rgb_to_gray(DiskImageView<PixelRGB<uint8> >(input_path)),
                         has_georef, georef,
                         has_nodata, nodata, opt,
                         ProgressCallback::dummy_instance(), keywords);
  return gray_path;

}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("camera-estimate",   po::value(&opt.camera_estimate)->default_value(""),
     "An estimated camera model used for location and pose estimate only.")
    ("max-translation",   po::value(&opt.max_translation)->default_value(10.0),
     "The maximum distance the camera solution is allowed to move from camera-estimate.")     
    ("camera-height",   po::value(&opt.camera_height)->default_value(-1.0),
     "The approximate height above the datum, in meters, at which the camera should be. If not specified, it will be read from the orthoimage metadata.")
    ("orthoimage-height",   po::value(&opt.orthoimage_height)->default_value(0.0),
     "The approximate height above the datum, in meters, at which the orthoimage is positioned. We assume flat ground.")
    ("ip-per-tile",             po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
    ("ip-detect-method",po::value(&opt.ip_detect_method)->default_value(1),
     "Interest point detection algorithm (0: Integral OBALoG, 1: OpenCV SIFT (default), 2: OpenCV ORB.")
    ("minimum-ip",             po::value(&opt.min_ip)->default_value(5),
     "Don't create camera if fewer than this many IP match.")
    ("ip-inlier-factor",   po::value(&opt.ip_inlier_factor)->default_value(1.0/15.0),
     "IP inlier factor.")
    ("individually-normalize",   po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
     "Individually normalize the input images instead of using common values.")
    ("skip-image-normalization", po::bool_switch(&opt.skip_image_normalization)->default_value(false)->implicit_value(true),
     "Skip the step of normalizing the values of input images.")
    ("short-circuit",   po::bool_switch(&opt.short_circuit)->default_value(false)->implicit_value(true),
     "No processing, just copy input intrinisc parameters to camera-estimate and write out.")
    ("show-error",   po::bool_switch(&opt.show_error)->default_value(false)->implicit_value(true),
     "Print point error.")
    ("keep-match-file",   po::bool_switch(&opt.keep_match_file)->default_value(false)->implicit_value(true),
     "Don't delete the .match file after running.")
    ("write-gcp-file",   po::bool_switch(&opt.write_gcp_file)->default_value(false)->implicit_value(true),
     "Write a bundle_adjust compatible gcp file.")
    ("reference-dem",             po::value(&opt.reference_dem)->default_value(""),
     "If provided, extract from this DEM the heights above the ground rather than assuming the value in --orthoimage-height.")
    ("crop-reference-dem", po::bool_switch(&opt.crop_reference_dem)->default_value(false)->implicit_value(true),
     "Crop the reference DEM to a generous area to make it faster to load.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("raw-image",  po::value(&opt.raw_image))
    ("orthoimage", po::value(&opt.ortho_image))
    ("input-cam",  po::value(&opt.input_cam))
    ("output-cam", po::value(&opt.output_cam));

  po::positional_options_description positional_desc;
  positional_desc.add("raw-image",  1);
  positional_desc.add("orthoimage", 1);
  positional_desc.add("input-cam",  1);
  positional_desc.add("output-cam", 1);

  std::string usage("<raw image> <ortho image> <input pinhole cam> <output pinhole cam> [options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Copy the IP settings to the global stereosettings() object
  asp::stereo_settings().ip_matching_method       = opt.ip_detect_method;
  asp::stereo_settings().individually_normalize   = opt.individually_normalize;
  asp::stereo_settings().skip_image_normalization = opt.skip_image_normalization;
  asp::stereo_settings().ip_inlier_factor         = opt.ip_inlier_factor;
  
  if ( opt.raw_image.empty() )
    vw_throw( ArgumentErr() << "Missing input raw image.\n" << usage << general_options );

  if ( opt.ortho_image.empty() )
    vw_throw( ArgumentErr() << "Missing input ortho image.\n" << usage << general_options );

  if ( opt.input_cam.empty() )
    vw_throw( ArgumentErr() << "Missing input pinhole camera.\n" << usage << general_options );

  if ( opt.output_cam.empty() )
    vw_throw( ArgumentErr() << "Missing output pinhole camera.\n" << usage << general_options );

  if (opt.camera_estimate != "") {
    if (!boost::filesystem::exists(opt.camera_estimate)) {
      vw_throw( ArgumentErr() << "Estimated camera file " << opt.camera_estimate << " does not exist!\n");
    }    
  }

  if (opt.short_circuit && opt.camera_estimate == "")
    vw_throw( ArgumentErr() << "Estimated camera file is required with the short-circuit option.\n");

  // Create the output directory
  vw::create_out_dir(opt.output_cam);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.output_cam);
  
}

// ================================================================================

int main(int argc, char* argv[]) {

  Options opt;
  //try {
  handle_arguments( argc, argv, opt );
   
  if (opt.short_circuit) {
    vw_out() << "Creating camera without using ortho image.\n";
  
    // Load input camera files
    vw_out() << "Loading: " << opt.input_cam << std::endl;
    PinholeModel input_cam(opt.input_cam);
    vw_out() << "Loading: " << opt.camera_estimate << std::endl;
    PinholeModel est_cam(opt.camera_estimate);
    
    // Copy camera position and pose from estimate camera to input camera
    input_cam.set_camera_center(est_cam.camera_center());
    input_cam.set_camera_pose  (est_cam.camera_pose  ());
    
    // Write to output camera
    vw_out() << "Writing: " << opt.output_cam << std::endl;
    input_cam.write(opt.output_cam);
    return 0;
  }
  
  opt.raw_image   = handle_rgb_input(opt.raw_image,   opt);
  opt.ortho_image = handle_rgb_input(opt.ortho_image, opt);
  
  ortho2pinhole(opt);
  return 0;
  //} ASP_STANDARD_CATCHES;
}
