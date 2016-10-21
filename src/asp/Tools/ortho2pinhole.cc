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
#include <asp/Sessions/ResourceLoader.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/InterestPointMatching.h>
#include <vw/Camera/PinholeModel.h>

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
  std::string raw_image, ortho_image, input_cam, output_cam, reference_dem;
  double camera_height, orthoimage_height;
  int ip_per_tile;
  int  ip_detect_method;
  bool individually_normalize;

  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): camera_height(-1), orthoimage_height(0), ip_per_tile(0),
             ip_detect_method(0), individually_normalize(false){}
};

void ortho2pinhole(Options const& opt){
  std::string out_prefix = "tmp-prefix";
  std::string match_filename = ip::match_filename(out_prefix, opt.raw_image, opt.ortho_image);
  
  boost::shared_ptr<DiskImageResource>
    rsrc1(asp::load_disk_image_resource(opt.raw_image, opt.input_cam)),
    rsrc2(asp::load_disk_image_resource(opt.ortho_image, opt.input_cam));
  if ( (rsrc1->channels() > 1) || (rsrc2->channels() > 1) )
    vw_throw(ArgumentErr() << "Error: Input images can only have a single channel!\n\n");
  std::string stereo_session_string = "pinhole";
  float nodata1, nodata2;
  SessionPtr session(asp::StereoSessionFactory::create(stereo_session_string, opt,
                                                       opt.raw_image,  opt.ortho_image,
                                                       opt.input_cam, opt.input_cam,
                                                       out_prefix));
  session->get_nodata_values(rsrc1, rsrc2, nodata1, nodata2);
  
  boost::shared_ptr<CameraModel> cam = session->camera_model(opt.raw_image, opt.input_cam);
  
  try{
    // IP matching may not succeed for all pairs
    
    // Get masked views of the images to get statistics from
    DiskImageView<float> image1_view(rsrc1), image2_view(rsrc2);
    ImageViewRef< PixelMask<float> > masked_image1
      = create_mask_less_or_equal(image1_view,  nodata1);
    ImageViewRef< PixelMask<float> > masked_image2
      = create_mask_less_or_equal(image2_view, nodata2);
    vw::Vector<vw::float32,6> image1_stats = asp::gather_stats(masked_image1, opt.raw_image);
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

  bool has_ref_dem = (opt.reference_dem != "");
  vw::cartography::GeoReference dem_georef;
  ImageViewRef< PixelMask<float> > dem;
  float dem_nodata = -std::numeric_limits<float>::max();
  if (has_ref_dem) {
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
    
    dem = create_mask(DiskImageView<float>(opt.reference_dem), dem_nodata);
  }
  
  vw::cartography::GeoReference ortho_georef;
  bool is_good = vw::cartography::read_georeference(ortho_georef, opt.ortho_image);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read georeference from: "
	     << opt.ortho_image << ".\n");
  }

  // The ortho image file must have the height of the camera above the ground.
  // This can be over-written from the command line.
  double cam_height = opt.camera_height;
  if (cam_height < 0){
    std::string alt_str;
    std::string alt_key = "Altitude";
    boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(opt.ortho_image));
    vw::cartography::read_header_string(*rsrc.get(), alt_key, alt_str);
    if (alt_str == "") 
      vw_throw( ArgumentErr() << "Cannot find the 'Altitude' metadata in the image: " 
		<< opt.ortho_image
                << ". It should then be specified on the command line as --camera-height.\n");
    
    cam_height = atof(alt_str.c_str());
  }

  std::vector<vw::ip::InterestPoint> raw_ip, ortho_ip;
  ip::read_binary_match_file(match_filename, raw_ip, ortho_ip);
  vw::camera::PinholeModel *pcam = dynamic_cast<vw::camera::PinholeModel*>(cam.get());
  if (pcam == NULL) {
    vw_throw(ArgumentErr() << "Expecting a pinhole camera model.\n");
  }

  if ( !(pcam->camera_pose() == Quaternion<double>(1, 0, 0, 0)) ) {
    // We like to start with a camera pointing along the z axis. That makes
    // the lfie easy.
    vw_throw(ArgumentErr() << "Expecting the input camera to have identity rotation.\n");
  }

  // Find pairs of points. First point is in the camera coordinate system,
  // second in the ground coordinate system. This will allow us to transform
  // the camera to the latter system.
  // We use two versions of ground points. In one we assume constant height,
  // in the second we pull the heights from a DEM. If there are enough
  // of the latter kind of points, we use those.
  std::vector<Vector3> points_in1, points_out1, llh1;
  std::vector<Vector3> points_in2, points_out2, llh2;
  std::vector<Vector2> pix1, pix2;
  
  for (size_t ip_iter = 0; ip_iter < raw_ip.size(); ip_iter++){
    try {
      Vector2 raw_pix(raw_ip[ip_iter].x, raw_ip[ip_iter].y);
      Vector3 ctr = pcam->camera_center(raw_pix);
      Vector3 dir = pcam->pixel_to_vector(raw_pix);
      
      // We assume the ground is flat. Intersect rays going from camera
      // with the plane z = cam_height.
      Vector3 point_in = ctr + (cam_height/dir[2])*dir;
      
      Vector2 ortho_pix(ortho_ip[ip_iter].x, ortho_ip[ip_iter].y);
      Vector2 ll = ortho_georef.pixel_to_lonlat(ortho_pix);
      
      // Use given assumed height of orthoimage above the datum.
      Vector3 llh(ll[0], ll[1], opt.orthoimage_height);
      Vector3 point_out = ortho_georef.datum().geodetic_to_cartesian(llh);

      points_in1.push_back(point_in);
      points_out1.push_back(point_out);
      llh1.push_back(llh);
      pix1.push_back(raw_pix);
      
      // Now let's see if the DEM is any good
      if (has_ref_dem) {
        PixelMask<float> dem_val; dem_val.invalidate();
        Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
        double x = dem_pix.x(), y = dem_pix.y();
        if (0 <= x && x <= dem.cols() - 1 && 0 <= y && y <= dem.rows() - 1 ) {
          InterpolationView<EdgeExtensionView< ImageViewRef< PixelMask<float> >, ConstantEdgeExtension >, BilinearInterpolation> interp_dem = interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());
          dem_val = interp_dem(x, y);
          if (is_valid(dem_val)) {
            Vector3 llh(ll[0], ll[1], dem_val.child());
            Vector3 point_out = dem_georef.datum().geodetic_to_cartesian(llh);
            points_in2.push_back(point_in);
            points_out2.push_back(point_out);
            llh2.push_back(llh);
            pix2.push_back(raw_pix);
          }
        } // end considering the reference DEM
        
      }
    }
    catch(...){} // camera operations can throw an exception
  }

  // See if enough points on the DEM were found
  if (has_ref_dem) {
    int num = 8;
    if (int(points_out2.size()) < num) {
      vw_out() << "Less than " << num << " interest points were on the DEM. Igorning the DEM "
               << "and using the constant height: " << opt.orthoimage_height << "\n";
    }else{
      points_in1  = points_in2;
      points_out1 = points_out2;
      llh1        = llh2;
      pix1        = pix2;
      points_in2.clear(); points_out2.clear(); llh2.clear(); pix2.clear();
    }
  }
  int num_pts = points_in1.size();
  vw_out() << "Using " << num_pts << " points to create the camera model.\n";
  vw::Matrix<double> points_in(3, num_pts), points_out(3, num_pts);
  typedef vw::math::MatrixCol<vw::Matrix<double> > ColView;
  for (int pt_iter = 0; pt_iter < num_pts; pt_iter++){
    ColView colIn (points_in,  pt_iter);  colIn  = points_in1[pt_iter];
    ColView colOut(points_out, pt_iter);  colOut = points_out1[pt_iter];
  }

  // Call function to compute a 3D affine transform between the two point sets
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale;
  asp::find_3D_affine_transform(points_in, points_out, rotation, translation, scale);

  vw_out() << "Determined camera extrinsics from orthoimage: " << std::endl;
  vw_out() << "rotation\n" << rotation << std::endl;
  vw_out() << "translation\n" << translation << std::endl;
  vw_out() << "scale\n" << scale << std::endl;

  // Estimate the error of this transform.
  double max_err = 0.0;
  for (size_t pt_iter = 0; pt_iter < num_pts; pt_iter++){
    ColView colIn (points_in,  pt_iter); 
    ColView colOut(points_out, pt_iter);
    vw::Vector3 point_in  = colIn;
    vw::Vector3 point_out = colOut;
    Vector3 in_trans =  scale*rotation*point_in + translation;
    max_err = std::max(max_err,  norm_2(in_trans - point_out));
  }
  vw_out() << "Max error on the ground in meters: " << max_err << std::endl;
  
  // Apply the transform to the camera.
  pcam->apply_transform(rotation, translation, scale);
  
  vw_out() << "Writing: " << opt.output_cam << std::endl;
  pcam->write(opt.output_cam);

  // Save a gcp file, later bundle_adjust can use it to improve upon this camera model
  std::string gcp_file = opt.output_cam + ".gcp";
  vw_out() << "Writing: " << gcp_file << std::endl;
  std::ofstream output_handle(gcp_file.c_str());
  int pts_count = 0;
  for (int pt_iter = 0; pt_iter < num_pts; pt_iter++) { // Loop through IPs

    Vector3 llh = llh1[pt_iter];
    Vector2 pix = pix1[pt_iter];
    Vector2 lonlat = subvector(llh, 0, 2);
    double height = llh[2];

    // The ground control point ID
    output_handle << pts_count;
    
    // Lat, lon, height
    output_handle << ", " << lonlat[1] << ", " << lonlat[0] << ", " << height;

    // Sigma values
    output_handle << ", " << 1 << ", " << 1 << ", " << 1;

    // Pixel value
    output_handle << ", " <<  opt.raw_image;
    output_handle << ", " << pix.x() << ", " << pix.y(); // IP location in image
    output_handle << ", " << 1 << ", " << 1; // Sigma values
    output_handle << std::endl; // Finish the line
    pts_count++;
    
  } // End loop through IPs
  output_handle.close();

  vw_out() << "Removing: " << match_filename << std::endl;
  boost::filesystem::remove(match_filename);
}  

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("camera-height",   po::value(&opt.camera_height)->default_value(-1.0),
     "The approximate height above the datum, in meters, at which the camera should be. If not specified, it will be read from the orthoimage metadata.")
    ("orthoimage-height",   po::value(&opt.orthoimage_height)->default_value(0.0),
     "The approximate height above the datum, in meters, at which the orthoimage is positioned. We assume flat ground.")
    ("ip-per-tile",             po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
    ("ip-detect-method",po::value(&opt.ip_detect_method)->default_value(1),
     "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("individually-normalize",   po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
     "Individually normalize the input images instead of using common values.")
    ("reference-dem",             po::value(&opt.reference_dem)->default_value(""),
     "If provided, extract from this DEM the heights above the ground rather than assuming the value in --orthoimage-height.");

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
  asp::stereo_settings().ip_matching_method     = opt.ip_detect_method;
  asp::stereo_settings().individually_normalize = opt.individually_normalize;
  
  if ( opt.raw_image.empty() )
    vw_throw( ArgumentErr() << "Missing input raw image.\n" << usage << general_options );

  if ( opt.ortho_image.empty() )
    vw_throw( ArgumentErr() << "Missing input ortho image.\n" << usage << general_options );

  if ( opt.input_cam.empty() )
    vw_throw( ArgumentErr() << "Missing input pinhole camera.\n" << usage << general_options );

  if ( opt.output_cam.empty() )
    vw_throw( ArgumentErr() << "Missing output pinhole camera.\n" << usage << general_options );

  // Create the output directory
  vw::create_out_dir(opt.output_cam);
}

// ================================================================================

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    ortho2pinhole(opt);
  } ASP_STANDARD_CATCHES;
}
