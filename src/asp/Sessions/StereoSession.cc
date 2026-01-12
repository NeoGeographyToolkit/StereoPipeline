// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file StereoSession.cc
///

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/AspStringUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/asp_config.h>

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <asp/Core/AlignmentUtils.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Geometry.h>
#include <vw/Image/PixelMask.h>
#include <vw/Image/PixelTypeInfo.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/FileIO/DiskImageUtils.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace fs = boost::filesystem;

namespace asp {

  // Pass over all the string variables we use
  void StereoSession::initialize(vw::GdalWriteOptions const& options,
                                 std::string const& left_image_file,
                                 std::string const& right_image_file,
                                 std::string const& left_camera_file,
                                 std::string const& right_camera_file,
                                 std::string const& out_prefix,
                                 std::string const& input_dem) {
    m_options           = options;
    m_left_image_file   = left_image_file;
    m_right_image_file  = right_image_file;
    m_left_camera_file  = left_camera_file;
    m_right_camera_file = right_camera_file;
    m_out_prefix        = out_prefix;
    m_input_dem         = input_dem;

    // Read the cameras used in mapprojection
    if (isMapProjected())
      read_mapproj_cams(m_left_image_file, m_right_image_file,
                        m_left_camera_file, m_right_camera_file,
                        m_input_dem, this->name(),
                        m_left_map_proj_model, m_right_map_proj_model);
  }

  // Read keywords that describe how the images were map-projected.
  // Do sanity checks.
  void read_mapproj_header_session(std::string const& map_file,
                                   std::string const& input_cam_file,
                                   std::string const& input_dem,
                                   std::string const& session_name,
                                   // Outputs
                                   std::string & adj_prefix,
                                   std::string & image_file, std::string & cam_type,
                                   std::string & cam_file, std::string & dem_file) {

    // Call a lower-level function to do the reading
    std::string adj_key, img_file_key, cam_type_key, cam_file_key, dem_file_key;
    asp::read_mapproj_header(map_file,
                             adj_key, img_file_key, cam_type_key, cam_file_key, dem_file_key,
                             adj_prefix, image_file, cam_type, cam_file, dem_file);

    // Sanity checks

    bool has_geoheader_cam_type = true;
    if (cam_type == "") {
      // If cam_type is empty, and session name is XmapY, use cam type Y.
      std::string tri_cam_type, mapproj_cam_type;
      asp::parseCamTypes(session_name, tri_cam_type, mapproj_cam_type);
      vw::vw_out(WarningMessage)
           << "Assuming mapprojection was done with camera type: "
           << mapproj_cam_type << "\n";
      cam_type = mapproj_cam_type;
      has_geoheader_cam_type = false;
    }

    bool has_geoheader_image = true;
    if (image_file == "") {
      image_file = map_file; // should be enough to load the camera
      has_geoheader_image = false;
    }

    // This is very hard to get right
    if (cam_file == "") {
      if (has_geoheader_cam_type && has_geoheader_image) {
        // The mapprojection info was saved, but the camera file was empty.
        // Then, the image file before mapprojection should have the camera.
        cam_file = image_file;
      } else {
        // The mapprojection info was not saved. Use the user-supplied
        // camera.
        cam_file = input_cam_file;
      }
    }

    if (dem_file == "") {
      vw::vw_out(WarningMessage) << "Missing field value for: " << dem_file_key
                                 << " in " << map_file << ".\n";
      dem_file = input_dem; // should be enough to load the DEM
      vw::vw_out(WarningMessage) << "Using DEM: " << dem_file << " instead.\n";
    }

    vw::Vector2 heights = asp::stereo_settings().ortho_heights;
    bool have_heights = (!std::isnan(heights[0]) && !std::isnan(heights[1]));

    // TODO(oalexan1): When do not have heights, the dem_file must be non-empty
    // and must exist.

    // The DEM the user provided better be the one used for map projection.
    // Give an error, as the results can be very different with the wrong DEM.
    if (input_dem != dem_file && !asp::stereo_settings().accept_provided_mapproj_dem
        && !have_heights)
      vw::vw_throw(ArgumentErr()
          << "The DEM used for map projection is different from the one "
          << "provided currently.\n"
          << "Mapprojection DEM: " << dem_file << "\n"
          << "Current DEM: " << input_dem << "\n"
          << "To override this error, use: --accept-provided-mapproj-dem.\n"
          << "Or, to change the DEM name in the geoheader of mapprojected images, run\n"
          << "  image_calc -c var_0 --mo DEM_FILE=myDem.tif in.tif -o out.tif -d float32\n"
          << "but only if sure that the DEMs are equivalent.\n");

    // If this is set to none, it means no bundle-adjust prefix was used
    if (adj_prefix == "NONE")
      adj_prefix = "";
  }

  // When loading camera models from the image files, we either use the sensor
  // model for the current session type or else the RPC model which is often
  // used as an approximation. If that fails, create a new session from scratch
  // and load the camera model with that.
  void StereoSession::read_mapproj_cam(std::string const& image_file,
                                       std::string const& cam_file,
                                       std::string const& adj_prefix,
                                       std::string const& cam_type,
                                       vw::CamPtr & map_proj_cam) {

    const Vector2 zero_pixel_offset(0,0);
    try {
      if (cam_type == "rpc") {
        map_proj_cam = load_rpc_camera_model(image_file, cam_file,
                                             adj_prefix, zero_pixel_offset);
      } else { // Use the native model
        map_proj_cam = load_camera_model(image_file, cam_file,
                                         adj_prefix, zero_pixel_offset);
      }
    } catch (std::exception const& e) {
      std::string msg = e.what();
      vw_out() << "Creating a new session to load the mapprojected camera model.\n";
      std::string session_type = cam_type;
      std::string out_prefix;
      try {
        asp::SessionPtr session(asp::StereoSessionFactory::create
                            (session_type, // may change
                             m_options, image_file, image_file, cam_file, cam_file,
                             out_prefix));
        map_proj_cam = session->load_camera_model(image_file, cam_file,
                                                  adj_prefix, zero_pixel_offset);
      } catch (std::exception const& e) {
        vw_throw(ArgumentErr() << "Failed to load the camera model from " << cam_file
                << " for image: " << image_file << ".\n"
                << "First error message: " << msg << "\n"
                << "Second error message: " << e.what() << "\n");
      }
    }

    return;
  }

  // Read the cameras used to undo the mapprojection. The header file of the
  // mapprojected images contain a lot of info that we load along the way,
  // including the bundle adjust prefix that was used to create these images,
  // which may be different than the one used later for triangulation.
  void StereoSession::read_mapproj_cams(std::string const& left_image_file,
                                        std::string const& right_image_file,
                                        std::string const& left_camera_file,
                                        std::string const& right_camera_file,
                                        std::string const& input_dem,
                                        std::string const& session_name,
                                        vw::CamPtr & left_map_proj_cam,
                                        vw::CamPtr & right_map_proj_cam) {

    std::string l_adj_prefix, r_adj_prefix, l_image_file, r_image_file,
    l_cam_type, r_cam_type, l_cam_file, r_cam_file, l_dem_file, r_dem_file;

    vw::Vector2 heights = asp::stereo_settings().ortho_heights;
    bool have_heights = (!std::isnan(heights[0]) && !std::isnan(heights[1]));

    if (have_heights) {
      // For ortho-ready images it is assumed the mapprojection was done
      // with the original cameras and no bundle adjustment prefix was used.
      l_cam_file = left_camera_file;
      r_cam_file = right_camera_file;
      l_cam_type = session_name;
      r_cam_type = session_name;

      // If not having cameras, use the images
      if (l_cam_file == "")
        l_cam_file = left_image_file;
      if (r_cam_file == "")
        r_cam_file = right_image_file;
    } else {
      // Load the name of the camera model, session, and DEM used in mapprojection
      // based on the record in that image. Load the bundle adjust prefix from the
      // mapprojected image. It can be empty, when such a prefix was not used in
      // mapprojection.
      read_mapproj_header_session(left_image_file, left_camera_file, input_dem,
                                  session_name, l_adj_prefix, l_image_file, l_cam_type,
                                  l_cam_file, l_dem_file);
      read_mapproj_header_session(right_image_file, right_camera_file, input_dem,
                                  session_name, r_adj_prefix, r_image_file, r_cam_type,
                                  r_cam_file, r_dem_file);
    }

    vw_out() << "Images before mapprojection: "
      << l_image_file << ' ' << r_image_file << "\n";
    vw_out() << "Mapprojection cameras: " << l_cam_file << ' ' << r_cam_file << "\n";
    vw_out() << "Mapprojected images bundle adjustment prefixes: "
              << l_adj_prefix << ' ' << r_adj_prefix << "\n";
    vw_out() << "Mapprojection cam types: " << l_cam_type << ' ' << r_cam_type << "\n";

    // Load either the current session camera type, or rpc, or form a new session.
    read_mapproj_cam(l_image_file, l_cam_file, l_adj_prefix, l_cam_type, left_map_proj_cam);
    read_mapproj_cam(r_image_file, r_cam_file, r_adj_prefix, r_cam_type, right_map_proj_cam);

    VW_ASSERT(left_map_proj_cam.get() && right_map_proj_cam.get(),
              ArgumentErr() << "StereoSession: Unable to locate map "
              << "projection camera model inside input files.");

    // Double check that we can read the DEM and that it has cartographic information.
    if (input_dem.empty() && !have_heights)
       vw_throw(ArgumentErr() << "StereoSession: An input DEM is required.");
    if (!boost::filesystem::exists(input_dem) && !have_heights)
      vw_throw(ArgumentErr() << "StereoSession: DEM '" << input_dem << "' does not exist.");
  }

  // Default implementation of this function.  Derived classes will probably override this.
  void StereoSession::camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                    boost::shared_ptr<vw::camera::CameraModel> &cam2) {
    cam1 = camera_model(m_left_image_file,  m_left_camera_file);
    cam2 = camera_model(m_right_image_file, m_right_camera_file);
  }

// This will be overridden in some sessions
bool StereoSession::have_datum() const {
  return !asp::stereo_settings().no_datum && !stereo_settings().correlator_mode;
}

// Returns the target datum to use for a given camera model.
// Can be overridden by derived classes.
// If no success finding the datum, will return WGS84.
// TODO(oalexan1): This must return a flag indicating if the datum was found.
vw::cartography::Datum StereoSession::get_datum(const vw::camera::CameraModel* cam,
                                                bool use_sphere_for_non_earth) const {

  if (!stereo_settings().datum.empty())
    return vw::cartography::Datum(stereo_settings().datum);

  // For ISIS, can query the camera. This code is invoked for stereo with
  // mapprojected ISIS cameras, as that use the mapprojected session, and not
  // the ISIS one. In the latter case the ISIS session has this own
  // implementation of this function.
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  vw::camera::IsisCameraModel const* isis_cam
    = dynamic_cast<vw::camera::IsisCameraModel const*>(unadjusted_model(cam));
  if (isis_cam != NULL)
    return isis_cam->get_datum_isis(use_sphere_for_non_earth);
#endif // ASP_HAVE_PKG_ISIS

  // Do same for csm
  asp::CsmModel const* csm_cam
    = dynamic_cast<asp::CsmModel const*>(unadjusted_model(cam));
  if (csm_cam != NULL)
    return csm_cam->get_datum_csm("unknown", use_sphere_for_non_earth);

  // Same for RPC
  asp::RPCModel const* rpc_cam
     = dynamic_cast<const asp::RPCModel*>(vw::camera::unadjusted_model(cam));
  if (rpc_cam != NULL)
     return rpc_cam->datum();

  // TODO(oalexan1): Need to find a systematic way of handling all
  // these cases.

  // Otherwise guess the datum based on the camera position.
  // If no luck, it will return the default WGS84 datum.
  // TODO(oalexan1): This may result in a bad datum for other planets.
  double cam_center_radius
      = norm_2(cam->camera_center(vw::Vector2()));
  vw::cartography::Datum datum;
  asp::guessDatum(cam_center_radius, datum);

  return datum;
}

// Peek inside the images and camera models and return the datum and projection,
// or at least the datum, packaged in a georef.
// If no success finding the datum, will return WGS84.
vw::cartography::GeoReference StereoSession::get_georef() {

  vw::cartography::GeoReference georef;

  // First try to see if the image is map-projected.
  bool has_georef = read_georeference(georef, m_left_image_file);

  bool has_datum = false;
  vw::cartography::Datum datum;
  if (!stereo_settings().correlator_mode) {
    has_datum = true;
    vw::CamPtr cam = this->camera_model(m_left_image_file, m_left_camera_file);
    // Spherical datum for non-Earth, as done usually. Used
    // consistently this way in bundle adjustment and stereo.
    bool use_sphere_for_non_earth = true;
    datum = this->get_datum(cam.get(), use_sphere_for_non_earth);
  }

  // Sanity check
  if (has_georef && has_datum) {
    // This check is very important, as it prevents a mixup of datums from
    // different planets. The guessed datum may be unreliable, so always warn only.
    bool warn_only = true;
    vw::checkDatumConsistency(georef.datum(), datum, warn_only);
  }

  if (!has_georef) {
    // The best we can do is to get the datum, even non-projected
    // images have that. Create however a fake valid georeference to
    // go with this datum, otherwise we can't read the datum when we
    // needed it later.

    georef = vw::cartography::GeoReference();
    Matrix3x3 transform = georef.transform();

    // assume these are degrees, does not mater much, but it needs be small enough
    double small = 1e-8;
    transform(0,0) = small;  // grid in x
    transform(1,1) = -small; // grid in y; y always goes down
    transform(0,2) = 0; // origin x
    transform(1,2) = 0; // origin y
    georef.set_transform(transform);

    georef.set_geographic(); // default projection

    if (has_datum)
      georef.set_datum(datum);
  }

  return georef;
}

boost::shared_ptr<vw::camera::CameraModel>
StereoSession::camera_model(std::string const& image_file, std::string const& camera_file,
                            bool quiet) {

  // TODO(oalexan1): ba_prefix better be passed from outside
  std::string ba_prefix = asp::stereo_settings().bundle_adjust_prefix;

  if (stereo_settings().correlator_mode) {
    // No cameras exist, so make some dummy cameras. Recall that we
    // set the session to rpc in this mode so that it is assumed that
    // the cameras may hiding in the images rather than kept
    // separately. In this mode the cameras should not actually get
    // used.
    vw::Vector<double, 20> v; v[0] = 1.0; // to a void a zero denominator
    vw::Vector2 v2(1.0, 1.0); // to avoid division by 0
    vw::Vector3 v3(1.0, 1.0, 1.0);
    boost::shared_ptr<vw::camera::CameraModel>
      cam(new RPCModel(vw::cartography::Datum("WGS84"), v, v, v, v, v2, v2, v3, v3));
    return cam;
  }

  // If the desired camera is already loaded, do not load it again.
  std::pair<std::string, std::string> image_cam_pair = std::make_pair(image_file, camera_file);

  auto map_it = m_camera_model.find(image_cam_pair);
  if (map_it != m_camera_model.end())
    return map_it->second;

  // Sometime when we do many attempts at loading cameras we don't want to print
  // this message.
  if (!quiet)
    vw_out() << "Loading camera model: " << image_file << ' ' << camera_file << "\n";

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 pixel_offset = camera_pixel_offset(isMapProjected(),
                                                 m_left_image_file,
                                                 m_right_image_file,
                                                 image_file);

  vw::CamPtr cam;
  if (camera_file == "") // No camera file provided, use the image file.
    cam = load_camera_model(image_file, image_file, ba_prefix, pixel_offset);
  else // Camera file provided
    cam  = load_camera_model(image_file, camera_file, ba_prefix, pixel_offset);

  {
    // Save the camera model in the map to not load it again. Ensure thread safety.
    vw::Mutex::Lock lock(m_camera_mutex);
    m_camera_model[image_cam_pair] = cam;
  }

  return cam;
}

// Find the masked images and stats. This will be reimplemented in StereoSessionIsis.
void StereoSession::
calcStatsMaskedImages(// Inputs
                      vw::ImageViewRef<float> const& left_cropped_image,
                      vw::ImageViewRef<float> const& right_cropped_image,
                      float left_nodata_value, float right_nodata_value,
                      std::string const& left_input_file,
                      std::string const& right_input_file,
                      std::string const& left_cropped_file,
                      std::string const& right_cropped_file,
                      // Outputs
                      vw::ImageViewRef<vw::PixelMask<float>> & left_masked_image,
                      vw::ImageViewRef<vw::PixelMask<float>> & right_masked_image,
                      vw::Vector6f & left_stats, 
                      vw::Vector6f & right_stats) const {

  left_masked_image = create_mask(left_cropped_image, left_nodata_value);
  right_masked_image = create_mask(right_cropped_image, right_nodata_value);

  // Compute input image statistics. This can be slow so use a timer.
  vw::Stopwatch sw1;
  sw1.start();
  left_stats = gather_stats(left_masked_image,  "left",
                            this->m_out_prefix, left_cropped_file,
                            asp::stereo_settings().force_reuse_match_files);
  sw1.stop();
  vw_out() << "Left image stats time: " << sw1.elapsed_seconds() << std::endl;
  vw::Stopwatch sw2;
  sw2.start();
  right_stats = gather_stats(right_masked_image, "right",
                             this->m_out_prefix, right_cropped_file,
                             asp::stereo_settings().force_reuse_match_files);
  sw2.stop();
  vw_out() << "Right image stats time: " << sw2.elapsed_seconds() << std::endl;

}

// Default preprocessing hook. Some sessions may override it.
void StereoSession::preprocessing_hook(bool adjust_left_image_size,
                                       std::string const& left_input_file,
                                       std::string const& right_input_file,
                                       std::string      & left_output_file,
                                       std::string      & right_output_file) {

  std::string left_cropped_file, right_cropped_file;
  ImageViewRef<float> left_cropped_image, right_cropped_image;
  vw::GdalWriteOptions options;
  float left_nodata_value, right_nodata_value;
  bool has_left_georef, has_right_georef;
  vw::cartography::GeoReference left_georef, right_georef;
  bool exit_early =
    StereoSession::shared_preprocessing_hook(options,
                                             left_input_file,    right_input_file,
                                             left_output_file,   right_output_file,
                                             left_cropped_file,  right_cropped_file,
                                             left_cropped_image, right_cropped_image,
                                             left_nodata_value,  right_nodata_value,
                                             has_left_georef,    has_right_georef,
                                             left_georef,        right_georef);

  if (exit_early)
    return;

  // Get the image sizes. Later alignment options can choose to
  // change this parameters (such as affine epipolar alignment).
  Vector2i left_size(left_cropped_image.cols(), left_cropped_image.rows());
  Vector2i right_size(right_cropped_image.cols(), right_cropped_image.rows());

  ImageViewRef<PixelMask<float>> left_masked_image, right_masked_image;
  Vector6f left_stats, right_stats;

  // Set up image masks. If the user provided a custom no-data value, values no
  // more than that have been masked by now in shared_preprocessing_hook.
  // This is reimplemented for ISIS.
  this->calcStatsMaskedImages(// Inputs
                              left_cropped_image, right_cropped_image,
                              left_nodata_value, right_nodata_value,
                              left_input_file, right_input_file,
                              left_cropped_file, right_cropped_file,
                              // Outputs
                              left_masked_image, right_masked_image,
                              left_stats, right_stats);

  // These stats will be needed later on
  if (stereo_settings().alignment_method == "local_epipolar")
    asp::saveStats(this->m_out_prefix, left_stats, right_stats);
  
  // Initialize the alignment matrices
  Matrix<double> align_left_matrix  = math::identity_matrix<3>();
  Matrix<double> align_right_matrix = math::identity_matrix<3>();

  ImageViewRef<PixelMask<float>> Limg, Rimg;
  bool isis_session = (this->name() == "isis" || this->name() == "isismapisis");
  
  // Use no-data in interpolation and edge extension
  // TODO(oalexan1): Maybe using 0 for nodata_pix is not good. May need to use
  // -32768.0.
  PixelMask<float>nodata_pix(0); nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> ext_nodata(nodata_pix);

  // Generate aligned versions of the input images according to the
  // options.
  vw_out() << "\t--> Applying alignment method: "
           << stereo_settings().alignment_method << "\n";
  if (stereo_settings().alignment_method == "epipolar") {

    if (isis_session)
      vw_throw(NoImplErr() << "StereoSessionISIS does not support epipolar rectification");
      
    epipolar_alignment(left_masked_image, right_masked_image, ext_nodata,
                       // Outputs
                       Limg, Rimg);

  } else if (stereo_settings().alignment_method == "homography"     ||
             stereo_settings().alignment_method == "affineepipolar" ||
             stereo_settings().alignment_method == "local_epipolar") {

    // Load the cameras
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    this->camera_models(left_cam, right_cam);

    imageAlignment(// Inputs
                   m_out_prefix, left_cropped_file, right_cropped_file,
                   left_input_file,
                   left_stats, right_stats, left_nodata_value, right_nodata_value,
                   left_cam, right_cam,
                   adjust_left_image_size,
                   // In-out
                   align_left_matrix, align_right_matrix, left_size, right_size);

    // Apply the alignment transform to both input images
    Limg = transform(left_masked_image,
                     HomographyTransform(align_left_matrix),
                     left_size.x(), left_size.y());
    Rimg = transform(right_masked_image,
                     HomographyTransform(align_right_matrix),
                     right_size.x(), right_size.y());

  } else {
    // No alignment, just provide the original files.
    Limg = left_masked_image;
    Rimg = right_masked_image;
  } // End of image alignment block

  // Apply our normalization options.
  bool use_percentile_stretch = false;
  bool do_not_exceed_min_max = isis_session; // for isis, do not exceed min/max
  // TODO(oalexan1): Should one add above "csm" and "csmmapcsm" / "csmmaprpc"?
  asp::normalize_images(stereo_settings().force_use_entire_range,
                        stereo_settings().individually_normalize,
                        use_percentile_stretch,
                        do_not_exceed_min_max,
                        left_stats, right_stats, Limg, Rimg);

  // The output no-data value must be < 0 as we scale the images to [0, 1].
  bool has_nodata = true;
  float output_nodata = -32768.0;
  vw_out() << "\t--> Writing pre-aligned images.\n";
  vw_out() << "\t--> Writing: " << left_output_file << "\n";
  vw::Stopwatch sw3;
  sw3.start();
  block_write_gdal_image(left_output_file, apply_mask(Limg, output_nodata),
                         has_left_georef, left_georef,
                         has_nodata, output_nodata, options,
                         TerminalProgressCallback("asp","\t  L:  "));
  sw3.stop();
  vw_out() << "Writing left image elapsed time: " << sw3.elapsed_seconds() << " s\n";

  vw_out() << "\t--> Writing: " << right_output_file << "\n";
  vw::Stopwatch sw4;
  sw4.start();
  
  // With no alignment, do not crop the right image to have the same dimensions
  // as the left image. Since there is no alignment, and images may not be
  // georeferenced, we do not know what portion of the right image corresponds
  // best to the left image, so cropping may throw away an area where the left
  // and right images overlap.
  if (stereo_settings().alignment_method != "none")
    Rimg = crop(edge_extend(Rimg, ext_nodata), bounding_box(Limg));
    
  block_write_gdal_image(right_output_file, apply_mask(Rimg, output_nodata),
                         has_right_georef, right_georef,
                         has_nodata, output_nodata, options,
                         TerminalProgressCallback("asp","\t  R:  "));
  sw4.stop();
  vw_out() << "Writing right image elapsed time: " << sw4.elapsed_seconds() << " s\n";

  // For bathy runs only
  if (this->do_bathymetry())
    this->align_bathy_masks(options);

} // End function preprocessing_hook

ImageViewRef<PixelMask<Vector2f>>
StereoSession::pre_pointcloud_hook(std::string const& input_file) {
  return DiskImageView<PixelMask<Vector2f> >(input_file);
}

// A little function whose goal is to avoid repeating same logic in a handful of places
void crop_bathy_mask(vw::GdalWriteOptions const& options,
                     std::string const& input_mask_file, std::string const& input_image_file,
                     BBox2i const& crop_win, std::string const& cropped_mask_file) {

  if (input_mask_file == "")
    vw_throw(ArgumentErr() << "Required bathy mask file was not specified.");

  // Sanity check, input image and mask must have same size
  DiskImageView<float> input_image(input_image_file);
  DiskImageView<float> input_bathy_mask(input_mask_file);
  if (input_bathy_mask.cols() != input_image.cols() ||
      input_bathy_mask.rows() != input_image.rows())
    vw_throw(ArgumentErr() << "Input image and input bathy mask don't have the same dimensions.");

  float mask_nodata_value = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(input_mask_file, mask_nodata_value))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from " << input_mask_file);

  vw::cartography::GeoReference georef;
  bool has_georef = read_georeference(georef, input_image_file);

  bool has_mask_nodata = true;
  vw_out() << "\t--> Writing cropped mask: " << cropped_mask_file << "\n";
  block_write_gdal_image(cropped_mask_file,
                         crop(input_bathy_mask, crop_win),
                         has_georef, crop(georef, crop_win),
                         has_mask_nodata, mask_nodata_value,
                         options,
                         TerminalProgressCallback("asp", "\t:  "));
}

bool StereoSession::
shared_preprocessing_hook(vw::GdalWriteOptions           & options,
                          std::string const              & left_input_file,
                          std::string const              & right_input_file,
                          std::string                    & left_output_file,
                          std::string                    & right_output_file,
                          std::string                    & left_cropped_file,
                          std::string                    & right_cropped_file,
                          vw::ImageViewRef<float>        & left_cropped_image,
                          vw::ImageViewRef<float>        & right_cropped_image,
                          float                          & left_nodata_value,
                          float                          & right_nodata_value,
                          bool                           & has_left_georef,
                          bool                           & has_right_georef,
                          vw::cartography::GeoReference  & left_georef,
                          vw::cartography::GeoReference  & right_georef) {

  // Retrieve nodata values and let the handles go out of scope right away.
  // For this to work the ISIS type must be registered with the
  // DiskImageResource class. This happens in "stereo.cc", so
  // these calls will create DiskImageResourceIsis objects.
  {
    boost::shared_ptr<DiskImageResource>
      left_rsrc (DiskImageResourcePtr(left_input_file)),
      right_rsrc(DiskImageResourcePtr(right_input_file));
    asp::get_nodata_values(left_rsrc, right_rsrc,
                           stereo_settings().nodata_value,
                           left_nodata_value, right_nodata_value);
  }

  // Set output file paths
  left_output_file  = this->m_out_prefix + "-L.tif";
  right_output_file = this->m_out_prefix + "-R.tif";

  // Enforce no predictor in compression, it works badly with L.tif and R.tif.
  options = this->m_options;
  options.gdal_options["PREDICTOR"] = "1";

  // Read the georef if available in the input images
  has_left_georef  = read_georeference(left_georef,  left_input_file);
  has_right_georef = read_georeference(right_georef, right_input_file);
  if (stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef = false;
    has_right_georef = false;
  }

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  // Here either the input image or the cropped images will be returned,
  // depending on whether the crop actually happens
  left_cropped_file = this->left_cropped_image();
  right_cropped_file = this->right_cropped_image();

  // If the output files already exist and are newer than the input files,
  // and we don't crop both left and right images, then there is nothing to do here.
  // Note: Must make sure all outputs are initialized before we
  // get to this part where we exit early.

  bool do_bathy = StereoSession::do_bathymetry();

  std::vector<std::string> check_files;
  check_files.push_back(left_input_file);
  check_files.push_back(right_input_file);
  check_files.push_back(m_left_camera_file);
  check_files.push_back(m_right_camera_file);
  bool rebuild = (!first_is_newer(left_output_file, check_files) ||
                  !first_is_newer(right_output_file, check_files));

  if (do_bathy) {
    rebuild = (rebuild ||
               (!first_is_newer(left_aligned_bathy_mask(), check_files) ||
                !first_is_newer(right_aligned_bathy_mask(), check_files)));
  }

  // Consider the case of multi-band images
  if (!rebuild) {
    int lc = vw::get_num_channels(left_input_file);
    int rc = vw::get_num_channels(right_input_file);
    if (lc > 1 || rc > 1) {
      vw_out(vw::WarningMessage)
        << "Always recomputing the inputs for multi-band images as the "
        << "provided band may change.\n";
      rebuild = true;
    }
  }

  if (!rebuild && !crop_left && !crop_right) {
    try {
      vw_log().console_log().rule_set().add_rule(-1, "fileio");
      DiskImageView<PixelGray<float32>> out_left (left_output_file);
      DiskImageView<PixelGray<float32>> out_right(right_output_file);

      if (do_bathy) {
        DiskImageView<float> left_bathy_mask (left_aligned_bathy_mask());
        DiskImageView<float> right_bathy_mask(right_aligned_bathy_mask());
      }

      vw_out(InfoMessage) << "\t--> Using cached normalized input images.\n";
      vw_settings().reload_config();
      return true; // Return true if we exist early since the images exist
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  // Load the desired band. Subtract 1 to make it start from 0.
  int ch = asp::stereo_settings().band - 1;
  ImageViewRef<float> left_orig_image = vw::read_channel<float>(left_input_file, ch);
  ImageViewRef<float> right_orig_image = vw::read_channel<float>(right_input_file, ch);

  // If the user provided a custom no-data value, values no more than that are
  // masked.
  float user_nodata = stereo_settings().nodata_value;
  if (!std::isnan(user_nodata)) {
    left_orig_image = apply_mask(create_mask_less_or_equal(left_orig_image, user_nodata),
                                  user_nodata);
    right_orig_image = apply_mask(create_mask_less_or_equal(right_orig_image, user_nodata),
                                   user_nodata);
  }

  // See if to crop the images
  if (crop_left) {
    // Crop and save the left image to left_cropped_file
    has_left_georef = read_georeference(left_georef, left_input_file);
    bool has_nodata = true;
    BBox2i left_win = stereo_settings().left_image_crop_win;
    left_win.crop(bounding_box(left_orig_image));
    // Return a handle to the cropped image
    left_cropped_image = crop(left_orig_image, left_win);

    if (stereo_settings().left_image_clip != "") {
      // Replace the crop with a given clip. This is a very rarely used option.
      // It can be handy when investigating CCD artifacts correction.
      left_cropped_image = DiskImageView<float>(stereo_settings().left_image_clip);
      if (left_cropped_image.cols() != left_win.width() ||
          left_cropped_image.rows() != left_win.height()) {
        vw_throw(ArgumentErr() << "The image specified via --left-image-clip has different "
                  << "dimensions than set via --left-image-crop-win.");
      }
    }
    
    // Crop the georef as well
    vw_out() << "\t--> Writing cropped image: " << left_cropped_file << "\n";
    block_write_gdal_image(left_cropped_file,
                           left_cropped_image,
                           has_left_georef, crop(left_georef, left_win),
                           has_nodata, left_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  } else {
    // Return a handle to the desired channel of the input image
    left_cropped_image = left_orig_image;
  }

  if (crop_right) {
    // Crop the right image and write to right_cropped_file
    has_right_georef = read_georeference(right_georef, right_input_file);
    bool has_nodata = true;
    BBox2i right_win = stereo_settings().right_image_crop_win;
    right_win.crop(bounding_box(right_orig_image));
    // Return a handle to the cropped image
    right_cropped_image = crop(right_orig_image, right_win);

    if (stereo_settings().right_image_clip != "") {
      // Replace the crop with a given clip. This is a very rarely used option.
      // It can be handy when investigating CCD artifacts correction.
      right_cropped_image = DiskImageView<float>(stereo_settings().right_image_clip);
      if (right_cropped_image.cols() != right_win.width() ||
          right_cropped_image.rows() != right_win.height()) {
        vw_throw(ArgumentErr() << "The image specified via --right-image-clip has different "
                  << "dimensions than set via --right-image-crop-win.");
      }
    }
    
    // Crop the georef as well
    vw_out() << "\t--> Writing cropped image: " << right_cropped_file << "\n";
    block_write_gdal_image(right_cropped_file,
                           right_cropped_image,
                           has_right_georef,
                           crop(right_georef, right_win),
                           has_nodata, right_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  } else {
    // Return a handle to the desired channel of the input image
    right_cropped_image = right_orig_image;
  }

  // Re-read the georef, since it may have changed above.
  has_left_georef  = read_georeference(left_georef,  left_cropped_file);
  has_right_georef = read_georeference(right_georef, right_cropped_file);
  if (stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef  = false;
    has_right_georef = false;
  }

  return false; // don't exit early
}

void StereoSession::read_bathy_masks(float & left_bathy_nodata, float & right_bathy_nodata,
                                     vw::ImageViewRef<vw::PixelMask<float>> & left_bathy_mask,
                                     vw::ImageViewRef<vw::PixelMask<float>> & right_bathy_mask) {

  std::string left_cropped_mask_file = left_cropped_bathy_mask();
  left_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(left_cropped_mask_file, left_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << left_cropped_mask_file);
  left_bathy_mask = create_mask(DiskImageView<float>(left_cropped_mask_file),
                                left_bathy_nodata);

  std::string right_cropped_mask_file = right_cropped_bathy_mask();
  right_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(right_cropped_mask_file, right_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << right_cropped_mask_file);
  right_bathy_mask = create_mask(DiskImageView<float>(right_cropped_mask_file),
                                right_bathy_nodata);

  // The left image (after crop) better needs to have the same dims
  // as the left mask after crop, and same for the right
  DiskImageView<float> left_image(this->left_cropped_image());
  DiskImageView<float> right_image(this->right_cropped_image());
  if (left_bathy_mask.cols() != left_image.cols()   ||
      left_bathy_mask.rows() != left_image.rows()   ||
      right_bathy_mask.cols() != right_image.cols() ||
      right_bathy_mask.rows() != right_image.rows()) {
    vw_throw(ArgumentErr() << "The dimensions of bathymetry masks don't agree "
              << "with the image sizes (after crop win, if applicable).");
  }

}

void StereoSession::read_aligned_bathy_masks
(vw::ImageViewRef<vw::PixelMask<float>> & left_aligned_bathy_mask_image,
 vw::ImageViewRef<vw::PixelMask<float>> & right_aligned_bathy_mask_image) {

  std::string left_aligned_mask_file = left_aligned_bathy_mask();
  float left_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(left_aligned_mask_file, left_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << left_aligned_mask_file);
  left_aligned_bathy_mask_image = create_mask(DiskImageView<float>(left_aligned_mask_file),
                                              left_bathy_nodata);

  std::string right_aligned_mask_file = right_aligned_bathy_mask();
  float right_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(right_aligned_mask_file, right_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << right_aligned_mask_file);
  right_aligned_bathy_mask_image = create_mask(DiskImageView<float>(right_aligned_mask_file),
                                right_bathy_nodata);
}

bool StereoSession::do_bathymetry() const {
  // For stereo will use left and right bathy masks. For bundle adjustment and
  // jitter_solve, will use a list of masks.
  return (stereo_settings().left_bathy_mask  != "" ||
          stereo_settings().right_bathy_mask != "" ||
          stereo_settings().bathy_mask_list  != "");
}

// Align the bathy masks. This will be called in stereo_pprc and, if
// needed, in stereo_tri. Skip this if the masks already exit and are
// not older than the images. This code mirrors very closely the logic
// for how the images are aligned.
// TODO(oalexan1): Such duplication of logic is not good.
void StereoSession::align_bathy_masks(vw::GdalWriteOptions const& options) {

  bool do_bathy = StereoSession::do_bathymetry();

  if (!do_bathy)
    return;

  // Check the timestamp of aligned masks
  std::vector<std::string> check_files;
  check_files.push_back(m_left_image_file);
  check_files.push_back(m_right_image_file);
  check_files.push_back(m_left_camera_file);
  check_files.push_back(m_right_camera_file);
  check_files.push_back(stereo_settings().left_bathy_mask);
  check_files.push_back(stereo_settings().right_bathy_mask);

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  bool rebuild = (!first_is_newer(left_aligned_bathy_mask(), check_files) ||
                  !first_is_newer(right_aligned_bathy_mask(), check_files));

  if (!rebuild && !crop_left && !crop_right) {
    try {
      vw_log().console_log().rule_set().add_rule(-1, "fileio");
      DiskImageView<float> left_bathy_mask (left_aligned_bathy_mask());
      DiskImageView<float> right_bathy_mask(right_aligned_bathy_mask());

      vw_out(InfoMessage) << "\t--> Using cached aligned bathy masks.\n";
      vw_settings().reload_config();
      return; // no need to rebuild, the results exit and are good

    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  // See if to crop the masks
  if (crop_left) {
    DiskImageView<float> left_orig_image(m_left_image_file);
    BBox2i left_win = stereo_settings().left_image_crop_win;
    left_win.crop(bounding_box(left_orig_image));
    crop_bathy_mask(options, stereo_settings().left_bathy_mask,
                    m_left_image_file, left_win, left_cropped_bathy_mask());
  }
  if (crop_right) {
    DiskImageView<float> right_orig_image(m_right_image_file);
    BBox2i right_win = stereo_settings().right_image_crop_win;
    right_win.crop(bounding_box(right_orig_image));
    crop_bathy_mask(options, stereo_settings().right_bathy_mask,
                    m_right_image_file, right_win, right_cropped_bathy_mask());
  }

  // Read the unaligned cropped masks
  ImageViewRef<PixelMask<float>> left_bathy_mask, right_bathy_mask;
  float left_bathy_nodata = -std::numeric_limits<float>::max();
  float right_bathy_nodata = -std::numeric_limits<float>::max();
  StereoSession::read_bathy_masks(left_bathy_nodata, right_bathy_nodata,
                                  left_bathy_mask, right_bathy_mask);

  // Use no-data in interpolation and edge extension.
  PixelMask<float>bathy_nodata_pix(0); bathy_nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> bathy_ext_nodata(bathy_nodata_pix);

  // Get the aligned size from the images already aligned
  Vector2i left_size;
  std::string left_aligned_file = this->m_out_prefix + "-L.tif";
  if (boost::filesystem::exists(left_aligned_file))
    left_size = file_image_size(left_aligned_file);
  else
    vw_throw(NoImplErr() << "Could not read: " << left_aligned_file);

  // Read alignment matrices
  vw::Matrix<double> align_left_matrix 
   = asp::alignmentMatrix(this->m_out_prefix, asp::stereo_settings().alignment_method,
                              "left");
  vw::Matrix<double> align_right_matrix
   = asp::alignmentMatrix(this->m_out_prefix, asp::stereo_settings().alignment_method,
                              "right");

  // Generate aligned versions of the masks according to the options.
  ImageViewRef<PixelMask<float>> left_aligned_bathy_mask, right_aligned_bathy_mask;
  if (stereo_settings().alignment_method == "homography"     ||
      stereo_settings().alignment_method == "affineepipolar" ||
      stereo_settings().alignment_method == "local_epipolar") {

    left_aligned_bathy_mask = transform(left_bathy_mask,
                                        HomographyTransform(align_left_matrix),
                                        left_size.x(), left_size.y());

    // Note how we use left_size and not right_size
    right_aligned_bathy_mask = transform(right_bathy_mask,
                                         HomographyTransform(align_right_matrix),
                                         left_size.x(), left_size.y());

  } else if (stereo_settings().alignment_method == "none") {
    // No alignment
    left_aligned_bathy_mask  = left_bathy_mask;
    right_aligned_bathy_mask = right_bathy_mask;
  } // End of image alignment block

  bool has_bathy_nodata = true;
  float output_nodata = -32768.0;

  // Read the georef of the cropped left and right images saved before the masks
  vw::cartography::GeoReference left_georef, right_georef;
  bool has_left_georef  = read_georeference(left_georef,  this->left_cropped_image());
  bool has_right_georef = read_georeference(right_georef, this->right_cropped_image());

  std::string left_aligned_bathy_mask_file = StereoSession::left_aligned_bathy_mask();
  vw_out() << "\t--> Writing: " << left_aligned_bathy_mask_file << "\n";
  block_write_gdal_image(left_aligned_bathy_mask_file,
                         apply_mask(left_aligned_bathy_mask, left_bathy_nodata),
                         has_left_georef, left_georef,
                         has_bathy_nodata, left_bathy_nodata, options,
                         TerminalProgressCallback("asp","\t  L bathy mask:  "));

  // Use same logic as when the right aligned image is written
  if (stereo_settings().alignment_method == "none") {
    std::string right_aligned_bathy_mask_file = StereoSession::right_aligned_bathy_mask();
    vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << "\n";
    block_write_gdal_image(right_aligned_bathy_mask_file,
                           apply_mask(right_aligned_bathy_mask, right_bathy_nodata),
                           has_right_georef, right_georef,
                           has_bathy_nodata, right_bathy_nodata, options,
                           TerminalProgressCallback("asp","\t  R bathy mask:  "));
  } else {
    std::string right_aligned_bathy_mask_file = StereoSession::right_aligned_bathy_mask();
    vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << "\n";
    block_write_gdal_image(right_aligned_bathy_mask_file,
                           apply_mask(crop(edge_extend(right_aligned_bathy_mask,
                                                       bathy_ext_nodata),
                                           // Note how we use the left aligned mask bbox
                                           bounding_box(left_aligned_bathy_mask)),
                                      right_bathy_nodata),
                           has_right_georef, right_georef,
                           has_bathy_nodata, right_bathy_nodata,
                           options,
                           TerminalProgressCallback("asp","\t  R bathy mask:  "));
  }
}

// Return the left and right cropped images. These are the same
// as the input images unless the cropping is on.
std::string StereoSession::left_cropped_image() const{
  std::string cropped_image = m_left_image_file;
  if (stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0))
    cropped_image = m_out_prefix + "-L-cropped.tif";
  return cropped_image;
}

std::string StereoSession::right_cropped_image() const{
  std::string cropped_image = m_right_image_file;
  if (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0))
    cropped_image = m_out_prefix + "-R-cropped.tif";
  return cropped_image;
}

// Apply epipolar alignment to images, if the camera models are pinhole. This will
// be reimplemented in StereoSessionPinhole.
void StereoSession::
epipolar_alignment(vw::ImageViewRef<vw::PixelMask<float>> left_masked_image,
                   vw::ImageViewRef<vw::PixelMask<float>> right_masked_image,
                   vw::ValueEdgeExtension<vw::PixelMask<float>> ext_nodata,
                   // Outputs
                   vw::ImageViewRef<vw::PixelMask<float>> & Limg,
                   vw::ImageViewRef<vw::PixelMask<float>> & Rimg) {
  vw_throw(ArgumentErr() << "Epipolar alignment is only implemented for pinhole cameras.");
}

std::string StereoSession::left_cropped_bathy_mask() const {
  if (!do_bathymetry())
    vw_throw(ArgumentErr() << "The left cropped bathy mask is requested when "
              << "bathymetry mode is not on.");

  bool crop_left = (stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0));
  if (!crop_left)
    return stereo_settings().left_bathy_mask;

  return this->m_out_prefix + "-L_cropped_bathy_mask.tif";
}

std::string StereoSession::right_cropped_bathy_mask() const {
  if (!do_bathymetry())
    vw_throw(ArgumentErr() << "The right cropped bathy mask is requested when "
              << "bathymetry mode is not on.");

  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  if (!crop_right)
    return stereo_settings().right_bathy_mask;

  return this->m_out_prefix + "-R_cropped_bathy_mask.tif";
}

std::string StereoSession::left_aligned_bathy_mask() const {
  return m_out_prefix + "-L_aligned_bathy_mask.tif";
}

std::string StereoSession::right_aligned_bathy_mask() const {
  return m_out_prefix + "-R_aligned_bathy_mask.tif";
}

void StereoSession::get_input_image_crops(vw::BBox2i &left_image_crop,
                                          vw::BBox2i &right_image_crop) const {

  // Set the ROIs to the entire image if the input crop windows are not set.
  Vector2i left_size  = file_image_size(m_left_image_file);
  Vector2i right_size = file_image_size(m_right_image_file);

  if (stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0))
    left_image_crop  = stereo_settings().left_image_crop_win;
  else
    left_image_crop = BBox2i(0, 0, left_size [0], left_size [1]);

  if (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0))
    right_image_crop = stereo_settings().right_image_crop_win;
  else
    right_image_crop = BBox2i(0, 0, right_size[0], right_size[1]);
}


vw::TransformPtr StereoSession::tx_left_homography() const {
  vw::Matrix<double> tx 
    = asp::alignmentMatrix(m_out_prefix, asp::stereo_settings().alignment_method,
                           "left");
  return vw::TransformPtr(new vw::HomographyTransform(tx));
}

vw::TransformPtr StereoSession::tx_right_homography() const {
  vw::Matrix<double> tx
    = asp::alignmentMatrix(m_out_prefix, asp::stereo_settings().alignment_method,
                           "right");
  return vw::TransformPtr(new vw::HomographyTransform(tx));
}

vw::TransformPtr StereoSession::tx_identity() const {
  Matrix<double> tx = math::identity_matrix<3>();
  return vw::TransformPtr(new vw::HomographyTransform(tx));
}

vw::TransformPtr StereoSession::tx_left_map_trans() const {

  std::string left_map_proj_image = this->left_cropped_image();
  if (!m_left_map_proj_model)
    vw_throw(ArgumentErr() << "Map projection model not loaded for image "
              << left_map_proj_image);
  return asp::transformFromMapProject(m_input_dem, left_map_proj_image,
                                      m_left_map_proj_model,
                                      m_options, "left", m_out_prefix,
                                      asp::stereo_settings().ortho_heights[0]);
}

vw::TransformPtr StereoSession::tx_right_map_trans() const {

  std::string right_map_proj_image = this->right_cropped_image();
  if (!m_right_map_proj_model)
    vw_throw(ArgumentErr() << "Map projection model not loaded for image "
              << right_map_proj_image);
  return asp::transformFromMapProject(m_input_dem, right_map_proj_image,
                                      m_right_map_proj_model,
                                      m_options, "right", m_out_prefix,
                                      asp::stereo_settings().ortho_heights[1]);
}

// Load an RPC model. Any adjustment in ba_prefix and pixel_offset
// will be applied.
boost::shared_ptr<vw::camera::CameraModel>
StereoSession::load_rpc_camera_model(std::string const& image_file,
                                     std::string const& camera_file,
                                     std::string const& ba_prefix,
                                     Vector2 pixel_offset) const {

  std::string err1, err2;
  try {
    if (camera_file != "")
      return load_adjusted_model(m_camera_loader.load_rpc_camera_model(camera_file),
                                 image_file, camera_file, ba_prefix, pixel_offset);
  } catch(std::exception const& e1) {
    err1 = e1.what();
  }
  try {
    return load_adjusted_model(m_camera_loader.load_rpc_camera_model(image_file),
                               image_file, camera_file, ba_prefix, pixel_offset);
  } catch(std::exception const& e2) {
    err2 = e2.what();
  }

  // For Cartosat, GDAL chokes. The user must move {image}_RPC_ORG.TXT
  // to {image}_RPC.TXT
  std::string truncated = fs::path(image_file).replace_extension("").string();
  std::string org_file = truncated + "_RPC_ORG.TXT";
  std::string rpc_file = truncated + "_RPC.TXT";
  std::string msg = "";
  if (boost::filesystem::exists(org_file))
    msg = "Detected file: " + org_file + ". If this is Cartosat data, any such files "
      "must be moved to names like " + rpc_file
      + " by overwriting those files if necessary.\n";

  // Raise a custom exception if both failed
  vw_throw(ArgumentErr() << "Unable to load RPC model from either " << image_file
          << " or " << camera_file << ".\n"
       << err1 << "\n" << err2 << "\n" << msg);
} // End function load_rpc_camera_model

} // End namespace asp
