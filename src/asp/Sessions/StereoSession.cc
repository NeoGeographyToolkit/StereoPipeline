// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/AspStringUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/AlignmentUtils.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Geometry.h>
#include <vw/Image/PixelMask.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Cartography/DatumUtils.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <map>
#include <utility>
#include <string>

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

ImageViewRef<PixelMask<Vector2f>>
StereoSession::pre_pointcloud_hook(std::string const& input_file) {
  return DiskImageView<PixelMask<Vector2f> >(input_file);
}

std::string StereoSession::left_cropped_image(bool do_crop) const{
  std::string cropped_image = m_left_image_file;
  if (do_crop) {
    // In stereo_dist_mode, the cropped and normalized image is L.tif, L-cropped.tif is skipped
    if (stereo_settings().stereo_dist_mode)
      cropped_image = m_out_prefix + "-L.tif";
    else
      cropped_image = m_out_prefix + "-L-cropped.tif";
  }
  std::cout << "left_cropped_image: " << cropped_image << "\n";
  return cropped_image;
}

std::string StereoSession::right_cropped_image(bool do_crop) const{
  std::string cropped_image = m_right_image_file;
  if (do_crop) {
    // In stereo_dist_mode, the cropped and normalized image is R.tif, R-cropped.tif is skipped
    if (stereo_settings().stereo_dist_mode)
      cropped_image = m_out_prefix + "-R.tif";
    else
      cropped_image = m_out_prefix + "-R-cropped.tif";
  }
  std::cout << "right_cropped_image: " << cropped_image << "\n";
  return cropped_image;
}

bool StereoSession::do_crop_left() const {
  return stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0);
}

bool StereoSession::do_crop_right() const {
  return stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0);
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

void StereoSession::get_input_image_crops(vw::BBox2i &left_image_crop,
                                          vw::BBox2i &right_image_crop) const {

  // Set the ROIs to the entire image if the input crop windows are not set.
  Vector2i left_size  = file_image_size(m_left_image_file);
  Vector2i right_size = file_image_size(m_right_image_file);

  if (do_crop_left())
    left_image_crop  = stereo_settings().left_image_crop_win;
  else
    left_image_crop = BBox2i(0, 0, left_size [0], left_size [1]);

  if (do_crop_right())
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

  bool crop_left = do_crop_left();
  std::string left_map_proj_image = this->left_cropped_image(crop_left);
  if (!m_left_map_proj_model)
    vw_throw(ArgumentErr() << "Map projection model not loaded for image "
              << left_map_proj_image);
  return asp::transformFromMapProject(m_input_dem, left_map_proj_image,
                                      m_left_map_proj_model,
                                      m_options, "left", m_out_prefix,
                                      asp::stereo_settings().ortho_heights[0]);
}

vw::TransformPtr StereoSession::tx_right_map_trans() const {

  bool crop_right = do_crop_right();
  std::string right_map_proj_image = this->right_cropped_image(crop_right);
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
