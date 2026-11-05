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

/// \file SatSimSensor.cc
/// Sensor projection and synthetic imagery generation for sat_sim.

#include <asp/Camera/SatSimSensor.h>
#include <asp/Camera/SatSimTrajectory.h>
#include <asp/Camera/SatSim.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/SyntheticLinescan.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/GdalUtils.h>
#include <asp/Core/CameraTransforms.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Geometry/baseUtils.h>
#include <vw/Image/Interpolation.h>
#include <vw/Image/Filter.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Core/StringUtils.h>

#include <boost/filesystem.hpp>
#include <iomanip>

namespace fs = boost::filesystem;

using namespace vw;

namespace asp {

void readPinholeCameras(SatSimOptions const& opt, 
                        std::vector<std::string> & cam_names,
                        std::vector<vw::CamPtr> & cams) {

  // Read the camera names
  vw::vw_out() << "Reading: " << opt.camera_list << "\n";
  asp::read_list(opt.camera_list, cam_names);

  // Sanity check
  if (cam_names.empty())
    vw::vw_throw(vw::ArgumentErr() << "No cameras were found.\n");

  cams.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++)
    cams[i] = vw::CamPtr(new vw::PinholeModel(cam_names[i])); // managed pointer

  return;
}

// A function to perturb the cameras. A lot of logic is in asp::genCamPoses().
void perturbCameras(SatSimOptions const& opt, 
                    std::string const& suffix,
                    vw::GeoReference const& georef,
                    std::vector<std::string> const& cam_names,
                    std::vector<vw::CamPtr> & cams) {

  // Must have as many cam names as cameras
  if (cam_names.size() != cams.size())
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many camera names as cameras.\n");
  
  std::vector<std::string> out_cam_names(cam_names.size());
  
  // Previous and current projected camera centers
  vw::Vector3 prev_proj_ctr(0, 0, 0), curr_proj_ctr(0, 0, 0);
  
  // Length along orbit from first to current camera
  double orbitLen = 0.0;
  
  for (int i = 0; i < int(cam_names.size()); i++) {
    
    // Camera must be pinhole
    vw::PinholeModel * pin 
      = dynamic_cast<vw::PinholeModel*>(cams[i].get());
    if (pin == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Expecting a Pinhole camera for: " 
                   << cam_names[i] << "\n");

    vw::Vector3 cam_ctr = pin->camera_center(vw::Vector2());
    vw::Matrix<double,3,3> cam2world = pin->get_rotation_matrix();
    
    vw::Vector3 llh = georef.datum().cartesian_to_geodetic(cam_ctr);

    // It is assume that the satellite points nadir. The camera is attached
    // to the satellite body at potentially an angle.
    vw::Matrix3x3 sat2world = georef.datum().lonlat_to_ned_matrix(llh);
    vw::Matrix3x3 cam2sat = inverse(sat2world) * cam2world;
    
    // The camera center in projected coordinates
    subvector(curr_proj_ctr, 0, 2) = georef.lonlat_to_point(subvector(llh, 0, 2)); 
    curr_proj_ctr[2] = llh[2];
    
    // The orbit length from the first to the current camera
    if (i > 0)
      orbitLen += calcOrbitLength(prev_proj_ctr, curr_proj_ctr, georef);
    
    // Jitter amplitude at the current location
    vw::Vector3 jitter_amp(0, 0, 0);
    if (std::isnan(opt.random_position_perturbation))
      jitter_amp = calcJitterAmplitude(opt, curr_proj_ctr, orbitLen);

    // The jitter vibration is applied to the camera in the satellite frame
    vw::Matrix3x3 R_rpy = asp::rollPitchYaw(jitter_amp[0], jitter_amp[1], jitter_amp[2]);
    cam2world = sat2world * R_rpy * cam2sat;
    
    // Replace the camera rotation
    pin->set_camera_pose(cam2world);
    
    if (!std::isnan(opt.random_position_perturbation)) {
      // Seed the random number generator to ensure being in a different location
      // in orbit results in a different perturbation for same camera index.
      srand(vw::norm_2(curr_proj_ctr));
      vw::Vector3 perturbation;
      for (int c = 0; c < 3; c++)
        perturbation[c] = (2 * double(rand())/double(RAND_MAX) - 1.0) 
                          * opt.random_position_perturbation;
      
      pin->set_camera_center(cam_ctr + perturbation);
    }
    
    // Will save the camera with the output prefix
    std::string camName = opt.out_prefix + "-" + fs::path(cam_names[i]).filename().string();
    
    if (!opt.save_as_csm) {
      // Save the camera as pinhole
      vw::vw_out() << "Writing: " << camName << "\n";
      pin->write(camName);
    } else {
      // Save as CSM
      cams[i] = boost::make_shared<asp::CsmModel>();
      auto const& d = georef.datum(); // alias
      auto csmPtr = boost::dynamic_pointer_cast<asp::CsmModel>(cams[i]);
      csmPtr->createFrameModel(*pin,
                               opt.image_size[0], opt.image_size[1],
                               d.semi_major_axis(), d.semi_minor_axis());
      
      // Replace extension with .json
      camName = fs::path(camName).replace_extension(".json").string();
      vw::vw_out() << "Writing: " << camName << "\n";
      csmPtr->saveState(camName);
    }
 
    out_cam_names[i] = camName;
    
    // The current camera becomes the previous one for next time
    prev_proj_ctr = curr_proj_ctr;
  }

  // Write the list
  std::string cam_list = opt.out_prefix + "-cameras" + suffix + ".txt"; 
  vw::vw_out() << "Writing: " << cam_list << "\n";
  asp::write_list(cam_list, out_cam_names);

  return;
}

// Check if we do a range
bool skipCamera(int i, SatSimOptions const& opt) {

  if (opt.first_index >= 0 && opt.last_index >= 0 &&
     (i < opt.first_index || i >= opt.last_index))
       return true;
  return false;
}

// Given a transform from ref sensor to world, the ref sensor to current sensor,
// create the transform from current sensor to world. Do it in-place.

void genPinholeCameras(SatSimOptions          const& opt,
                vw::GeoReference const& dem_georef,
                std::vector<vw::Vector3>      const& positions,
                std::vector<vw::Matrix3x3>    const& cam2world,
                std::vector<vw::Matrix3x3>    const& ref_cam2world,
                std::vector<double>           const& cam_times,
                bool                                 have_rig,
                Eigen::Affine3d               const& ref2sensor,
                std::string                   const& suffix, 
                // Outputs
                std::vector<std::string> & cam_names,
                std::vector<vw::CamPtr>  & cams) {

  // Ensure we have as many camera positions as we have camera orientations
  if (positions.size() != cam2world.size() || positions.size() != ref_cam2world.size()
      || positions.size() != cam_times.size())
    vw::vw_throw(vw::ArgumentErr() 
                 << "Expecting as many camera positions as camera orientations.\n");
  
  cams.resize(positions.size());
  cam_names.resize(positions.size());
  for (int i = 0; i < int(positions.size()); i++) {

    vw::Vector3 P    = positions[i];
    vw::Matrix3x3 R  = cam2world[i];
    vw::Vector3 P0   = positions[i];
    vw::Matrix3x3 R0 = ref_cam2world[i];

    if (have_rig) {
      // Must adjust for the rig
      applyRigTransform(ref2sensor, P, R);
      applyRigTransform(ref2sensor, P0, R0);
    }
    
    // Always create the cameras, but only save them if we are not skipping
    vw::Datum d = dem_georef.datum();
    if (opt.save_as_csm) {
      cams[i] = boost::make_shared<asp::CsmModel>();
      auto csmPtr = boost::dynamic_pointer_cast<asp::CsmModel>(cams[i]);
      csmPtr->createFrameModel(opt.image_size[0], opt.image_size[1],
                              opt.optical_center[0], opt.optical_center[1],
                              opt.focal_length, 
                              d.semi_major_axis(), d.semi_minor_axis(),
                              P, R);
    } else {
      cams[i] = boost::make_shared<vw::PinholeModel>
                  (P, R, opt.focal_length, opt.focal_length,
                   opt.optical_center[0], opt.optical_center[1]);
    }

    // This is useful for understanding things in the satellite frame
    vw::PinholeModel pinRefCam;
    asp::CsmModel csmRefCam;
    if (opt.save_ref_cams) {
      if (opt.save_as_csm) 
        csmRefCam.createFrameModel(opt.image_size[0], opt.image_size[1],
                                   opt.optical_center[0], opt.optical_center[1],
                                   opt.focal_length, 
                                   d.semi_major_axis(), d.semi_minor_axis(),
                                   P0, R0);
      else
        pinRefCam = vw::PinholeModel(P0, R0,
                                     opt.focal_length, opt.focal_length,
                                     opt.optical_center[0], opt.optical_center[1]);
    }

    std::string ext;
    if (opt.save_as_csm)
      ext = ".json";
    else
      ext = ".tsai"; 

    // The suffix is used with the rig
    bool isRef = false, isFrame = true;
    std::string camName = camPrefix(opt, i, cam_times[i], isRef, isFrame, suffix) + ext;
    cam_names[i] = camName;

    // Check if we do a range
    if (skipCamera(i, opt)) continue;

    vw::vw_out() << "Writing: " << camName << "\n";
    if (opt.save_as_csm) {
      auto csmPtr = boost::dynamic_pointer_cast<asp::CsmModel>(cams[i]);
      csmPtr->saveState(camName);
    } else {
      auto pinPtr = boost::dynamic_pointer_cast<vw::PinholeModel>(cams[i]);
      pinPtr->write(camName);
    }

    if (opt.save_ref_cams) {
      bool isRef = true;
      std::string refCamName = camPrefix(opt, i, cam_times[i], isRef, isFrame, suffix) + ext;
      vw::vw_out() << "Writing: " << refCamName << "\n";
      if (opt.save_as_csm)
        csmRefCam.saveState(refCamName);
      else
        pinRefCam.write(refCamName);
    }
  }

  // Write the list of cameras only if we are not skipping the first camera
  // Otherwise the same file may be written by multiple processes. 
  if (!skipCamera(0, opt)) {
    std::string cam_list = opt.out_prefix + "-cameras" + suffix + ".txt"; 
    vw::vw_out() << "Writing: " << cam_list << "\n";
    asp::write_list(cam_list, cam_names);
  } else {
     // Print a warning message that the list won't be saved
     vw::vw_out(vw::WarningMessage) << "The camera list is saved only when " 
        << "--first-index is 0, to avoid a race condition.\n";
  }

  return;
}

// Bring crops in memory. It greatly helps with multi-threading speed.  
// This function is used only for small tiles, to avoid running out of memory
// (which did happen).
void setupCroppedDemAndOrtho(vw::Vector2 const& image_size,
    vw::CamPtr const& cam,
    vw::ImageViewRef<vw::PixelMask<float>> const& dem,
    vw::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> const& ortho,
    vw::GeoReference const& ortho_georef,
    double blur_sigma,
    // Outputs
    vw::ImageView<vw::PixelMask<float>> & crop_dem,
    vw::GeoReference & crop_dem_georef,
    vw::ImageView<vw::PixelMask<float>> & crop_ortho,
    vw::GeoReference & crop_ortho_georef) {

    // Find the bounding box of the dem and ortho portions seen in the camera,
    // in projected coordinates
    float mean_gsd = 0.0;    
    bool quick = true; // Assumes a big DEM fully containing the image    
    vw::BBox2 dem_box;
    try {
      // This is a bugfix. The camera_bbox function can fail if we want an
      // extent that does not fit fully with the input DEM/orthoimage.
      // We could use above quick = false, but then we'd get only partial
      // synthetic images, which isn't good.
      dem_box = vw::camera_bbox(dem, dem_georef, dem_georef,
        cam, image_size[0], image_size[1], mean_gsd, quick);
    } catch (const vw::Exception& e) {
       vw::vw_throw(vw::ArgumentErr() << "sat_sim: Failed to compute a synthetic image. "
       << "The most likely cause is that the desired image is out of bounds given "
       << "the input DEM and orthoimage.\n");
    }

    vw::GeoTransform d2o(dem_georef, ortho_georef);
    vw::BBox2 ortho_box = d2o.point_to_point_bbox(dem_box);

    // Find the DEM pixel box and expand it in case there was some inaccuracies
    // in finding the box
    vw::BBox2i dem_pixel_box = dem_georef.point_to_pixel_bbox(dem_box);
    int expand = 50;
    dem_pixel_box.expand(expand);
    dem_pixel_box.crop(vw::bounding_box(dem));

    // Same for the ortho
    vw::BBox2i ortho_pixel_box = ortho_georef.point_to_pixel_bbox(ortho_box);
    ortho_pixel_box.expand(expand);
    
    // Adjust for any blur
    int kernel_size = 0;
    if (blur_sigma > 0) {
      kernel_size = vw::compute_kernel_size(blur_sigma);
      ortho_pixel_box.expand(kernel_size); // to avoid edge effects
    }

    ortho_pixel_box.crop(vw::bounding_box(ortho));

    // Crop
    crop_dem = vw::crop(dem, dem_pixel_box);
    crop_dem_georef = crop(dem_georef, dem_pixel_box);

    if (blur_sigma > 0) 
      crop_ortho = vw::crop(vw::gaussian_filter(ortho, blur_sigma), ortho_pixel_box);
    else
      crop_ortho = vw::crop(ortho, ortho_pixel_box);
      
    crop_ortho_georef = crop(ortho_georef, ortho_pixel_box);
}

// Create a synthetic image with multiple threads
typedef vw::ImageView<vw::PixelMask<float>> ImageT;
class SynImageView: public vw::ImageViewBase<SynImageView> {
  
  typedef typename ImageT::pixel_type PixelT;
  SatSimOptions const& m_opt;
  vw::CamPtr m_cam;
  vw::GeoReference m_dem_georef; // make a copy to be thread-safe
  vw::ImageViewRef<vw::PixelMask<float>> m_dem;
  double m_height_guess;
  vw::GeoReference m_ortho_georef; // make a copy to be thread-safe
  vw::ImageViewRef<vw::PixelMask<float>> m_ortho;
  float m_ortho_nodata_val;

public:
  SynImageView(SatSimOptions const& opt,
               vw::CamPtr    const& cam,
               vw::GeoReference   const& dem_georef,
               vw::ImageViewRef<vw::PixelMask<float>> dem,
               double height_guess,
               vw::GeoReference   const& ortho_georef,
               vw::ImageViewRef<vw::PixelMask<float>> ortho,
               float ortho_nodata_val):
                m_opt(opt), m_cam(cam), 
                m_dem_georef(dem_georef), m_dem(dem),
                m_height_guess(height_guess),
                m_ortho_georef(ortho_georef), m_ortho(ortho),
                m_ortho_nodata_val(ortho_nodata_val) {}

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef vw::ProceduralPixelAccessor<SynImageView> pixel_accessor;

  inline vw::int32 cols() const { return m_opt.image_size[0]; }
  inline vw::int32 rows() const { return m_opt.image_size[1]; }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double/*i*/, double/*j*/, vw::int32/*p*/ = 0) const {
    vw::vw_throw(vw::NoImplErr() 
      << "SynImageView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef vw::CropView<vw::ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(vw::BBox2i const& bbox) const {

    // Expand the box a bit, to help with interpolation in the ortho image later
    vw::BBox2i extra_bbox = bbox;
    extra_bbox.expand(10);
    extra_bbox.crop(vw::BBox2i(0, 0, cols(), rows()));
    vw::Vector2 crop_start = extra_bbox.min();
    vw::Vector2 crop_image_size = extra_bbox.max() - extra_bbox.min();

    // Must adjust the camera to be able work with the current box, which 
    // does not necessarily start at (0,0). We only adjust the starting
    // position, and not any other params
    vw::Vector3 translation(0, 0, 0);
    vw::Quat    rotation(vw::identity_matrix<3>());
    vw::Vector2 pixel_offset = crop_start;
    double      scale = 1.0;
    vw::CamPtr crop_cam(new vw::AdjustedCameraModel(m_cam, translation, rotation, 
                        pixel_offset, scale));

    // Bring crops in memory. It greatly helps with multi-threading speed.  
    vw::GeoReference crop_dem_georef; // make a copy to be thread-safe
    vw::ImageView<vw::PixelMask<float>> crop_dem;
    vw::GeoReference crop_ortho_georef; // make a copy to be thread-safe
    vw::ImageView<vw::PixelMask<float>> crop_ortho;
    setupCroppedDemAndOrtho(crop_image_size,
      crop_cam, m_dem, m_dem_georef, m_ortho, m_ortho_georef, m_opt.blur_sigma,
      // Outputs
      crop_dem, crop_dem_georef, crop_ortho, crop_ortho_georef);

    // Create interpolated image with bicubic interpolation with invalid pixel 
    // edge extension
    vw::PixelMask<float> nodata_mask = vw::PixelMask<float>(); // invalid value
    nodata_mask.invalidate();
    auto interp_ortho = vw::interpolate(crop_ortho, vw::BicubicInterpolation(),
                                        vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_mask));

    // The location where the ray intersects the ground. We will use each obtained
    // location as initial guess for the next ray. This may not be always a great
    // guess, but it is better than starting nowhere. It should work decently
    // if the camera is high, and with a small footprint on the ground.
    vw::Vector3 xyz_guess(0, 0, 0);

    vw::ImageView<result_type> tile(bbox.width(), bbox.height());

    for (int col = bbox.min().x(); col < bbox.max().x(); col++) {
      for (int row = bbox.min().y(); row < bbox.max().y(); row++) {

        // These will use to index into the tile 
        int c = col - bbox.min().x();
        int r = row - bbox.min().y();

        // Start with an invalid pixel
        tile(c, r) = vw::PixelMask<float>();
        tile(c, r).invalidate();

        // Here use the full image pixel indices
        vw::Vector2 pix(col, row);
        // Also use original camera
        vw::Vector3 cam_ctr = m_cam->camera_center(pix);
        vw::Vector3 cam_dir = m_cam->pixel_to_vector(pix);

        // Intersect the ray going from the given camera pixel with a DEM
        // Use xyz_guess as initial guess and overwrite it with the new value
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        vw::Vector3 xyz = vw::camera_pixel_to_dem_xyz
          (cam_ctr, cam_dir, crop_dem,
            crop_dem_georef, treat_nodata_as_zero,
            has_intersection, m_opt.dem_height_error_tol,
            xyz_guess, m_height_guess);

        if (!has_intersection)
          continue; // will result in nodata pixels

        // Update the guess for nex time, now that we have a valid intersection point
        xyz_guess = xyz;

        // Find the texture value at the intersection point by interpolation.
        // This will result in an invalid value if if out of range or if the
        // image itself has invalid pixels.
        vw::Vector3 llh = crop_dem_georef.datum().cartesian_to_geodetic(xyz);
        vw::Vector2 ortho_pix = crop_ortho_georef.lonlat_to_pixel
                                 (vw::Vector2(llh[0], llh[1]));
        tile(c, r) = interp_ortho(ortho_pix[0], ortho_pix[1]);
      }
    }

    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, vw::BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

// Generate images by projecting rays from the sensor to the ground
void genImages(SatSimOptions      const& opt,
    bool external_cameras,
    std::vector<std::string>      const& cam_names,
    std::vector<vw::CamPtr>       const& cams,
    std::string                   const& suffix, 
    vw::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> dem,
    double height_guess,
    vw::GeoReference const& ortho_georef,
    vw::ImageViewRef<vw::PixelMask<float>> ortho,
    float ortho_nodata_val) {

  vw::vw_out() << "Generating images.\n";

  // Generate image names from camera names by replacing the extension
  std::vector<std::string> image_names;
  image_names.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++) {
    if (external_cameras)
      image_names[i] = opt.out_prefix + "-" 
      + fs::path(cam_names[i]).filename().replace_extension(".tif").string();
    else // just replace the extension
     image_names[i] = fs::path(cam_names[i]).replace_extension(".tif").string();
  }

  for (size_t i = 0; i < cams.size(); i++) {

    // Check if we do a range
    if (skipCamera(i, opt)) continue;

    // Save the image using the block write function with multiple threads
    // Increase the tile size, as otherwise this code becomes very slow
    // since a time-consuming camera box calculation happens in each tile.
    // TODO(oalexan1): Figure out why the slowdown.
    vw::vw_out() << "Writing: " << image_names[i] << "\n";
    bool has_georef = false; // the produced image is raw, it has no georef
    bool has_nodata = true;
    SatSimOptions local_opt = opt;
    local_opt.raster_tile_size = vw::Vector2i(512, 512);
    block_write_gdal_image(image_names[i], 
                           vw::apply_mask(SynImageView(opt, cams[i], dem_georef, dem, 
                                                       height_guess, ortho_georef, 
                                                       ortho, ortho_nodata_val), 
                                          ortho_nodata_val),
                           has_georef, ortho_georef, has_nodata, ortho_nodata_val, 
                           local_opt, vw::TerminalProgressCallback("", "\t--> "));
  }  

  // Write the list of images only if we are not skipping the first camera
  // Otherwise the same file may be written by multiple processes. Also
  // do this only for pinhole cameras, as for linescan there is only one 
  // image/camera rather than a set.
  if (opt.sensor_type == "pinhole") {
    if (!skipCamera(0, opt)) {
      std::string image_list = opt.out_prefix + "-images" + suffix + ".txt"; 
      vw::vw_out() << "Writing: " << image_list << "\n";
      asp::write_list(image_list, image_names);
    } else {
      // Print a warning message that the list won't be saved
      vw::vw_out(vw::WarningMessage) << "The image list is saved only when " 
        << "--first-index is 0, to avoid a race condition.\n";
    }
  }

  return;
}

// If sensor type is one value, make a vector of that value with
// as many entries as sensor names. Otherwise split by comma,
// and there must be as many entries as sensors.
void handleSensorType(int num_sensors, 
                      std::string const& sensor_type, 
                      std::vector<std::string> & sensor_types) { 
  boost::split(sensor_types, sensor_type, boost::is_any_of(","));
  
  // If only one, fill to make num_sensors
  if (sensor_types.size() == 1) {
    std::string val = sensor_types[0];
    sensor_types.clear();
    for (int i = 0; i < num_sensors; i++)
      sensor_types.push_back(val);
  }
  
  // There must be as many as sensors
  if (int(sensor_types.size()) != num_sensors)
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many sensor types as sensors.\n");
  
  // Each must be linescan or pinhole
  for (int i = 0; i < int(sensor_types.size()); i++) {
    if (sensor_types[i] == "frame")
      sensor_types[i] = "pinhole"; // frame is synonymous with pinhole
    if (sensor_types[i] != "linescan" && sensor_types[i] != "pinhole")
      vw::vw_throw(vw::ArgumentErr() << "Expecting sensor type to be linescan "
                   << "or pinhole/frame.\n");
  }
}

// Generate cameras and images for a sensor

} // end namespace asp
