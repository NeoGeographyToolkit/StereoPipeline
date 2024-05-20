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

#include <asp/Core/Common.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/SyntheticLinescan.h>
#include <asp/Camera/SatSim.h>
#include <asp/Camera/CsmUtils.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Math/Functors.h>
#include <vw/Core/Stopwatch.h>

namespace asp {

// Allow finding the time at any line, even negative ones. Here a
// simple slope-intercept formula is used rather than a table. 
// This was a temporary function used for debugging
// double get_time_at_line(double line) const {
//     csm::ImageCoord csm_pix;
//     asp::toCsmPixel(vw::Vector2(0, line), csm_pix);
//     return ls_model->getImageTime(csm_pix);
// }

// The pointing vector in sensor coordinates, before applying cam2world. This
// is for testing purposes. Normally CSM takes care of this internally.
// This was a temporary function used for debugging
// vw::Vector3 get_local_pixel_to_vector(vw::Vector2 const& pix) const {

//   vw::Vector3 result(pix[0] + detector_origin[0], 
//                       detector_origin[1], 
//                       ls_model->m_focalLength);
//   // Make the direction have unit length
//   result = normalize(result);
//   return result;
// }

// Compare the camera center and direction with pinhole. A very useful
// test.
void PinLinescanTest(SatSimOptions                const & opt, 
                     asp::CsmModel                const & ls_cam,
                     std::map<int, vw::Vector3>   const & positions,
                     std::map<int, vw::Matrix3x3> const & cam2world) {
                        
  for (int i = 0; i < int(positions.size()); i++) {

    auto pin_cam 
      = vw::camera::PinholeModel(asp::mapVal(positions, i),
                                 asp::mapVal(cam2world, i),
                                 opt.focal_length, opt.focal_length,
                                 opt.optical_center[0], opt.optical_center[1]);
  
    double line = (opt.image_size[1] - 1.0) * i / std::max((positions.size() - 1.0), 1.0);
  
    // Need care here. ls_pix is on the line.
    vw::Vector2 pin_pix(opt.optical_center[0], opt.optical_center[1]);
    vw::Vector2 ls_pix (opt.optical_center[0], line);

    // The differences below must be 0
    vw::Vector3 ls_ctr  = ls_cam.camera_center(ls_pix);
    vw::Vector3 pin_ctr = pin_cam.camera_center(pin_pix);
    std::cout << "ls ctr and and pin - ls ctr diff: " << ls_ctr << " "
              << norm_2(pin_ctr - ls_ctr) << std::endl;

    vw::Vector3 ls_dir = ls_cam.pixel_to_vector(ls_pix);
    vw::Vector3 pin_dir = pin_cam.pixel_to_vector(pin_pix);
    std::cout << "ls dir and pin - ls dir diff: " << ls_dir << " "
              << norm_2(pin_dir - ls_dir) << std::endl;
  }
}

// Wrapper for logic to intersect DEM with ground. The xyz provided on input serves
// as initial guess and gets updated on output if the intersection succeeds. Return
// true on success.
bool intersectDemWithRay(SatSimOptions const& opt,
                         vw::cartography::GeoReference const& dem_georef,vw::ImageViewRef<vw::PixelMask<float>> dem,
                         vw::Vector3 const& cam_ctr, 
                         vw::Vector3 const& cam_dir,
                         double height_guess,
                         // Output
                         vw::Vector3 & xyz) {

    // Find the intersection of this ray with the ground
    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double max_abs_tol = std::min(opt.dem_height_error_tol, 1e-14);
    double max_rel_tol = max_abs_tol;
    int num_max_iter = 100;

    vw::Vector3 local_xyz 
      = vw::cartography::camera_pixel_to_dem_xyz
        (cam_ctr, cam_dir, dem, dem_georef, treat_nodata_as_zero, has_intersection, 
        // Below we use a prudent approach. Try to make the solver work
        // hard. It is not clear if this is needed.
        std::min(opt.dem_height_error_tol, 1e-8),
        max_abs_tol, max_rel_tol, 
        num_max_iter, xyz, height_guess);

    if (!has_intersection)
      return false;

    // Update xyz with produced value if we succeeded
    xyz = local_xyz;
    return true;
}

// Estimate pixel aspect ratio (width / height) of a pixel on the ground
double pixelAspectRatio(SatSimOptions                 const & opt,     
                        vw::cartography::GeoReference const & dem_georef,
                        asp::CsmModel                 const & ls_cam,
                        vw::ImageViewRef<vw::PixelMask<float>>  dem,  
                        double height_guess) {

  // Put here a stop watch
  //vw::Stopwatch sw;
  //sw.start();

  // We checked that the image width and height is at least 2 pixels. That is
  // needed to properly create the CSM model. Now do some samples to see how the
  // pixel width and height are on the ground. Use a small set of samples. Should be good
  // enough. Note how we go a little beyond each sample, while still not exceeding
  // the designed image size. 
  double samp_x = (opt.image_size[0] - 1.0) / 10.0;
  double samp_y = (opt.image_size[1] - 1.0) / 10.0;

  std::vector<double> ratios; 
  vw::Vector3 xyz(0, 0, 0); // intersection with DEM, will be updated below
  
  for (double x = 0; x < opt.image_size[0] - 1.0; x += samp_x) {
    for (double y = 0; y < opt.image_size[1] - 1.0; y += samp_y) {

      // Find the intersection of the ray from this pixel with the ground
      vw::Vector2 pix(x, y);
      vw::Vector3 ctr = ls_cam.camera_center(pix);
      vw::Vector3 dir = ls_cam.pixel_to_vector(pix);
      bool ans = intersectDemWithRay(opt, dem_georef, dem, ctr, dir, 
         height_guess, xyz);
      if (!ans) 
        continue;
      vw::Vector3 P0 = xyz;

      // Add a little to the pixel, but stay within the image bounds
      double dx = std::min(samp_x, 0.5);
      double dy = std::min(samp_y, 0.5);

      // See pixel width on the ground
      pix = vw::Vector2(x + dx, y);
      ctr = ls_cam.camera_center(pix);
      dir = ls_cam.pixel_to_vector(pix);
      ans = intersectDemWithRay(opt, dem_georef, dem, ctr, dir, 
         height_guess, xyz);
      if (!ans) 
        continue;
      vw::Vector3 Px = xyz;

      // See pixel height on the ground
      pix = vw::Vector2(x, y + dy);
      ctr = ls_cam.camera_center(pix);
      dir = ls_cam.pixel_to_vector(pix);
      ans = intersectDemWithRay(opt, dem_georef, dem, ctr, dir, 
         height_guess, xyz);
      if (!ans)
        continue;
      vw::Vector3 Py = xyz;

      double ratio = norm_2(Px - P0) / norm_2(Py - P0);
      if (std::isnan(ratio) || std::isinf(ratio) || ratio <= 0.0)
        continue;
      ratios.push_back(ratio);
    }
  }

  if (ratios.empty())
    vw::vw_throw(vw::ArgumentErr() << "No valid samples found to compute "
             << "the pixel width and height on the ground.\n");

  double ratio = vw::math::destructive_median(ratios);

  //sw.stop();
  //std::cout << "Time to compute pixel aspect ratio: " << sw.elapsed_seconds() << std::endl;

  return ratio;
}

// Form a linescan camera taking into account the reference sensor to current
// rig sensor transform.
void populateCsmLinescanRig(double first_line_time, double dt_line, 
                            double t0_ephem, double dt_ephem,
                            double t0_quat, double dt_quat, 
                            double focal_length,
                            vw::Vector2                const & optical_center,
                            vw::Vector2i               const & image_size,
                            vw::cartography::Datum     const & datum, 
                            std::string                const & sensor_id, 
                            std::vector<vw::Vector3>   const & positions,
                            std::vector<vw::Vector3>   const & velocities,
                            std::vector<vw::Matrix3x3> const & cam2world,
                            bool                               have_rig,
                            Eigen::Affine3d            const & ref2sensor,
                            // Outputs
                            asp::CsmModel                    & model) {

  // If there is a rig, must apply the proper transforms to positions and orientations
  if (!have_rig) {
      populateCsmLinescan(first_line_time, dt_line, t0_ephem, dt_ephem, t0_quat, dt_quat,
                      focal_length, optical_center, image_size, datum, sensor_id,
                      positions, velocities, cam2world, model);
  } else {
    std::vector<vw::Vector3> trans_positions(positions.size());
    std::vector<vw::Matrix3x3> trans_cam2world(cam2world.size());
    for (size_t i = 0; i < positions.size(); i++) {
      vw::Vector3   P = positions[i];
      vw::Matrix3x3 R = cam2world[i]; 
      
      applyRigTransform(ref2sensor, P, R);
      trans_positions[i] = P;
      trans_cam2world[i] = R;
    }
    populateCsmLinescan(first_line_time, dt_line, t0_ephem, dt_ephem, t0_quat, dt_quat,
                      focal_length, optical_center, image_size, datum, sensor_id,
                      trans_positions, velocities, trans_cam2world, model);
  }
  
}

// Create and save a linescan camera with given camera positions and orientations.
// There will be just one of them, as all poses are part of the same linescan camera.
void genLinescanCameras(double                                 first_line_time,
                        double                                 orbit_len,     
                        vw::cartography::GeoReference  const & dem_georef,
                        vw::ImageViewRef<vw::PixelMask<float>> dem,
                        int                                    first_pos,
                        std::vector<vw::Vector3>       const & positions,
                        std::vector<vw::Matrix3x3>     const & cam2world,
                        std::vector<vw::Matrix3x3>     const & cam2world_no_jitter,
                        std::vector<vw::Matrix3x3>     const & ref_cam2world,
                        std::vector<double>            const & cam_times,
                        double                                 height_guess,
                        bool                                   have_rig,
                        Eigen::Affine3d                const & ref2sensor,
                        std::string                    const & suffix, 
                        // Outputs
                        SatSimOptions                        & opt, 
                        std::vector<std::string>             & cam_names,
                        std::vector<vw::CamPtr>              & cams) {
  
  // Sanity checks
  if (cam2world.size() != positions.size() || 
      cam2world_no_jitter.size() != positions.size() ||
      ref_cam2world.size() != positions.size() ||
      cam_times.size() != positions.size())
    vw::vw_throw(vw::ArgumentErr() 
                 << "Expecting as many camera orientations as positions.\n");
    
  // Initialize the outputs
  cam_names.clear();
  cams.clear();
  
  // Measure time based on the length of the segment between 1st and last line.
  // Must be consistent with calcTrajectory().
  double orbit_segment_time = orbit_len / opt.velocity;
  double dt_line = orbit_segment_time / (opt.image_size[1] - 1.0);

  // Positions and orientations use the same starting time and spacing. Note
  // that opt.num_cameras is the number of cameras within the desired orbital
  // segment of length orbit_len, and also the number of cameras for which we
  // have image lines. We will have extra cameras beyond that segment to make it
  // easy to interpolate the camera position and orientation at any time and
  // also to solve for jitter. Here we adjust things so that the camera at first
  // image line has time 0. 
  if (first_pos > 0)
    vw::vw_throw(vw::ArgumentErr() << "First position index must be non-positive.\n");
  double dt_ephem = orbit_segment_time / (opt.num_cameras - 1.0);
  double dt_quat = dt_ephem;
  double t0_ephem = first_line_time + first_pos * dt_ephem;
  double t0_quat = t0_ephem;

  // Sanity check. The linescan time logic here must agree with the logic in
  // calcTrajectory(), including for frame cameras.
  for (size_t i = 0; i < positions.size(); i++) {
    if (opt.model_time && std::abs(t0_ephem + dt_ephem * i - cam_times[i]) > 1e-8)
      vw::vw_throw(vw::ArgumentErr() << "Time mismatch. Use a smaller --reference-time.\n");
  }
  
  // We have no velocities in this context, so set them to 0
  std::vector<vw::Vector3> velocities(positions.size(), vw::Vector3(0, 0, 0));
  
  // Create the camera. Will be later owned by a smart pointer.
  asp::CsmModel * ls_cam = new asp::CsmModel;
  std::string sensor_id = "SyntheticLinescan";

  // We assume no optical offset in y for this synthetic camera
  vw::Vector2 local_optical_center(opt.optical_center[0], 0.0);

  // If creating square pixels, must use the camera without jitter to estimate
  // the image height. Otherwise the image height produced from the camera with
  // jitter will be inconsistent with the one without jitter. This is a bugfix. 
  if (!opt.non_square_pixels) 
    populateCsmLinescanRig(first_line_time, dt_line, t0_ephem, dt_ephem, t0_quat, dt_quat,
                           opt.focal_length, local_optical_center,
                           opt.image_size, dem_georef.datum(), sensor_id,
                           positions, velocities, cam2world_no_jitter, have_rig, ref2sensor, 
                           *ls_cam); // output
  else
    populateCsmLinescanRig(first_line_time, dt_line, t0_ephem, dt_ephem, t0_quat, dt_quat,
                           opt.focal_length, local_optical_center,
                           opt.image_size, dem_georef.datum(), sensor_id,
                           positions, velocities, cam2world, have_rig, ref2sensor, 
                           *ls_cam); // output 

  // Sanity check (very useful)
  // PinLinescanTest(opt, *ls_cam, positions, cam2world);

  if (!opt.non_square_pixels) {
    // Find the pixel aspect ratio on the ground (x/y)
    vw::vw_out() << "Adjusting the linescan image height from " << opt.image_size[1] << " to ";
    double ratio = pixelAspectRatio(opt, dem_georef, *ls_cam, dem, height_guess);
    // Adjust the image height to make the pixels square
    opt.image_size[1] = std::max(round(opt.image_size[1] / ratio), 2.0);
    // Must adjust the time spacing to match the new image height
    dt_line = orbit_segment_time / (opt.image_size[1] - 1.0);
    vw::vw_out() << opt.image_size[1] << " pixels, to make the ground "
                 << "projection of an image pixel be roughly square.\n";

    // Recreate the camera with this aspect ratio. This time potentially use the 
    // camera with jitter. 
    populateCsmLinescanRig(first_line_time, dt_line, t0_ephem, dt_ephem, t0_quat, dt_quat,
                           opt.focal_length, local_optical_center,
                           opt.image_size, dem_georef.datum(), sensor_id,
                           positions, velocities, cam2world, have_rig, ref2sensor,
                           *ls_cam); // output
    // Sanity check (very useful for testing, the new ratio must be close to 1.0)
    // ratio = pixelAspectRatio(opt, dem_georef, *ls_cam, dem, height_guess);
  }
  
  std::string filename = opt.out_prefix + suffix + ".json";
  ls_cam->saveState(filename);

  if (opt.save_ref_cams) {
      asp::CsmModel ref_cam;
      populateCsmLinescanRig(first_line_time, dt_line, t0_ephem, dt_ephem, t0_quat, dt_quat,
                             opt.focal_length, local_optical_center, 
                             opt.image_size, dem_georef.datum(), sensor_id,
                             positions,  velocities, ref_cam2world, have_rig, ref2sensor,
                             ref_cam); // output
    std::string ref_filename = opt.out_prefix + "-ref" + suffix + ".json";
    ref_cam.saveState(ref_filename);
  }

  // Save the camera name and smart pointer to camera
  cam_names.push_back(filename);
  cams.push_back(vw::CamPtr(ls_cam));

  return;
}

// A function to read Linescan cameras from disk. There will
// be just one of them, but same convention is kept as for Pinhole
// where there is many of them. Note that the camera is created as of CSM type,
// rather than asp::CsmModel type. This is not important as we will
// abstract it right away to the base class.
void readLinescanCameras(SatSimOptions const& opt, 
    std::vector<std::string> & cam_names,
    std::vector<vw::CamPtr> & cams) {

  // Read the camera names
  vw::vw_out() << "Reading: " << opt.camera_list << std::endl;
  asp::read_list(opt.camera_list, cam_names);

  // Sanity checks
  if (cam_names.empty())
    vw::vw_throw(vw::ArgumentErr() << "No cameras were found.\n");
  if (cam_names.size() != 1)
    vw::vw_throw(vw::ArgumentErr() << "Only one linescan camera is expected.\n");

  cams.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++)
    cams[i] = vw::CamPtr(new asp::CsmModel(cam_names[i]));

  return;
}

} // end namespace asp
