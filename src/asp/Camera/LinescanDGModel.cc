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

#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/CsmModel.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/Utilities.h>

using namespace vw;

namespace asp {

// -----------------------------------------------------------------
// LinescanDGModel supporting functions

boost::posix_time::ptime parse_dg_time(std::string str) {
  try{
    return boost::posix_time::time_from_string(str);
  }catch(...){
    // This is a useful error, it tells the user an XML camera file is
    // trying to be interpreted as a DG camera file.
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse time from string: " << str << ". "
                 << "If you are not using Digital Globe images, you may need to "
                 << "specify the session type, such as -t rpc, -t rpcmaprpc, -t aster, etc.\n");
  }
  return boost::posix_time::time_from_string(str); // Never reached!
}

vw::CamPtr load_dg_camera_model_from_xml(std::string const& path){

  // Parse the Digital Globe XML file
  GeometricXML geo;
  AttitudeXML  att;
  EphemerisXML eph;
  ImageXML     img;
  RPCXML       rpc;

  try {
    read_xml(path, geo, att, eph, img, rpc);
  } catch (const std::exception& e){
    vw::vw_throw(vw::ArgumentErr() << "Invalid Digital Globe XML file: " << path << ". "
                 << "If you are not using Digital Globe images, you may "
                 << "need to specify the session type, such as -t rpc, "
                 << "-t rpcmaprpc, -t aster, etc.\n"
		 << e.what() << "\n");
  }
  
  // Get an estimate of the surface elevation from the corners specified in the file.
  // - Not every file has this information, in which case we will just use zero.
  double mean_ground_elevation = 0.0;
  vw::BBox3 bbox = rpc.get_lon_lat_height_box();
  if (!bbox.empty())
    mean_ground_elevation = (bbox.min()[2] + bbox.max()[2]) / 2.0;
  
  // Convert measurements in millimeters to pixels.
  geo.principal_distance /= geo.detector_pixel_pitch;
  geo.detector_origin    /= geo.detector_pixel_pitch;

  // Convert all time measurements to something that boost::date_time can read.
  boost::replace_all(eph.start_time,            "T", " ");
  boost::replace_all(img.tlc_start_time,        "T", " ");
  boost::replace_all(img.first_line_start_time, "T", " ");
  boost::replace_all(att.start_time,            "T", " ");

  // Convert UTC time measurements to line measurements. Ephemeris
  // start time will be our reference frame to calculate seconds against.
  SecondsFromRef convert(parse_dg_time(eph.start_time));

  // It is assumed that EPH and ATT are sampled at the same rate and time.
  VW_ASSERT(eph.satellite_position_vec.size() == att.satellite_quat_vec.size(),
            vw::MathErr() << "Ephemeris and attitude don't have the same number of samples.");
  VW_ASSERT(eph.start_time == att.start_time && eph.time_interval == att.time_interval,
            vw::MathErr() << "Ephemeris and attitude don't seem to use the same t0 or dt.");

  // Load up the time interpolation class. If the TLCList only has
  // one entry, then we have to manually drop in the slope and offset.
  if (img.tlc_vec.size() == 1) {
    double direction = 1;
    if (boost::to_lower_copy(img.scan_direction) != "forward") {
      direction = -1;
    }
    img.tlc_vec.push_back(std::make_pair(img.tlc_vec.front().first +
                                         img.avg_line_rate, direction));
  }

  // Build the TLCTimeInterpolation object and do a quick sanity check.
  vw::camera::TLCTimeInterpolation
    tlc_time_interpolation(img.tlc_vec, convert(parse_dg_time(img.tlc_start_time)));
  
  VW_ASSERT(fabs(convert(parse_dg_time(img.first_line_start_time)) -
  tlc_time_interpolation(0)) < fabs(1.0 / (10.0 * img.avg_line_rate)),
	     vw::MathErr()
	     << "First line time and output from TLC lookup table "
	     << "do not agree of the ephemeris time for the first line of the image. "
	     << "If your XML camera files are not from the WorldView satellites, "
	     << "you may try the switch -t rpc to use the RPC camera model.\n"
	     << "The first image line ephemeris time is: "
  	     << convert(parse_dg_time(img.first_line_start_time)) << ".\n"
	     << "The TLC look up table time is: " << tlc_time_interpolation(0) << ".\n"
	     << "Maximum allowed difference is 1/10 of avg line rate, which is: "
	     << fabs(1.0 / (10.0 * img.avg_line_rate))
	     << ".\n");

  double et0 = convert(parse_dg_time(eph.start_time));
  double at0 = convert(parse_dg_time(att.start_time));
  double edt = eph.time_interval;
  double adt = att.time_interval;

  if ((stereo_settings().enable_correct_velocity_aberration ||
       stereo_settings().enable_correct_atmospheric_refraction) &&
      stereo_settings().dg_use_csm)
    vw::vw_throw(vw::ArgumentErr() << "Cannot correct velocity aberration or "
                 << "atmospheric refraction with the CSM model.\n");
  
  vw::Quat sensor_rotation = vw::math::euler_xyz_to_quaternion
    (vw::Vector3(0,0,geo.detector_rotation - M_PI/2)); // explained earlier
  vw::Quat sensor_to_body = geo.camera_attitude * sensor_rotation;
  vw::Vector2 final_detector_origin
    = subvector(inverse(sensor_rotation).rotate(vw::Vector3(geo.detector_origin[0],
							      geo.detector_origin[1],
                                                            0)), 0, 2);
  
  // Convert ephemeris to be position of camera. Change attitude to be
  // the rotation from camera frame to world frame. We also add an
  // additional 90 degree rotation to the camera frame so X is the horizontal
  // direction to the picture and +Y points down the image (in the
  // direction of flight).
  std::vector<vw::Vector3> camera_position_vec(eph.satellite_position_vec.size());
  std::vector<vw::Quat>    camera_quat_vec(att.satellite_quat_vec.size());
  for (size_t i = 0; i < eph.satellite_position_vec.size(); i++) {
    auto const& qv = att.satellite_quat_vec[i];
    vw::Quat q(qv[3], qv[0], qv[1], qv[2]); // Note the swapping, the order is now w, x, y, z.
    camera_position_vec[i] = eph.satellite_position_vec[i] + q.rotate(geo.perspective_center);
    camera_quat_vec[i] = q * sensor_to_body;
  }

  vw::CamPtr cam_ptr
    (new DGCameraModel(vw::camera::PiecewiseAPositionInterpolation(camera_position_vec,
                                                                   eph.velocity_vec, et0, edt),
                       vw::camera::LinearPiecewisePositionInterpolation(eph.velocity_vec,
                                                                        et0, edt),
                       vw::camera::SLERPPoseInterpolation(camera_quat_vec, at0, adt),
                       tlc_time_interpolation, img.image_size, final_detector_origin,
                       geo.principal_distance, mean_ground_elevation,
                       stereo_settings().enable_correct_velocity_aberration,
                       stereo_settings().enable_correct_atmospheric_refraction));
     
  return cam_ptr;
} // End function load_dg_camera_model()

// Constructor
DGCameraModel::DGCameraModel
  (vw::camera::PiecewiseAPositionInterpolation      const& position,
   vw::camera::LinearPiecewisePositionInterpolation const& velocity,
   vw::camera::SLERPPoseInterpolation               const& pose,
   vw::camera::TLCTimeInterpolation                 const& time,
   vw::Vector2i                                     const& image_size, 
   vw::Vector2                                      const& detector_origin,
   double                                           const  focal_length,
   double                                           const  mean_ground_elevation,
   bool                                                    correct_velocity,
   bool                                                    correct_atmosphere):
    DGCameraModelBase(position, velocity, pose, time, image_size, detector_origin, focal_length,
                      mean_ground_elevation, correct_velocity, correct_atmosphere) {

  if (stereo_settings().dg_use_csm) 
    vw_out() << "Using the CSM model with DigitalGlobe cameras.\n";

  // It is convenient to have the CSM model exist even if it is not used.
  // The cam_test.cc and jitter_solve.cc tools uses this assumption.
  populateCsmModel();
}
  
// This is a lengthy function that does many initializations  
void DGCameraModel::populateCsmModel() {

  // Using a desired precision of 1e-8 will result in about this much
  // agreement between image to ground and back. Pushing this to
  // something lower will result in the CSM ground-to-image
  // computation failing due to numerical precision issues, which can
  // be traced to the DG camera using a focal length (in pixels) of
  // 2,002,252.25.
  m_csm_model.reset(new CsmModel);
  m_csm_model->m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISISON;
  vw::cartography::Datum datum("WGS84"); // this sensor is used for Earth only
  m_csm_model->m_semi_major_axis = datum.semi_major_axis();
  m_csm_model->m_semi_minor_axis = datum.semi_minor_axis();
    
  // Create a linescan model as a smart pointer, and do smart pointer
  // casting Follow the CSM API. The type of m_gm_model is
  // csm::RasterGM, which is a base class. UsgsAstroLsSensorModel is
  // derived from it. A smart pointer to m_gm_model is held by
  // m_csm_model.
  m_csm_model->m_gm_model.reset(new UsgsAstroLsSensorModel);
  m_ls_model = boost::dynamic_pointer_cast<UsgsAstroLsSensorModel>
    (m_csm_model->m_gm_model);
  if (m_ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");
    
  // This performs many initializations apart from the above. Note that this is
  // not a boost::shared_ptr reset, it is UsgsAstroLsSensorModel reset.
  m_ls_model->reset();
    
  m_ls_model->m_nSamples = m_image_size[0]; 
  m_ls_model->m_nLines   = m_image_size[1];

  double f = m_focal_length;

  m_ls_model->m_platformFlag = 1; // For order 8 Lagrange interpolation
  m_ls_model->m_maxElevation =  10000.0; //  10 km
  m_ls_model->m_minElevation = -10000.0; // -10 km
  m_ls_model->m_focalLength  =  f;
  m_ls_model->m_zDirection   = 1.0;
  m_ls_model->m_halfSwath    = 1.0;
  m_ls_model->m_sensorIdentifier = "DigitalGlobeLinescan";
  m_ls_model->m_majorAxis = m_csm_model->m_semi_major_axis;
  m_ls_model->m_minorAxis = m_csm_model->m_semi_minor_axis;

  // The choices below are because of how DigitalGlobe's
  // get_local_pixel_vector() interacts with the
  // UsgsAstroLsSensorModel function
  // computeDistortedFocalPlaneCoordinates(). For Pleiades
  // get_local_pixel_vector() is computed differently, and hence
  // different choices here as well. Also keep in mind that a CSM
  // pixel has extra 0.5 added to it.
  m_ls_model->m_iTransL[0]             = 0.0;  
  m_ls_model->m_iTransL[1]             = 0.0;
  m_ls_model->m_iTransL[2]             = 1.0;
  m_ls_model->m_iTransS[0]             = 0.0;
  m_ls_model->m_iTransS[1]             = 1.0;
  m_ls_model->m_iTransS[2]             = 0.0;
  m_ls_model->m_detectorLineOrigin     = 0.0;
  m_ls_model->m_detectorSampleOrigin   = 0.0;
  m_ls_model->m_detectorLineSumming    = 1.0;
  // TODO(oalexan1): Consider subtracting 0.5 below, and then adding
  // only 0.5 to tlc[i].first, further down. Looks to produce similar
  // but not quite the same result, but there was no time for a lot of
  // testing doing it that way.
  m_ls_model->m_startingDetectorLine   = m_detector_origin[1];
  m_ls_model->m_detectorSampleSumming  = 1.0;
  m_ls_model->m_startingDetectorSample = (m_detector_origin[0] - 0.5);

  // Time
  auto const& tlc = m_time_func.m_tlc;
  double time_offset = m_time_func.m_time_offset;
  if (tlc.size() < 2) 
    vw::vw_throw(vw::ArgumentErr()
                 << "Expecting at least two line and time sample pairs.\n");
  m_ls_model->m_intTimeLines.clear(); // not needed, but best for clarity
  m_ls_model->m_intTimeStartTimes.clear();
  m_ls_model->m_intTimes.clear();
  for (size_t i = 0; i < tlc.size(); i++) {
    m_ls_model->m_intTimeLines.push_back(tlc[i].first + 1.0); // line
    m_ls_model->m_intTimeStartTimes.push_back(tlc[i].second + time_offset); // time
    // Slope
    if (i + 1 < tlc.size()) {
      // Compute the slope between this time instance and the next
      double slope = (tlc[i+1].second - tlc[i].second) / (tlc[i+1].first - tlc[i].first);
      m_ls_model->m_intTimes.push_back(slope);
    } else{
      // Cannot have a slope for the last value, as there's no next one to use,
      // but for consistency, borrow the last slope. This is consistent
      // with lines out of range using the slopes closest to them.
      double slope = m_ls_model->m_intTimes.back();
      m_ls_model->m_intTimes.push_back(slope);
    }
  }

  // Compute positions and velocities. DigitalGlobe uses linear
  // interpolation based on tabulated positions and velocities to find
  // any given position, via the class
  // PiecewiseAPositionInterpolation. But UsgsAstroLsSensorModel
  // expects Lagrange interpolation of positions only, and makes no
  // use of velocities for that. For backward compatibility, provide
  // UsgsAstroLsSensorModel with more position samples by a given
  // factor, with each sample obtained with
  // PiecewiseAPositionInterpolation, so it makes use of both
  // positions and velocities.
  // TODO(oalexan1): Are positions sampled finely enough?
  int factor = 1;
  double old_t0 = m_position_func.get_t0();
  double old_dt = m_position_func.get_dt();
  double old_tend = m_position_func.get_tend();
  int num_old_pos = round((old_tend - old_t0)/old_dt) + 1;
  int num_new_pos = factor * (num_old_pos - 1) + 1; // care here, to not go out of bounds
  m_ls_model->m_numPositions = 3 * num_new_pos; // concatenate all coordinates
  m_ls_model->m_t0Ephem = old_t0; // same starting position
  m_ls_model->m_dtEphem = old_dt / factor; // finer sampling
  m_ls_model->m_positions.resize(m_ls_model->m_numPositions);
  m_ls_model->m_velocities.resize(m_ls_model->m_numPositions);
  for (int pos_it = 0; pos_it < num_new_pos; pos_it++) {
    // The largest value of t will be no more than old_tend, within numerical precision
    double t = m_ls_model->m_t0Ephem + pos_it * m_ls_model->m_dtEphem;
    vw::Vector3 P = m_position_func(t);
    vw::Vector3 V = m_velocity_func(t);
    for (int coord = 0; coord < 3; coord++) {
      m_ls_model->m_positions [3*pos_it + coord] = P[coord];
      m_ls_model->m_velocities[3*pos_it + coord] = V[coord];
    }
  }

  // Quaternions
  // TODO(oalexan1): Are the quaternions sampled finely enough?
  // Since DG has a lot of them, we assume there's no need for more.
  // ASP's old approach used linear quaternion interpolation,
  // but CSM uses Lagrange interpolation.
  int num_quat = m_pose_func.m_pose_samples.size();
  m_ls_model->m_numQuaternions = 4 * num_quat; // concatenate all coordinates
  m_ls_model->m_t0Quat = m_pose_func.m_t0; // quaternion t0
  m_ls_model->m_dtQuat = m_pose_func.m_dt; // quaternion dt
  m_ls_model->m_quaternions.resize(m_ls_model->m_numQuaternions);

  for (int pos_it = 0; pos_it < m_ls_model->m_numQuaternions / 4; pos_it++) {
    double t = m_ls_model->m_t0Quat + pos_it * m_ls_model->m_dtQuat;
    vw::Quat q = m_pose_func.m_pose_samples[pos_it];
    // ASP stores the quaternions as (w, x, y, z). CSM wants them as
    // x, y, z, w.
    int coord = 0;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.x(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.y(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.z(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.w(); coord++;
  }

  // Re-creating the model from the state forces some operations to
  // take place which are inaccessible otherwise.
  std::string modelState = m_ls_model->getModelState();
  m_ls_model->replaceModelState(modelState);
}

// Re-implement base class functions
  
double DGCameraModel::get_time_at_line(double line) const {
  
  if (stereo_settings().dg_use_csm) {
    csm::ImageCoord csm_pix;
    vw::Vector2 pix(0, line);
    asp::toCsmPixel(pix, csm_pix);
    return m_ls_model->getImageTime(csm_pix);
  }
  
  return m_time_func(line);
}

vw::Vector3 DGCameraModel::get_camera_center_at_time(double time) const {
  if (stereo_settings().dg_use_csm) {
    csm::EcefCoord ecef = m_ls_model->getSensorPosition(time);
    return vw::Vector3(ecef.x, ecef.y, ecef.z);
  }
  
  return m_position_func(time);
}

vw::Vector3 DGCameraModel::get_camera_velocity_at_time(double time) const {
  if (stereo_settings().dg_use_csm) {
    csm::EcefVector ecef = m_ls_model->getSensorVelocity(time);
    return vw::Vector3(ecef.x, ecef.y, ecef.z);
  }
  
  return m_velocity_func(time);
}

// Function to interpolate quaternions with the CSM model. This is used
// for CSM model validation but not in production.
// TODO(oalexan1): Move this to a new CsmModelUtils.cc file and call it from here.
void DGCameraModel::getQuaternions(const double& time, double q[4]) const {

  if (!stereo_settings().dg_use_csm)
    vw::vw_throw(vw::ArgumentErr()
                 << "getQuaternions: It was expected the CSM model was used.\n");
    
  int nOrder = 8;
  if (m_ls_model->m_platformFlag == 0)
    nOrder = 4;
  int nOrderQuat = nOrder;
  if (m_ls_model->m_numQuaternions < 6 && nOrder == 8)
    nOrderQuat = 4;

  lagrangeInterp(m_ls_model->m_numQuaternions / 4,
                 &m_ls_model->m_quaternions[0],
                 m_ls_model->m_t0Quat, m_ls_model->m_dtQuat,
                 time, 4, nOrderQuat, q);
}

vw::Quat DGCameraModel::get_camera_pose_at_time(double time) const {
  if (stereo_settings().dg_use_csm) {
    vw::Quat old_q = m_pose_func(time);
    double q[4];
    getQuaternions(time, q);
    return vw::Quat(q[3], q[0], q[1], q[2]); // go from (x, y, z, w) to (w, x, y, z)
  }
  
  return m_pose_func(time);
}
  
// Gives a pointing vector in the world coordinates.
vw::Vector3 DGCameraModel::pixel_to_vector(vw::Vector2 const& pix) const {

  if (stereo_settings().dg_use_csm) {
    csm::ImageCoord csm_pix;
    asp::toCsmPixel(pix, csm_pix);
    csm::EcefLocus locus = m_ls_model->imageToRemoteImagingLocus(csm_pix);
    return vw::Vector3(locus.direction.x, locus.direction.y, locus.direction.z);
  }
  
  return vw::camera::LinescanModel::pixel_to_vector(pix);
}

// As pixel_to_vector, but in the local camera frame.
vw::Vector3 DGCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {
  if (stereo_settings().dg_use_csm)
    vw::vw_throw(vw::ArgumentErr()
                 << "get_local_pixel_vector(): Cannot be called in CSM mode.\n");
  
  vw::Vector3 local_vec(pix[0] + m_detector_origin[0], m_detector_origin[1], m_focal_length);
  return normalize(local_vec);
}

// See the .h file for the documentation.
double DGCameraModel::errorFunc(double y, vw::Vector3 const& point) const {

  double t = get_time_at_line(y);
  vw::Quat q = get_camera_pose_at_time(t);
  vw::Vector3 pt = inverse(q).rotate(point - get_camera_center_at_time(t));

  return pt.y() / pt.z() - m_detector_origin[1] / m_focal_length;
}
  
// Point to pixel with no initial guess
vw::Vector2 DGCameraModel::point_to_pixel(vw::Vector3 const& point) const {
  if (stereo_settings().dg_use_csm) {

    csm::EcefCoord ecef(point[0], point[1], point[2]);
    
    // Do not show warnings, it becomes too verbose
    double achievedPrecision = -1.0;
    csm::WarningList warnings;
    csm::WarningList * warnings_ptr = NULL;
    bool show_warnings = false;
    // Do not use here a desired precision than than 1e-8, as
    // then CSM can return junk.
    csm::ImageCoord csm_pix
      = m_ls_model->groundToImage(ecef,
                                  m_csm_model->m_desired_precision,
                                  &achievedPrecision, warnings_ptr);
    
    vw::Vector2 asp_pix;
    asp::fromCsmPixel(asp_pix, csm_pix);
    
#if 0
    // This logic is from UsgsAstroLsSensorModel, with fixes to make
    // it more robust for DG cameras. Not used yet.
    
    // TODO(oalexan1): Use this logic
    // with non-CSM cameras.  Should be much faster than the current
    // approach of solving a minimization problem.  Do not set
    // desired_precision to less than 1e-8 as then the algorithm will
    // produce junk due to numerical precision issues with the large
    // DG focal length.

    // For non-CSM logic, need to start iterating from sensor
    // midpoint.  asp_pix[1] = m_image_size.y()/2;
    
    double L0 = 0.0; // Line increment
    double lineErr0 = errorFunc(L0 + asp_pix[1], point);
    double L1 = 0.1;
    double lineErr1 = errorFunc(L1 + asp_pix[1], point);
    
    for (int count = 0; count < 15; count++) {
      
      if (lineErr1 == lineErr0)
        break; // avoid division by 0
      
      // Secant method update
      // https://en.wikipedia.org/wiki/Secant_method
      double increment = lineErr1 * (L1 - L0) / (lineErr1 - lineErr0);
      double L2 = L1 - increment;
      double lineErr2 = errorFunc(L2 + asp_pix[1], point);
      
      // Update for the next step
      L0 = L1; lineErr0 = lineErr1;
      L1 = L2; lineErr1 = lineErr2;
      
      // If the solution changes by less than this, we achieved the desired line precision
      if (increment < m_csm_model->m_desired_precision) 
        break;
    }
    
    asp_pix[1] += L1;
    
    // Solve for sample location now that we know the correct line
    double t = get_time_at_line(asp_pix[1]);
    vw::Quat q = get_camera_pose_at_time(t);
    vw::Vector3 pt = inverse(q).rotate(point - get_camera_center_at_time(t));
    pt *= m_focal_length / pt.z();
    asp_pix = vw::Vector2(pt.x() - m_detector_origin[0], asp_pix[1]);
#endif

    return asp_pix;
  }
  
  // Non-CSM version
  return vw::camera::LinescanModel::point_to_pixel(point);

}
  
// TODO(oalexan1): Wipe this and use the logic above, after much testing.  
vw::Vector2 DGCameraModel::point_to_pixel(vw::Vector3 const& point, double starty) const {

  if (stereo_settings().dg_use_csm)
    vw::vw_throw(vw::ArgumentErr()
                 << "point_to_pixel(point, starty): Cannot be called in CSM mode.\n");
    
  // Use the uncorrected function to get a fast but good starting seed.
  vw::camera::CameraGenericLMA model(this, point);
  int status = -1;
  vw::Vector2 start = point_to_pixel_uncorrected(point, starty);
  
  // Run the solver
  vw::Vector3 objective(0, 0, 0);
  const double ABS_TOL = 1e-16;
  const double REL_TOL = 1e-16;
  const int    MAX_ITERATIONS = 1e+5;
  vw::Vector2 solution = vw::math::levenberg_marquardtFixed<vw::camera::CameraGenericLMA, 2,3>
    (model, start, objective, status,
     ABS_TOL, REL_TOL, MAX_ITERATIONS);
  VW_ASSERT(status > 0,
            vw::camera::PointToPixelErr() << "Unable to project point into LinescanDG model.");
  return solution;
}
  
// Camera pose
vw::Quaternion<double> DGCameraModel::camera_pose(vw::Vector2 const& pix) const {

  if (stereo_settings().dg_use_csm) {
    vw_throw(vw::NoImplErr() << "camera_pose() is not implemented with CSM.");
    return vw::Quaternion<double>();
  }
  
  return vw::camera::LinescanModel::camera_pose(pix);
}

// Gives the camera position in world coordinates.
vw::Vector3 DGCameraModel::camera_center(vw::Vector2 const& pix) const {

  if (stereo_settings().dg_use_csm) {
    csm::ImageCoord csm_pix;
    asp::toCsmPixel(pix, csm_pix);
    
    double time = m_ls_model->getImageTime(csm_pix);
    csm::EcefCoord ecef = m_ls_model->getSensorPosition(time);
    
    return vw::Vector3(ecef.x, ecef.y, ecef.z);
  }
  
  return vw::camera::LinescanModel::camera_center(pix);
}
    
} // end namespace asp

