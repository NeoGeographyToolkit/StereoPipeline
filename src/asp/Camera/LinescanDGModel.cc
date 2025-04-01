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

// TODO(oalexan1): Make this inherit from CsmModel.

#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/Covariance.h>

#include <vw/Camera/OrbitalCorrections.h>

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

vw::CamPtr load_dg_camera_model_from_xml(std::string const& path) {
  
  // Parse the Digital Globe XML file
  GeometricXML geo;
  AttitudeXML  att;
  EphemerisXML eph;
  ImageXML     img;
  RPCXML       rpc;

  // Check that cameras end in .xml.
  std::string ext = boost::filesystem::path(path).extension().string();
  boost::algorithm::to_lower(ext);
  if (ext != ".xml")
    vw::vw_throw(vw::ArgumentErr() 
      << "Digital Globe camera files must end in .xml. Likely need to specify "
      << "another session type than -t dg.\n");
    
  try {
    read_xml(path, geo, att, eph, img, rpc);
  } catch (...) {
    // The XML parser may throw exceptions other than std::exception. Catch them
    // all here.
    vw::vw_throw(vw::ArgumentErr() 
                 << "Invalid Digital Globe XML file: " << path << ". "
                 << "If you are not using Digital Globe cameras, you may "
                 << "need to specify the appropriate session type, such as -t rpc, "
                 << "-t rpcmaprpc, -t aster, etc.\n");
  }

  // For WV, only Stereo1B and Basic1B products are supported. Users
  // often say wrong results are produced with other products.
  std::string sat_id = img.sat_id; 
  boost::algorithm::to_lower(sat_id);
  std::string image_descriptor = img.image_descriptor; 
  boost::algorithm::to_lower(image_descriptor); 
  if (sat_id.size() >= 2 && sat_id.substr(0, 2) == "wv" && 
      image_descriptor != "stereo1b" && image_descriptor != "basic1b") {
    vw::vw_throw(vw::ArgumentErr() << "For WorldView images, only Stereo1B and Basic1B products are supported.\n");
  } 

  // Get an estimate of the surface elevation from the corners specified in the
  // file. Not every file has this information, in which case we will just use
  // zero. Also estimate the local Earth radius. We assume the WGS84 ellipsoid.
  // These will be used to apply velocity aberration and atmospheric refraction
  // corrections.
  double local_earth_radius = vw::DEFAULT_EARTH_RADIUS;
  double mean_ground_elevation = vw::DEFAULT_SURFACE_ELEVATION;
  vw::BBox3 bbox = rpc.get_lon_lat_height_box();
  if (!bbox.empty()) {
    mean_ground_elevation = (bbox.min()[2] + bbox.max()[2]) / 2.0;
    double lon = (bbox.min()[0] + bbox.max()[0])/2.0;
    double lat = (bbox.min()[1] + bbox.max()[1])/2.0;
    vw::cartography::Datum datum("WGS84"); 
    vw::Vector3 xyz = datum.geodetic_to_cartesian(vw::Vector3(lon, lat, 0));
    local_earth_radius = norm_2(xyz);
  }

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
            vw::MathErr() 
              << "Ephemeris and attitude don't have the same number of samples.");
  VW_ASSERT(eph.start_time == att.start_time && eph.time_interval == att.time_interval,
            vw::MathErr() 
              << "Ephemeris and attitude don't seem to use the same t0 or dt.");

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
  double first_line_time = convert(parse_dg_time(img.first_line_start_time));
  double tol = std::abs(1.0 / (10.0 * img.avg_line_rate));
  VW_ASSERT(std::abs(first_line_time - tlc_time_interpolation(0)) < tol,
	     vw::ArgumentErr()
        << "First line time does not agree with TLC time for file: " << path << ".\n"
        << "This suggests that your camera file is incorrect. To make this error go away, "
        << "edit the camera file and make the value of <TLCTIME> equal to the value of "
        << "<FIRSTLINETIME>. Use the produced cameras with caution.\n"
        << "Or consider using the RPC camera model (option -t rpc).\n"
        << "First line time is: " << first_line_time << ".\n"
        << "TLC time is: " << tlc_time_interpolation(0) << ".\n");

  double et0 = convert(parse_dg_time(eph.start_time));
  double at0 = convert(parse_dg_time(att.start_time));
  double edt = eph.time_interval;
  double adt = att.time_interval;

  vw::Quat sensor_rotation = vw::math::euler_xyz_to_quaternion
    (vw::Vector3(0,0,geo.detector_rotation - M_PI/2)); // explained earlier
  vw::Quat sensor_to_body = geo.camera_attitude * sensor_rotation;
  vw::Vector2 final_detector_origin
    = subvector(inverse(sensor_rotation).rotate(vw::Vector3(geo.detector_origin[0],
							      geo.detector_origin[1], 0)), 0, 2);

  // We will create one camera model in regular use, and 14 more of
  // them with slight perturbations if needed for error propagation
  // (covariance computation). This approach results in avoiding
  // writing a lot of new code which would be in some places similar
  // and in others different than existing one. See Covariance.h for
  // more details.
  vw::CamPtr nominal_cam;
  std::vector<vw::CamPtr> perturbed_cams;
  int num_cams = 1;
  if (asp::stereo_settings().propagate_errors)
    num_cams = numCamsForCovariance();

  for (int cam_it = 0; cam_it < num_cams; cam_it++) {
    vw::Vector<double, 3> dp = asp::positionDelta(cam_it);
    vw::Vector<double, 4> dq = asp::quatDelta(cam_it);

    // Convert ephemeris from satellite to camera position. Change
    // attitude to be the rotation from camera frame to world
    // frame. We also add an additional 90 degree rotation to the
    // camera frame so X is the horizontal direction to the picture
    // and +Y points down the image (in the direction of flight). Must
    // apply any perturbations when still in satellite coordinates, to
    // be consistent with input covariances.
    std::vector<vw::Vector3> camera_position_vec(eph.satellite_position_vec.size());
    std::vector<vw::Quat>    camera_quat_vec(att.satellite_quat_vec.size());
    for (size_t i = 0; i < eph.satellite_position_vec.size(); i++) {
      Vector<double, 3> p = eph.satellite_position_vec[i] + dp; // add the perturbation
      Vector<double, 4> q = att.satellite_quat_vec[i];
      // The dq perturbations are chosen under the assumption that q is normalized
      double len_q = norm_2(q);
      if (len_q > 0 && asp::stereo_settings().propagate_errors) 
        q = q / len_q; // Normalization is not needed without covariance logic
      q = q + dq;
      vw::Quat qt(q[3], q[0], q[1], q[2]); // Note the swap. The order is now w, x, y, z.
      camera_position_vec[i] = p + qt.rotate(geo.perspective_center);
      camera_quat_vec[i] = qt * sensor_to_body;
    }

    vw::CamPtr cam_ptr
      (new DGCameraModel(vw::camera::PiecewiseAPositionInterpolation
                           (camera_position_vec, eph.velocity_vec, et0, edt),
                         vw::camera::LinearPiecewisePositionInterpolation
                           (eph.velocity_vec, et0, edt),
                         vw::camera::SLERPPoseInterpolation(camera_quat_vec, at0, adt),
                         tlc_time_interpolation, img.image_size, final_detector_origin,
                         geo.principal_distance, mean_ground_elevation, local_earth_radius));

    if (cam_it == 0) 
      nominal_cam = cam_ptr;
    else
      perturbed_cams.push_back(cam_ptr);
  }

  DGCameraModel * cam = (DGCameraModel*)nominal_cam.get();
  
  // Store the starting time and spacing for the satellite, in case later some resampling
  // happens to the cameras when those would use a different spacing.
  // TODO(oalexan1): This code needs to be deleted, as there is no longer any 
  // resampling of the cameras.
  cam->m_satellite_pos_t0 = et0;
  cam->m_satellite_pos_dt = edt;
  cam->m_satellite_quat_t0 = at0;
  cam->m_satellite_quat_dt = adt;
  if (asp::stereo_settings().propagate_errors) {
    cam->m_perturbed_cams = perturbed_cams; 
    cam->m_satellite_pos_cov = eph.satellite_pos_cov;
    cam->m_satellite_quat_cov = att.satellite_quat_cov;
  }
  
  return nominal_cam;
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
   double                                           const  local_earth_radius):
 m_position_func(position), m_velocity_func(velocity),
 m_pose_func(pose), m_time_func(time), m_image_size(image_size),
 m_detector_origin(detector_origin),
 m_focal_length(focal_length), m_mean_ground_elevation(mean_ground_elevation),
 m_local_earth_radius(local_earth_radius) {
   // Populate the underlying CSM model
   populateCsmModel();
}
  
// This is a lengthy function that does many initializations  
// TODO(oalexan1): Replace this with a call to populateCsmLinescan()
// from CsmUtils.cc, but that will need some refactoring first to handle
// this case. 
void DGCameraModel::populateCsmModel() {

  // Using a desired precision of 1e-8 will result in about this much
  // agreement between image to ground and back. Pushing this to
  // something lower will result in the CSM ground-to-image
  // computation failing due to numerical precision issues, which can
  // be traced to the DG camera using a focal length (in pixels) of
  // 2,002,252.25.

  // This sensor is used for Earth only
  vw::cartography::Datum datum("WGS84"); 
  this->m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  this->m_semi_major_axis = datum.semi_major_axis(); 
  this->m_semi_minor_axis = datum.semi_minor_axis();
    
  // Create a linescan model as a smart pointer, and do smart pointer
  // casting. Follow the CSM API. The type of m_gm_model is
  // csm::RasterGM, which is a base class. UsgsAstroLsSensorModel is
  // derived from it. A smart pointer to m_gm_model is held by
  // this model.
  this->m_gm_model.reset(new UsgsAstroLsSensorModel);
  m_ls_model = boost::dynamic_pointer_cast<UsgsAstroLsSensorModel>
    (this->m_gm_model);
  if (m_ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");
    
  // This performs many initializations apart from the above. Note that this is
  // not a boost::shared_ptr reset, it is UsgsAstroLsSensorModel reset.
  m_ls_model->reset();
    
  m_ls_model->m_nSamples         = m_image_size[0];
  m_ls_model->m_nLines           = m_image_size[1];
  m_ls_model->m_platformFlag     = 1; // For order 8 Lagrange interpolation
  m_ls_model->m_maxElevation     = 10000.0; //  10 km
  m_ls_model->m_minElevation     = -10000.0; // -10 km
  m_ls_model->m_focalLength      = m_focal_length;
  m_ls_model->m_zDirection       = 1.0;
  m_ls_model->m_halfSwath        = 1.0;
  m_ls_model->m_sensorIdentifier = "DigitalGlobeLinescan";
  m_ls_model->m_majorAxis        = this->m_semi_major_axis;
  m_ls_model->m_minorAxis        = this->m_semi_minor_axis;

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
  m_ls_model->m_detectorLineSumming    = 1.0;
  m_ls_model->m_detectorSampleSumming  = 1.0;
  
  // Keep these as is. Modify instead m_detectorLineOrigin and
  // m_detectorSampleOrigin. The effect is same as all USGSCSM code uses
  // m_detectorLineOrigin - m_startingDetectorLine, and the same for the sample.
  m_ls_model->m_startingDetectorLine   = 0.0;
  m_ls_model->m_startingDetectorSample = 0.0;

  // Optical center
  m_ls_model->m_detectorLineOrigin     = -m_detector_origin[1];
  m_ls_model->m_detectorSampleOrigin   = -m_detector_origin[0] + 0.5;

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
    // The 1.0 below is what is needed to make CSM agree with the older ASP linescan
    // implementation. Part of it is a 0.5 pixel offset in the line. 
    // TODO(oalexan1): This needs more thinking.
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

  // Pass to CSM the positions and velocities. CSM uses Lagrange interpolation.
  m_ls_model->m_numPositions = 3 * m_position_func.m_position_samples.size();
  m_ls_model->m_t0Ephem = m_position_func.m_t0; // position t0
  m_ls_model->m_dtEphem = m_position_func.m_dt; // position dt
  m_ls_model->m_positions.resize(m_ls_model->m_numPositions);
  m_ls_model->m_velocities.resize(m_ls_model->m_numPositions);
  for (size_t pos_it = 0; pos_it < m_position_func.m_position_samples.size(); pos_it++) {
    vw::Vector3 P = m_position_func.m_position_samples[pos_it];
    vw::Vector3 V = m_velocity_func.m_position_samples[pos_it];
    for (int coord = 0; coord < 3; coord++) {
      m_ls_model->m_positions [3*pos_it + coord] = P[coord];
      m_ls_model->m_velocities[3*pos_it + coord] = V[coord];
    }
  }

  // Quaternions. CSM uses Lagrange interpolation.
  int num_quat = m_pose_func.m_pose_samples.size();
  m_ls_model->m_numQuaternions = 4 * num_quat; // concatenate all coordinates
  m_ls_model->m_t0Quat = m_pose_func.m_t0; // quaternion t0
  m_ls_model->m_dtQuat = m_pose_func.m_dt; // quaternion dt
  m_ls_model->m_quaternions.resize(m_ls_model->m_numQuaternions);

  for (int pos_it = 0; pos_it < m_ls_model->m_numQuaternions / 4; pos_it++) {
    vw::Quat q = m_pose_func.m_pose_samples[pos_it];
    // ASP stores the quaternions as (w, x, y, z). CSM wants them as
    // x, y, z, w.
    int coord = 0;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.x(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.y(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.z(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.w(); coord++;
  }

  // Quaternions must always be normalized and not change suddenly in sign.
  asp::normalizeQuaternions(m_ls_model.get());
  
  // Re-creating the model from the state forces some operations to
  // take place which are inaccessible otherwise.
  std::string modelState = m_ls_model->getModelState();
  m_ls_model->replaceModelState(modelState);

  // Adjust the CSM model to correct for velocity aberration and atmospheric
  // refraction. This can only happen after the model is fully initialized, as
  // need to create rays from the camera center to the ground.
  bool correct_velocity_aberration = true;
  bool correct_atmospheric_refraction = true;
  asp::orbitalCorrections(this, 
                          correct_velocity_aberration, correct_atmospheric_refraction,
                          m_local_earth_radius, m_mean_ground_elevation);
  
  return;
}

// Interpolate the satellite position covariance at given pixel
void DGCameraModel::interpSatellitePosCov(vw::Vector2 const& pix,
                                          double p_cov[SAT_POS_COV_SIZE]) const {
  
  double time = asp::get_time_at_line(pix.y(), m_ls_model.get());
  int numCov = m_satellite_pos_cov.size() / SAT_POS_COV_SIZE;

  int nOrder = 8;
  if (m_ls_model->m_platformFlag == 0)
    nOrder = 4;

  lagrangeInterp(numCov, &m_satellite_pos_cov[0], m_satellite_pos_t0, m_satellite_pos_dt,
                 time, SAT_POS_COV_SIZE, nOrder, p_cov);
  //asp::nearestNeibInterp(numCov, &m_satellite_pos_cov[0], m_satellite_pos_t0, 
  //                       m_satellite_pos_dt, time, SAT_POS_COV_SIZE, p_cov);
}

// Interpolate the satellite quaternion covariance at given pixel
void DGCameraModel::interpSatelliteQuatCov(vw::Vector2 const& pix,
                                           double q_cov[SAT_QUAT_COV_SIZE]) const {

  double time = asp::get_time_at_line(pix.y(), m_ls_model.get());
  int numCov = m_satellite_quat_cov.size() / SAT_QUAT_COV_SIZE;
  
  int nOrder = 8;
  if (m_ls_model->m_platformFlag == 0)
    nOrder = 4;
  int nOrderQuat = nOrder;
  if (numCov < 6 && nOrder == 8)
    nOrderQuat = 4;

  // The covariances are not guaranteed to be smooth, so use nearest neighbor.
  // Interpolation may even create matrices with negative eigenvalues.
  //lagrangeInterp(numCov, &m_satellite_quat_cov[0], m_satellite_quat_t0, m_satellite_quat_dt,
  //               time, SAT_QUAT_COV_SIZE, nOrderQuat, q_cov);
  asp::nearestNeibInterp(numCov, &m_satellite_quat_cov[0], 
                    m_satellite_quat_t0, m_satellite_quat_dt,
                    time, SAT_QUAT_COV_SIZE, q_cov);
}

} // end namespace asp

