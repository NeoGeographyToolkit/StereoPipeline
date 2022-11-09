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

boost::shared_ptr<vw::camera::CameraModel> load_dg_camera_model_from_xml(std::string const& path){

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

  // I'm going make the assumption that EPH and ATT are sampled at the same rate and time.
  VW_ASSERT(eph.position_vec.size() == att.quat_vec.size(),
            vw::MathErr() << "Ephemeris and Attitude don't have the same number of samples.");
  VW_ASSERT(eph.start_time == att.start_time && eph.time_interval == att.time_interval,
            vw::MathErr() << "Ephemeris and Attitude don't seem to sample with the same t0 or dt.");

  // Convert ephemeris to be position of camera. Change attitude to
  // be the rotation from camera frame to world frame. We also add an
  // additional rotation to the camera frame so X is the horizontal
  // direction to the picture and +Y points down the image (in the direction of flight).
  vw::Quat sensor_coordinate = vw::math::euler_xyz_to_quaternion
    (vw::Vector3(0,0,geo.detector_rotation - M_PI/2));
  for (size_t i = 0; i < eph.position_vec.size(); i++) {
    eph.position_vec[i] += att.quat_vec[i].rotate(geo.perspective_center);
    att.quat_vec[i] = att.quat_vec[i] * geo.camera_attitude * sensor_coordinate;
  }

  // Load up the time interpolation class. If the TLCList only has
  // one entry ... then we have to manually drop in the slope and offset.
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
    tlc_time_interpolation(img.tlc_vec,
                           convert(parse_dg_time(img.tlc_start_time)));
  
  VW_ASSERT(fabs(convert(parse_dg_time(img.first_line_start_time)) -
  tlc_time_interpolation(0)) < fabs(1.0 / (10.0 * img.avg_line_rate)),
	     vw::MathErr()
	     << "First Line Time and output from TLC lookup table "
	     << "do not agree of the ephemeris time for the first line of the image. "
	     << "If your XML camera files are not from the WorldView satellites, "
	     << "you may try the switch -t rpc to use the RPC camera model.\n"
	     << "The first image line ephemeris time is: "
  	     << convert(parse_dg_time(img.first_line_start_time)) << ".\n"
	     << "The TLC look up table time is: " << tlc_time_interpolation(0) << ".\n"
	     << "Maximum allowed difference is 1/10 of avg line rate, which is: "
	     << fabs(1.0 / (10.0 * img.avg_line_rate))
	     << ".\n");
   
  vw::Vector2 final_detector_origin
    = subvector(inverse(sensor_coordinate).rotate(vw::Vector3(geo.detector_origin[0],
							      geo.detector_origin[1],
							      0)), 0, 2);

  double et0 = convert(parse_dg_time(eph.start_time));
  double at0 = convert(parse_dg_time(att.start_time));
  double edt = eph.time_interval;
  double adt = att.time_interval;

  // This is where we could set the Earth radius if we have that info.

  return boost::shared_ptr<vw::camera::CameraModel>
    (new DGCameraModel
     (vw::camera::PiecewiseAPositionInterpolation(eph.position_vec,
                                                  eph.velocity_vec, et0, edt),
      vw::camera::LinearPiecewisePositionInterpolation(eph.velocity_vec,
                                                       et0, edt),
      vw::camera::SLERPPoseInterpolation(att.quat_vec, at0, adt),
      tlc_time_interpolation, img.image_size, final_detector_origin,
      geo.principal_distance, mean_ground_elevation,
      stereo_settings().enable_correct_velocity_aberration,
      stereo_settings().enable_correct_atmospheric_refraction));
} // End function load_dg_camera_model()

// TODO(oalexan1): This is temporary logic. Eventually this class will
// replace LinescanDGModel, will be de-templated, moved to .cc,
// and the class PiecewiseAdjustedLinescanModel will go away.
struct DgCsmModel: public CsmModel {
      
DgCsmModel() {}

// This is a lengthy function that does many initializations  
void populateCsmModel(DGCameraModel * dg_model) {
    
  m_desired_precision = 1.0e-12; // Need high precision for bundle adjustment
  vw::cartography::Datum datum("WGS84"); // this sensor is used for Earth only
  m_semi_major_axis = datum.semi_major_axis();
  m_semi_minor_axis = datum.semi_minor_axis();
    
  // Create a linescan model as a smart pointer, and do smart pointer casting
  m_csm_model.reset(new UsgsAstroLsSensorModel); // follow CSM api, type is csm::RasterGM
  m_ls_model = boost::dynamic_pointer_cast<UsgsAstroLsSensorModel>(m_csm_model);
  if (m_ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");
    
  // This performs many initializations apart from the above
  m_ls_model->reset();
    
  m_ls_model->m_nSamples = dg_model->m_image_size[0]; 
  m_ls_model->m_nLines   = dg_model->m_image_size[1];
    
  m_ls_model->m_platformFlag = 1; // For order 8 Lagrange interpolation
  m_ls_model->m_maxElevation =  10000.0; //  10 km
  m_ls_model->m_minElevation = -10000.0; // -10 km
  m_ls_model->m_focalLength  = 1.0;
  m_ls_model->m_zDirection   = 1.0;
  m_ls_model->m_halfSwath    = 1.0;
  m_ls_model->m_sensorIdentifier = "DigitalGlobeLinescan";
  m_ls_model->m_majorAxis = m_semi_major_axis;
  m_ls_model->m_minorAxis = m_semi_minor_axis;
    
  m_ls_model->m_iTransL[0]   = 0.0;  
  m_ls_model->m_iTransL[1]   = 1.0; // no scale
  m_ls_model->m_iTransL[2]   = 0.0; // no skew
  m_ls_model->m_iTransS[0]   = 0.0;
  m_ls_model->m_iTransS[1]   = 0.0; // no skew
  m_ls_model->m_iTransS[2]   = 1.0; // no scale
  m_ls_model->m_detectorLineOrigin   = 0.0;
  m_ls_model->m_detectorSampleOrigin = 0.0;
    
  // Quantities needed to find the ray direction in the sensor plane.
  // This needs to be consistent with usgscsm functions
  // computeDistortedFocalPlaneCoordinates() and
  // createCameraLookVector(), which requires a lot of care.
  // Need to emulate this
  // double x = m_coeff_psi_x[0] + m_coeff_psi_x[1] * (col  + m_ref_col);
  // double y = m_coeff_psi_y[0] + m_coeff_psi_y[1] * (col  + m_ref_col);
  // Using this:
  // double detSample = (col + 0.5) * sampleSumming + startingSample;
  // double detLine = line * lineSumming + startingLine; // but it will use line = 0
  vw::Vector2 m_coeff_psi_x(0, 0), m_coeff_psi_y(0, 0); // temporary
  double m_ref_col = 0;
  m_ls_model->m_detectorLineSumming    = 1.0;
  m_ls_model->m_startingDetectorLine   =  m_coeff_psi_y[0]; // note that m_coeff_psi_y[1] = 0
  m_ls_model->m_detectorSampleSumming  = -m_coeff_psi_x[1];
  m_ls_model->m_startingDetectorSample
    = -m_coeff_psi_x[0] - m_coeff_psi_x[1] * (m_ref_col - 0.5);
    
  // Time
  auto const& tlc = dg_model->m_time_func.m_tlc;
  double time_offset = dg_model->m_time_func.m_time_offset;
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
    
#if 0
  // Quaternions
  // TODO(oalexan1): Are the quaternions sampled finely enough?
  int num_quat = dg_model->m_pose_func.m_pose_samples.size();
  m_ls_model->m_numQuaternions = 4 * num_quat; // concatenate all coordinates
  m_ls_model->m_t0Quat = dg_model->m_pose_func.m_t0; // quaternion t0
  m_ls_model->m_dtQuat = dg_model->m_pose_func.m_dt; // quaternion dt
  m_ls_model->m_quaternions.resize(m_ls_model->m_numQuaternions);
#endif
  std::cout << "--Enable this!" << std::endl;
#if 0
  // Re-creating the model from the state forces some operations to
  // take place which are inaccessible otherwise.
  std::string modelState = m_ls_model->getModelState();
  m_ls_model->replaceModelState(modelState);
#endif
    
  //         m_pose_func
  //           std::vector<Quat> m_pose_samples;
  //         double m_t0, m_dt, m_tend;
  //         bool m_use_splines;
}

  // Pointer to linescan sensor.
  boost::shared_ptr<UsgsAstroLsSensorModel> m_ls_model;
      
};
  
// Constructor
DGCameraModel::DGCameraModel
  (vw::camera::PiecewiseAPositionInterpolation      const& position,
   vw::camera::LinearPiecewisePositionInterpolation const& pose,
   vw::camera::SLERPPoseInterpolation               const& velocity,
   vw::camera::TLCTimeInterpolation                 const& time,
   vw::Vector2i                                     const& image_size, 
   vw::Vector2                                      const& detector_origin,
   double                                           const  focal_length,
   double                                           const  mean_ground_elevation,
   bool                                                    correct_velocity,
   bool                                                    correct_atmosphere):
DGCameraModelBase(position, pose, velocity, time, image_size, detector_origin, focal_length,
                  mean_ground_elevation, correct_velocity, correct_atmosphere) {
    
    if (stereo_settings().dg_use_csm) {
      m_dg_csm_model.reset(new DgCsmModel);
      m_dg_csm_model->populateCsmModel(this);
    }
}
  
// Re-implement base class functions
  
double DGCameraModel::get_time_at_line(double line) const {
  
  if (stereo_settings().dg_use_csm) {
    csm::ImageCoord csm_pix;
    vw::Vector2 pix(0, line);
    asp::toCsmPixel(pix, csm_pix);
    return m_dg_csm_model->m_ls_model->getImageTime(csm_pix);
  }
  
  return m_time_func(line);
}

vw::Vector3 DGCameraModel::get_camera_center_at_time(double time) const {
  return m_position_func(time);
}

// TODO(oalexan1): This is not called, so it is tricky to test.
vw::Vector3 DGCameraModel::get_camera_velocity_at_time(double time) const {
  return m_velocity_func(time);
}

vw::Quat DGCameraModel::get_camera_pose_at_time(double time) const {
  return m_pose_func(time);
}
  
// Gives a pointing vector in the world coordinates.
vw::Vector3 DGCameraModel::pixel_to_vector(vw::Vector2 const& pix) const {
  return vw::camera::LinescanModel::pixel_to_vector(pix);
}
    
// As pixel_to_vector, but in the local camera frame.
vw::Vector3 DGCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {
  vw::Vector3 local_vec(pix[0]+m_detector_origin[0], m_detector_origin[1], m_focal_length);
  return normalize(local_vec);
}
    
// Override this implementation with a faster, more specialized implementation.
vw::Vector2 DGCameraModel::point_to_pixel(vw::Vector3 const& point, double starty) const {
  // Use the uncorrected function to get a fast but good starting seed.
  vw::camera::CameraGenericLMA model(this, point);
  int status;
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
  return vw::camera::LinescanModel::camera_pose(pix);
}

// Gives the camera position in world coordinates.
vw::Vector3 DGCameraModel::camera_center(vw::Vector2 const& pix) const {
  return vw::camera::LinescanModel::camera_center(pix);
}
    
} // end namespace asp

