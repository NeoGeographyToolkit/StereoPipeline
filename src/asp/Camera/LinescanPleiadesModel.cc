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

#include <asp/Core/StereoSettings.h>
#include <asp/Camera/PleiadesXML.h>
#include <asp/Camera/LinescanPleiadesModel.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/CsmUtils.h>
#include <vw/Camera/CameraSolve.h>
#include <vw/Camera/OrbitalCorrections.h>
#include <usgscsm/UsgsAstroLsSensorModel.h>

// This class implements the Pleiades linescan model
// based on the CSM model.
// TODO(oalexan1): Need to think more about the desired precision of the CSM
// model. Need high accuracy for bundle adjustment, but may get away
// with less for mapprojection. For now, err towards more accuracy.
namespace asp {
// Constructor
PleiadesCameraModel::
PleiadesCameraModel(vw::camera::LinearTimeInterpolation const& time,
                    vw::camera::LagrangianInterpolation const& position,
                    vw::camera::LagrangianInterpolation const& velocity,
                    bool isNeo, double t0Quat, double dtQuat,
                    double quat_offset_time, double quat_scale,
                    std::vector<vw::Vector<double, 4>>  const& quaternion_coeffs,
                    vw::Vector2                         const& coeff_psi_x,
                    vw::Vector2                         const& coeff_psi_y,
                    vw::Vector2i                        const& image_size,
                    double min_time, double max_time,
                    int ref_col, int ref_row, double accuracy_stdv):
  m_position_func(position), m_velocity_func(velocity),
  m_isNeo(isNeo), m_t0Quat(t0Quat), m_dtQuat(dtQuat),
  m_quat_offset_time(quat_offset_time), m_quat_scale(quat_scale),
  m_quaternion_coeffs(quaternion_coeffs),
  m_time_func(time), m_coeff_psi_x(coeff_psi_x), m_coeff_psi_y(coeff_psi_y),
  m_min_time(min_time), m_max_time(max_time), m_ref_col(ref_col), m_ref_row(ref_row),
  m_image_size(image_size), m_accuracy_stdv(accuracy_stdv) {

  populateCsmModel();
}

void PleiadesCameraModel::populateCsmModel() {

  // Populate CsmModel class members
  // Do not use a precision below 1.0-e8 as then the linescan model will return junk.
  m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  vw::cartography::Datum datum("WGS84"); // this sensor is used for Earth only
  m_semi_major_axis = datum.semi_major_axis();
  m_semi_minor_axis = datum.semi_minor_axis();
  
  // Create the linescan model
  m_gm_model.reset(new UsgsAstroLsSensorModel); // m_gm_model will manage the deallocation
  m_ls_model = dynamic_cast<UsgsAstroLsSensorModel*>(m_gm_model.get()); // pointer to ls model
  if (m_ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");

  // This performs many initializations apart from the above
  m_ls_model->reset();

  // Override some initializations
  m_ls_model->m_nSamples = m_image_size[0]; 
  m_ls_model->m_nLines   = m_image_size[1];
  m_ls_model->m_platformFlag = 1; // explicitly set to 1, to have order 8 Lagrange interpolation
  m_ls_model->m_maxElevation =  10000.0; //  10 km
  m_ls_model->m_minElevation = -10000.0; // -10 km
  m_ls_model->m_focalLength  = 1.0;
  m_ls_model->m_zDirection   = 1.0;
  m_ls_model->m_halfSwath    = 1.0;
  m_ls_model->m_sensorIdentifier = "Pleiades";
  m_ls_model->m_majorAxis = m_semi_major_axis;
  m_ls_model->m_minorAxis = m_semi_minor_axis;
  
  m_ls_model->m_iTransL[0]   = 0.0;  
  m_ls_model->m_iTransL[1]   = 1.0; // no scale
  m_ls_model->m_iTransL[2]   = 0.0; // no skew
  m_ls_model->m_iTransS[0]   = 0.0;
  m_ls_model->m_iTransS[1]   = 0.0; // no skew
  m_ls_model->m_iTransS[2]   = 1.0; // no scale

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
  
  m_ls_model->m_detectorLineSumming   = 1.0;
  m_ls_model->m_detectorSampleSumming = -m_coeff_psi_x[1];
  
  // Keep entries below at 0. Modify instead the detector line and sample
  // origin. The effect is same as all USGSCSM code uses m_detectorLineOrigin -
  // m_startingDetectorLine, and the same for the sample.
  m_ls_model->m_startingDetectorLine   = 0.0;
  m_ls_model->m_startingDetectorSample = 0.0;
  
  // Optical center
  m_ls_model->m_detectorLineOrigin   = -m_coeff_psi_y[0]; // note: m_coeff_psi_y[1] = 0
  m_ls_model->m_detectorSampleOrigin = m_coeff_psi_x[0] + m_coeff_psi_x[1] * (m_ref_col - 0.5);

  // Time
  m_ls_model->m_intTimeLines.push_back(1.0); // to offset CSM's quirky 0.5 additions
  m_ls_model->m_intTimeStartTimes.push_back(m_time_func.m_t0);
  m_ls_model->m_intTimes.push_back(m_time_func.m_dt);
  int num_pos = m_position_func.m_samples.size();
  if ((size_t)num_pos != m_velocity_func.m_samples.size())
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many positions as velocities.\n");

  // Positions and velocities
  m_ls_model->m_numPositions = 3 * num_pos; // concatenate all coordinates
  m_ls_model->m_t0Ephem = m_position_func.get_t0();
  
  m_ls_model->m_dtEphem = m_position_func.get_dt();
  m_ls_model->m_positions.resize(m_ls_model->m_numPositions);
  m_ls_model->m_velocities.resize(m_ls_model->m_numPositions);
  for (int pos_it = 0; pos_it < num_pos; pos_it++) {
    for (int coord = 0; coord < 3; coord++) {
      m_ls_model->m_positions [3*pos_it + coord] = m_position_func.m_samples[pos_it][coord];
      m_ls_model->m_velocities[3*pos_it + coord] = m_velocity_func.m_samples[pos_it][coord];
    }
  }

  // Quaternions. For NEO these are tabulated, while for 1A/1B they come
  // from sampling a polynomial. In the latter case, 
  // quaternions are sampled over the range of times for which the position
  // and velocity are available, which a way longer range than the time spent
  // acquiring image lines.
  // TODO(oalexan1): What is the right factor (inverse of sampling rate)?
  if (m_isNeo) {
    m_ls_model->m_numQuaternions = 4 * m_quaternion_coeffs.size();
    m_ls_model->m_t0Quat = m_t0Quat;
    m_ls_model->m_dtQuat = m_dtQuat;
  } else {
    int factor = 100;
    // quaternion t0, borrow from position t0
    m_ls_model->m_t0Quat = m_ls_model->m_t0Ephem; 
    // quaternion dt, borrow from position dt
    m_ls_model->m_dtQuat = m_ls_model->m_dtEphem / factor; 
    // Great care is needed here. Number of samples is obtained
    // by dividing the range of times by the sampling rate, and
    // then adding one. 
    double beg_time = m_ls_model->m_t0Ephem;
    double end_time = beg_time + m_ls_model->m_dtEphem * (num_pos - 1);
    int num_new_pos = (int)round((end_time - beg_time) / m_ls_model->m_dtQuat) + 1;
    m_ls_model->m_numQuaternions = 4 * num_new_pos;
  }

  m_ls_model->m_quaternions.resize(m_ls_model->m_numQuaternions);
  for (int pos_it = 0; pos_it < m_ls_model->m_numQuaternions / 4; pos_it++) {
    vw::Quat q;
    if (m_isNeo) {
      vw::Vector<double, 4> const& v = m_quaternion_coeffs[pos_it]; // alias
      q = vw::Quat(v[0], v[1], v[2], v[3]); // order is w, x, y, z
    } else {
      // Sample the polynomial
      double t = m_ls_model->m_t0Quat + pos_it * m_ls_model->m_dtQuat;
      q = get_camera_pose_at_time(t);
    }

    // ASP stores the quaternions as (w, x, y, z). CSM wants them as
    // x, y, z, w.
    int coord = 0;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.x(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.y(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.z(); coord++;
    m_ls_model->m_quaternions[4*pos_it + coord] = q.w(); coord++;
  }

  // Quaternions must always be normalized and not change suddenly in sign.
  asp::normalizeQuaternions(m_ls_model);

  // Re-creating the model from the state forces some operations to
  // take place which are inaccessible otherwise.
  std::string modelState = m_ls_model->getModelState();
  m_ls_model->replaceModelState(modelState);
  
  // Adjust the CSM model to correct for velocity aberration and atmospheric
  // refraction, if desired. This can only happen after the model is fully initialized, as
  // need to create rays from the camera center to the ground.
  // TODO(oalexan1): Use more precise values below.
  double local_earth_radius = vw::DEFAULT_EARTH_RADIUS;
  double mean_ground_elevation = vw::DEFAULT_SURFACE_ELEVATION;
  asp::orbitalCorrections(this, 
                          asp::stereo_settings().enable_velocity_aberration_correction,
                          asp::stereo_settings().enable_atmospheric_refraction_correction,
                          local_earth_radius, mean_ground_elevation);
}
  
vw::Vector3 PleiadesCameraModel::camera_center(vw::Vector2 const& pix) const {
  csm::ImageCoord csm_pix;
  asp::toCsmPixel(pix, csm_pix);

  double time = m_ls_model->getImageTime(csm_pix);
  csm::EcefCoord ecef = m_ls_model->getSensorPosition(time);

  return vw::Vector3(ecef.x, ecef.y, ecef.z);
}

vw::Vector3 PleiadesCameraModel::pixel_to_vector(vw::Vector2 const& pix) const {
  csm::ImageCoord csm_pix;
  asp::toCsmPixel(pix, csm_pix);
  
  csm::EcefLocus locus = m_ls_model->imageToRemoteImagingLocus(csm_pix);
  return vw::Vector3(locus.direction.x, locus.direction.y, locus.direction.z);
}
  
vw::Vector2 PleiadesCameraModel::point_to_pixel(vw::Vector3 const& point) const {

  csm::EcefCoord ecef(point[0], point[1], point[2]);
  
  // Do not show warnings, it becomes too verbose
  double achievedPrecision = -1.0;
  csm::WarningList warnings;
  csm::WarningList * warnings_ptr = NULL;
  bool show_warnings = false;
  csm::ImageCoord csm_pix = m_ls_model->groundToImage(ecef, m_desired_precision,
                                                      &achievedPrecision, warnings_ptr);
  
  vw::Vector2 asp_pix;
  asp::fromCsmPixel(asp_pix, csm_pix);
  return asp_pix;
}
  
// This function is tricky to implement 
vw::Quaternion<double> PleiadesCameraModel::camera_pose(vw::Vector2 const& pix) const {
  // This is not implemented for now for the CSM model
  vw_throw(vw::NoImplErr() << "LinescanPleiadesModel: Cannot retrieve camera_pose!");
  return vw::Quaternion<double>();
}
  
// Allow finding the time at any line, even negative ones. Here a
// simple slope-intercept formula is used rather than a table so one
// cannot run out of bounds. Page 76 in the doc.
double PleiadesCameraModel::get_time_at_line(double line) const {
  csm::ImageCoord csm_pix;
  asp::toCsmPixel(vw::Vector2(0, line), csm_pix);
  return m_ls_model->getImageTime(csm_pix);
}
 
// Throw an exception if the input time is outside the given
// bounds. The valid range is much bigger than the range of times
// at which image lines are recorded. It is rather the range at
// which positions, velocities, and quaternions are tabulated.
void PleiadesCameraModel::check_time(double time, std::string const& location) const {
  if ((time < m_min_time) || (time > m_max_time))
    vw::vw_throw(vw::ArgumentErr() << "PleiadesCameraModel::"<<location
                 << ": Requested time "<<time<<" is out of bounds ("
                 << m_min_time << " <-> "<<m_max_time<<")\n");
}

vw::Vector3 PleiadesCameraModel::get_camera_center_at_time(double time) const {
  // TODO(oalexan1): This needs more testing. Normally it is not invoked.
  csm::EcefCoord ecef = m_ls_model->getSensorPosition(time);
  return vw::Vector3(ecef.x, ecef.y, ecef.z);
}
  
vw::Vector3 PleiadesCameraModel::get_camera_velocity_at_time(double time) const { 
  // TODO(oalexan1): This needs testing.
  csm::EcefVector ecef = m_ls_model->getSensorVelocity(time);
  return vw::Vector3(ecef.x, ecef.y, ecef.z);
}

// Compute the quaternion at given time using the polynomial
// expression (doc page 77). This should work whether or not the CSM
// model is used. This applies only to L1A/B products, and not to the NEO
// product, when the quaternions are tabulated instead.
vw::Quat PleiadesCameraModel::get_camera_pose_at_time(double time) const {

  if (m_isNeo)
    vw::vw_throw(vw::NoImplErr() << "PleiadesCameraModel: Cannot compute camera pose "
      << "from polynomial for NEO products. This is a programmer error.\n");

  double scaled_t = (time - m_quat_offset_time) / m_quat_scale;

  if (m_quaternion_coeffs.size() != 4)
    vw::vw_throw(vw::ArgumentErr() << "Expecting four quaternions.\n");
  
  vw::Vector<double, 4> v(0, 0, 0, 0); // future quaternion
  double tn = 1.0; // scaled_t to the power of n
  for (size_t it = 0; it < 4; it++) {

    for (size_t coord = 0; coord < 4; coord++)
      v[coord] += m_quaternion_coeffs[coord][it] * tn;
    
    tn *= scaled_t;
  }
  
  return vw::Quaternion<double>(v[0], v[1], v[2], v[3]); // order is w, x, y, z
}

// The pointing vector in sensor coordinates. This function is now not used,
// because CSM does the work. 
// Page 76 in the doc.
vw::Vector3 PleiadesCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {

  double col = pix[0];
  double row = pix[1];
  
  // The doc says to subtract m_ref_col, while it was found
  // experimentally that one needs to add it.
  double x = m_coeff_psi_x[0] + m_coeff_psi_x[1] * (col  + m_ref_col);

  // Note how below col (and not row) is used, per the Pleiades doc.
  // Note that m_coeff_psi_y[1] is 0.
  double y = m_coeff_psi_y[0] + m_coeff_psi_y[1] * (col  + m_ref_col);

  vw::Vector3 result = vw::Vector3(y, -x, 1.0);

  // Make the direction have unit length
  result = normalize(result);

  return result;
}

boost::shared_ptr<PleiadesCameraModel>
load_pleiades_camera_model_from_xml(std::string const& path) {

  // Parse the Pleiades XML file
  PleiadesXML xml_reader;
  xml_reader.read_xml(path);

  // Get all the initial functors
  vw::camera::LinearTimeInterpolation
    time_func = xml_reader.setup_time_func();
  vw::camera::LagrangianInterpolation
    position_func = xml_reader.setup_position_func(time_func);
  vw::camera::LagrangianInterpolation
    velocity_func = xml_reader.setup_velocity_func(time_func);
  
  // Set up the quaternions for NEO. This will create xml_reader m_t0Quat, 
  // m_dtQuat, m_quaternion_coeffs.
  if (xml_reader.m_isNeo)
    xml_reader.setup_pose_func(time_func);

  // Find the range of times for which we can solve for position and pose
  double min_position_time = position_func.get_t0();
  double max_position_time = position_func.get_tend();
  double min_velocity_time = velocity_func.get_t0();
  double max_velocity_time = velocity_func.get_tend();

  double min_time = std::max(min_position_time, min_velocity_time);
  double max_time = std::min(max_position_time, max_velocity_time);

  // Create the model. This can throw an exception.
  boost::shared_ptr<PleiadesCameraModel> cam
    (new PleiadesCameraModel(time_func, position_func, velocity_func,
                             xml_reader.m_isNeo,
                             xml_reader.m_t0Quat,
                             xml_reader.m_dtQuat,
                             xml_reader.m_quat_offset_time,
                             xml_reader.m_quat_scale,
                             xml_reader.m_quaternion_coeffs,
                             xml_reader.m_coeff_psi_x,
                             xml_reader.m_coeff_psi_y,
                             xml_reader.m_image_size,
                             min_time, max_time,
                             xml_reader.m_ref_col, xml_reader.m_ref_row,
                             xml_reader.m_accuracy_stdv));
  
  return cam;
} // End function load_pleiades_camera_model()

} // end namespace asp

