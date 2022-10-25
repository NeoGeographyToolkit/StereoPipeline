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

#include <vw/Camera/CameraSolve.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/PleiadesXML.h>
#include <asp/Camera/LinescanPleiadesModel.h>
#include <asp/Camera/CsmModel.h>
#include <usgscsm/UsgsAstroLsSensorModel.h>

// TODO(oalexan1): Reset the usgs linescan model after construction,
// as it initializes many things

namespace asp {

// Constructor
PleiadesCameraModel::
PleiadesCameraModel(vw::camera::LinearTimeInterpolation const& time,
                    vw::camera::LagrangianInterpolation const& position,
                    vw::camera::LagrangianInterpolation const& velocity,
                    double                                     quat_offset_time,
                    double                                     quat_scale,
                    std::vector<vw::Vector<double, 4>>  const& quaternion_coeffs,
                    vw::Vector2                         const& coeff_psi_x,
                    vw::Vector2                         const& coeff_psi_y,
                    vw::Vector2i                        const& image_size,
                    double min_time, double max_time,
                    int ref_col, int ref_row,
                    bool   correct_velocity, bool correct_atmosphere):
  vw::camera::LinescanModel(image_size, correct_velocity, correct_atmosphere),
  m_position_func(position), m_velocity_func(velocity),
  m_quat_offset_time(quat_offset_time), m_quat_scale(quat_scale),
  m_quaternion_coeffs(quaternion_coeffs),
  m_time_func(time), m_coeff_psi_x(coeff_psi_x), m_coeff_psi_y(coeff_psi_y),
  m_min_time(min_time), m_max_time(max_time), m_ref_col(ref_col), m_ref_row(ref_row) {

  if (!asp::stereo_settings().linescan_no_csm_model)
    populateCsmModel();
}

void PleiadesCameraModel::populateCsmModel() {

  // Model creation
  m_csm_model.reset(new UsgsAstroLsSensorModel);

  // TODO(oalexan1): May not need this
  m_csm_no_adjustment.assign(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);

  // This performs many initializations
  m_csm_model->reset();

  // Override some initializations
  m_csm_model->m_nSamples = m_image_size[0]; 
  m_csm_model->m_nLines   = m_image_size[1];
  m_csm_model->m_platformFlag = 1; // explicitly set to 1, to have order 8 Lagrange interpolation
  
  // Datum
  vw::cartography::Datum datum("WGS84");
  m_csm_model->m_majorAxis = datum.semi_major_axis();
  m_csm_model->m_minorAxis = datum.semi_minor_axis();

  // Time
  m_csm_model->m_intTimeLines.push_back(1.0); // to offset CSM's quirky 0.5 additions in places
  m_csm_model->m_intTimeStartTimes.push_back(m_time_func.m_t0);
  m_csm_model->m_intTimes.push_back(m_time_func.m_dt);
  int num_pos = m_position_func.m_samples.size();
  if ((size_t)num_pos != m_velocity_func.m_samples.size())
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many positions as velocities.\n");

  // Positions and velocities
  m_csm_model->m_numPositions = 3 * num_pos;
  m_csm_model->m_t0Ephem = m_position_func.get_t0();
  m_csm_model->m_dtEphem = m_position_func.get_dt();
  m_csm_model->m_positions.resize(m_csm_model->m_numPositions);
  m_csm_model->m_velocities.resize(m_csm_model->m_numPositions);
  for (int pos_it = 0; pos_it < num_pos; pos_it++) {
    for (int coord = 0; coord < 3; coord++) {
      m_csm_model->m_positions [3*pos_it + coord] = m_position_func.m_samples[pos_it][coord];
      m_csm_model->m_velocities[3*pos_it + coord] = m_velocity_func.m_samples[pos_it][coord];
    }
  }

  // Sanity checks
  double max_pos_diff = 0.0;
  double max_time_diff = 0.0;
  for (int l = -3000; l < 30000; l+= 1000) {
    vw::Vector2 a(l, l);
    csm::ImageCoord b;
    asp::toCsmPixel(a, b);
    //std::cout << "ASP time " << l << ' ' << m_time_func(l) << std::endl;
    //std::cout << "CSM time " << l << ' ' << m_csm_model->getImageTime(b) << std::endl;
    max_time_diff = std::max(max_time_diff,
                             std::abs(m_time_func(l) - m_csm_model->getImageTime(b)));
                             
    double time = m_csm_model->getImageTime(b);
    bool calc_vel = false;
    double xc, yc, zc;
    double vx, vy, vz;
    double xl, yl, zl;
    
    std::cout.precision(18);
    csm::EcefCoord xyz = m_csm_model->getSensorPosition(time);
    vw::Vector3 xyz1(xyz.x, xyz.y, xyz.z);
    
    vw::Vector3 xyz2 = this->get_camera_center_at_time(time);
    max_pos_diff = std::max(max_pos_diff, vw::math::norm_2(xyz1 - xyz2));
  }

  std::cout << "--max time diff " << max_time_diff << std::endl;
  std::cout << "--max pos diff " << max_pos_diff << std::endl;
}

vw::Vector2 PleiadesCameraModel::point_to_pixel(vw::Vector3 const& point) const {
  return point_to_pixel(point, -1); // Redirect to the function with no initial guess
}
  
// TODO: Port these changes to the base class
vw::Vector2 PleiadesCameraModel::point_to_pixel(vw::Vector3 const& point, double starty) const {

  // Use the generic solver to find the pixel 
  // - This method will be slower but works for more complicated geometries
  vw::camera::CameraGenericLMA model(this, point);
  int status;
  vw::Vector2 start = m_image_size / 2.0; // Use the center as the initial guess
  if (starty >= 0) // If the user provided a line number guess, use it.
    start[1] = starty;

  // Solver constants
  const double ABS_TOL = 1e-16;
  const double REL_TOL = 1e-16;
  const int    MAX_ITERATIONS = 1e+5;
  const double MAX_ERROR = 0.01;

  vw::Vector3 objective(0, 0, 0);
  vw::Vector2 solution = vw::math::levenberg_marquardtFixed<vw::camera::CameraGenericLMA, 2,3>
    (model, start, objective, status, ABS_TOL, REL_TOL, MAX_ITERATIONS);

  double  error = norm_2(model(solution));

  VW_ASSERT( (status > 0) && (error < MAX_ERROR),
             vw::camera::PointToPixelErr()
             << "Unable to project point into LinescanPleiades model" );

  return solution;
}

void PleiadesCameraModel::check_time(double time, std::string const& location) const {
  if ((time < m_min_time) || (time > m_max_time))
    vw::vw_throw(vw::ArgumentErr() << "PleiadesCameraModel::"<<location
                 << ": Requested time "<<time<<" is out of bounds ("
                 << m_min_time << " <-> "<<m_max_time<<")\n");
}

vw::Vector3 PleiadesCameraModel::get_camera_center_at_time(double time) const {
  check_time(time, "get_camera_center_at_time");
  return m_position_func(time);
}
vw::Vector3 PleiadesCameraModel::get_camera_velocity_at_time(double time) const { 
  check_time(time, "get_camera_velocity_at_time");
  return m_velocity_func(time); 
}

// Compute the quaternion at given time using the polynomial
// expression (doc page 77)
vw::Quat PleiadesCameraModel::get_camera_pose_at_time(double time) const {

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
  
  vw::Quaternion<double> q(v[0], v[1], v[2], v[3]); // order is w, x, y, z
  
  return q;
}

// Allow finding the time at any line, even negative ones. Here a
// simple slope-intercept formula is used rather than a table so one
// cannot run out of bounds. Page 76 in the doc.
double PleiadesCameraModel::get_time_at_line(double line) const {
  return m_time_func(line); 
}

// Page 76 in the doc
vw::Vector3 PleiadesCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {

  double col = pix[0];
  double row = pix[1];
  
  // The doc says to subtract m_ref_col, while it was found
  // experimentally that one needs to add it.
  double x   = m_coeff_psi_x[0] + m_coeff_psi_x[1] * (col  + m_ref_col);

  // Note how below col (and not row) is used, per the doc.
  double y   = m_coeff_psi_y[0] + m_coeff_psi_y[1] * (col  + m_ref_col);

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
    time_func      = xml_reader.setup_time_func();
  vw::camera::LagrangianInterpolation
    position_func  = xml_reader.setup_position_func(time_func);
  vw::camera::LagrangianInterpolation
    velocity_func  = xml_reader.setup_velocity_func(time_func);

  // Find the range of times for which we can solve for position and pose
  double min_position_time = position_func.get_t0();
  double max_position_time = position_func.get_tend();
  double min_velocity_time = velocity_func.get_t0();
  double max_velocity_time = velocity_func.get_tend();

  double min_time = std::max(min_position_time, min_velocity_time);
  double max_time = std::min(max_position_time, max_velocity_time);

  // Without these corrections the RPC and exact models agree to within
  // 0.003 pixels as shown by cam_test.
  bool correct_velocity_aberration = false;
  bool correct_atmospheric_refraction = false;
  
  // Create the model. This can throw an exception.
  boost::shared_ptr<PleiadesCameraModel> cam
    (new PleiadesCameraModel(time_func, position_func, velocity_func,
                             xml_reader.m_quat_offset_time,
                             xml_reader.m_quat_scale,
                             xml_reader.m_quaternion_coeffs,
                             xml_reader.m_coeff_psi_x,
                             xml_reader.m_coeff_psi_y,
                             xml_reader.m_image_size,
                             min_time, max_time,
                             xml_reader.m_ref_col, xml_reader.m_ref_row,
                             correct_velocity_aberration,
                             correct_atmospheric_refraction));
  
  return cam;
} // End function load_pleiades_camera_model()

} // end namespace asp

