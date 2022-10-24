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

namespace asp {

using vw::Vector3;
using vw::Matrix3x3;

// TODO: Port these changes to the base class

vw::Vector2 PleiadesCameraModel::point_to_pixel(Vector3 const& point, double starty) const {

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

  Vector3 objective(0, 0, 0);
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
// expression (page 77)
vw::Quat PleiadesCameraModel::get_camera_pose_at_time(double time) const {

  double scaled_t = (time - m_quat_offset_time) / m_quat_scale;
  vw::Quaternion<double> q = m_quaternion_coeffs[0];
  double tn = 1.0; // scaled_t to the power of n
  std::cout << "q = " << q << std::endl;
  for (size_t it = 1; it < m_quaternion_coeffs.size(); it++) {
    tn *= scaled_t;
    std::cout << "--t = " << scaled_t << std::endl;
    std::cout << "tn = " << tn << std::endl;
    //q += tn * m_quaternion_coeffs[it];
    q = q + m_quaternion_coeffs[it] * tn;
    std::cout << "--q is " << q << std::endl;
  }

  return q;
}

double PleiadesCameraModel::get_time_at_line(double line) const {
  // Allow finding the time at any line, even negative ones.  Here a
  // simple slope-intercept formula is used rather than a table so one
  // cannot run out of bounds.
  return m_time_func(line); 
}


Vector3 PleiadesCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {

  // According to Modelo%20Orbital%20Pleiades-1.pdf:
  
  // psi_x = tan_psi_x[0] * (col - col_ref) + tan_psi_x[1]
  // psi_y = tan_psi_y[0] * (col - col_ref) + tan_psi_y[1]

  // where those coefficients are given in LINE_OF_SIGHT_TANPSIX
  // and LINE_OF_SIGHT_TANPSIY.

  // The doc says col_ref is 1, so for us it will be 0 since our
  // columns start form 0.

  double col     = pix[0];
  double psi_x   = m_tan_psi_x[0] * col + m_tan_psi_x[1];
  double psi_y   = m_tan_psi_y[0] * col + m_tan_psi_y[1];
  Vector3 result = Vector3(tan(psi_y), -tan(psi_x), 1.0);

  // Make the direction have unit length
  result = normalize(result);
  
  // Go from sensor coordinates to satellite body coordinates.
  // The instrument biases were not documented at all and took a while
  // to figure out.
  result = m_inverse_instrument_biases.rotate(result);

  return result;
}

boost::shared_ptr<PleiadesCameraModel> load_pleiades_camera_model_from_xml(std::string const& path){

  vw_out(vw::DebugMessage,"asp") << "Loading Pleiades camera file: " << path << std::endl;
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
  
  std::cout << "3 min time " << min_time << std::endl;
  std::cout << "--3 max time " << max_time << std::endl;
  
  bool correct_velocity_aberration = false;
  bool correct_atmospheric_refraction = false;
  
  // Create the model. This can throw an exception.
  boost::shared_ptr<PleiadesCameraModel> cam
    (new PleiadesCameraModel(position_func, velocity_func,
                             xml_reader.m_quat_offset_time,
                             xml_reader.m_quat_scale,
                             xml_reader.m_quaternion_coeffs,
                             time_func, 
                             xml_reader.m_tan_psi_x,
                             xml_reader.m_tan_psi_y,
                             xml_reader.m_instrument_biases,
                             xml_reader.m_image_size,
                             min_time, max_time,
                             correct_velocity_aberration,
                             correct_atmospheric_refraction));
  
  return cam;
} // End function load_pleiades_camera_model()

} // end namespace asp

