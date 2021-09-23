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
#include <asp/Camera/PeruSatXML.h>
#include <asp/Camera/LinescanPeruSatModel.h>

namespace asp {

// While static variables are not thread-safe, this will be changed only once,
// the first time a PeruSatCameraModel is loaded, and model loading is a serial
// process. 
static bool perusat_correction_note_printed = false;

using vw::Vector3;
using vw::Matrix3x3;

// TODO: Port these changes to the base class

vw::Vector2 PeruSatCameraModel::point_to_pixel(Vector3 const& point, double starty) const {

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
             << "Unable to project point into LinescanPeruSat model" );

  return solution;
}

void PeruSatCameraModel::check_time(double time, std::string const& location) const {
  if ((time < m_min_time) || (time > m_max_time))
    vw::vw_throw(vw::ArgumentErr() << "PeruSatCameraModel::"<<location
                 << ": Requested time "<<time<<" is out of bounds ("
                 << m_min_time << " <-> "<<m_max_time<<")\n");
}

vw::Vector3 PeruSatCameraModel::get_camera_center_at_time(double time) const {
  check_time(time, "get_camera_center_at_time");
  return m_position_func(time);
}
vw::Vector3 PeruSatCameraModel::get_camera_velocity_at_time(double time) const { 
  check_time(time, "get_camera_velocity_at_time");
  return m_velocity_func(time); 
}
vw::Quat PeruSatCameraModel::get_camera_pose_at_time(double time) const {
  check_time(time, "get_camera_pose_at_time");
 return m_pose_func(time); 
}

double PeruSatCameraModel::get_time_at_line(double line) const {
  // Allow finding the time at any line, even negative ones.  Here a
  // simple slope-intercept formula is used rather than a table so one
  // cannot run out of bounds.
  return m_time_func(line); 
}


Vector3 PeruSatCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {

  // According to Modelo%20Orbital%20PeruSAT-1.pdf:
  
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

boost::shared_ptr<PeruSatCameraModel> load_perusat_camera_model_from_xml(std::string const& path){

  vw_out(vw::DebugMessage,"asp") << "Loading PeruSat camera file: " << path << std::endl;
  // Parse the PeruSat XML file
  PeruSatXML xml_reader;
  xml_reader.read_xml(path);

  // Get all the initial functors
  vw::camera::LinearTimeInterpolation
    time_func      = xml_reader.setup_time_func();
  vw::camera::LagrangianInterpolation
    position_func  = xml_reader.setup_position_func(time_func);
  vw::camera::LagrangianInterpolation
    velocity_func  = xml_reader.setup_velocity_func(time_func);
  vw::camera::SLERPPoseInterpolation
    pose_func      = xml_reader.setup_pose_func(time_func);

  // Find the range of times for which we can solve for position and pose
  double min_position_time = position_func.get_t0();
  double max_position_time = position_func.get_tend();
  double min_velocity_time = velocity_func.get_t0();
  double max_velocity_time = velocity_func.get_tend();
  double min_pose_time = pose_func.get_t0();
  double max_pose_time = pose_func.get_tend();
  double min_time = std::max(min_position_time, std::max(min_velocity_time, min_pose_time));
  double max_time = std::min(max_position_time, std::min(max_velocity_time, max_pose_time));

  // See note on this below
  bool correct_velocity_aberration = false;
  bool correct_atmospheric_refraction = false;
  
  // Create the model. This can throw an exception.
  boost::shared_ptr<PeruSatCameraModel> cam
    (new PeruSatCameraModel(position_func, velocity_func, 
                            pose_func, time_func, 
                            xml_reader.m_tan_psi_x,
                            xml_reader.m_tan_psi_y,
                            xml_reader.m_instrument_biases,
                            xml_reader.m_image_size,
                            min_time, max_time,
                            correct_velocity_aberration,
                            correct_atmospheric_refraction));

  // Print this note only if PeruSat model loading was successful, as
  // sometimes this camera loading function is invoked when querying
  // an unknown XML model and we may not end up using this session if
  // loading fails.
  if (!perusat_correction_note_printed) {
    vw::vw_out() << "Not using atmospheric and velocity aberration correction "
             << "with PeruSat cameras to maintain closer agreement with "
             << "the RPC approximation to this model.\n";
    perusat_correction_note_printed = true;
  }
  
  return cam;
} // End function load_perusat_camera_model()

} // end namespace asp

