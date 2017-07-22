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


#include <asp/Camera/SPOT_XML.h>
#include <vw/Camera/CameraSolve.h>
#include <asp/Camera/LinescanSpotModel.h>

namespace asp {

using vw::Vector3;
using vw::Matrix3x3;

// TODO: Port these changes to the base class

vw::Vector2 SPOTCameraModel::point_to_pixel(Vector3 const& point, double starty) const {

  // Use the generic solver to find the pixel 
  // - This method will be slower but works for more complicated geometries
  vw::camera::CameraGenericLMA model( this, point );
  int status;
  vw::Vector2 start = m_image_size / 2.0; // Use the center as the initial guess
  if (starty >= 0) // If the user provided a line number guess..
    start[1] = starty;

  // Solver constants
  const double ABS_TOL = 1e-16;
  const double REL_TOL = 1e-16;
  const int    MAX_ITERATIONS = 1e+5;
  const double MAX_ERROR = 0.01;

  Vector3 objective(0, 0, 0);
  vw::Vector2 solution = vw::math::levenberg_marquardt(model, start, objective, status,
                                               ABS_TOL, REL_TOL, MAX_ITERATIONS);
  // Check the error - If it is too high then the solver probably got stuck at the edge of the image.
  double  error = norm_2(model(solution));
  VW_ASSERT( (status > 0) && (error < MAX_ERROR),
	           vw::camera::PointToPixelErr() << "Unable to project point into LinescanSPOT model" );

  return solution;
}


void SPOTCameraModel::check_time(double time, std::string const& location) const {
  if ((time < m_min_time) || (time > m_max_time))
    vw::vw_throw(vw::ArgumentErr() << "SPOTCameraModel::"<<location
                 << ": Requested time "<<time<<" is out of bounds ("
                 << m_min_time << " <-> "<<m_max_time<<")\n");
}

vw::Vector3 SPOTCameraModel::get_camera_center_at_time(double time) const {
  check_time(time, "get_camera_center_at_time");
  return m_position_func(time);
}
vw::Vector3 SPOTCameraModel::get_camera_velocity_at_time(double time) const { 
  check_time(time, "get_camera_velocity_at_time");
  return m_velocity_func(time); 
}
vw::Quat SPOTCameraModel::get_camera_pose_at_time(double time) const {
  check_time(time, "get_camera_pose_at_time");
 return m_pose_func(time); 
}
double SPOTCameraModel::get_time_at_line(double line) const {
  if ((line < 0.0) || (static_cast<int>(line) >= m_image_size[1]))
    vw::vw_throw(vw::ArgumentErr() << "SPOTCameraModel::get_time_at_line"
                 << ": Requested line "<<line<<" is out of bounds (0"
                 << " <-> "<<m_image_size[1]<<")\n");
 return m_time_func(line); 
}



Vector3 SPOTCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {
  
  // psi_x Is the angle from nadir line in along-track direction (lines)
  // psi_y Is the angle from nadir line in across-track direction (cols)
  // psi_x is nearly constant.  psi_y starts negative and increases with column.
  
  // Interpolate the pixel angle from the adjacent values in the lookup table.
  // - Probably should have a simple 2D interp function somewhere.  
  double     col    = pix[0];
  double min_col    = floor(col);
  double max_col    = min_col + 1.0;
  size_t min_index  = static_cast<size_t>(min_col);
  size_t max_index  = static_cast<size_t>(max_col);
  double min_weight = max_col - col;
  double max_weight = col-min_col;
  
  // Check bounds
  if ((col < 0) || (col > static_cast<double>(m_look_angles.size())-1.0))
    vw::vw_throw(vw::ArgumentErr() << "SPOTCameraModel:::get_local_pixel_vector: Requested pixel " << col << " is out of bounds!\n");
  
  double psi_x = (m_look_angles[max_index].second[0]*max_weight + 
                  m_look_angles[min_index].second[0]*min_weight  );
  double psi_y = (m_look_angles[max_index].second[1]*max_weight + 
                  m_look_angles[min_index].second[1]*min_weight  );
  
  // This vector is in the SPOT5 O1 Navigation Coordinate Sytem, which 
  // differs from how we usually set up our coordinates.
  //Vector3 result = normalize(Vector3(-tan(psi_y), tan(psi_x), -1));
  
  // Convert the local vector so that it follows our usual conventions:
  //  Z down, Y flight direction, X increasing sample direction. 
  Vector3 result = normalize(Vector3(tan(psi_y), tan(psi_x), 1.0));
  //std::cout << "Pixel: " << pix << std::endl;
  //std::cout << "Col: " << col << ", min col: " << min_col << ", max col: " << max_col << std::endl;
  //std::cout << "min = " << m_look_angles[min_index].second << std::endl;
  //std::cout << "max = " << m_look_angles[max_index].second << std::endl;
  //std::cout << "Local pixel vector: " << result << std::endl;
  return result;
}

Matrix3x3 SPOTCameraModel::get_local_orbital_frame(Vector3 const& position, Vector3 const& velocity) {
  // These calculations are copied from the SPOT 123-4-58 Geometry Handbook (GAEL-P135-DOC-001)
  Vector3 Z2 = vw::math::normalize(position);
  Vector3 X2 = vw::math::normalize(vw::math::cross_prod(velocity, Z2));
  Vector3 Y2 = vw::math::cross_prod(Z2, X2);
  Matrix3x3 out;
  for (int r=0; r<3; ++r) {
    out(r,0) = X2[r];
    out(r,1) = Y2[r];
    out(r,2) = Z2[r];
  }
  return out;
}

Matrix3x3 SPOTCameraModel::get_look_rotation_matrix(double yaw, double pitch, double roll) {
/*
  // These calculations are copied from the SPOT 123-4-58 Geometry Handbook (GAEL-P135-DOC-001)
  Matrix3x3 Mp, Mr, My;
  Mp(0,0) = 1.0;         Mp(0,1) = 0.0;           Mp(0,2) = 0.0;
  Mp(1,0) = 0.0;         Mp(1,1) = cos(pitch);    Mp(1,2) = sin(pitch);
  Mp(2,0) = 0.0;         Mp(2,1) = -sin(pitch);   Mp(2,2) = cos(pitch);

  Mr(0,0) = cos(roll);   Mr(0,1) = 0.0;           Mr(0,2) = -sin(roll);
  Mr(1,0) = 0.0;         Mr(1,1) = 1.0;           Mr(1,2) = 0.0;
  Mr(2,0) = sin(roll);   Mr(2,1) = 0.0;           Mr(2,2) = cos(roll);
  
  My(0,0) = cos(yaw);    My(0,1) = -sin(yaw);     My(0,2) = 0.0;
  My(1,0) = sin(yaw);    My(1,1) = cos(yaw);      My(1,2) = 0.0;
  My(2,0) = 0.0;         My(2,1) = 0.0;           My(2,2) = 1.0; 

  Matrix3x3 out = Mp*Mr*My;
  return out;
*/  

  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);
  double cy = cos(yaw);
  double sy = sin(yaw);

  Matrix3x3 M;
  M(0,0) = (cr*cy);            M(0,1) = (-cr*sy);           M(0,2) = (-sr);
  M(1,0) = (cp*sy+sp*sr*cy);   M(1,1) = (cp*cy-sp*sr*sy);   M(1,2) = (sp*cr);
  M(2,0) = (-sp*sy+cp*sr*cy);  M(2,1) = (-sp*cy-cp*sr*sy);  M(2,2) = cp*cr; 
  return M;
}


/*
Notes on interpolation:

Line period = 7.5199705115e-04 = 0.000751997051

Paper recommends:
position = lagrangian interpolation --> The times happen to be spaced exactly 30 secs apart.
velocity = lagrangian interpolation
pose = linear interpolation --> The times are spaced ALMOST exactly 1.0000 seconds apart
time = Linear is only option.
*/


boost::shared_ptr<SPOTCameraModel> load_spot5_camera_model_from_xml(std::string const& path)
{

  // XYZ coordinates are in the ITRF coordinate frame which means GCC coordinates.
  // - The velocities are in the same coordinate frame, not in some local frame.

  vw_out(vw::DebugMessage,"asp") << "Loading SPOT5 camera file: " << path << std::endl;

  // Parse the SPOT5 XML file
  SpotXML xml_reader;
  xml_reader.read_xml(path);

  // Get all the initial functors
  vw::camera::LagrangianInterpolation position_func  = xml_reader.setup_position_func();
  vw::camera::LagrangianInterpolation velocity_func  = xml_reader.setup_velocity_func();
  vw::camera::LinearTimeInterpolation time_func      = xml_reader.setup_time_func();
  vw::camera::LinearPiecewisePositionInterpolation spot_pose_func = xml_reader.setup_pose_func(time_func);
  
  // The SPOT5 camera uses a different pose convention than we do, so we create
  //  a new pose interpolation functor that will return the pose in an easy to use format.
  
  // Get some information about the pose data
  double min_time      = spot_pose_func.get_t0();
  double max_time      = spot_pose_func.get_tend();
  double time_delta    = spot_pose_func.get_dt();
  size_t num_pose_vals = static_cast<size_t>(round((max_time - min_time) / time_delta));
  
  // This matrix rotates the axes of the SPOT5 model so that it is oriented with
  //  our standard linescanner coordinate frame.
  Matrix3x3 R;
  R(0,0) = -1.0; R(0,1) = 0.0; R(0,2) =  0.0;
  R(1,0) =  0.0; R(1,1) = 1.0; R(1,2) =  0.0;
  R(2,0) =  0.0; R(2,1) = 0.0; R(2,2) = -1.0;
  
  // Make a new vector of pose values in the GCC coordinate frame.
  // - This saves us from having to do all of the coordinate transforms
  //   each time a camera position is needed.
  std::vector<vw::Quat> gcc_pose(num_pose_vals);
  Vector3 position, velocity, yaw_pitch_roll;
  Matrix3x3 lo_frame, look_rotation, combined_rotation;  
  for (size_t i=0; i<num_pose_vals; ++i) {
    // Get info at this time
    double time = min_time + time_delta * static_cast<double>(i);
    position       = position_func(time);
    velocity       = velocity_func(time);
    yaw_pitch_roll = spot_pose_func(time);
    
    // TODO: There may be a small (~1 meter offset) between ITRF coordinates and WGS84 coordinates!
    
    // Get the two of rotation matrices we need
    lo_frame      = SPOTCameraModel::get_local_orbital_frame(position, velocity);
    look_rotation = SPOTCameraModel::get_look_rotation_matrix(yaw_pitch_roll[0], 
                                          yaw_pitch_roll[1], yaw_pitch_roll[2]);
    //look_rotation.set_identity(); // DEBUG assume perfect path following
    // By their powers combined these form the GCC rotation we need.
    combined_rotation = lo_frame * look_rotation*R;

    gcc_pose[i] = vw::Quat(combined_rotation);    
  }
  
  vw::camera::SLERPPoseInterpolation pose_func(gcc_pose, min_time, time_delta);

  // Feed everything into a new camera model.
  return boost::shared_ptr<SPOTCameraModel>(new SPOTCameraModel(position_func, velocity_func, 
                                                                pose_func, time_func, 
                                                                xml_reader.look_angles,
                                                                xml_reader.image_size,
                                                                min_time, max_time));

} // End function load_spot_camera_model()


} // end namespace asp

