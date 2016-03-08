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
#include <asp/Camera/LinescanSpotModel.h>

namespace asp {

using vw::Vector3;
using vw::Matrix3x3;


Vector3 SPOTCameraModel::get_local_pixel_vector(vw::Vector2 const& pix) const {
  
  // psi_x Is the angle from nadir line in along-track direction (lines)
  // psi_y Is the angle from nadir line in across-track direction (cols)
  
  // Interpolate the pixel angle from the adjacent values in the lookup table.
  // - Probably should have a simple 2D interp function somewhere.  
  double     col = pix[0];
  double min_col = floor(col);
  double max_col = ceil (col);
  size_t min_index = static_cast<size_t>(min_col);
  size_t max_index = static_cast<size_t>(max_col);
  
  double psi_x = (m_look_angles[min_index].second[0]*(col-min_col) + 
                  m_look_angles[max_index].second[0]*(max_col-col)  );
  double psi_y = (m_look_angles[min_index].second[1]*(col-min_col) + 
                  m_look_angles[max_index].second[1]*(max_col-col)  );
  
  // This vector is in the SPOT5 O1 Navigation Coordinate Sytem, which 
  // differs from how we usually set up our coordinates.
  return normalize(Vector3(-tan(psi_y), tan(psi_x), -1));
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
}

boost::shared_ptr<SPOTCameraModel> load_spot5_camera_model(std::string const& path)
{

  // XYZ coordinates are in the ITRF coordinate frame which means GCC coordinates.
  // - The velocities are in the same coordinate frame, not in some local frame.

  vw::vw_out() << "DEBUG - Loading SPOT5 camera file: " << path << std::endl;

  // Parse the SPOT5 XML file
  SpotXML xml_reader;
  xml_reader.read_xml(path);

  // Get all the initial functors
  vw::camera::LagrangianInterpolation              position_func  = xml_reader.setup_position_func();
  vw::camera::LagrangianInterpolation              velocity_func  = xml_reader.setup_velocity_func();
  vw::camera::LinearPiecewisePositionInterpolation spot_pose_func = xml_reader.setup_pose_func();
  vw::camera::LinearTimeInterpolation              time_func      = xml_reader.setup_time_func();
  
  // The SPOT5 camera uses a different pose convention than we do, so we create
  //  a new pose interpolation functor that will return the pose in an easy to use format.
  
  // Get some information about the pose data
  double min_time      = spot_pose_func.get_t0();
  double max_time      = spot_pose_func.get_tend();
  double time_delta    = spot_pose_func.get_dt();
  size_t num_pose_vals = static_cast<size_t>(round((max_time - min_time) / time_delta));
  
  // Make a new vector of pose values in the GCC coordinate frame.
  std::vector<vw::Quat> gcc_pose(num_pose_vals);
  Vector3 position, velocity, yaw_pitch_roll;
  Matrix3x3 lo_frame, look_rotation, combined_rotation;  
  for (size_t i=0; i<num_pose_vals; ++i) {
    // Get info at this time
    double time = min_time + time_delta * static_cast<double>(i);
    position       = position_func(time);
    velocity       = velocity_func(time);
    yaw_pitch_roll = spot_pose_func(time);
    
    // Get the two of rotation matrices we need
    lo_frame      = SPOTCameraModel::get_local_orbital_frame(position, velocity);
    look_rotation = SPOTCameraModel::get_look_rotation_matrix(yaw_pitch_roll[0], 
                                          yaw_pitch_roll[1], yaw_pitch_roll[2]);
    // By their powers combined thes form the GCC rotation we need.
    combined_rotation = look_rotation * lo_frame;
    gcc_pose[i] = vw::Quat(combined_rotation);
  }
  vw::camera::SLERPPoseInterpolation pose_func(gcc_pose, min_time, time_delta);

  // Feed everything into a new camera model.
  return boost::shared_ptr<SPOTCameraModel>(new SPOTCameraModel(position_func, velocity_func, 
                                                                pose_func, time_func, 
                                                                xml_reader.look_angles,
                                                                xml_reader.image_size));

} // End function load_spot_camera_model()


} // end namespace asp

