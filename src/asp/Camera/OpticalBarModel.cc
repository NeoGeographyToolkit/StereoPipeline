// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#include <vw/Camera/CameraModel.h>
#include <vw/Camera/CameraSolve.h>
#include <asp/Camera/OpticalBarModel.h>

#include <iomanip>
#include <boost/filesystem/convenience.hpp>


namespace asp {
namespace camera {

using namespace vw;
using namespace vw::camera;


Vector2 OpticalBarModel::pixel_to_sensor_plane(Vector2 const& pixel) const {
  return (pixel - m_center_offset_pixels) * m_pixel_size;
}

double OpticalBarModel::pixel_to_time_delta(Vector2 const& pix) const {

  // This is the amount of time required for one complete image scan.
  const double scan_time = m_scan_angle_radians / m_scan_rate_radians;

  // Since the camera sweeps a scan through columns, use that to
  //  determine the fraction of the way it is through the image.
  double scan_fraction = pix[0] / m_image_size[0]; // TODO: Add 0.5 pixels?
  double time_delta    = scan_fraction * scan_time;
  return time_delta;
}

Vector3 OpticalBarModel::get_velocity(vw::Vector2 const& pixel) const {
  
  // TODO: For speed, store the pose*velocity vector.
  // Convert the velocity from sensor coords to GCC coords
  Matrix3x3 pose = transpose(camera_pose(pixel).rotation_matrix());

  return pose*Vector3(0,m_velocity,0);
}

Vector3 OpticalBarModel::camera_center(Vector2 const& pix) const {
  // We model with a constant velocity.
  double dt = pixel_to_time_delta(pix);

  return m_initial_position + dt*get_velocity(pix);
}


Quat OpticalBarModel::camera_pose(Vector2 const& pix) const {
  // Camera pose is treated as constant for the duration of a scan.
  return axis_angle_to_quaternion(m_initial_orientation);
}

Vector3 OpticalBarModel::pixel_to_vector_uncorrected(Vector2 const& pixel) const {
 
  Vector2 sensor_plane_pos = pixel_to_sensor_plane(pixel);
  Vector3 cam_center       = camera_center(pixel);
  Quat    cam_pose         = camera_pose  (pixel);

  // This is the horizontal angle away from the center point (from straight out of the camera)
  double alpha = sensor_plane_pos[0] / m_focal_length;

  // Distance from the camera center to the ground.
  double H = norm_2(cam_center) - m_mean_surface_elevation;

  // TODO: Make this optional!
  // Distortion caused by compensation for the satellite's forward motion during the image.
  // - The film was actually translated underneath the lens to compensate for the motion.
  double image_motion_compensation = ((-m_focal_length * m_velocity) / (H*m_scan_rate_radians))
                                     * sin(alpha);
  
  Matrix3x3 M_inv = transpose(cam_pose.rotation_matrix());
  
  Vector3 r(m_focal_length * sin(alpha),
            sensor_plane_pos[1] + image_motion_compensation,
            -m_focal_length * cos(alpha));
  
  Vector3 result = M_inv * r; // == scale(gcc_point - cam_center)
  
  return result;
}

Vector3 OpticalBarModel::pixel_to_vector(Vector2 const& pixel) const {
  try {
    Vector3 output_vector = pixel_to_vector_uncorrected(pixel);

    Vector3 cam_ctr = camera_center(pixel);
    if (!m_correct_atmospheric_refraction) 
      output_vector = apply_atmospheric_refraction_correction(cam_ctr, m_mean_earth_radius,
                                                              m_mean_surface_elevation, output_vector);

    if (!m_correct_velocity_aberration) 
      return output_vector;
    else
      return apply_velocity_aberration_correction(cam_ctr, get_velocity(pixel),
                                                  m_mean_earth_radius, output_vector);

  } catch(const vw::Exception &e) {
    // Repackage any of our exceptions thrown below this point as a 
    //  pixel to ray exception that other code will be able to handle.
    vw_throw(vw::camera::PixelToRayErr() << e.what());
  }
}

Vector2 OpticalBarModel::point_to_pixel(Vector3 const& point) const {

  // Use the generic solver to find the pixel 
  // - This method will be slower but works for more complicated geometries
  CameraGenericLMA model( this, point );
  int status;
  Vector2 start = m_image_size / 2.0; // Use the center as the initial guess

  // Solver constants
  const double ABS_TOL = 1e-16;
  const double REL_TOL = 1e-16;
  const int    MAX_ITERATIONS = 1e+5;

  Vector3 objective(0, 0, 0);
  Vector2 solution = math::levenberg_marquardt(model, start, objective, status,
                                               ABS_TOL, REL_TOL, MAX_ITERATIONS);
  VW_ASSERT( status > 0,
             camera::PointToPixelErr() << "Unable to project point into Linescan model" );

  return solution;
}

void OpticalBarModel::read(std::string const& filename) {

  // TODO: Make compatible with .tsai!
  
  // Open the input file
  std::ifstream cam_file;
  cam_file.open(filename.c_str());
  if (cam_file.fail())
    vw_throw( IOErr() << "OpticalBarModel::read_file: Could not open file: " << filename );

  // Check for version number on the first line
  std::string line;
  std::getline(cam_file, line);
  if (line.find("VERSION") == std::string::npos) {
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Version missing!\n" );
  }

  int file_version = 1;
  sscanf(line.c_str(),"VERSION_%d", &file_version); // Parse the version of the input file

  // Right now there is only one version (VERSION_3) so if we find the version
  //  we just skip it and move on to the next line.  If the version is changed,
  //  handler logic needs to be implemented here.
  std::getline(cam_file, line);


  // Start parsing all the parameters from the lines.
  if (!cam_file.good() || sscanf(line.c_str(),"image_size = %d %d",
      &m_image_size[0], &m_image_size[1]) != 2) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the image size\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"image_center = %lf %lf",
      &m_center_offset_pixels[0], &m_center_offset_pixels[1]) != 2) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the image center\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"pitch = %lf", &m_pixel_size) != 1) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the pixel pitch\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"f = %lf", &m_focal_length) != 1) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the focal_length\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"scan_angle = %lf", &m_scan_angle_radians) != 1) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the scan angle\n" );
  }
  
  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"scan_rate = %lf", &m_scan_rate_radians) != 1) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the scan rate\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"iC = %lf %lf %lf", 
        &m_initial_position(0), &m_initial_position(1), &m_initial_position(2)) != 3) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the initial position\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"iR = %lf %lf %lf", 
        &m_initial_orientation(0), &m_initial_orientation(1), &m_initial_orientation(2)) != 3) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the initial orientation\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"velocity = %lf", &m_velocity) != 1) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the velocity\n" );
  }

  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"mean_earth_radius = %lf", &m_mean_earth_radius) != 1) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the mean earth radius\n" );
  }
  
  std::getline(cam_file, line);
  if (!cam_file.good() || sscanf(line.c_str(),"mean_surface_elevation = %lf", &m_mean_surface_elevation) != 1) {
    cam_file.close();
    vw_throw( IOErr() << "OpticalBarModel::read_file(): Could not read the mean surface elevation\n" );
  }
  
  cam_file.close();
}


void OpticalBarModel::write(std::string const& filename) const {

  // TODO: Make compatible with .tsai files!

  // Set the path an open the output file for writing
  std::string file_path = boost::filesystem::path(filename).replace_extension(".opb").string();
  std::ofstream cam_file(file_path.c_str());
  if( !cam_file.is_open() ) 
    vw_throw( IOErr() << "OpticalBarModel::write: Could not open file: " << file_path );

  std::string VERSION = "VERSION_1";
  
  // Write the pinhole camera model parts
  //   # digits to survive double->text->double conversion
  const size_t ACCURATE_DIGITS = 17; // = std::numeric_limits<double>::max_digits10
  cam_file << std::setprecision(ACCURATE_DIGITS); 
  cam_file << VERSION << "\n";

  cam_file << "image_size = "   << m_image_size[0] << " " 
                                << m_image_size[1]<< "\n";
  cam_file << "image_center = " << m_center_offset_pixels[0] << " "
                                << m_center_offset_pixels[1] << "\n";
  cam_file << "pitch = "        << m_pixel_size             << "\n";
  cam_file << "f = "            << m_focal_length           << "\n";
  cam_file << "scan_angle = "   << m_scan_angle_radians     << "\n";
  cam_file << "scan_rate = "    << m_scan_rate_radians      << "\n";
  cam_file << "iC = " << m_initial_position[0] << " "
                      << m_initial_position[1] << " "
                      << m_initial_position[2] << "\n";
  cam_file << "iR = " << m_initial_orientation[0] << " "
                      << m_initial_orientation[1] << " "
                      << m_initial_orientation[2] << "\n";
  cam_file << "velocity = " << m_velocity << "\n";
  cam_file << "mean_earth_radius = "      << m_mean_earth_radius      << "\n";
  cam_file << "mean_surface_elevation = " << m_mean_surface_elevation << "\n";

  cam_file.close();
}



std::ostream& operator<<( std::ostream& os, OpticalBarModel const& camera_model) {
  os << "\n------------------------ Optical Bar Model -----------------------\n\n";
  os << " Image size :            " << camera_model.m_image_size             << "\n";
  os << " Center offset (pixels): " << camera_model.m_center_offset_pixels   << "\n";
  os << " Pixel size (m) :        " << camera_model.m_pixel_size             << "\n";
  os << " Focal length (m) :      " << camera_model.m_focal_length           << "\n";
  os << " Scan angle (rad):       " << camera_model.m_scan_angle_radians     << "\n";
  os << " Scan rate (rad/s):      " << camera_model.m_scan_rate_radians      << "\n";
  os << " Initial position:       " << camera_model.m_initial_position       << "\n";
  os << " Initial pose:           " << camera_model.m_initial_orientation    << "\n";
  os << " Velocity:               " << camera_model.m_velocity               << "\n";
  os << " Mean earth radius:      " << camera_model.m_mean_earth_radius      << "\n";
  os << " Mean surface elevation: " << camera_model.m_mean_surface_elevation << "\n";

  os << "\n------------------------------------------------------------------------\n\n";
  return os;
}


}} // namespace asp::camera

