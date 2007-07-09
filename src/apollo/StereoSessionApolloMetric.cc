#include <boost/shared_ptr.hpp>

#include <vw/Camera/PinholeModel.h> 
#include <vw/Math/EulerAngles.h>
#include <vw/FileIO.h>

#include "apollo/StereoSessionApolloMetric.h"

#include "stereo.h"
#include "file_lib.h"
#include "Spice.h"

#include <list>

using namespace std;
using namespace vw;
using namespace vw::camera;

static void load_apollo_metric_kernels() {
  //  Constants 
  const std::string spice_database = "./AS15_Kernels/";

  std::list<std::string> spice_kernels;
  spice_kernels.push_back( spice_database + "ap15.bc" );
  spice_kernels.push_back( spice_database + "ap15.bsp" );     
  spice_kernels.push_back( spice_database + "ap15.tsc" );
  spice_kernels.push_back( spice_database + "ap15_v02.tf" );
  spice_kernels.push_back( spice_database + "ap15m_v01.ti" );
  spice_kernels.push_back( spice_database + "de414.bsp" );
  spice_kernels.push_back( spice_database + "naif0008.tls" );
  spice_kernels.push_back( spice_database + "pck00008.tpc" );
  spice::load_kernels(spice_kernels);
}

void apollo_metric_intrinsics(double &f, double &cx, double &cy) {
  double focal_length;
  double pixels_per_mm;
  Vector2 ccd_center;
  spice::kernel_param("INS-915240_FOCAL_LENGTH", focal_length); // units: mm
  spice::kernel_param("INS-915240_K", pixels_per_mm);           // units: pixels/mm
  spice::kernel_param("INS-915240_CCD_CENTER", ccd_center);     // units: pixels
  f = focal_length * pixels_per_mm;
  cx = ccd_center[0];
  cy = ccd_center[1];
}

// Load the state of the MOC camera for a given time range, returning 
// observations of the state for the given time interval.
void apollo_metric_state(double time,
                         vw::Vector3 &position,
                         vw::Vector3 &velocity, 
                         vw::Quaternion<double> &pose) {  
  spice::body_state(time, position, velocity, pose,
                    "APOLLO 15", "IAU_MOON", "MOON", "A15_METRIC");
}

void StereoSessionApolloMetric::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                              boost::shared_ptr<camera::CameraModel> &cam2) {
  
  std::cout << "Loading kernels\n";
  load_apollo_metric_kernels();

  // Hard coded values for AS15-M-0081 and AS15-M-0082 for now
  std::string utc1 = "1971-07-30T02:20:24.529";
  std::string utc2 = "1971-07-30T02:20:44.876";

  std::cout << "Converting to et\n";
  double et1 = spice::utc_to_et(utc1);
  double et2 = spice::utc_to_et(utc2);
  std::cout << "\t" << et1 << "   " << et2 << "\n";

  // Intrinsics are shared by the two images since it's the same imager
  std::cout << "Computing intrinsics\n";
  double f, cx, cy;
  apollo_metric_intrinsics(f, cx, cy);

  // Scale the intrinsics by the actual size of the supplied apollo
  // image.  Sometimes we supply a supsampled image, and we would like
  // to adjust these parameters to match the reduced resolution if
  // needed.
  double width = cx * 2;
  vw::DiskImageView<PixelGray<float> > left_image(m_left_image_file);
  double scale = left_image.cols() / width;
  std::cout << "Scale factor: " << scale << "\n";
  f *= scale;
  cx *= scale;
  cy *= scale;

  std::cout << "\tf = " << f << "   cx = " << cx << "   cy = " << cy << "\n";

  Vector3 camera_center;
  Vector3 camera_velocity;
  Quaternion<double> camera_pose;

  std::cout << "Initializing camera 1\n";
  // Initialize camera 1
  apollo_metric_state(et1, camera_center, camera_velocity, camera_pose);
  PinholeModel* cam_ptr1 = new PinholeModel(camera_center, camera_pose.rotation_matrix(),
                                            f, f, cx, cy);


  std::cout << "Initializing camera 2\n";
  // Initialize camera 2
  apollo_metric_state(et2, camera_center, camera_velocity, camera_pose);
  PinholeModel* cam_ptr2 = new PinholeModel(camera_center, camera_pose.rotation_matrix(),
                                            f, f, cx, cy);

  cam1 = boost::shared_ptr<camera::CameraModel>(cam_ptr1);
  cam2 = boost::shared_ptr<camera::CameraModel>(cam_ptr2);
}

// void StereoSessionApolloMetric::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
//                                               boost::shared_ptr<camera::CameraModel> &cam2) {
//   PinholeModel* cam_ptr1 = new PinholeModel(m_left_camera_file);
//   PinholeModel* cam_ptr2 = new PinholeModel(m_right_camera_file);

//   Quaternion<double> pose = cam_ptr1->camera_pose();
//   cam_ptr1->set_camera_pose(pose*math::euler_to_quaternion(37.73*M_PI/180, 0, 0, "zxy"));
//   pose = cam_ptr2->camera_pose();
//   cam_ptr2->set_camera_pose(pose*math::euler_to_quaternion(37.73*M_PI/180, 0, 0, "zxy"));

//   cam1 = boost::shared_ptr<camera::CameraModel>(cam_ptr1);
//   cam2 = boost::shared_ptr<camera::CameraModel>(cam_ptr2);
// }


void StereoSessionApolloMetric::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {
  
  boost::shared_ptr<camera::CameraModel> left_camera, right_camera;
  this->camera_models(left_camera, right_camera);
  write_orbital_reference_model(m_out_prefix + "-OrbitViz.vrml", *left_camera, *right_camera);
  StereoSessionKeypoint::pre_pointcloud_hook(input_file, output_file);

}


