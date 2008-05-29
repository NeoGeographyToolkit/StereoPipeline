#include <boost/shared_ptr.hpp>

#include <vw/Camera/PinholeModel.h> 
#include <vw/Math/EulerAngles.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>

#include "apollo/StereoSessionApolloMetric.h"

#include "stereo.h"
#include "file_lib.h"
#include "SpiceUtilities.h"
#include "CameraAdjust.h"
#include "KML.h"

#include <list>

using namespace std;
using namespace vw;
using namespace vw::camera;


static void load_tie_points(std::string filename, std::vector<Vector3> &ground_pts, std::vector<Vector2> &image_pts, double subsample) {

  const double LUNAR_RADIUS = 1737400;

  std::ifstream input(filename.c_str());
  if (!(input.good())) {
    std::cout << "Could not open tie point file: " << filename << "\nExiting.\n\n";
    exit(1);
  }

  ground_pts.clear();
  image_pts.clear();

  while (!input.eof()) {
    float lon, lat, u, v;
    input >> lon >> lat >> u >> v;
    ground_pts.push_back(cartography::lon_lat_radius_to_xyz(Vector3(lon,lat,LUNAR_RADIUS)));
    image_pts.push_back(Vector2(u/subsample,v/subsample));
  }
}

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

void apollo_metric_intrinsics(double &f, double &cx, double &cy, double &pixels_per_mm, double subsample) {
  double focal_length;
  Vector2 ccd_center;
  spice::kernel_param("INS-915240_FOCAL_LENGTH", focal_length); // units: mm
  spice::kernel_param("INS-915240_K", pixels_per_mm);           // units: pixels/mm
  spice::kernel_param("INS-915240_CCD_CENTER", ccd_center);     // units: pixels

  f = focal_length * pixels_per_mm / subsample;
  cx = ccd_center[0]/subsample;
  cy = ccd_center[1]/subsample;
  pixels_per_mm /= subsample;
}


class MetricCameraLensDistortion : public LensDistortionBase<MetricCameraLensDistortion, PinholeModel> {
  Vector3 m_radial;
  Vector3 m_tangential;
  double m_pixels_per_mm;
  public:
  MetricCameraLensDistortion(Vector3 radial_params, Vector3 tangential_params, double pixels_per_mm) : 
    m_radial(radial_params),
    m_tangential(tangential_params),
    m_pixels_per_mm(pixels_per_mm) {}
  
  virtual ~MetricCameraLensDistortion() {}

  virtual Vector2 get_undistorted_coordinates(Vector2 const& v) const {
    vw_throw(NoImplErr() << "MetricCameraLensDistortion::get_undistorted_coordinates() not yet implemented.");
    return Vector2();
  }

  //  Location where the given pixel would have appeared if there
  //  were no lens distortion.
  virtual Vector2 get_distorted_coordinates(Vector2 const& p) const {
    //      vw_out(0)<< "Pix: " << p << "\n";
      double fu, fv, cu, cv;
      this->camera_model().intrinsic_parameters(fu, fv, cu, cv);
      
      double x = (p[0] - cu) / m_pixels_per_mm;
      double y = (p[1] - cv) / m_pixels_per_mm;
      
      double r2 = x * x + y * y;
      double r4 = r2 * r2;
      double r6 = r2 * r4;
      
      double a = (1 + m_radial[0]*r2 + m_radial[1]*r4 + m_radial[2]*r6);
      double xp = a * x - (m_tangential[0]*r2 + m_tangential[1]*r4)*sin(m_tangential[2]);
      double yp = a * y + (m_tangential[0]*r2 + m_tangential[1]*r4)*cos(m_tangential[2]);

      Vector2 result(xp * m_pixels_per_mm + cu,
                     yp * m_pixels_per_mm + cv);
      //      vw_out(0)<< "     " << result << "\n";
      return result;
    }
  
  virtual void write(std::ostream & os) const {
    os << "radial[0] = " << m_radial[0] << "\n";
    os << "radial[1] = " << m_radial[1] << "\n";
    os << "radial[2] = " << m_radial[2] << "\n";
    os << "tangential[0] = " << m_tangential[0] << "\n";
    os << "tangential[1] = " << m_tangential[1] << "\n";
    os << "tangential[2] = " << m_tangential[2] << "\n";
    os << "pixels per mm = " << m_pixels_per_mm << "\n";
  }

  friend std::ostream & operator<<(std::ostream & os, const MetricCameraLensDistortion mcld){
    mcld.write(os);
    return os;
  }
  
};


// Load the state of the MOC camera for a given time range, returning 
// observations of the state for the given time interval.
void apollo_metric_state(double time,
                         vw::Vector3 &position,
                         vw::Vector3 &velocity, 
                         vw::Quaternion<double> &pose) {  
  spice::body_state(time, position, velocity, pose,
                    "APOLLO 15", "IAU_MOON", "MOON", "A15_METRIC");
}

void bundle_adjust_pose(std::string tie_file1, std::string tie_file2,
                        boost::shared_ptr<camera::CameraModel> &cam1,
                        boost::shared_ptr<camera::CameraModel> &cam2,
                        double subsample = 1) {

    std::vector<Vector3> ground_pt1, ground_pt2;
    std::vector<Vector2> image_pt1, image_pt2;
    load_tie_points(tie_file1, ground_pt1, image_pt1, subsample);
    load_tie_points(tie_file2, ground_pt2, image_pt2, subsample);

    // Optimize pointing for camera 1
    Vector<double,4> seed(1.0,0,0,0);
    Vector<double,4> scale(0.1,0.1,0.1,0.1);
    CameraToGroundOptimizePoseFunc optimize_functor1(cam1, ground_pt1, image_pt1);
    int status;
    std::cout << "starting optization 1\n";
    Vector4 quaternion_adjustment = vw::math::nelder_mead(optimize_functor1, seed, scale, status, true, 3, 1e-8, 300);
    Vector4 v1 = normalize(quaternion_adjustment);
    std::cout << "result: " << v1 << "\n\n";
    Quaternion<double> q1(v1[0], v1[1], v1[2], v1[3]);
    TransformedCameraModel *bundle_adjusted_camera1 = new TransformedCameraModel(cam1);
    bundle_adjusted_camera1->set_rotation(q1);

    // Optimize pointing for camera 2
    seed = Vector4(1.0, 0, 0, 0);
    CameraToGroundOptimizePoseFunc optimize_functor2(cam2, ground_pt2, image_pt2);
    std::cout << "starting optization 1\n";
    quaternion_adjustment = vw::math::nelder_mead(optimize_functor2, seed, scale, status, true, 3, 1e-8, 300);
    v1 = normalize(quaternion_adjustment);
    std::cout << "result: " << v1 << "\n\n";
    Quaternion<double> q2(v1[0], v1[1], v1[2], v1[3]);
    TransformedCameraModel *bundle_adjusted_camera2 = new TransformedCameraModel(cam2);
    bundle_adjusted_camera2->set_rotation(q2);

    cam1 = boost::shared_ptr<camera::CameraModel>(bundle_adjusted_camera1);
    cam2 = boost::shared_ptr<camera::CameraModel>(bundle_adjusted_camera2);
}

void bundle_adjust_position(std::string tie_file1, std::string tie_file2,
                            boost::shared_ptr<camera::CameraModel> &cam1,
                            boost::shared_ptr<camera::CameraModel> &cam2,
                            double subsample = 1) {

    std::vector<Vector3> ground_pt1, ground_pt2;
    std::vector<Vector2> image_pt1, image_pt2;
    load_tie_points(tie_file1, ground_pt1, image_pt1, subsample);
    load_tie_points(tie_file2, ground_pt2, image_pt2, subsample);

    Vector3 seed(0,0,0); // start with zero translation offset
    Vector3 scale(100.0,100.0,100.0);
    int status;

    // Optimize pointing for camera 1
    CameraToGroundOptimizePositionFunc optimize_functor1(cam1, ground_pt1, image_pt1);
    std::cout << "starting optization 1 with starting position " << cam1->camera_center(Vector2(0,0)) << "\n";
    Vector3 position_adjustment = vw::math::nelder_mead(optimize_functor1, seed, scale, status, true, 3, 1e-8, 300);
    std::cout << "result: " << position_adjustment << "\n\n";
    TransformedCameraModel *bundle_adjusted_camera1 = new TransformedCameraModel(cam1);
    bundle_adjusted_camera1->set_translation(-position_adjustment);

    // Optimize pointing for camera 2
    CameraToGroundOptimizePositionFunc optimize_functor2(cam2, ground_pt2, image_pt2);
    std::cout << "starting optization 2 with starting position " << cam2->camera_center(Vector2(0,0)) << "\n";
    position_adjustment = vw::math::nelder_mead(optimize_functor2, seed, scale, status, true, 3, 1e-8, 300);
    std::cout << "result: " << position_adjustment << "\n\n";
    TransformedCameraModel *bundle_adjusted_camera2 = new TransformedCameraModel(cam2);
    bundle_adjusted_camera2->set_translation(-position_adjustment);

    cam1 = boost::shared_ptr<camera::CameraModel>(bundle_adjusted_camera1);
    cam2 = boost::shared_ptr<camera::CameraModel>(bundle_adjusted_camera2);
}


void bundle_adjust_position_and_pose(std::string tie_file1, std::string tie_file2,
                                     boost::shared_ptr<camera::CameraModel> &cam1,
                                     boost::shared_ptr<camera::CameraModel> &cam2,
                                     double subsample = 1) {

    std::vector<Vector3> ground_pt1, ground_pt2;
    std::vector<Vector2> image_pt1, image_pt2;
    load_tie_points(tie_file1, ground_pt1, image_pt1, subsample);
    load_tie_points(tie_file2, ground_pt2, image_pt2, subsample);

    // Optimize pointing for camera 1
    Vector<double,7> seed;
    seed(0) = 1.0;
    Vector<double,7> scale;
    scale(0) = 0.1;
    scale(1) = 0.1;
    scale(2) = 0.1;
    scale(3) = 0.1;

    scale(4) = 100.0;
    scale(5) = 100.0;
    scale(6) = 100.0;

    CameraToGroundOptimizeFunc optimize_functor1(cam1, ground_pt1, image_pt1);
    int status;
    std::cout << "starting optization 1\n";
    Vector<double,7> adjustment = vw::math::nelder_mead(optimize_functor1, seed, scale, status, true, 8, 1e-8, 800);
    Vector4 v1 = normalize(subvector(adjustment,0,4));
    Vector3 p1 = subvector(adjustment,4,3);
    std::cout << "result: " << v1 << "   " << p1 << "\n\n";
    Quaternion<double> q1(v1[0], v1[1], v1[2], v1[3]);
    TransformedCameraModel *bundle_adjusted_camera1 = new TransformedCameraModel(cam1);
    bundle_adjusted_camera1->set_rotation(q1);
    bundle_adjusted_camera1->set_translation(-p1);

    // Optimize pointing for camera 2
    CameraToGroundOptimizeFunc optimize_functor2(cam2, ground_pt2, image_pt2);
    std::cout << "starting optization 1\n";
    adjustment = vw::math::nelder_mead(optimize_functor2, seed, scale, status, true, 8, 1e-8, 800);
    v1 = normalize(subvector(adjustment,0,4));
    Vector3 p2 = subvector(adjustment,4,3);
    std::cout << "result: " << v1 << "   " << p2 << "\n\n";
    Quaternion<double> q2(v1[0], v1[1], v1[2], v1[3]);
    TransformedCameraModel *bundle_adjusted_camera2 = new TransformedCameraModel(cam2);
    bundle_adjusted_camera2->set_rotation(q2);
    bundle_adjusted_camera2->set_translation(-p2);

    cam1 = boost::shared_ptr<camera::CameraModel>(bundle_adjusted_camera1);
    cam2 = boost::shared_ptr<camera::CameraModel>(bundle_adjusted_camera2);
}

boost::shared_ptr<vw::camera::CameraModel> StereoSessionApolloMetric::camera_model(std::string image_file, 
                                                                                   std::string camera_file) {
  std::cout << "Loading kernels\n";
  load_apollo_metric_kernels();

  // Hard coded values for now...
//   std::string utc1 = "1971-07-30T02:20:24.529"; // AS15-M-0081
//   std::string utc2 = "1971-07-30T02:20:44.876"; // AS15-M-0082
//   std::string utc1 = "1971-07-30T02:26:31.865"; // AS15-M-0099
//   std::string utc2 = "1971-07-30T02:26:52.293"; // AS15-M-0100
  std::string utc = camera_file;
  double et = spice::utc_to_et(utc);

  // Intrinsics are shared by the two images since it's the same imager
  std::cout << "Computing intrinsics\n";
  double f, cx, cy, pixels_per_mm;
  double subsample = 1;
  if (m_extra_argument1 != "") 
    subsample = atoi(m_extra_argument1.c_str());

  apollo_metric_intrinsics(f, cx, cy, pixels_per_mm, subsample);
  std::cout << "\tf = " << f << "   cx = " << cx << "   cy = " << cy << "  pixels_per_mm = " << pixels_per_mm << "   subsample = " << subsample << "\n";

  Vector3 camera_center;
  Vector3 camera_velocity;
  Quaternion<double> camera_pose;

  // Set up lens distortion
  //
  // Note: These values are taken from the Apollo 16 camera
  // calibration document, so they probably aren't correct for the
  // AS15 camera.
  MetricCameraLensDistortion distortion_model(Vector3(0.13678194e-5, 0.53824020e-9, -0.52793282e-13),
                                              Vector3(0.12275363e-5, -0.24596243e-9, 1.8859721),
                                              pixels_per_mm);
                                              
  std::cout << "Initializing camera for UTC: " << utc << "\n";
  apollo_metric_state(et, camera_center, camera_velocity, camera_pose);
  PinholeModel pinhole_cam(camera_center, camera_pose.rotation_matrix(),
                           f, f, cx, cy,
                           Vector3(1,0,0),
                           Vector3(0,1,0),
                           Vector3(0,0,1),
                           NullLensDistortion());
                            //                            distortion_model);


  return boost::shared_ptr<CameraModel>(new PinholeModel(pinhole_cam));


  // Disabled for now... -mbroxton
  //
//   // If the user has supplied tie point files, we adjust the camera
//   // pointing information here using a nelder-mead optimizer.
//   if (m_extra_argument4 == "pose" && m_extra_argument2 != "" && m_extra_argument3 != "") {
//     std::cout << "Optimizing by adjusting pose...\n";
//     bundle_adjust_pose(m_extra_argument2, m_extra_argument3, cam1, cam2, subsample);
//   } else if (m_extra_argument4 == "position" && m_extra_argument2 != "" && m_extra_argument3 != "") {
//     std::cout << "Optimizing by adjusting position...\n";
//     bundle_adjust_position(m_extra_argument2, m_extra_argument3, cam1, cam2, subsample);
//   } else if (m_extra_argument4 == "both" && m_extra_argument2 != "" && m_extra_argument3 != "") {
//     std::cout << "Optimizing by adjusting position and pose...\n";
//     bundle_adjust_position_and_pose(m_extra_argument2, m_extra_argument3, cam1, cam2, subsample);
//   }

}

void StereoSessionApolloMetric::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {
  
  boost::shared_ptr<camera::CameraModel> left_camera, right_camera;
  this->camera_models(left_camera, right_camera);
  KMLStateVectorViz viz(m_out_prefix+"-OrbitViz.kml", m_out_prefix+"-OrbitViz");
  viz.append_body_state("Left Camera", left_camera->camera_center(Vector2(0,0)), left_camera->camera_pose(Vector2(0,0)));
  viz.append_body_state("Right Camera", right_camera->camera_center(Vector2(0,0)), right_camera->camera_pose(Vector2(0,0)));
  viz.close();
  write_orbital_reference_model(m_out_prefix + "-OrbitViz.vrml", *left_camera, *right_camera);
  StereoSessionKeypoint::pre_pointcloud_hook(input_file, output_file);
}


