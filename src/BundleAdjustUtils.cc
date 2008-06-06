#include "BundleAdjustUtils.h"

#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ip;

void read_adjustments(std::string const& filename, Vector3& position_correction, Vector3& pose_correction) {
  std::ifstream istr(filename.c_str());
  istr >> position_correction[0] >> position_correction[1] >> position_correction[2];
  istr >> pose_correction[0] >> pose_correction[1] >> pose_correction[2];
}

void write_adjustments(std::string const& filename, Vector3 const& position_correction, Vector3 const& pose_correction) {
  std::ofstream ostr(filename.c_str());
  ostr << position_correction[0] << " " << position_correction[1] << " " << position_correction[2] << "\n";
  ostr << pose_correction[0] << " " << pose_correction[1] << " " << pose_correction[2] << " " << "\n";
}

void compute_stereo_residuals(std::vector<boost::shared_ptr<CameraModel> > const& camera_models,
                              ControlNetwork const& cnet) {

  // Compute pre-adjustment residuals and convert to bundles
  int n = 0;
  double error_sum = 0; 
  double min_error = ScalarTypeLimits<double>::highest();
  double max_error = ScalarTypeLimits<double>::lowest();
  for (unsigned i = 0; i < cnet.size(); ++i) {
    for (unsigned j = 0; j+1 < cnet[i].size(); ++j) {
      ++n;
      int cam1 = cnet[i][j].image_id();
      int cam2 = cnet[i][j+1].image_id();
      Vector2 pix1 = cnet[i][j].position();
      Vector2 pix2 = cnet[i][j+1].position();

      StereoModel sm(*(camera_models[cam1]), *(camera_models[cam2]));
      double error;
      Vector3 pos = sm(pix1,pix2,error);
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
  }  
  std::cout << "\nStereo Intersection Residuals -- Min: " << min_error << "  Max: " << max_error << "  Average: " << (error_sum/n) << "\n";
}

void add_matched_points(ControlNetwork& cnet,
                        std::vector<InterestPoint> const& ip1,
                        std::vector<InterestPoint> const& ip2,
                        int camera_id1, int camera_id2,
                        std::vector<boost::shared_ptr<CameraModel> > const& camera_models) {

  for (unsigned i=0; i < ip1.size(); ++i) {
    ControlMeasure m1(ip1[i].x, ip1[i].y, ip1[i].scale, ip1[i].scale, camera_id1);
    ControlMeasure m2(ip2[i].x, ip2[i].y, ip2[i].scale, ip2[i].scale, camera_id2);
    
    unsigned pos1 = cnet.find_measure(m1);
    unsigned pos2 = cnet.find_measure(m2);
    
    if ( pos1 != cnet.size() && pos2 == cnet.size() ) {        // Contains m1 aready
      cnet[pos1].add_measure(m2);
    } else if ( pos1 == cnet.size() && pos2 != cnet.size() ) { // Contains m2 aready
      cnet[pos2].add_measure(m1);
    } else if ( pos1 == cnet.size() && pos2 == cnet.size() ) { // Contains neither
      // ... create a stereo model for this image pair...
      StereoModel sm(*(camera_models[camera_id1]), *(camera_models[camera_id2]));
      ControlPoint cpoint;
      double error;
      cpoint.set_position(sm(m1.position(), m2.position(), error));
      cpoint.set_sigma(error,error,error);
      cpoint.add_measure(m1);
      cpoint.add_measure(m2);
      cnet.add_control_point(cpoint);
    } else { 
      std::cout << "Houston, we have a control point loop!\n";
    }
  }
}


int add_ground_control_points(vw::camera::ControlNetwork& cnet,
                              std::string filename, int camera_id) {

  std::ifstream istr(filename.c_str());
  int count = 0;
  while (!istr.eof()) {
    Vector2 pix;
    Vector3 loc;
    Vector3 sigma;

    istr >> pix[0] >> pix[1] >> loc[0] >> loc[1] >> loc[2] >> sigma[0] >> sigma[1] >> sigma[2];
    ControlMeasure m(pix[0], pix[1], 1.0, 1.0, camera_id);
    ControlPoint cpoint;
    cpoint.set_position(loc[0],loc[1],loc[2]);
    cpoint.set_sigma(sigma[0],sigma[1],sigma[2]);
    cpoint.add_measure(m);
    cnet.add_control_point(cpoint);
    ++count;
  }
  istr.close();
  return count;
}
