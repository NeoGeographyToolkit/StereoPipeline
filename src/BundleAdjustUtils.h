#ifndef __BUNDLE_ADJUST_UTILS_H__
#define __BUNDLE_ADJUST_UTILS_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Camera/ControlNetwork.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>

void read_adjustments(std::string const& filename, vw::Vector3& position_correction, vw::Vector3& pose_correction);
void write_adjustments(std::string const& filename, vw::Vector3 const& position_correction, vw::Vector3 const& pose_correction);

void compute_stereo_residuals(std::vector<boost::shared_ptr<vw::camera::CameraModel> > const& camera_models,
                              vw::camera::ControlNetwork const& cnet);

void add_matched_points(vw::camera::ControlNetwork& cnet,
                        std::vector<vw::ip::InterestPoint> const& ip1,
                        std::vector<vw::ip::InterestPoint> const& ip2,
                        int camera_id1, int camera_id2,
                        std::vector<boost::shared_ptr<vw::camera::CameraModel> > const& camera_models);

int add_ground_control_points(vw::camera::ControlNetwork& cnet, std::string filename, int camera_id);

#endif // __BUNDLE_ADJUST_UTILS_H__
