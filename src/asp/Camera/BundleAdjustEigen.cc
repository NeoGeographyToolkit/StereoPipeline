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

/// \file BundleAdjustEigen.cc
///

#include <asp/Camera/BundleAdjustEigen.h>
#include <asp/Core/Nvm.h>

#include <vw/Camera/PinholeModel.h>
namespace asp {

// Export the latest xyz values from param_storage to a vector of Vector3
void exportTriPoints(asp::BAParams                const& param_storage, 
                     std::vector<Eigen::Vector3d>      & tri_vec) {
  tri_vec.resize(param_storage.num_points());
  for (int i = 0; i < param_storage.num_points(); i++) {
    const double* point = param_storage.get_point_ptr(i);
    tri_vec[i] = Eigen::Vector3d(point[0], point[1], point[2]);
  }
}

// Calculate the optical offsets for each image. For pinhole cameras,
// read these from the camera model. For other cameras, use half of the
// image size.
void calcOpticalOffsets(std::vector<std::string>               const& image_files,
                        std::vector<vw::CamPtr>                const& cams,
                        std::map<std::string, Eigen::Vector2d>      & optical_offsets) {
  optical_offsets.clear();
  for (size_t i = 0; i < image_files.size(); i++) {
    vw::camera::PinholeModel * pinhole_model
       = dynamic_cast<vw::camera::PinholeModel*>
         (vw::camera::unadjusted_model(cams[i].get()));
    if (pinhole_model != NULL) {
      vw::Vector2 offset = pinhole_model->point_offset(); 
      optical_offsets[image_files[i]] = Eigen::Vector2d(offset[0], offset[1]);
    } else {
      vw::DiskImageView<float> img(image_files[i]);
      optical_offsets[image_files[i]] = Eigen::Vector2d(img.cols()/2.0, img.rows()/2.0);
    }
  }
}

// Given a pinhole camera model, find the world-to-camera transform
// TODO(oalexan1): Move this out
Eigen::Affine3d calcWorldToCam(vw::camera::PinholeModel const& pin) {

  // The rotation and translation matrices in the pinhole camera  
  vw::Matrix3x3 R = pin.get_rotation_matrix();
  vw::Vector3 T = pin.camera_center();
  
  // Eigen rotation
  Eigen::Matrix3d ER;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      ER(r, c) = R(r, c);
      
  // Eigen translation    
  Eigen::Vector3d ET;
  for (int r = 0; r < 3; r++)
    ET(r) = T(r);
  
  // Populate the rotation + translation matrix
  Eigen::Matrix4d E;
  E.setIdentity();
  E.block<3,3>(0,0) = ER;
  E.block<3,1>(0,3) = ET;

  // This is camera-to-world, so invert it
  Eigen::Affine3d world_to_cam;
  world_to_cam.matrix() = E.inverse();  
  
  return world_to_cam;
}

// Based on the current camera models, calculate the world-to-camera transforms.
// For pinhole cameras, fetch them directly from the camera models. For adjusted
// pinhole cameras, must combine the adjustments with the pose. For others,
// return the identity matrix.
void calcCameraPoses(std::vector<vw::CamPtr>      const& cams,
                     std::vector<Eigen::Affine3d>      & world_to_cam) {
  world_to_cam.resize(cams.size());
  
  for (size_t i = 0; i < cams.size(); i++) {
    vw::camera::PinholeModel * pinhole_model
       = dynamic_cast<vw::camera::PinholeModel*>(cams[i].get());
    if (pinhole_model != NULL) {
      world_to_cam[i] = calcWorldToCam(*pinhole_model);
      continue;
    }
    
    // Try adjusted pinhole
    vw::camera::AdjustedCameraModel * adj_cam = 
      dynamic_cast<vw::camera::AdjustedCameraModel*>(cams[i].get());
    vw::camera::PinholeModel const* pin 
      = dynamic_cast<vw::camera::PinholeModel const*>(adj_cam->unadjusted_model().get());
    if (pin == NULL) {
      // Use the identity matrix for non-pinhole cameras
      world_to_cam[i] = Eigen::Affine3d::Identity();
      continue;
    }
    // Apply the adjustment to the pinhole camera    
    vw::Matrix4x4 ecef_transform = adj_cam->ecef_transform();
    // Make a copy of the pinhole camera
    vw::camera::PinholeModel local_pin = *pin;
    // Apply the transform
    local_pin.apply_transform(ecef_transform);
    world_to_cam[i] = calcWorldToCam(local_pin);
  }
}

// Once bundle adjustment is done, export the outlier list, camera poses,
// triangulated points, and optical offsets. This is needed for saving 
// an NVM file.
void saveNvm(asp::BaBaseOptions                const& opt, 
             vw::ba::ControlNetwork            const& cnet,
             asp::BAParams                     const& param_storage,
             std::vector<Eigen::Affine3d>           & world_to_cam,
             std::map<std::string, Eigen::Vector2d> & optical_offsets) {

  std::set<int> outliers;
  updateOutliers(cnet, param_storage, outliers);
 
  std::vector<Eigen::Vector3d> tri_vec;
  exportTriPoints(param_storage, tri_vec);

  // Find latest poses (return the identity for non-pinhole cameras)
  std::vector<vw::CamPtr> optimized_cams;
  asp::calcOptimizedCameras(opt, param_storage, optimized_cams);
  calcCameraPoses(optimized_cams, world_to_cam);

  // Find the optical centers if not loaded from nvm
  if (optical_offsets.empty())
    calcOpticalOffsets(opt.image_files, optimized_cams, optical_offsets);
    
  // Write the nvm
  std::string nvm_file = opt.out_prefix + ".nvm"; 
  nvmData nvm;
  asp::cnetToNvm(cnet, optical_offsets, world_to_cam, nvm, tri_vec, outliers);
  asp::writeNvm(nvm, nvm_file);
}

} // end namespace asp

