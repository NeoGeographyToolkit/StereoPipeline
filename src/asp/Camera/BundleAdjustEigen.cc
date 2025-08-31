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
#include <asp/Core/EigenTransformUtils.h>
#include <asp/Camera/BundleAdjustOutliers.h>
#include <asp/Rig/nvm.h>

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

// Calculate the optical offsets for each image. For pinhole and csm frame
// cameras, read these from the camera model. For other cameras, use half of the
// image size.
void calcOpticalOffsets(std::vector<std::string>               const& image_files,
                        std::vector<vw::CamPtr>                const& cams,
                        std::map<std::string, Eigen::Vector2d>      & optical_offsets) {
  optical_offsets.clear();
  for (size_t i = 0; i < image_files.size(); i++) {
    
    vw::camera::PinholeModel * pinhole_model 
      = dynamic_cast<vw::camera::PinholeModel*>(vw::camera::unadjusted_model(cams[i].get()));
    if (pinhole_model != NULL) {
      vw::Vector2 offset = pinhole_model->point_offset(); 
      optical_offsets[image_files[i]] = Eigen::Vector2d(offset[0], offset[1]);
      continue;
    } 
    
    asp::CsmModel const* csm_model 
      = dynamic_cast<asp::CsmModel const*>(vw::camera::unadjusted_model(cams[i].get()));    
    if (csm_model != NULL) {
      // For CSM, for frame can do, for others cannot
      if (csm_model->isFrameCam()) {
        vw::Vector2 offset = csm_model->optical_center();
        optical_offsets[image_files[i]] = Eigen::Vector2d(offset[0], offset[1]);
        continue;
      }
    }
    
    // Fallback to half of the image size
    vw::DiskImageView<float> img(image_files[i]);
    optical_offsets[image_files[i]] = Eigen::Vector2d(img.cols()/2.0, img.rows()/2.0);
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

// Given a csm frame camera model, find the world-to-camera transform.
Eigen::Affine3d calcWorldToCam(asp::CsmModel const& csm) {

  // Initialize world-to-camera as the identity matrix
  Eigen::Matrix4d E;
  E.setIdentity();
  Eigen::Affine3d world_to_cam;
  world_to_cam.matrix() = E.inverse(); // for consistency with what is below

  // Find the camera center
  double x = 0, y = 0, z = 0;
  double qx = 0, qy = 0, qz = 0, qw = 0;
  try {
    csm.frame_position(x, y, z);
    csm.frame_quaternion(qx, qy, qz, qw);
  } catch (const std::exception & e) {
    // Not a frame camera, return the identity matrix
    return world_to_cam;
  }

  E = calcTransform(x, y, z, qx, qy, qz, qw).matrix();
  
  // This is camera-to-world, so invert it
  world_to_cam.matrix() = E.inverse();  
  
  return world_to_cam;
}

// Based on the current camera models, calculate the world-to-camera transforms.
// For pinhole cameras, fetch them directly from the camera models. For adjusted
// pinhole cameras, must combine the adjustments with the pose. For others,
// return the identity matrix.
void calcCameraPoses(bool                                no_poses_from_nvm,
                     std::vector<vw::CamPtr>      const& cams,
                     std::vector<Eigen::Affine3d>      & world_to_cam) {
  world_to_cam.resize(cams.size());
  
  for (size_t i = 0; i < cams.size(); i++) {
    
    if (no_poses_from_nvm) {
      // Use the identity matrix for non-pinhole when told to do so
      world_to_cam[i] = Eigen::Affine3d::Identity();
      continue;
    }
    
    // Try pinhole
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
    if (pin != NULL && adj_cam != NULL) {
      // Apply the adjustment to the pinhole camera    
      vw::Matrix4x4 ecef_transform = adj_cam->ecef_transform();
      // Make a copy of the pinhole camera
      vw::camera::PinholeModel local_pin = *pin;
      // Apply the transform
      local_pin.apply_transform(ecef_transform);
      world_to_cam[i] = calcWorldToCam(local_pin);
      continue;
    }
    
    // Try csm, with and without adjustment
    asp::CsmModel const* csm_model 
      = dynamic_cast<asp::CsmModel const*>(adj_cam->unadjusted_model().get());
    if (csm_model != NULL && csm_model->isFrameCam()) {
      
      vw::Matrix4x4 ecef_transform = vw::math::identity_matrix<4>();
      if (adj_cam != NULL) 
        ecef_transform = adj_cam->ecef_transform();
        
      asp::CsmModel local_csm;
      csm_model->deep_copy(local_csm);
      local_csm.applyTransform(ecef_transform); 
      world_to_cam[i] = calcWorldToCam(local_csm);
      continue;
    }
    
    // Use the identity matrix if no luck
    world_to_cam[i] = Eigen::Affine3d::Identity();
    continue;  
  }
  
}

// Once bundle adjustment is done, export the outlier list, camera poses,
// triangulated points, and optical offsets. This is needed for saving 
// an NVM file.
void saveNvm(asp::BaBaseOptions                const& opt, 
             bool                                     no_poses_from_nvm,
             vw::ba::ControlNetwork            const& cnet,
             asp::BAParams                     const& param_storage,
             std::vector<Eigen::Affine3d>           & world_to_cam,
             std::map<std::string, Eigen::Vector2d> & optical_offsets) {

  std::set<int> outliers;
  asp::updateOutliers(cnet, param_storage, outliers);
 
  std::vector<Eigen::Vector3d> tri_vec;
  exportTriPoints(param_storage, tri_vec);

  // Find latest poses (return the identity for non-pinhole cameras)
  std::vector<vw::CamPtr> optimized_cams;
  asp::calcOptimizedCameras(opt, param_storage, optimized_cams);
  calcCameraPoses(no_poses_from_nvm, optimized_cams, world_to_cam);

  // Find the optical centers if not loaded from nvm
  if (optical_offsets.empty())
    calcOpticalOffsets(opt.image_files, optimized_cams, optical_offsets);
    
  // Write the nvm
  std::string nvm_file = opt.out_prefix + ".nvm"; 
  rig::nvmData nvm;
  rig::cnetToNvm(cnet, optical_offsets, world_to_cam, nvm, tri_vec, outliers);
  rig::writeNvm(nvm, nvm_file);
}

// Given pinhole cameras and camera-to-world transforms, update the camera poses
// in the pinhole cameras.
void updateCameraPoses(std::vector<Eigen::Affine3d> const& world_to_cam,
                       std::vector<vw::CamPtr>           & cams) {

  // Must have as many cameras as transforms
  if (cams.size() != world_to_cam.size())
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many cameras as transforms.\n");
    
  // Iterate over the cameras
  for (size_t i = 0; i < cams.size(); i++) {
    
    // Find the camera-to-world transform as a 4x4 matrix. Must invert the world-to-camera.
    Eigen::Matrix4d E = world_to_cam[i].matrix().inverse();
    
    // Find the rotation as the upper-left 3x3 matrix as Matrix3x3
    vw::Matrix3x3 R;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        R(r, c) = E(r, c);
    
    // Find the translation as the right-most column as Vector3
    vw::Vector3 T;
    for (int r = 0; r < 3; r++)
      T(r) = E(r, 3);
      
    vw::camera::PinholeModel * pin
       = dynamic_cast<vw::camera::PinholeModel*>(cams[i].get());
    if (pin != NULL) {
      pin->set_camera_pose(R);
      pin->set_camera_center(T);
      continue;
    }
    
    // Try adjusted pinhole camera
    vw::camera::AdjustedCameraModel * adj_cam = 
      dynamic_cast<vw::camera::AdjustedCameraModel*>(cams[i].get());
    
    // If null, we are doing something wrong
    if (adj_cam != NULL) {
      // We expect the adjustment to be the identity, as this function is called
      // before any adjustment is applied.
      vw::Matrix4x4 ecef_transform = adj_cam->ecef_transform();
      if (ecef_transform != vw::math::identity_matrix(4))
        vw::vw_throw(vw::ArgumentErr() << "Expecting the adjustment to be the identity.\n");
      
      // Get the unadjusted pinhole camera
      pin = dynamic_cast<vw::camera::PinholeModel*>(adj_cam->unadjusted_model().get());
      if (pin != NULL) {
        pin->set_camera_pose(R);
        pin->set_camera_center(T);
        continue;
      }
    }

    // Try CSM frame
    asp::CsmModel* csm_model = dynamic_cast<asp::CsmModel*>(cams[i].get());
    if (csm_model != NULL && csm_model->isFrameCam()) {
      // For CSM, for frame can do, for others cannot
      csm_model->set_frame_position(T[0], T[1], T[2]);
      // Convert R to Eigen
      Eigen::Matrix3d ER;
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
          ER(r, c) = R(r, c);
      Eigen::Quaterniond q(ER);
      csm_model->set_frame_quaternion(q.x(), q.y(), q.z(), q.w());
      continue;
    }
    
    if (i == 0)
     vw::vw_out() << "Ignoring the camera poses in the NVM file.\n";
  }
    
}  

} // end namespace asp

