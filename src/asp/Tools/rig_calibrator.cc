// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/Core/AspLog.h>
#include <asp/Core/Macros.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/Nvm.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Rig/RigOptions.h>
#include <asp/Rig/RigParseOptions.h>
#include <asp/Rig/RigTypeDefs.h>
#include <asp/Rig/Triangulation.h>
#include <asp/Rig/RigThread.h>
#include <asp/Rig/RigParseUtils.h>
#include <asp/Rig/BasicAlgs.h>
#include <asp/Rig/RigUtils.h>
#include <asp/Rig/ImageLookup.h>
#include <asp/Rig/SystemUtils.h>
#include <asp/Rig/TransformUtils.h>
#include <asp/Rig/InterpolationUtils.h>
#include <asp/Rig/InterestPoint.h>
#include <asp/Rig/RigOutlier.h>
#include <asp/Rig/TextureProcessing.h>
#include <asp/Rig/CameraImage.h>
#include <asp/Rig/RigConfig.h>
#include <asp/Rig/RigCameraUtils.h>
#include <asp/Rig/RigData.h>
#include <asp/Rig/RigIo.h>
#include <asp/Rig/RigImageIO.h>
#include <asp/Rig/RigOptimizer.h>

#include <vw/FileIO/FileUtils.h>
#include <vw/Core/Log.h>
#include <vw/Cartography/GeoReference.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <glog/logging.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <oneapi/tbb/task_arena.h>
#include <boost/filesystem.hpp>

#include <string>
#include <map>
#include <iostream>
#include <fstream>

void run_rig_calibrator(int argc, char** argv) {

  // Some dependencies may still use google logging
  google::InitGoogleLogging(argv[0]);
  // Force linking to tbb for some dependencies
  tbb::task_arena schedule(tbb::task_arena::automatic);

  rig::RigOptions opt;
  rig::handleRigArgs(argc, argv, opt);
  rig::parameterValidation(opt);

  // Create the output directory, turn on logging
  std::string out_prefix = opt.out_prefix + "/run"; // part of the api
  vw::create_out_dir(out_prefix);
  asp::log_to_file(argc, argv, "", out_prefix);

  // Rig configuration. The rig transforms may not exist yet.
  rig::RigSet R;
  rig::readRigConfig(opt.rig_config, opt.use_initial_rig_transforms, R);

  // Parse auxiliary rig options that depend on R
  rig::parseAuxRigOptions(opt, R);

  // Sanity check
  size_t max_num_sensors_per_rig = 0;
  for (size_t rig_it = 0; rig_it < R.cam_set.size(); rig_it++)
    max_num_sensors_per_rig = std::max(max_num_sensors_per_rig, R.cam_set[rig_it].size());
  if (opt.extra_list != "" && opt.num_overlaps < max_num_sensors_per_rig)
    LOG(FATAL) << "If inserting extra images, must have --num_overlaps be at least "
                << "the number of sensors in the rig, and ideally more, to be able "
                << "to tie well the new images with the existing ones.\n";

  // Optionally load the mesh
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  if (opt.mesh != "")
    rig::loadMeshBuildTree(opt.mesh, mesh, mesh_info, graph, bvh_tree);

  // Read camera poses from nvm file or a list.
  std::vector<rig::MsgMap> image_maps;
  std::vector<rig::MsgMap> depth_maps;
  asp::nvmData nvm;
  rig::readListOrNvm(opt.camera_poses, opt.nvm,
                     opt.image_sensor_list, opt.extra_list,
                     opt.use_initial_rig_transforms,
                     opt.bracket_len, opt.nearest_neighbor_interp,
                     opt.read_nvm_no_shift, opt.num_overlaps, R,
                     nvm, image_maps, depth_maps); // out

  // Poses for all the cameras (extrinsics)
  rig::Extrinsics cams;
  // Keep here the images, timestamps, and bracketing information
  std::vector<rig::cameraImage> imgData;
  //  The range of R.ref_to_cam_timestamp_offsets[cam_type] before
  //  getting out of the bracket.
  std::vector<double> min_timestamp_offset, max_timestamp_offset;
  std::vector<double> ref_timestamps; // Timestamps for the ref cameras
  // Select the images to use. If the rig is used, keep non-ref images
  // only within the bracket.
  rig::lookupImages(// Inputs
                    opt.no_rig, opt.bracket_len,
                    opt.timestamp_offsets_max_change,
                    opt.bracket_single_image,
                    R, image_maps, depth_maps,
                    // Outputs
                    ref_timestamps, cams.world_to_ref,
                    imgData, cams.world_to_cam, min_timestamp_offset, max_timestamp_offset);
  // De-allocate data we no longer need
  image_maps = std::vector<rig::MsgMap>();
  depth_maps = std::vector<rig::MsgMap>();

  if (opt.no_rig && opt.use_initial_rig_transforms)
    LOG(FATAL) << "Cannot use initial rig transforms without a rig.\n";

  // If we can use the initial rig transform, compute and overwrite
  // cams.world_to_cam, the transforms from the world to each non-reference camera.
  // TODO(oalexan1): Test if this works with --no_rig. For now this combination
  // is not allowed.
  if (!opt.no_rig && opt.use_initial_rig_transforms)
    rig::calcWorldToCamWithRig(// Inputs
                               !opt.no_rig,
                               imgData, cams.world_to_ref, ref_timestamps,
                               R.ref_to_cam_trans,
                               R.ref_to_cam_timestamp_offsets,
                               // Output
                               cams.world_to_cam);

  // If desired, initalize the rig based on median of transforms for the sensors
  if (!opt.no_rig && !opt.use_initial_rig_transforms)
    rig::calc_rig_trans(imgData, cams.world_to_ref, cams.world_to_cam, ref_timestamps,
                        R); // out

  // Determine if a given camera type has any depth information
  int num_cam_types = R.cam_names.size();
  std::vector<bool> has_depth(num_cam_types, false);
  for (size_t cid = 0; cid < imgData.size(); cid++) {
    int cam_type = imgData[cid].camera_type;
    if (imgData[cid].depth_cloud.cols > 0 && imgData[cid].depth_cloud.rows > 0)
      has_depth[cam_type] = true;
  }

  // Transform to world coordinates if control points were provided. Note:
  // applyRegistration modifies cams.world_to_ref (Affine3d), which will then be
  // converted to state.world_to_ref_vec (primary storage) below.
  Eigen::Affine3d registration_trans;
  registration_trans.matrix() = Eigen::Matrix4d::Identity(); // default
  bool registration_applied = false;
  if (opt.registration && opt.hugin_file != "" && opt.xyz_file != "") {
    // Keep user's R.depth_to_image transforms, and only transform only the image
    // cameras from Theia's abstract coordinate system to world coordinates.
    bool scale_depth = false;
    registration_applied = true;
    applyRegistration(opt.no_rig, scale_depth, opt.hugin_file, opt.xyz_file,
                      has_depth, imgData,
                      // Outputs
                      registration_trans, cams.world_to_ref, cams.world_to_cam, R);
  }

  int num_ref_cams = cams.world_to_ref.size();
  if (cams.world_to_ref.size() != ref_timestamps.size())
    LOG(FATAL) << "Must have as many ref cam timestamps as ref cameras.\n";

  // Set up the variable blocks to optimize for BracketedDepthError
  int num_depth_params = rig::NUM_RIGID_PARAMS;
  if (opt.affine_depth_to_image)
    num_depth_params = rig::NUM_AFFINE_PARAMS;

  // Separate the initial scale. This is convenient if cam_depth_to_image is
  // scale * rotation + translation and if it is desired to keep the scale
  // fixed. In either case, the scale will be multiplied back when needed.
  // TODO(oalexan1): affine_depth_to_image must allow fixing the scale. 
  std::vector<double> depth_to_image_scales;
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    double depth_to_image_scale
      = pow(R.depth_to_image[cam_type].matrix().determinant(), 1.0 / 3.0);
    R.depth_to_image[cam_type].linear() /= depth_to_image_scale;
    depth_to_image_scales.push_back(depth_to_image_scale);
  }

  // Detect and match features if --num_overlaps > 0. Append the features
  // read from the nvm. Triangulate all points.
  rig::KeypointVec keypoint_vec;
  rig::PidCidFid pid_to_cid_fid;
  bool filter_matches_using_cams = true;
  std::vector<std::pair<int, int>> input_image_pairs; // will use num_overlaps instead
  // Do not save these matches. Only inlier matches will be saved later.
  bool local_save_matches = false;
  std::vector<Eigen::Vector3d> xyz_vec;
  rig::detectAddFeatures(// Inputs
                         imgData, R.cam_params, opt.out_prefix, local_save_matches,
                         filter_matches_using_cams, cams.world_to_cam,
                         opt.num_overlaps, input_image_pairs,
                         opt.initial_max_reprojection_error,
                         opt.num_match_threads,
                         opt.read_nvm_no_shift, opt.no_nvm_matches,
                         opt.verbose,
                         // Outputs
                         keypoint_vec, pid_to_cid_fid, xyz_vec, nvm);
  if (pid_to_cid_fid.empty())
    LOG(FATAL) << "No interest points were found. Must specify either "
               << "--nvm or positive --num_overlaps.\n";

  // Inlier flag. Once an inlier becomes an outlier, it stays that way
  rig::PidCidFidMap pid_cid_fid_inlier;

  // TODO(oalexan1): Must initialize all points as inliers outside this function,
  // as now this function resets those.
  rig::flagOutlierByExclusionDist(// Inputs
                                  R.cam_params, imgData, pid_to_cid_fid,
                                  keypoint_vec,
                                  // Outputs
                                  pid_cid_fid_inlier);

  // Ensure that the triangulated points are kept in sync with the cameras
  if (opt.use_initial_triangulated_points && registration_applied)
    rig::transformInlierTriPoints(registration_trans, pid_to_cid_fid,
                                  pid_cid_fid_inlier, xyz_vec);

  // Run several optimization passes with outlier filtering
  for (int pass = 0; pass < opt.num_passes; pass++) {
    std::cout << "\nOptimization pass " << pass + 1 << " / " << opt.num_passes << "\n";
    runOptPass(pass, num_depth_params, opt, imgData, ref_timestamps,
               keypoint_vec, pid_to_cid_fid,
               min_timestamp_offset, max_timestamp_offset, mesh, bvh_tree,
               depth_to_image_scales, cams, R, xyz_vec, pid_cid_fid_inlier); // out
  }  // End optimization passes

  // Put back the scale in R.depth_to_image
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    R.depth_to_image[cam_type].linear() *= depth_to_image_scales[cam_type];

  if (opt.save_matches)
    rig::saveInlierMatchPairs(imgData, opt.num_overlaps, pid_to_cid_fid,
                              keypoint_vec, pid_cid_fid_inlier, opt.out_prefix);

  // Redo the registration unless told not to.
  // Note: applyRegistration modifies cams.world_to_ref (Affine3d form), which was updated
  // from state.world_to_ref_vec after the optimization loop above.
  if (!opt.skip_post_registration && opt.registration &&
      opt.hugin_file != "" && opt.xyz_file != "") {
    // This time adjust the depth-to-image scale to be consistent with optimized cameras
    bool scale_depth = true;
    Eigen::Affine3d registration_trans;
    applyRegistration(opt.no_rig, scale_depth, opt.hugin_file, opt.xyz_file,
                      has_depth, imgData,
                      // Outputs
                      registration_trans, cams.world_to_ref, cams.world_to_cam, R);

    // Transform accordingly the triangulated points
    rig::transformInlierTriPoints(registration_trans, pid_to_cid_fid,
                                        pid_cid_fid_inlier, xyz_vec);
  }

  if (opt.out_texture_dir != "")
    rig::meshProjectCameras(R.cam_names, R.cam_params, imgData, cams.world_to_cam,
                            mesh, bvh_tree, opt.out_texture_dir);

  rig::saveCameraPoses(opt.out_prefix, imgData, cams.world_to_cam);

  bool model_rig = (!opt.no_rig);
  rig::writeRigConfig(opt.out_prefix + "/rig_config.txt", model_rig, R);

  std::string nvm_file = opt.out_prefix + "/cameras.nvm";
  bool shift_keypoints = true;
  rig::writeInliersToNvm(nvm_file, shift_keypoints, R.cam_params, imgData,
                         cams.world_to_cam, keypoint_vec,
                         pid_to_cid_fid, pid_cid_fid_inlier, xyz_vec);

  if (opt.save_nvm_no_shift) {
    std::string nvm_file = opt.out_prefix + "/cameras_no_shift.nvm";
    bool shift_keypoints = false;
    rig::writeInliersToNvm(nvm_file, shift_keypoints, R.cam_params, imgData,
                           cams.world_to_cam, keypoint_vec,
                           pid_to_cid_fid, pid_cid_fid_inlier, xyz_vec);
  }

  if (opt.export_to_voxblox)
    rig::exportToVoxblox(R.cam_names, imgData, R.depth_to_image,
                         cams.world_to_cam, opt.out_prefix);

  if (opt.save_transformed_depth_clouds)
    rig::saveTransformedDepthClouds(R.cam_names, imgData, R.depth_to_image,
                                    cams.world_to_cam, opt.out_prefix);

  // Save the list of images (useful for bundle_adjust)
  std::string image_list = opt.out_prefix + "/image_list.txt";
  rig::saveImageList(imgData, image_list);

  if (opt.save_pinhole_cameras)
    rig::writePinholeCameras(R.cam_names, R.cam_params, imgData,
                             cams.world_to_cam, opt.out_prefix);

  std::string conv_angles_file = opt.out_prefix + "/convergence_angles.txt";
  rig::savePairwiseConvergenceAngles(pid_to_cid_fid, keypoint_vec,
                                     imgData, cams.world_to_cam,
                                     xyz_vec,  pid_cid_fid_inlier,
                                     conv_angles_file);
  return;
}

int main(int argc, char * argv[]) {

  try {
    run_rig_calibrator(argc, argv);
  } ASP_STANDARD_CATCHES;

  return 0;

}
