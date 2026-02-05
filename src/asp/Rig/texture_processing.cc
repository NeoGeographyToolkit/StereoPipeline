/* Copyright (c) 2021-2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <asp/Rig/texture_processing.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/basic_algs.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Eigen includes
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <glog/logging.h>

// Boost includes
#include <boost/filesystem.hpp>

// texrecon includes
#include <mve/image_io.h>
#include <mve/image_tools.h>
#include <tex/defines.h>
#include <tex/obj_model.h>
#include <util/file_system.h>

// System includes
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <utility>
#include <set>

// TODO(oalexan1): Consider applying TILE_PADDING not to at the atlas
// forming stage, when the patch is padded, but earlier, right when
// the patch is created from a mesh triangle. That will ensure that
// while the same overall buffer is used, the padding region will have
// actual image samples, which may improve the rendering in meshlab
// when one zooms out, where now for some reason the triangle edges
// show up.  Can test the output with the test_texture_gen tool.

namespace rig {

void loadMeshBuildTree(std::string const& mesh_file, mve::TriangleMesh::Ptr& mesh,
                       std::shared_ptr<mve::MeshInfo>& mesh_info,
                       std::shared_ptr<tex::Graph>& graph,
                       std::shared_ptr<BVHTree>& bvh_tree) {
  std::cout << "Loading mesh: " << mesh_file << std::endl;
  mesh = mve::geom::load_ply_mesh(mesh_file);

  mesh_info = std::shared_ptr<mve::MeshInfo>(new mve::MeshInfo(mesh));
  tex::prepare_mesh(mesh_info.get(), mesh);

  std::size_t const num_faces = mesh->get_faces().size() / 3;
  if (num_faces > std::numeric_limits<std::uint32_t>::max())
    throw std::runtime_error("Exeeded maximal number of faces");

  // TODO(oalexan1): Is this necessary?
  graph = std::shared_ptr<tex::Graph>(new tex::Graph(num_faces));
  tex::build_adjacency_graph(mesh, *mesh_info.get(), graph.get());

  util::WallTimer timer;
  std::vector<unsigned int> const& faces = mesh->get_faces();
  std::vector<math::Vec3f> const& vertices = mesh->get_vertices();
  std::cout << "Number of faces: " << faces.size() / 3 << " faces.\n";
  bvh_tree = std::shared_ptr<BVHTree>(new BVHTree(faces, vertices));
  std::cout << "Building the tree took: " << timer.get_elapsed() / 1000.0 << " seconds\n";
}

Eigen::Vector3d vec3f_to_eigen(math::Vec3f const& v) {
  Eigen::Vector3d V;
  for (size_t it = 0; it < 3; it++) V[it] = v[it];
  return V;
}

math::Vec3f eigen_to_vec3f(Eigen::Vector3d const& V) {
  math::Vec3f v;
  for (size_t it = 0; it < 3; it++) v[it] = V[it];
  return v;
}

void formMtl(std::string const& out_prefix, std::string& mtl_str) {
  std::ostringstream ofs;
  ofs << "newmtl material0000\n";
  ofs << "Ka 0.200000 0.200000 0.200000\n";
  ofs << "Kd 1.000000 1.000000 1.000000\n";
  ofs << "Ks 1.000000 1.000000 1.000000\n";
  ofs << "Tr 0.000000\n";
  ofs << "illum 2\n";
  ofs << "Ns 0.000000\n";
  ofs << "map_Kd " << out_prefix << ".png\n";
  mtl_str = ofs.str();
}

void formObjCustomUV(mve::TriangleMesh::ConstPtr mesh, 
                     std::vector<Eigen::Vector3i> const& face_vec,
                     std::map<int, Eigen::Vector2d> const& uv_map, 
                     std::string const& out_prefix,
                     std::string& obj_str) {
  // Get handles to the vertices and vertex normals
  std::vector<math::Vec3f> const& vertices = mesh->get_vertices();
  std::vector<math::Vec3f> const& mesh_normals = mesh->get_vertex_normals();
  if (vertices.size() != mesh_normals.size())
    LOG(FATAL) << "A mesh must have as many vertices as vertex normals.";

  // The uv_map assigns to each of the mesh vertices that is visible
  // in the texture its (u, v) pair. Find the map from each vertex
  // index to the index in the list of (u, v) pairs.
  std::map<int, int> vertex_to_uv;
  int64_t count = 0;
  for (auto it = uv_map.begin(); it != uv_map.end(); it++) {
    vertex_to_uv[it->first] = count;
    count++;
  }

  std::ostringstream out;

  out << "mtllib " << out_prefix << ".mtl\n";

  out << std::setprecision(16);
  for (std::size_t i = 0; i < vertices.size(); i++) {
    out << "v " << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << "\n";
  }

  for (auto it = uv_map.begin(); it != uv_map.end(); it++)
    out << "vt " << (it->second)[0] << " " << (it->second)[1] << "\n";

  for (std::size_t i = 0; i < mesh_normals.size(); i++)
    out << "vn " << mesh_normals[i][0] << " " << mesh_normals[i][1] << " "
        << mesh_normals[i][2] << "\n";

  int64_t OBJ_INDEX_OFFSET = 1;  // have indices start from 1
  for (std::size_t j = 0; j < face_vec.size(); j++) {
    out << "f";
    for (std::size_t k = 0; k < 3; ++k) {
      out << " " << face_vec[j][k] + OBJ_INDEX_OFFSET << "/"
          << vertex_to_uv[face_vec[j][k]] + OBJ_INDEX_OFFSET << "/"
          << face_vec[j][k] + OBJ_INDEX_OFFSET;
    }
    out << "\n";
  }
  obj_str = out.str();
}

// Project texture and find the UV coordinates
void projectTexture(mve::TriangleMesh::ConstPtr mesh, std::shared_ptr<BVHTree> bvh_tree,
                    cv::Mat const& image,
                    Eigen::Affine3d const& world_to_cam,
                    rig::CameraParameters const& cam_params,
                    // outputs
                    std::vector<double>& smallest_cost_per_face,
                    std::vector<Eigen::Vector3i>& face_vec,
                    std::map<int, Eigen::Vector2d>& uv_map) {

  // Wipe the outputs
  face_vec.clear();
  uv_map.clear();

  // Compute camera center from world_to_cam transform
  Eigen::Vector3d cam_ctr = world_to_cam.inverse().translation();

  // Here need to take into account that for real (not simulated)
  // images the camera may have been calibrated at 1/4 the original
  // image resolution. We will use the calibrated dimensions when
  // normalizing u and v, because when projecting 3D points in the
  // camera we will the calibrated camera model.
  int64_t raw_image_cols = image.cols;
  int64_t raw_image_rows = image.rows;
  int64_t calib_image_cols = cam_params.GetDistortedSize()[0];
  int64_t calib_image_rows = cam_params.GetDistortedSize()[1];

  int64_t factor = raw_image_cols / calib_image_cols;

  if ((raw_image_cols != calib_image_cols * factor) ||
      (raw_image_rows != calib_image_rows * factor)) {
    LOG(FATAL) << "Published image width and height are: "
               << raw_image_cols << ' ' << raw_image_rows << "\n"
               << "Calibrated image width and height are: "
               << calib_image_cols << ' ' << calib_image_rows << "\n"
               << "These must be equal up to an integer factor.\n";
  }

  std::vector<math::Vec3f> const& vertices = mesh->get_vertices();
  std::vector<math::Vec3f> const& mesh_normals = mesh->get_vertex_normals();
  if (vertices.size() != mesh_normals.size())
    LOG(FATAL) << "A mesh must have as many vertices as vertex normals.";

  std::vector<unsigned int> const& faces = mesh->get_faces();
  std::vector<math::Vec3f> const& face_normals = mesh->get_face_normals();

  if (smallest_cost_per_face.size() != faces.size())
    LOG(FATAL) << "There must be one cost value per face.";

#pragma omp parallel for
  for (std::size_t face_id = 0; face_id < faces.size() / 3; face_id++) {
    math::Vec3f const& v1 = vertices[faces[3 * face_id + 0]];
    math::Vec3f const& v2 = vertices[faces[3 * face_id + 1]];
    math::Vec3f const& v3 = vertices[faces[3 * face_id + 2]];
    math::Vec3f const& face_normal = face_normals[face_id];
    math::Vec3f const face_center = (v1 + v2 + v3) / 3.0f;

    // Do some geometric checks and compute the cost for this face and camera

    Eigen::Vector3d cam_to_face_vec = (vec3f_to_eigen(face_center) - cam_ctr).normalized();
    Eigen::Vector3d face_to_cam_vec = -cam_to_face_vec;
    double face_normal_to_cam_dot_prod = face_to_cam_vec.dot(vec3f_to_eigen(face_normal));

    if (face_normal_to_cam_dot_prod <= 0.0) continue;  // The face points away from the camera

    // Angle between face normal and ray from face center to camera center
    // is bigger than 75 degrees.
    // TODO(oalexan1): Make this a parameter.
    // TODO(oalexan1): Filter by distance from each of
    // v1, v2, v3 to view_pos.
    double face_normal_to_cam_angle = std::acos(face_normal_to_cam_dot_prod);  // radians
    if (face_normal_to_cam_angle > 75.0 * M_PI / 180.0) continue;

    // The further a camera is and the bigger then angle between the
    // camera direction and the face normal, the less we want this
    // camera's texture for this triangle.
    double cost_val = face_normal_to_cam_angle + (vec3f_to_eigen(face_center) - cam_ctr).norm();
    if (cost_val >= smallest_cost_per_face[face_id]) {
      continue;
    }

    // A mesh triangle is visible if rays from its vertices do not
    // intersect the mesh somewhere else before hitting the camera,
    // and hit the camera inside the image bounds.
    bool visible = true;
    math::Vec3f const* samples[] = {&v1, &v2, &v3};

    std::vector<Eigen::Vector2d> UV;
    for (std::size_t vertex_it = 0; vertex_it < 3; vertex_it++) {
      BVHTree::Ray ray;
      ray.origin = *samples[vertex_it];
      ray.dir = eigen_to_vec3f(cam_ctr) - ray.origin;
      ray.tmax = ray.dir.norm();
      ray.tmin = ray.tmax * 0.0001f;
      ray.dir.normalize();

      BVHTree::Hit hit;
      if (bvh_tree->intersect(ray, &hit)) {
        visible = false;
        break;
      }

      // Triangle vertex in world coordinates
      Eigen::Vector3d world_pt = vec3f_to_eigen(*samples[vertex_it]);

      // Transform the vertex to camera coordinates
      Eigen::Vector3d cam_pt = world_to_cam * world_pt;

      // Skip points that project behind the camera
      if (cam_pt.z() <= 0) {
        visible = false;
        break;
      }

      // Get the undistorted pixel
      Eigen::Vector2d undist_centered_pix =
        cam_params.GetFocalVector().cwiseProduct(cam_pt.hnormalized());
      if (std::abs(undist_centered_pix[0]) > cam_params.GetUndistortedHalfSize()[0] ||
          std::abs(undist_centered_pix[1]) > cam_params.GetUndistortedHalfSize()[1]) {
        // If we are out of acceptable undistorted region, there's some uncertainty whether
        // the distortion computation in the next operation will work, so quit early.
        visible = false;
        break;
      }

      // Get the distorted pixel value
      Eigen::Vector2d dist_pix;
      cam_params.Convert<rig::UNDISTORTED_C, rig::DISTORTED>
        (undist_centered_pix, &dist_pix);

      // Skip pixels that don't project in the window of dimensions
      // dist_crop_size centered at the image center. Note that
      // dist_crop_size is read from the camera configuration, and is
      // normally either the full image or something smaller if the
      // user restricts the domain of validity of the distortion
      // model.
      Eigen::Vector2i dist_size      = cam_params.GetDistortedSize();
      Eigen::Vector2i dist_crop_size = cam_params.GetDistortedCropSize();
      if (std::abs(dist_pix[0] - dist_size[0] / 2.0) > dist_crop_size[0] / 2.0  ||
          std::abs(dist_pix[1] - dist_size[1] / 2.0) > dist_crop_size[1] / 2.0) {
        visible = false;
        break;
      }
      
      // Find the u, v coordinates of each vertex
      double u = dist_pix.x() / calib_image_cols;
      // TODO(oalexan1): Maybe use:
      // v = (calib_image_rows - 1 - dist_pix.y())/image_rows ?
      double v = 1.0 - dist_pix.y() / calib_image_rows;

      UV.push_back(Eigen::Vector2d(u, v));
    }

    if (!visible) continue;

#pragma omp critical
    {
      // Here we integrate the results from each loop iteration so have
      // to use a lock (critical section).

      // Sanity check, to ensure nothing got mixed up with the multiple
      // threads
      if (cost_val >= smallest_cost_per_face[face_id])
        LOG(FATAL) << "Book-keeping error in estimating cost per face.";

      smallest_cost_per_face[face_id] = cost_val;

      for (std::size_t vertex_it = 0; vertex_it < 3; vertex_it++)
        uv_map[faces[3 * face_id + vertex_it]] = UV[vertex_it];

      face_vec.push_back(Eigen::Vector3i(faces[3 * face_id + 0], faces[3 * face_id + 1], faces[3 * face_id + 2]));
    }
  }  // End loop over mesh faces
}

// Calculate camera center and ray direction in world coordinates from a distorted pixel
void calcCamCtrDir(rig::CameraParameters const& cam_params,
                   Eigen::Vector2d const& dist_pix,
                   Eigen::Affine3d const& world_to_cam,
                   // Output
                   Eigen::Vector3d& cam_ctr,
                   Eigen::Vector3d& world_ray) {
  
  // Undistort the pixel
  Eigen::Vector2d undist_centered_pix;
  cam_params.Convert<rig::DISTORTED, rig::UNDISTORTED_C>(dist_pix, &undist_centered_pix);

  // Ray from camera going through the undistorted pand centered pixel
  Eigen::Vector3d cam_ray(undist_centered_pix.x() / cam_params.GetFocalVector()[0],
                          undist_centered_pix.y() / cam_params.GetFocalVector()[1], 1.0);
  cam_ray.normalize();

  Eigen::Affine3d cam_to_world = world_to_cam.inverse();
  world_ray = cam_to_world.linear() * cam_ray;
  cam_ctr = cam_to_world.translation();
}

// Intersect ray with a mesh. Return true on success.
bool ray_mesh_intersect(Eigen::Vector2d const& dist_pix,
                        rig::CameraParameters const& cam_params,
                        Eigen::Affine3d const& world_to_cam,
                        mve::TriangleMesh::Ptr const& mesh,
                        std::shared_ptr<BVHTree> const& bvh_tree,
                        double min_ray_dist, double max_ray_dist,
                        // Output
                        Eigen::Vector3d& intersection) {

  // Initialize the output
  intersection = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Calculate camera center and ray direction
  Eigen::Vector3d cam_ctr, world_ray;
  calcCamCtrDir(cam_params, dist_pix, world_to_cam, cam_ctr, world_ray);

  // Set up the ray structure for the mesh
  BVHTree::Ray bvh_ray;
  bvh_ray.origin = rig::eigen_to_vec3f(cam_ctr);
  bvh_ray.dir = rig::eigen_to_vec3f(world_ray);
  bvh_ray.dir.normalize();
  bvh_ray.tmin = min_ray_dist;
  bvh_ray.tmax = max_ray_dist;

  // Intersect the ray with the mesh
  BVHTree::Hit hit;
  if (bvh_tree->intersect(bvh_ray, &hit)) {
    double cam_to_mesh_dist = hit.t;
    intersection = cam_ctr + cam_to_mesh_dist * world_ray;
    return true;
  }

  return false;
}

// Project and save a mesh as an obj file to out_prefix.obj,
// out_prefix.mtl, out_prefix.png.
void meshProject(mve::TriangleMesh::Ptr const& mesh, std::shared_ptr<BVHTree> const& bvh_tree,
                 cv::Mat const& image, Eigen::Affine3d const& world_to_cam,
                 rig::CameraParameters const& cam_params,
                 std::string const& out_prefix) {
  // Create the output directory, if needed
  std::string out_dir = boost::filesystem::path(out_prefix).parent_path().string();
  if (out_dir != "") rig::createDir(out_dir);

  std::vector<Eigen::Vector3i> face_vec;
  std::map<int, Eigen::Vector2d> uv_map;

  std::vector<unsigned int> const& faces = mesh->get_faces();
  int64_t num_faces = faces.size();
  std::vector<double> smallest_cost_per_face(num_faces, 1.0e+100);

  // Find the UV coordinates and the faces having them
  rig::projectTexture(mesh, bvh_tree, image, world_to_cam, cam_params,
                      smallest_cost_per_face, face_vec, uv_map);

  // Strip the directory name, according to .obj file conventions.
  std::string suffix = boost::filesystem::path(out_prefix).filename().string();

  std::string obj_str;
  rig::formObjCustomUV(mesh, face_vec, uv_map, suffix, obj_str);

  std::string mtl_str;
  rig::formMtl(suffix, mtl_str);

  std::string obj_file = out_prefix + ".obj";
  std::cout << "Writing: " << obj_file << std::endl;
  std::ofstream obj_handle(obj_file);
  obj_handle << obj_str;
  obj_handle.close();

  std::string mtl_file = out_prefix + ".mtl";
  std::cout << "Writing: " << mtl_file << std::endl;
  std::ofstream mtl_handle(mtl_file);
  mtl_handle << mtl_str;
  mtl_handle.close();

  std::string texture_file = out_prefix + ".png";
  std::cout << "Writing: " << texture_file << std::endl;
  cv::imwrite(texture_file, image);
}

// Project given images with optimized cameras onto mesh. It is
// assumed that the most up-to-date cameras were copied/interpolated
// form the optimizer structures into the world_to_cam vector.
void meshProjectCameras(std::vector<std::string> const& cam_names,
                        std::vector<rig::CameraParameters> const& cam_params,
                        std::vector<rig::cameraImage> const& cam_images,
                        std::vector<Eigen::Affine3d> const& world_to_cam,
                        mve::TriangleMesh::Ptr const& mesh,
                        std::shared_ptr<BVHTree> const& bvh_tree,
                        std::string const& out_dir) {
  if (cam_names.size() != cam_params.size())
    LOG(FATAL) << "There must be as many camera names as sets of camera parameters.\n";
  if (cam_images.size() != world_to_cam.size())
    LOG(FATAL) << "There must be as many camera images as camera poses.\n";
  if (out_dir.empty())
    LOG(FATAL) << "The output directory is empty.\n";
  
  char filename_buffer[1000];

  for (size_t cid = 0; cid < cam_images.size(); cid++) {
    double timestamp = cam_images[cid].timestamp;
    int cam_type = cam_images[cid].camera_type;

    // Must use the 10.7f format for the timestamp as everywhere else in the code
    snprintf(filename_buffer, sizeof(filename_buffer), "%s/%10.7f_%s",
             out_dir.c_str(), timestamp, cam_names[cam_type].c_str());
    std::string out_prefix = filename_buffer;  // convert to string

    std::cout << "Creating texture for: " << out_prefix << std::endl;
    meshProject(mesh, bvh_tree, cam_images[cid].image, world_to_cam[cid], 
                cam_params[cam_type], out_prefix);
  }
}

// Intersect them with a mesh instead, and average those intersections.
// This will be used to for a mesh-to-triangulated-points constraint.
void meshTriangulations(std::vector<rig::CameraParameters> const& cam_params,
                        std::vector<rig::cameraImage> const& cams,
                        std::vector<Eigen::Affine3d> const& world_to_cam,
                        rig::PidCidFid const& pid_to_cid_fid,
                        PidCidFidMap const& pid_cid_fid_inlier,
                        rig::KeypointVec const& keypoint_vec,
                        double min_ray_dist, double max_ray_dist,
                        mve::TriangleMesh::Ptr const& mesh, 
                        std::shared_ptr<BVHTree> const& bvh_tree,
                        // Outputs
                        rig::PidCidFidToMeshXyz & pid_cid_fid_mesh_xyz,
                        std::vector<Eigen::Vector3d> & pid_mesh_xyz) {
  
  // Initialize the outputs
  pid_cid_fid_mesh_xyz.resize(pid_to_cid_fid.size());
  pid_mesh_xyz.resize(pid_to_cid_fid.size());

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    Eigen::Vector3d avg_mesh_xyz(0, 0, 0);
    int num_intersections = 0;

    for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
         cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      // Initialize this
      pid_cid_fid_mesh_xyz[pid][cid][fid] = rig::badMeshXyz();

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
        continue;

      // Intersect the ray with the mesh
      Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first, keypoint_vec[cid][fid].second);
      Eigen::Vector3d mesh_xyz(0.0, 0.0, 0.0);
      bool have_mesh_intersection
        = rig::ray_mesh_intersect(dist_ip, cam_params[cams[cid].camera_type],
                                  world_to_cam[cid], mesh, bvh_tree,
                                  min_ray_dist, max_ray_dist,
                                  // Output
                                  mesh_xyz);

      if (have_mesh_intersection) {
        pid_cid_fid_mesh_xyz[pid][cid][fid] = mesh_xyz;
        avg_mesh_xyz += mesh_xyz;
        num_intersections += 1;
      }
    }

    // Average the intersections of all rays with the mesh
    if (num_intersections >= 1)
      avg_mesh_xyz /= num_intersections;
    else
      avg_mesh_xyz = rig::badMeshXyz();

    pid_mesh_xyz[pid] = avg_mesh_xyz;
  }

  return;
}

}  // namespace rig
