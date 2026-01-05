/* Copyright (c) 2021, United States Government, as represented by the
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

#ifndef ASP_RIG_TEXTURE_PROCESSING_H
#define ASP_RIG_TEXTURE_PROCESSING_H

#include <asp/Rig/RigTypeDefs.h>
#include <asp/Rig/RigCameraModel.h>
#include <asp/Rig/rig_utils.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

// texrecon includes
#include <mve/mesh_io_ply.h>
#include <mve/mesh_info.h>
#include <mve/mesh.h>
#include <mve/image.h>
#include <acc/bvh_tree.h>
#include <tex/timer.h>
#include <tex/texturing.h>
#include <tex/tri.h>
#include <tex/texture_patch.h>
#include <tex/rectangular_bin.h>
#include <tex/material_lib.h>
#include <util/exception.h>
#include <math/vector.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <map>
#include <limits>
#include <string>

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

namespace rig {

// In order to sample a face it is easier to first rotate it so it
// is in a plane so that the face normal points along the positive
// x axis. All points in the transformed face will have a constant
// x. Store the that value of x and together with the
// lower-left value of the bounding box in that plane (so min_y, and min_z),
// and the number of samples needed (with given pixel size) to
// sample that face (with a small padding on both sides).
// Also store the transform from that plane to the face itself.

// We use 'int64_t' values instead of 'int', as the latter is 32-bit and
// the area of some images we encounter can then overflow.

struct FaceInfo {
  double x, min_y, min_z;

  // We will have [min_y, min_y + width * pixel_size] x [min_z, min_z + height * pixel_size]
  // contain the face transformed in that plane with the normal pointing along the z axis.
  int64_t width, height;

  // The padding to apply to each face bounding box before sampling it
  int64_t face_info_padding;

  // The pixel at (x, min_y, min_z) in the plane will end up at location (shift_u, shift_v)
  // in the texture.
  int64_t shift_u, shift_v;

  // Used to flag a valid face
  bool valid;

  // The transform which makes a version of the face in the y-z plane to the actual triangle
  Eigen::Affine3d YZPlaneToTriangleFace;

  // The vertices of a face after being transformed to a plane with x constant
  std::vector<Eigen::Vector3d> TransformedVertices;

  // Initialize all members. Invalid or unprocessed faces will have
  // shift_u == invalid_shift_u.
  FaceInfo() {
    x       = 0.0;
    min_y   = 0.0;
    min_z   = 0.0;
    width   = 0L;
    height  = 0L;
    face_info_padding = 1L;
    valid = true;
    shift_u = std::numeric_limits<int>::max();
    shift_v = 0L;
    YZPlaneToTriangleFace = Eigen::Affine3d::Identity();
    TransformedVertices.resize(3);
  }
};

// Small utilities
Eigen::Vector3d vec3f_to_eigen(math::Vec3f const& v);
math::Vec3f eigen_to_vec3f(Eigen::Vector3d const& V);

// A texture patch without holding a buffer to the texture but only vertex and face info
class IsaacTexturePatch {
 public:
  typedef std::shared_ptr<IsaacTexturePatch> Ptr;
  typedef std::shared_ptr<const IsaacTexturePatch> ConstPtr;
  typedef std::vector<std::size_t> Faces;
  typedef std::vector<Eigen::Vector2d> Texcoords;

 private:
  int64_t label;
  Faces faces;
  Texcoords texcoords;
  int64_t width, height;

 public:
  /** Constructs a texture patch. */
  IsaacTexturePatch(int64_t label, std::vector<std::size_t> const& faces,
                    std::vector<Eigen::Vector2d> const& texcoords,
                    int64_t width, int64_t height);

  IsaacTexturePatch(IsaacTexturePatch const& texture_patch);

  static IsaacTexturePatch::Ptr create(IsaacTexturePatch::ConstPtr texture_patch);
  static IsaacTexturePatch::Ptr create(int64_t label, std::vector<std::size_t> const& faces,
                                       std::vector<Eigen::Vector2d> const& texcoords,
                                       int64_t width, int64_t height);

  IsaacTexturePatch::Ptr duplicate(void);

  std::vector<std::size_t>& get_faces(void);
  std::vector<std::size_t> const& get_faces(void) const;
  std::vector<Eigen::Vector2d>      & get_texcoords(void);
  std::vector<Eigen::Vector2d> const& get_texcoords(void) const;

  int64_t get_label(void) const;
  int64_t get_width(void) const;
  int64_t get_height(void) const;
  int64_t get_size(void) const;
};

inline IsaacTexturePatch::IsaacTexturePatch(int64_t label, std::vector<std::size_t> const& faces,
                                            std::vector<Eigen::Vector2d> const& texcoords,
                                            int64_t width, int64_t height)
    : label(label), faces(faces), texcoords(texcoords), width(width), height(height) {}

IsaacTexturePatch::IsaacTexturePatch(IsaacTexturePatch const& texture_patch) {
  label = texture_patch.label;
  faces = std::vector<std::size_t>(texture_patch.faces);
  texcoords = std::vector<Eigen::Vector2d>(texture_patch.texcoords);
  width = texture_patch.width;
  height = texture_patch.height;
}

inline IsaacTexturePatch::Ptr IsaacTexturePatch::create(IsaacTexturePatch::ConstPtr texture_patch) {
  return std::make_shared<IsaacTexturePatch>(*texture_patch);
}

inline IsaacTexturePatch::Ptr IsaacTexturePatch::create(
  int64_t label, std::vector<std::size_t> const& faces,
  std::vector<Eigen::Vector2d> const& texcoords, int64_t width, int64_t height) {
  return std::make_shared<IsaacTexturePatch>(label, faces, texcoords, width, height);
}

inline IsaacTexturePatch::Ptr IsaacTexturePatch::duplicate(void) {
  return Ptr(new IsaacTexturePatch(*this));
}

inline int64_t IsaacTexturePatch::get_label(void) const { return label; }

inline int64_t IsaacTexturePatch::get_width(void) const { return width; }

inline int64_t IsaacTexturePatch::get_height(void) const { return height; }

inline std::vector<Eigen::Vector2d>& IsaacTexturePatch::get_texcoords(void) {
  return texcoords;
}

inline std::vector<std::size_t>& IsaacTexturePatch::get_faces(void) { return faces; }

inline std::vector<Eigen::Vector2d> const& IsaacTexturePatch::get_texcoords(void) const {
  return texcoords;
}

inline std::vector<std::size_t> const& IsaacTexturePatch::get_faces(void) const { return faces; }

inline int64_t IsaacTexturePatch::get_size(void) const { return get_width() * get_height(); }

/**
 * Class representing a texture atlas. Have to duplicate the code from texrecon
 * as there are custom changes for ISAAC.
 */
class IsaacTextureAtlas {
 public:
  typedef std::shared_ptr<IsaacTextureAtlas> Ptr;

  typedef std::vector<std::size_t> Faces;
  typedef std::vector<std::size_t> TexcoordIds;
  typedef std::vector<Eigen::Vector2d> Texcoords;

  int64_t get_width();
  int64_t get_height();

 private:
  int64_t width, height, determined_height;
  bool finalized;

  Faces faces;
  Texcoords texcoords;
  TexcoordIds texcoord_ids;

  mve::ByteImage::Ptr image;

  RectangularBin::Ptr bin;

  void resize_atlas(void);

 public:
  IsaacTextureAtlas(int64_t width, int64_t height);

  static IsaacTextureAtlas::Ptr create(int64_t width, int64_t height);

  Faces& get_faces(void);
  TexcoordIds& get_texcoord_ids(void);
  Texcoords& get_texcoords(void);
  mve::ByteImage::Ptr& get_image(void);

  bool insert(IsaacTexturePatch::ConstPtr texture_patch);

  void scale_texcoords(void);

  void merge_texcoords(void);

  void finalize(void);
};

inline IsaacTextureAtlas::Ptr IsaacTextureAtlas::create(int64_t width, int64_t height) {
  return Ptr(new IsaacTextureAtlas(width, height));
}

inline IsaacTextureAtlas::Faces& IsaacTextureAtlas::get_faces(void) { return faces; }

inline IsaacTextureAtlas::TexcoordIds& IsaacTextureAtlas::get_texcoord_ids(void) {
  return texcoord_ids;
}

inline IsaacTextureAtlas::Texcoords& IsaacTextureAtlas::get_texcoords(void) { return texcoords; }

inline mve::ByteImage::Ptr& IsaacTextureAtlas::get_image(void) { return image; }

inline int64_t IsaacTextureAtlas::get_width() { return width; }

inline int64_t IsaacTextureAtlas::get_height() { return height; }


/**
 * Class representing a obj model. Have to duplicate the texrecon structure, as
 * we use double-precision texcoords.
  */
class IsaacObjModel {
 public:
  struct Face {
    std::size_t vertex_ids[3];
    std::size_t texcoord_ids[3];
    std::size_t normal_ids[3];
  };

    struct Group {
        std::string material_name;
        std::vector<Face> faces;
    };

    typedef std::vector<math::Vec3f> Vertices;
    typedef std::vector<Eigen::Vector2d> TexCoords;
    typedef std::vector<math::Vec3f> Normals;
    typedef std::vector<Group> Groups;

 private:
    Vertices vertices;
    TexCoords texcoords;
    Normals normals;
    Groups groups;
    MaterialLib material_lib;

 public:
    /** Saves the obj model to an .obj file, its material lib and the
        materials with the given prefix. */
    void save_to_files(std::string const & prefix) const;

    MaterialLib & get_material_lib(void);
    Vertices & get_vertices(void);
    TexCoords & get_texcoords(void);
    Normals & get_normals(void);
    Groups & get_groups(void);

    static void save(IsaacObjModel const & model, std::string const & prefix);
};

inline
MaterialLib &
IsaacObjModel::get_material_lib(void) {
    return material_lib;
}

inline
IsaacObjModel::Vertices &
IsaacObjModel::get_vertices(void) {
    return vertices;
}

inline
IsaacObjModel::TexCoords &
IsaacObjModel::get_texcoords(void) {
    return texcoords;
}

inline
IsaacObjModel::Normals &
IsaacObjModel::get_normals(void) {
    return normals;
}

inline
IsaacObjModel::Groups &
IsaacObjModel::get_groups(void) {
    return groups;
}

// Load and prepare a mesh
void loadMeshBuildTree(std::string const& mesh_file, mve::TriangleMesh::Ptr& mesh,
                       std::shared_ptr<mve::MeshInfo>& mesh_info,
                       std::shared_ptr<tex::Graph>& graph,
                       std::shared_ptr<BVHTree>& bvh_tree);

// Put an textured mesh obj file in a string
void formObjCustomUV(mve::TriangleMesh::ConstPtr mesh, std::vector<Eigen::Vector3i> const& face_vec,
                     std::map<int, Eigen::Vector2d> const& uv_map,
                     std::string const& out_prefix, std::string& obj_str);

void formMtl(std::string const& out_prefix, std::string& mtl_str);

// Project texture and find the UV coordinates
void projectTexture(mve::TriangleMesh::ConstPtr mesh, std::shared_ptr<BVHTree> bvh_tree,
                    cv::Mat const& image, rig::CameraModel const& cam,
                    // outputs
                    std::vector<double>& smallest_cost_per_face,
                    std::vector<Eigen::Vector3i>& face_vec,
                    std::map<int, Eigen::Vector2d>& uv_map);

// Find where ray emanating from a distorted pixel intersects a mesh. Return true
// on success.
bool ray_mesh_intersect(Eigen::Vector2d const& dist_pix,
                        rig::CameraParameters const& cam_params,
                        Eigen::Affine3d const& world_to_cam,
                        mve::TriangleMesh::Ptr const& mesh,
                        std::shared_ptr<BVHTree> const& bvh_tree,
                        double min_ray_dist, double max_ray_dist,
                        // Output
                        Eigen::Vector3d& intersection);

void meshProject(mve::TriangleMesh::Ptr const& mesh, std::shared_ptr<BVHTree> const& bvh_tree,
                 cv::Mat const& image,
                 Eigen::Affine3d const& world_to_cam, rig::CameraParameters const& cam_params,
                 std::string const& out_prefix);

void meshProjectCameras(std::vector<std::string> const& cam_names,
                        std::vector<rig::CameraParameters> const& cam_params,
                        std::vector<rig::cameraImage> const& cam_images,
                        std::vector<Eigen::Affine3d> const& world_to_cam,
                        mve::TriangleMesh::Ptr const& mesh,
                        std::shared_ptr<BVHTree> const& bvh_tree,
                        std::string const& out_dir);

void meshTriangulations(// Inputs
  std::vector<rig::CameraParameters> const& cam_params,
  std::vector<rig::cameraImage>      const& cams,
  std::vector<Eigen::Affine3d>       const& world_to_cam,
  rig::PidCidFid                     const& pid_to_cid_fid,
  PidCidFidMap                       const& pid_cid_fid_inlier,
  rig::KeypointVec                   const& keypoint_vec,
  Eigen::Vector3d                    const& bad_xyz, double min_ray_dist, double max_ray_dist,
  mve::TriangleMesh::Ptr             const& mesh, std::shared_ptr<BVHTree> const& bvh_tree,
  // Outputs
  rig::PidCidFidToMeshXyz                 & pid_cid_fid_mesh_xyz,
  std::vector<Eigen::Vector3d>            & pid_mesh_xyz);
  
}  // namespace rig

#endif  // ASP_RIG_TEXTURE_PROCESSING_H
