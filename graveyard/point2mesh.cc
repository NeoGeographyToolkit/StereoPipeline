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


/// \file point2mesh.cc
///

#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <stdlib.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
using namespace vw;

#include <asp/Tools/nff_terrain.h>

// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_pointcloud_filename(std::string const& filename) {
  std::string result = filename;

  // First case: filenames that match <prefix>-PC.<suffix>
  int index = result.rfind("-PC.");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // Second case: filenames that match <prefix>.<suffix>
  index = result.rfind(".");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // No match
  return result;
}

/// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

/// Erases a file suffix if one exists and returns the base string
static std::string suffix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(0, index);
  return result;
}

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}


template <class ViewT>
BBox<3> pointcloud_bbox(ImageViewBase<ViewT> const& point_image) {
  // Compute bounding box
  BBox<3> result;
  typename ViewT::pixel_accessor row_acc = point_image.impl().origin();
  for( int32 row=0; row < point_image.impl().rows(); ++row ) {
    typename ViewT::pixel_accessor col_acc = row_acc;
    for( int32 col=0; col < point_image.impl().cols(); ++col ) {
      if (*col_acc != Vector3())
        result.grow(*col_acc);
      col_acc.next_col();
    }
    row_acc.next_row();
  }
  return result;
}



// Apply an offset to the points in the PointImage
class PointOffsetFunc : public UnaryReturnSameType {
  Vector3 m_offset;

public:
  PointOffsetFunc(Vector3 const& offset) : m_offset(offset) {}

  template <class T>
  T operator()(T const& p) const {
    if (p == T()) return p;
    return p + m_offset;
  }
};

template <class ImageT>
UnaryPerPixelView<ImageT, PointOffsetFunc>
inline point_image_offset( ImageViewBase<ImageT> const& image, Vector3 const& offset) {
  return UnaryPerPixelView<ImageT,PointOffsetFunc>( image.impl(), PointOffsetFunc(offset) );
}



class PointTransFunc : public ReturnFixedType<Vector3> {
  Matrix3x3 m_trans;
public:
  PointTransFunc(Matrix3x3 trans) : m_trans(trans) {}
  Vector3 operator() (Vector3 const& pt) const { return m_trans*pt; }
};

int main( int argc, char *argv[] ) {
  set_debug_level(VerboseDebugMessage+11);

  std::string pointcloud_filename, out_prefix = "", output_file_type, texture_filename;
  unsigned cache_size, max_triangles;
  float mesh_tolerance;
  unsigned simplemesh_h_step, simplemesh_v_step;
  int debug_level;
  double phi_rot, omega_rot, kappa_rot;
  std::string rot_order;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("simple-mesh", "Generate simple (non-adaptive) mesh")
    ("simplemesh-h-step", po::value<unsigned>(&simplemesh_h_step)->default_value(1), "Horizontal step size for simple meshing algorithm")
    ("simplemesh-v-step", po::value<unsigned>(&simplemesh_v_step)->default_value(1), "Vertical step size for simple meshing algorithm")
    ("mesh-tolerance", po::value<float>(&mesh_tolerance)->default_value(0.001), "Tolerance for the adaptive meshing algorithm")
    ("max_triangles", po::value<unsigned>(&max_triangles)->default_value(1000000), "Maximum triangles for the adaptive meshing algorithm")
    ("center", "Center the model around the origin.  Use this option if you are experiencing numerical precision issues.")
    ("cache", po::value<unsigned>(&cache_size)->default_value(2048), "Cache size, in megabytes")
    ("input-file", po::value<std::string>(&pointcloud_filename), "Explicitly specify the input file")
    ("texture-file", po::value<std::string>(&texture_filename), "Specify texture filename")
    ("grayscale-texture", "Use grayscale image processing when modifying the texture image (for .iv and .vrml files only)")
    ("output-prefix,o", po::value<std::string>(&out_prefix), "Specify the output prefix")
    ("output-filetype,t", po::value<std::string>(&output_file_type)->default_value("ive"), "Specify the output file")
    ("debug-level,d", po::value<int>(&debug_level)->default_value(vw::DebugMessage-1), "Set the debugging output level. (0-50+)")

    ("rotation-order", po::value<std::string>(&rot_order)->default_value("xyz"),"Set the order of an euler angle rotation applied to the 3D points prior to DEM rasterization")
    ("phi-rotation", po::value<double>(&phi_rot)->default_value(0),"Set a rotation angle phi")
    ("omega-rotation", po::value<double>(&omega_rot)->default_value(0),"Set a rotation angle omega")
    ("kappa-rotation", po::value<double>(&kappa_rot)->default_value(0),"Set a rotation angle kappa")
                ("flip-triangles", "Use clockwise vertex ordering (.iv and .tri files only)");

  po::positional_options_description p;
  p.add("input-file", 1);
  p.add("texture-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  // Set the Vision Workbench debug level
  //set_debug_level(debug_level);
  //vw_system_cache().resize( cache_size*1024*1024 );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <pointcloud> <texture file> ...";
  usage << std::endl << std::endl << desc << std::endl;

  if( vm.count("help") ) {
    vw_out(0) << usage.str();
    return 1;
  }

  if( vm.count("input-file") != 1 || (vm.count("texture-file") != 1 && output_file_type != "tri") ) {
    vw_out(0) << "Error: Must specify exactly one pointcloud file and one texture file!" << std::endl;
    vw_out(0) << usage.str();
    return 1;
  }

  if( out_prefix == "" ) {
    out_prefix = prefix_from_pointcloud_filename(pointcloud_filename);
  }

  DiskImageView<Vector3> point_disk_image(pointcloud_filename);
  ImageViewRef<Vector3> point_image = point_disk_image;

  if (vm.count("center")) {
    BBox<float,3> bbox = pointcloud_bbox(point_disk_image);
    std::cout << "\t--> Centering model around the origin.\n";
    std::cout << "\t    Initial point image bounding box: " << bbox << "\n";
    Vector3 midpoint = (bbox.max() + bbox.min()) / 2.0;
    std::cout << "\t    Midpoint: " << midpoint << "\n";
    point_image = point_image_offset(point_image, -midpoint);
    BBox<float,3> bbox2 = pointcloud_bbox(point_image);
    std::cout << "\t    Re-centered point image bounding box: " << bbox2 << "\n";
  }

  // Apply an (optional) rotation to the 3D points before building the mesh.
  if (phi_rot != 0 || omega_rot != 0 || kappa_rot != 0) {
    std::cout << "Applying rotation sequence: " << rot_order << "      Angles: " << phi_rot << "   " << omega_rot << "  " << kappa_rot << "\n";
    Matrix3x3 rotation_trans = math::euler_to_rotation_matrix(phi_rot,omega_rot,kappa_rot,rot_order);
    point_image = per_pixel_filter(point_image, PointTransFunc(rotation_trans));
  }

  std::cout << "\nGenerating 3D mesh from point cloud:\n";
  Mesh mesh_maker;
  if(vm.count("simple-mesh")) {
    mesh_maker.build_simple_mesh(point_image, simplemesh_h_step, simplemesh_v_step);
  } else {
    mesh_maker.build_adaptive_mesh(point_image, mesh_tolerance, max_triangles);
  }

  // New style (open scene graph) 3D models
  if (output_file_type == "ive")
    mesh_maker.write_osg(out_prefix+".ive", texture_filename);

  // Old style (open inventor) 3D models
  else if(output_file_type == "iv" || output_file_type == "vrml") {
    std::string corrected_texture_filename = prefix_from_filename(texture_filename)+".jpg";
    if (suffix_from_filename(texture_filename) != "jpg")
      if (vm.count("grayscale-texture") ) {
        DiskImageView<PixelGray<uint8> > texture(texture_filename);
        write_image(corrected_texture_filename, texture);
      } else {
        DiskImageView<PixelRGB<uint8> > texture(texture_filename);
        write_image(corrected_texture_filename, texture);
      }

    if(output_file_type == "iv")
      mesh_maker.write_inventor(out_prefix+".iv", corrected_texture_filename, vm.count("flip-triangles") != 0);

    if(output_file_type == "vrml")
      mesh_maker.write_vrml(out_prefix+".vrml", corrected_texture_filename);

  }

  // Simple trimesh (Stanford robotics libraries) 3D models
  else if (output_file_type == "tri") {
    mesh_maker.write_trimesh(out_prefix+".tri", vm.count("flip-triangles") != 0);
  } else {
    std::cout << "Unsupported 3D file type.\n";
  }

  return 0;
}
