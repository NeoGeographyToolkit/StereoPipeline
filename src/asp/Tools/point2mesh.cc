// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file point2mesh2.cc
///

//Standard Stuff
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>

//Boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

//VisionWorkbench
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <asp/Core/Macros.h>
using namespace vw;

//OpenSceneGraph
#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/Simplifier>
#include <osg/Node>
#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/TexGen>
#include <osg/Material>
#include <osgViewer/Viewer>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>

// ---------------------------------------------------------
// UTILITIES
// ---------------------------------------------------------

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

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

// ---------------------------------------------------------
// BUILD MESH
//
// Calculates a way to build color data on the DEM.
// ---------------------------------------------------------
osg::StateSet* create1DTexture( osg::Node* loadedModel , const osg::Vec3f& Direction ){
  const osg::BoundingSphere& bs = loadedModel->getBound();

  osg::Image* image = new osg::Image;
  int noPixels = 1000;
  int divider = 200;

  // Now I am going to build the image data
  image->allocateImage( noPixels , 1 , 1 , GL_RGBA , GL_FLOAT );
  image->setInternalTextureFormat( GL_RGBA );

  std::vector< osg::Vec4 > colour;
  colour.push_back( osg::Vec4( 0.0f , 1.0f , 0.0f , 1.0f ) );
  colour.push_back( osg::Vec4( 0.0f , 0.0f , 0.0f , 1.0f ) );

  // file in the image data
  osg::Vec4* dataPtr = (osg::Vec4*)image->data();
  for ( int i = 0 ; i < divider ; ++i ){
    osg::Vec4 color = colour[0];
    *dataPtr++ = color;
  }
  for ( int i=divider ; i < noPixels ; ++i ) {
    osg::Vec4 color = colour[1];
    *dataPtr++ = color;
  }

  // Texture 1D
  osg::Texture1D* texture = new osg::Texture1D;
  texture->setWrap( osg::Texture1D::WRAP_S , osg::Texture1D::MIRROR );
  texture->setFilter( osg::Texture1D::MIN_FILTER , osg::Texture1D::LINEAR );
  texture->setFilter( osg::Texture1D::MAG_FILTER , osg::Texture1D::LINEAR );
  texture->setImage( image );

  float zBase = bs.center().z() - bs.radius();
  float zScale = 0.04;

  // Texture Coordinate Generator
  osg::TexGen* texgen = new osg::TexGen;
  texgen->setMode( osg::TexGen::OBJECT_LINEAR );
  std::cout << "Direction Vector being used " << Direction.x() << " " << Direction.y() << " " << Direction.z() << std::endl;
  texgen->setPlane( osg::TexGen::S , osg::Plane( zScale*Direction.x() , zScale*Direction.y() , zScale*Direction.z() , -zBase ) );

  osg::Material* material = new osg::Material;

  osg::StateSet* stateset = new osg::StateSet;
  stateset->setTextureAttribute( 0 , texture , osg::StateAttribute::OVERRIDE );
  stateset->setTextureMode( 0 , GL_TEXTURE_1D , osg::StateAttribute::ON  | osg::StateAttribute::OVERRIDE );
  stateset->setTextureMode( 0 , GL_TEXTURE_2D , osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
  stateset->setTextureMode( 0 , GL_TEXTURE_3D , osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

  stateset->setTextureAttribute( 0 , texgen , osg::StateAttribute::OVERRIDE );
  stateset->setTextureMode( 0 , GL_TEXTURE_GEN_S , osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

  stateset->setAttribute( material , osg::StateAttribute::OVERRIDE );

  return stateset;
}

// ---------------------------------------------------------
// BUILD MESH
//
// Takes in an image and builds geodes for every triangle strip.
// ---------------------------------------------------------
template <class ViewT>
osg::Node* build_mesh( vw::ImageViewBase<ViewT> const& point_image, const int& step_size, const std::string& init_tex_file, osg::Vec3f& dataNormal, bool light) {

  //const ViewT& point_image_impl = point_image.impl();
  osg::Geode* mesh = new osg::Geode();
  osg::Geometry* geometry = new osg::Geometry();
  osg::Vec3Array* vertices = new osg::Vec3Array();
  osg::Vec2Array* texcoords = new osg::Vec2Array();
  osg::Vec3Array* normals = new osg::Vec3Array();

  dataNormal = osg::Vec3f( 0.0f , 0.0f , 0.0f );

  std::cout << "\t--> Orginal size: [" << point_image.impl().cols() << ", " << point_image.impl().rows() << "]\n";
  std::cout << "\t--> Subsampled:   [" << point_image.impl().cols()/step_size << ", "
            << point_image.impl().rows()/step_size << "]\n";

  //////////////////////////////////////////////////
  // Deciding how to reduce the texture size
  //   Max texture width or height is 4096
  std::string tex_file;
  if ( init_tex_file.size() ) {
    DiskImageView<PixelGray<uint8> > previous_texture(init_tex_file);
    tex_file = prefix_from_pointcloud_filename(init_tex_file) + "-tex";
    if (point_image.impl().cols() > 4096 ||
        point_image.impl().rows() > 4096 ) {
      std::cout << "Resampling to reduce texture size:\n";
      float tex_sub_scale = 4096.0/float(std::max(previous_texture.cols(),previous_texture.rows()));
      ImageViewRef<PixelGray<uint8> > new_texture = resample(previous_texture,tex_sub_scale);
      std::cout << "\t--> Texture size: [" << new_texture.cols() << ", " << new_texture.rows() << "]\n";
      DiskImageResourceGDAL tex_rsrc(tex_file+".tif",new_texture.format(),
                                     Vector2i(256,256) );
      block_write_image(tex_rsrc,new_texture,
                        TerminalProgressCallback("asp","\tSubsampling:") );
    } else {
      // Always saving as an 8bit texture. These second handedly
      // normalizes the data for us (which is a problem for datasets
      // like HiRISE which will feed us tiffs with values outside of
      // 0-1).
      DiskImageResourceGDAL tex_rsrc(tex_file+".tif",previous_texture.format(),
                                     Vector2i(256,256) );
      block_write_image(tex_rsrc, previous_texture,
                        TerminalProgressCallback("asp","\tNormalizing:") );
    }
    // When we subsample, we use tiff because we can block write the image.
    // However, trying to load the tiff with osg causes problems because
    // osg uses libtiff to load the images. libtiff conflicts with gdal
    // if gdal was compiled with internal tiff, and osg will fail to load
    // the texture. To avoid all this, we resave our subsampled texture as a jpg
    DiskImageView<PixelGray<uint8> > new_texture(tex_file+".tif");
    write_image(tex_file+".jpg", new_texture);
    unlink((tex_file+".tif").c_str());
    tex_file += ".jpg";
  }

  //////////////////////////////////////////////////
  /// Setting name of geode
  {
    std::ostringstream os;
    os << "Simple Mesh" << std::endl;
    mesh->setName( os.str() );
  }

  //////////////////////////////////////////////////
  /// PUSHING ALL VERTICES & Also texture coordinates
  {

    // Some constants for the calculation in here
    unsigned num_rows = point_image.impl().rows()/step_size;
    unsigned num_cols = point_image.impl().cols()/step_size;

    TerminalProgressCallback progress("asp", "\tVertices:   ");
    double progress_mult = 1.0/double(num_rows*num_cols);

    for (unsigned r = 0; r < (num_rows); ++r ){
      for (unsigned c = 0; c < (num_cols); ++c ){
        progress.report_progress((r*num_cols+c)*progress_mult);

        unsigned r_step = r * step_size;
        unsigned c_step = c * step_size;

        vertices->push_back( osg::Vec3f( point_image.impl()(c_step,r_step)[0] ,
                                         point_image.impl()(c_step,r_step)[1] ,
                                         point_image.impl()(c_step,r_step)[2] ) );

        // Calculating normals, if the user wants shading
        if (light) {
          Vector3 temp_normal;

          // These calculations seems backwards from what they should
          // be. Its because for the indexing of the image is weird,
          // its column then row.

          // Is quadrant 1 normal calculation possible?
          if ( (r > 0) && ( (c+1) < (num_cols)) ) {
            if ( (point_image.impl()(c_step+step_size,r_step) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step,r_step-step_size) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step+step_size,r_step) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step,r_step-step_size) -
                                                                point_image.impl()(c_step,r_step) ) );
            }
          }
          // Is quadrant 2 normal calculation possible?
          if ( ( (c+1) < (num_cols) ) && ((r+1) < (num_rows))){
            if ( (point_image.impl()(c_step,r_step+step_size) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step+step_size,r_step) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step,r_step+step_size) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step+step_size,r_step) -
                                                                point_image.impl()(c_step,r_step) ) );
            }
          }
          // Is quadrant 3 normal calculation possible?
          if ( ((r+1) < (num_rows)) && (c>0) ) {
            if ( (point_image.impl()(c_step-step_size,r_step) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step,r_step+step_size) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step-step_size,r_step) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step,r_step+step_size) -
                                                                point_image.impl()(c_step,r_step) ) );
            }
          }
          // Is quadrant 4 normal calculation possible?
          if ( (c>0) && (r>0) ) {
            if ( (point_image.impl()(c_step,r_step-step_size) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step-step_size,r_step) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step,r_step-step_size) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step-step_size,r_step) -
                                                                point_image.impl()(c_step,r_step) ) );
            }
          }

          temp_normal = normalize( temp_normal );
          normals->push_back( osg::Vec3f( temp_normal[0],
                                          temp_normal[1],
                                          temp_normal[2] ) );
        }

        if ( tex_file.size() ) {
          texcoords->push_back( osg::Vec2f ( (float)c_step / (float)point_image.impl().cols() ,
                                             1-(float)r_step / (float)point_image.impl().rows() ) );
        } else if ( (point_image.impl()(c_step,r_step)[0] != 0 ) &&
                    (point_image.impl()(c_step,r_step)[1] != 0 ) &&
                    (point_image.impl()(c_step,r_step)[2] != 0 ) ) {
          //I'm calculating the main normal for the data.
          dataNormal[0] += point_image.impl()(c_step,r_step)[0];
          dataNormal[1] += point_image.impl()(c_step,r_step)[1];
          dataNormal[2] += point_image.impl()(c_step,r_step)[2];
        }
      }
    }
    progress.report_finished();

    std::cout << "\t > size: " << vertices->size() << " vertices\n";

    geometry->setVertexArray( vertices );

    if (light)
      geometry->setNormalArray( normals );

    if ( tex_file.size() ) {
      geometry->setTexCoordArray( 0,texcoords );
    } else {
      dataNormal.normalize();
    }

    osg::Vec4Array* colour = new osg::Vec4Array();
    colour->push_back( osg::Vec4f( 1.0f, 1.0f, 1.0f, 1.0f ) );
    geometry->setColorArray( colour );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

  }

  //////////////////////////////////////////////////
  /// Deciding How to draw triangle strips
  std::cout << "Drawing Triangle Strips\n";
  {
    unsigned col_steps = point_image.impl().cols()/step_size;

    for (int r = 0; r < ( point_image.impl().rows()/step_size - 1); ++r){

      bool add_direction_down = true;
      osg::DrawElementsUInt* dui = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP);

      for (unsigned c = 0; c < ( col_steps ); ++c){

        unsigned pointing_index = r*(col_steps) + c;

        //std::cout << "V: " << vertices->at(pointing_index)[0] << " " << vertices->at(pointing_index)[1] << " " << vertices->at(pointing_index)[2] << std::endl;

        if (add_direction_down) {

          // Adding top point ...
          if ( (vertices->at(pointing_index)[0] != 0) &&
               (vertices->at(pointing_index)[1] != 0) &&
               (vertices->at(pointing_index)[2] != 0) ) {

            dui->push_back( pointing_index );
          }

          // Adding bottom point ..
          if ( (vertices->at(pointing_index+col_steps)[0] != 0) &&
               (vertices->at(pointing_index+col_steps)[1] != 0) &&
               (vertices->at(pointing_index+col_steps)[2] != 0) ) {

            dui->push_back( pointing_index+col_steps );
          } else {
            // If there's a drop out here... we switch adding direction.
            add_direction_down = false;
          }

        } else {

          // Adding bottom point ..
          if ( (vertices->at(pointing_index+col_steps)[0] != 0) &&
               (vertices->at(pointing_index+col_steps)[1] != 0) &&
               (vertices->at(pointing_index+col_steps)[2] != 0) ) {

            dui->push_back( pointing_index+col_steps );
          }

          // Adding top point ...
          if ( (vertices->at(pointing_index)[0] != 0) &&
               (vertices->at(pointing_index)[1] != 0) &&
               (vertices->at(pointing_index)[2] != 0) ) {

            dui->push_back( pointing_index );
          } else {
            // If there's a drop out here... we switch adding direction.
            add_direction_down = true;
          }
        }
      }

      geometry->addPrimitiveSet(dui);

    }
  }

  ////////////////////////////////////////////////
  /// Adding texture to the DTM
  if (tex_file.size()){

    std::cout << "Attaching Texture Data\n";

    osg::Image* textureImage = osgDB::readImageFile(tex_file.c_str());

    if ( textureImage ) {
      if ( textureImage->valid() ){
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(textureImage);
        osg::StateSet* stateset = geometry->getOrCreateStateSet();
        stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
      } else {
        std::cout << "Failed to open texture data in " << tex_file << std::endl;
      }
    } else {
      std::cout << "Failed to open texture data in " << tex_file << std::endl;
    }
  }

  mesh->addDrawable( geometry );

  return mesh;

}

// ---------------------------------------------------------
// POINT IMAGE TRANSFORM
// ---------------------------------------------------------
class PointTransFunc : public ReturnFixedType<Vector3> {
  Matrix3x3 m_trans;
public:
  PointTransFunc(Matrix3x3 trans) : m_trans(trans) {}
  Vector3 operator() (Vector3 const& pt) const { return m_trans*pt; }
};

// ---------------------------------------------------------
// POINT IMAGE OFFSET
// ---------------------------------------------------------

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

template <class ViewT>
BBox<float,3> point_image_bbox(ImageViewBase<ViewT> const& point_image) {
  // Compute bounding box
  BBox<float,3> result;
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

// MAIN
// ---------------------------------------------------------

struct Options {
  Options() : root( new osg::Group() ), simplify_percent(0) {};
  // Input
  std::string pointcloud_filename, texture_file_name;

  // Settings
  unsigned step_size;
  osg::ref_ptr<osg::Group> root;
  float simplify_percent;
  osg::Vec3f dataNormal;
  std::string rot_order;
  double phi_rot, omega_rot, kappa_rot;
  bool center, enable_lighting, smooth_mesh, simplify_mesh;

  // Output
  std::string output_prefix, output_file_type;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("simplify-mesh", po::value(&opt.simplify_percent),
       "Run OSG Simplifier on mesh, 1.0 = 100%")
    ("smooth-mesh", "Run OSG Smoother on mesh")
    ("use-delaunay", "Uses the delaunay triangulator to create a surface from the point cloud. This is not recommended for point clouds with serious noise issues.")
    ("step,s", po::value(&opt.step_size)->default_value(10),
       "Step size for mesher, sets the polygons size per point")
    ("output-prefix,o", po::value(&opt.output_prefix),
       "Specify the output prefix")
    ("output-filetype,t",
       po::value(&opt.output_file_type)->default_value("ive"),
       "Specify the output file")
    ("enable-lighting,l", "Enables shades and light on the mesh" )
    ("center", "Center the model around the origin.  Use this option if you are experiencing numerical precision issues.")
    ("rotation-order", po::value(&opt.rot_order)->default_value("xyz"),
       "Set the order of an euler angle rotation applied to the 3D points prior to DEM rasterization")
    ("phi-rotation", po::value(&opt.phi_rot)->default_value(0),
     "Set a rotation angle phi")
    ("omega-rotation", po::value(&opt.omega_rot)->default_value(0),
     "Set a rotation angle omega")
    ("kappa-rotation", po::value(&opt.kappa_rot)->default_value(0),
     "Set a rotation angle kappa")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.pointcloud_filename),
       "Explicitly specify the input file")
    ("texture-file", po::value(&opt.texture_file_name),
       "Explicity specify the texture file");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);
  positional_desc.add("texture-file", 1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <pointcloud> <texture-file> ...\n";

  if ( vm.count("help" ) )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.pointcloud_filename.empty() )
    vw_throw( ArgumentErr() << "Missing point cloud.\n"
              << usage.str() << general_options );
  if ( opt.output_prefix.empty() )
    opt.output_prefix =
      prefix_from_pointcloud_filename( opt.pointcloud_filename );
  opt.center = vm.count("center");
  opt.enable_lighting = vm.count("enable-lighting");
  opt.smooth_mesh = vm.count("smooth-mesh");
  opt.simplify_mesh = vm.count("simplify-mesh");
}

int main( int argc, char *argv[] ){

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Loading point cloud!
    DiskImageView<Vector3> point_disk_image(opt.pointcloud_filename);
    ImageViewRef<Vector3> point_image = point_disk_image;

    // Centering Option (helpful if you are experiencing round-off error...)
    if (opt.center) {
      BBox<float,3> bbox = point_image_bbox(point_disk_image);
      std::cout << "\t--> Centering model around the origin.\n";
      std::cout << "\t    Initial point image bounding box: " << bbox << "\n";
      Vector3 midpoint = (bbox.max() + bbox.min()) / 2.0;
      std::cout << "\t    Midpoint: " << midpoint << "\n";
      point_image = point_image_offset(point_image, -midpoint);
      BBox<float,3> bbox2 = point_image_bbox(point_image);
      std::cout << "\t    Re-centered point image bounding box: " << bbox2 << "\n";
    }

    // Applying option rotations before hand
    if ( opt.phi_rot != 0 || opt.omega_rot != 0 || opt.kappa_rot != 0 ) {
      std::cout << "Applying rotation sequence: " << opt.rot_order
                << "\tAngles: " << opt.phi_rot << "   " << opt.omega_rot
                << "   " << opt.kappa_rot << std::endl;
      Matrix3x3 rotation_trans =
        math::euler_to_rotation_matrix( opt.phi_rot, opt.omega_rot,
                                        opt.kappa_rot, opt.rot_order );
      point_image =
        vw::per_pixel_filter(point_image, PointTransFunc( rotation_trans ) );
    }

    // Building Mesh
    std::cout << "\nGenerating 3D mesh from point cloud:\n";
    {
      opt.root->addChild(build_mesh(point_image, opt.step_size, opt.texture_file_name, opt.dataNormal, opt.enable_lighting ));

      if ( !opt.texture_file_name.empty() ) {
        // Turning off lighting and other likes
        osg::StateSet* stateSet = new osg::StateSet();
        if ( !opt.enable_lighting )
          stateSet->setMode( GL_LIGHTING , osg::StateAttribute::OFF );
        stateSet->setMode( GL_BLEND , osg::StateAttribute::ON );
        opt.root->setStateSet( stateSet );

      } else {
        std::cout << "Adding contour coloring\n";
        osg::StateSet* stateSet =
          create1DTexture( opt.root.get() , opt.dataNormal );
        if ( !opt.enable_lighting )
          stateSet->setMode( GL_LIGHTING , osg::StateAttribute::OFF );
        stateSet->setMode( GL_BLEND , osg::StateAttribute::ON );
        opt.root->setStateSet( stateSet );
      }
    }

    // Smooth Option
    if ( opt.smooth_mesh ) {
      std::cout << "Smoothing Data\n";
      osgUtil::SmoothingVisitor sv;
      opt.root->accept(sv);
    }

    // Simplify Option
    if ( opt.simplify_mesh ) {
      if ( opt.simplify_percent == 0.0 )
        opt.simplify_percent = 1.0;

      std::cout << "Simplifying Data\n";
      osgUtil::Simplifier simple;
      simple.setSmoothing( opt.smooth_mesh );
      simple.setSampleRatio( opt.simplify_percent );
      opt.root->accept(simple);
    }

    // Optimizing Data
    {
      std::cout << "Optimizing Data\n";
      osgUtil::Optimizer optimizer;
      optimizer.optimize( opt.root.get() );
    }

    // Saving Data
    {
      std::cout << "\nSaving Data:\n";
      std::ostringstream os;
      os << opt.output_prefix << "." << opt.output_file_type;
      osgDB::writeNodeFile( *opt.root.get() , os.str() );
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
