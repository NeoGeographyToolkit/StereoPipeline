// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
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
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Math/Vector.h>
#include <vw/Math.h>
#include <vw/Stereo.h>
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
osg::Node* build_mesh( vw::ImageViewBase<ViewT> const& point_image, const int& step_size, const std::string& tex_file, osg::Vec3f& dataNormal, bool light) {

  const ViewT& point_image_impl = point_image.impl();
  osg::Geode* mesh = new osg::Geode();
  osg::Geometry* geometry = new osg::Geometry();
  osg::Vec3Array* vertices = new osg::Vec3Array();
  osg::Vec2Array* texcoords = new osg::Vec2Array();
  osg::Vec3Array* normals = new osg::Vec3Array();

  dataNormal = osg::Vec3f( 0.0f , 0.0f , 0.0f );

  std::cout << "In Rows: " << point_image_impl.rows() << " Cols: " << point_image_impl.cols() << std::endl;

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
    std::cout << "Creating Vertice Data\n";

    // Some constants for the calculation in here
    unsigned num_rows = point_image_impl.rows()/step_size;
    unsigned num_cols = point_image_impl.cols()/step_size;

    for (unsigned r = 0; r < (num_rows); ++r ){
      for (unsigned c = 0; c < (num_cols); ++c ){
	
	unsigned r_step = r * step_size;
	unsigned c_step = c * step_size;

	vertices->push_back( osg::Vec3f( point_image_impl.impl()(c_step,r_step)[0] ,
					 point_image_impl.impl()(c_step,r_step)[1] ,
					 point_image_impl.impl()(c_step,r_step)[2] ) );

	// Calculating normals, if the user wants shading
	if (light) {
	  Vector3 temp_normal;
	  
	  // These calculations seems backwards from what they should
	  // be. Its because for the indexing of the image is weird,
	  // its column then row.

	  // Is quadrant 1 normal calculation possible?
	  if ( (r > 0) && ( (c+1) < (num_cols)) ) {
	    if ( (point_image_impl.impl()(c_step+step_size,r_step) != Vector3(0,0,0)) &&
		 (point_image_impl.impl()(c_step,r_step-step_size) != Vector3(0,0,0)) ) {
	      temp_normal = temp_normal + normalize(cross_prod( point_image_impl.impl()(c_step+step_size,r_step) -
								point_image_impl.impl()(c_step,r_step),
								point_image_impl.impl()(c_step,r_step-step_size) - 
								point_image_impl.impl()(c_step,r_step) ) );
	    }
	  }
	  // Is quadrant 2 normal calculation possible?
	  if ( ( (c+1) < (num_cols) ) && ((r+1) < (num_rows))){
	    if ( (point_image_impl.impl()(c_step,r_step+step_size) != Vector3(0,0,0)) &&
		 (point_image_impl.impl()(c_step+step_size,r_step) != Vector3(0,0,0)) ) {
	      temp_normal = temp_normal + normalize(cross_prod( point_image_impl.impl()(c_step,r_step+step_size) -
								point_image_impl.impl()(c_step,r_step),
								point_image_impl.impl()(c_step+step_size,r_step) -
								point_image_impl.impl()(c_step,r_step) ) );
	    }
	  }
	  // Is quadrant 3 normal calculation possible?
	  if ( ((r+1) < (num_rows)) && (c>0) ) {
	    if ( (point_image_impl.impl()(c_step-step_size,r_step) != Vector3(0,0,0)) &&
		 (point_image_impl.impl()(c_step,r_step+step_size) != Vector3(0,0,0)) ) {
	      temp_normal = temp_normal + normalize(cross_prod( point_image_impl.impl()(c_step-step_size,r_step) -
								point_image_impl.impl()(c_step,r_step),
								point_image_impl.impl()(c_step,r_step+step_size) -
								point_image_impl.impl()(c_step,r_step) ) );
	    }
	  }
	  // Is quadrant 4 normal calculation possible?
	  if ( (c>0) && (r>0) ) {
	    if ( (point_image_impl.impl()(c_step,r_step-step_size) != Vector3(0,0,0)) &&
		 (point_image_impl.impl()(c_step-step_size,r_step) != Vector3(0,0,0)) ) {
	      temp_normal = temp_normal + normalize(cross_prod( point_image_impl.impl()(c_step,r_step-step_size) -
								point_image_impl.impl()(c_step,r_step),
								point_image_impl.impl()(c_step-step_size,r_step) -
								point_image_impl.impl()(c_step,r_step) ) );
	    }
	  }
	  
	  temp_normal = normalize( temp_normal );
	  normals->push_back( osg::Vec3f( temp_normal[0],
					  temp_normal[1],
					  temp_normal[2] ) );
	}
      
	if ( tex_file.size() ) {
	  texcoords->push_back( osg::Vec2f ( (float)c_step / (float)point_image_impl.cols() ,
					     1-(float)r_step / (float)point_image_impl.rows() ) );
	  //texcoords->push_back( osg::Vec2f ( (float)r / (float)point_image_impl.rows() , 
	  //				     1 - (float)c / (float)point_image_impl.cols() ) );
	} else if ( (point_image_impl.impl()(c_step,r_step)[0] != 0 ) &&
		    (point_image_impl.impl()(c_step,r_step)[1] != 0 ) &&
		    (point_image_impl.impl()(c_step,r_step)[2] != 0 ) ) {
	  //I'm calculating the main normal for the data.
	  dataNormal[0] += point_image_impl.impl()(c_step,r_step)[0];
	  dataNormal[1] += point_image_impl.impl()(c_step,r_step)[1];
	  dataNormal[2] += point_image_impl.impl()(c_step,r_step)[2];
	}
      }
    }

    std::cout << " > size: " << vertices->size() << std::endl;

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
    unsigned col_steps = point_image_impl.cols()/step_size;

    for (int r = 0; r < ( point_image_impl.rows()/step_size - 1); ++r){

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

      //std::cout << "Adding a " << dui->getNumIndices() << " .... yeah\n";
      geometry->addPrimitiveSet(dui);

    }
  }

  ////////////////////////////////////////////////
  /// Adding texture to the DTM
  if (tex_file.size()){

    std::cout << "Attaching Texture Data\n";

    osg::Image* textureImage = osgDB::readImageFile(tex_file.c_str());

    if ( textureImage->valid() ){
      osg::Texture2D* texture = new osg::Texture2D;
      texture->setImage(textureImage);
      osg::StateSet* stateset = geometry->getOrCreateStateSet();
      stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    } else {
      std::cout << "Failed to open texture data in " << tex_file << std::endl;
    }
  }

  mesh->addDrawable( geometry );

  std::cout << "...Done!\n\n";
  
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

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------

int main( int argc, char *argv[] ){
  //Main Variables
  set_debug_level(VerboseDebugMessage+11);
  std::string pointcloud_filename, output_prefix = "", output_file_type, texture_file_name;
  unsigned step_size, cache_size;
  osg::ref_ptr<osg::Group> root = new osg::Group();
  float simplify_percent = 0.0;
  osg::Vec3f dataNormal;
  std::string rot_order;
  double phi_rot, omega_rot, kappa_rot;

  //Boost Program options
  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("cache",             po::value<unsigned>(&cache_size)->default_value(2048), 
       "Cache size, in megabytes")
    ("simplify-mesh",     po::value<float>(&simplify_percent), 
       "Run OSG Simplifier on mesh, 1.0 = 100%")
    ("smooth-mesh", 
       "Run OSG Smoother on mesh")
    ("use-delaunay", 
       "Uses the delaunay triangulator to create a surface from the point cloud. This is not recommended for point clouds with serious noise issues.")
    ("step,s",            po::value<unsigned>(&step_size)->default_value(10), 
       "Step size for mesher, sets the polygons size per point")
    ("input-file",        po::value<std::string>(&pointcloud_filename), 
       "Explicitly specify the input file")
    ("texture-file",      po::value<std::string>(&texture_file_name), 
       "Explicity specify the texture file")
    ("output-prefix,o",   po::value<std::string>(&output_prefix), 
       "Specify the output prefix")
    ("output-filetype,t", po::value<std::string>(&output_file_type)->default_value("ive"), 
       "Specify the output file")
    ("enable-lighting,l",
     "Enables shades and light on the mesh" )
    ("center", "Center the model around the origin.  Use this option if you are experiencing numerical precision issues.")
    ("rotation-order",    po::value<std::string>(&rot_order)->default_value("xyz"),
       "Set the order of an euler angle rotation applied to the 3D points prior to DEM rasterization")
    ("phi-rotation",      po::value<double>(&phi_rot)->default_value(0), 
       "Set a rotation angle phi")
    ("omega-rotation",    po::value<double>(&omega_rot)->default_value(0), 
       "Set a rotation angle omega")
    ("kappa-rotation",    po::value<double>(&kappa_rot)->default_value(0), 
       "Set a rotation angle kappa");

  po::positional_options_description p;
  p.add("input-file",   1);
  p.add("texture-file", 1);
  
  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <input-file> <texture-file> ... " << std::endl << std::endl;
  usage << desc << std::endl;

  //Setting the Vision Workbench cache size?
  vw_system_cache().resize( cache_size*1024*1024 );

  if( vm.count("help") ) {
    std::cout << desc << std::endl;
    return 1;
  } else if ( vm.count("input-file") != 1 ) {
    std::cout << usage.str();
    return 1;
  }

  if( output_prefix == "" ) {
    output_prefix = prefix_from_pointcloud_filename(pointcloud_filename);
  }

  // Loading point cloud!
  DiskImageView<Vector3> point_disk_image(pointcloud_filename);
  ImageViewRef<Vector3> point_image = point_disk_image;

  // Centering Option (helpful if you are experiencing round-off error...)
  if (vm.count("center")) {
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
  if ( phi_rot != 0 || omega_rot != 0 || kappa_rot != 0 ) {
    std::cout << "Applying rotation sequence: " << rot_order << "\tAngles: " << phi_rot << "   " << omega_rot << "   " << kappa_rot << std::endl;
    Matrix3x3 rotation_trans = math::euler_to_rotation_matrix( phi_rot, omega_rot , kappa_rot , rot_order );
    point_image = vw::per_pixel_filter(point_image, PointTransFunc( rotation_trans ) );
  }

  // Building Mesh
  std::cout << "\nGenerating 3D mesh from point cloud:\n";
  {
    root->addChild(build_mesh(point_image, step_size, texture_file_name , dataNormal, vm.count("enable-lighting") ));

    if ( vm.count( "texture-file" ) ) {

      // Turning off lighting and other likes
      osg::StateSet* stateSet = new osg::StateSet();
      if ( !vm.count("enable-lighting") )
	stateSet->setMode( GL_LIGHTING , osg::StateAttribute::OFF );
      stateSet->setMode( GL_BLEND , osg::StateAttribute::ON );
      root->setStateSet( stateSet );

    } else {

      std::cout << "Adding contour coloring\n";
      osg::StateSet* stateSet = create1DTexture( root.get() , dataNormal );
      if ( !vm.count("enable-lighting") )
	stateSet->setMode( GL_LIGHTING , osg::StateAttribute::OFF );
      stateSet->setMode( GL_BLEND , osg::StateAttribute::ON );
      root->setStateSet( stateSet );

    }
  }

  // Smooth Option
  if ( vm.count( "smooth-mesh" ) ) {
    
    std::cout << "Smoothing Data\n";
    osgUtil::SmoothingVisitor sv;
    root->accept(sv);

  }

  // Simplify Option
  if ( vm.count( "simplify-mesh" ) ) {
    
    if ( simplify_percent == 0.0 )
      simplify_percent = 1.0;

    std::cout << "Simplifying Data\n";
    osgUtil::Simplifier simple;
    simple.setSmoothing( vm.count( "smooth-mesh" ) );
    simple.setSampleRatio( simplify_percent );
    root->accept(simple);

  }

  // Optimizing Data
  {
    std::cout << "Optimizing Data\n";
    osgUtil::Optimizer optimizer;
    optimizer.optimize( root.get() );
  }

  // Saving Data
  std::cout << "\nSaving Data:\n";
  {
    std::ostringstream os;
    os << output_prefix << "." << output_file_type;
    osgDB::writeNodeFile( *root.get() , os.str() );

    //osgViewer::Viewer viewer;
    //viewer.setSceneData(root.get());
    //viewer.realize();
    //viewer.run();
  }
  
}
