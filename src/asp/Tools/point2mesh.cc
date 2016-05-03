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


/// \file point2mesh2.cc
///

//Standard Stuff
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>

//VisionWorkbench & ASP
#include <vw/Image/Transform.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Image/MaskViews.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;

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
#include <osgSim/Version>
#include <osgFX/Version>
#include <osgTerrain/Version>
#include <osgVolume/Version>


// ---------------------------------------------------------
// UTILITIES
// ---------------------------------------------------------

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : vw::cartography::GdalWriteOptions {
  Options() : root( new osg::Group() ), simplify_percent(0) {};
  // Input
  std::string pointcloud_filename, texture_file_name;

  // Settings
  uint32 step_size;
  osg::ref_ptr<osg::Group> root;
  float simplify_percent;
  osg::Vec3f dataNormal;
  std::string rot_order;
  double phi_rot, omega_rot, kappa_rot;
  bool center, enable_lighting, smooth_mesh, simplify_mesh;
  std::string osg_version;

  // Output
  std::string output_prefix, output_file_type;
};

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
  vw_out() << "Direction Vector being used " << Direction.x() << " " << Direction.y() << " " << Direction.z() << std::endl;
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
osg::Node* build_mesh( vw::ImageViewBase<ViewT> const& point_image,
                       Options& opt ) {

  //const ViewT& point_image_impl = point_image.impl();
  osg::Geode* mesh = new osg::Geode();
  osg::Geometry* geometry = new osg::Geometry();
  osg::Vec3Array* vertices = new osg::Vec3Array();
  osg::Vec2Array* texcoords = new osg::Vec2Array();
  osg::Vec3Array* normals = new osg::Vec3Array();

  opt.dataNormal = osg::Vec3f( 0.0f , 0.0f , 0.0f );

  vw_out() << "\t--> Orginal size: [" << point_image.impl().cols() << ", " << point_image.impl().rows() << "]\n";
  vw_out() << "\t--> Subsampled:   [" << point_image.impl().cols()/opt.step_size << ", "
            << point_image.impl().rows()/opt.step_size << "]\n";

  //////////////////////////////////////////////////
  // Deciding how to reduce the texture size
  //   Max texture width or height is 4096
  std::string tex_file;
  if ( opt.texture_file_name.size() ) {
    DiskImageView<PixelGray<uint8> > previous_texture(opt.texture_file_name);
    tex_file = asp::prefix_from_pointcloud_filename(opt.output_prefix) + "-tex";
    if (point_image.impl().cols() > 4096 ||
        point_image.impl().rows() > 4096 ) {
      vw_out() << "Resampling to reduce texture size:\n";
      float tex_sub_scale = 4096.0/float(std::max(previous_texture.cols(),previous_texture.rows()));
      ImageViewRef<PixelGray<uint8> > new_texture = resample(previous_texture,tex_sub_scale);
      vw_out() << "\t--> Texture size: [" << new_texture.cols() << ", " << new_texture.rows() << "]\n";
      vw_out() << "Writing temporary file: " << tex_file+".tif" << "\n";
      vw::cartography::block_write_gdal_image( tex_file+".tif", new_texture, opt,
                                   TerminalProgressCallback("asp","\tSubsampling:") );
    } else {
      // Always saving as an 8bit texture. These second handedly
      // normalizes the data for us (which is a problem for datasets
      // like HiRISE which will feed us tiffs with values outside of
      // 0-1).
      vw_out() << "Writing temporary file: " << tex_file+".tif" << "\n";
      vw::cartography::block_write_gdal_image( tex_file+".tif", previous_texture, opt,
                                   TerminalProgressCallback("asp","\tNormalizing:") );
    }
    // When we subsample, we use tiff because we can block write the image.
    // However, trying to load the tiff with osg causes problems because
    // osg uses libtiff to load the images. libtiff conflicts with gdal
    // if gdal was compiled with internal tiff, and osg will fail to load
    // the texture. To avoid all this, we resave our subsampled texture as a jpg
    DiskImageView<PixelGray<uint8> > new_texture(tex_file+".tif");
    vw_out() << "Writing temporary file: " << tex_file+".jpg" << "\n";
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
    uint32 num_rows = point_image.impl().rows()/opt.step_size;
    uint32 num_cols = point_image.impl().cols()/opt.step_size;

    TerminalProgressCallback progress("asp", "\tVertices:   ");
    double progress_mult = 1.0/double(num_rows*num_cols);

    for ( uint32 r = 0; r < (num_rows); ++r ){
      for (uint32 c = 0; c < (num_cols); ++c ){
        progress.report_progress((r*num_cols+c)*progress_mult);

        uint32 r_step = r * opt.step_size;
        uint32 c_step = c * opt.step_size;

        vertices->push_back( osg::Vec3f( point_image.impl()(c_step,r_step)[0] ,
                                         point_image.impl()(c_step,r_step)[1] ,
                                         point_image.impl()(c_step,r_step)[2] ) );

        // Calculating normals, if the user wants shading
        if (opt.enable_lighting) {
          Vector3 temp_normal;

          // These calculations seems backwards from what they should
          // be. Its because for the indexing of the image is weird,
          // its column then row.

          // Is quadrant 1 normal calculation possible?
          if ( (r > 0) && ( (c+1) < (num_cols)) ) {
            if ( (point_image.impl()(c_step+opt.step_size,r_step) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step,r_step-opt.step_size) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step+opt.step_size,r_step) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step,r_step-opt.step_size) -
                                                                point_image.impl()(c_step,r_step) ) );
            }
          }
          // Is quadrant 2 normal calculation possible?
          if ( ( (c+1) < (num_cols) ) && ((r+1) < (num_rows))){
            if ( (point_image.impl()(c_step,r_step+opt.step_size) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step+opt.step_size,r_step) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step,r_step+opt.step_size) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step+opt.step_size,r_step) -
                                                                point_image.impl()(c_step,r_step) ) );
            }
          }
          // Is quadrant 3 normal calculation possible?
          if ( ((r+1) < (num_rows)) && (c>0) ) {
            if ( (point_image.impl()(c_step-opt.step_size,r_step) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step,r_step+opt.step_size) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step-opt.step_size,r_step) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step,r_step+opt.step_size) -
                                                                point_image.impl()(c_step,r_step) ) );
            }
          }
          // Is quadrant 4 normal calculation possible?
          if ( (c>0) && (r>0) ) {
            if ( (point_image.impl()(c_step,r_step-opt.step_size) != Vector3(0,0,0)) &&
                 (point_image.impl()(c_step-opt.step_size,r_step) != Vector3(0,0,0)) ) {
              temp_normal = temp_normal + normalize(cross_prod( point_image.impl()(c_step,r_step-opt.step_size) -
                                                                point_image.impl()(c_step,r_step),
                                                                point_image.impl()(c_step-opt.step_size,r_step) -
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
          opt.dataNormal[0] += point_image.impl()(c_step,r_step)[0];
          opt.dataNormal[1] += point_image.impl()(c_step,r_step)[1];
          opt.dataNormal[2] += point_image.impl()(c_step,r_step)[2];
        }
      }
    }
    progress.report_finished();

    vw_out() << "\t > size: " << vertices->size() << " vertices\n";

    geometry->setVertexArray( vertices );

    if (opt.enable_lighting)
      geometry->setNormalArray( normals );

    if ( tex_file.size() ) {
      geometry->setTexCoordArray( 0,texcoords );
    } else {
      opt.dataNormal.normalize();
    }

    osg::Vec4Array* colour = new osg::Vec4Array();
    colour->push_back( osg::Vec4f( 1.0f, 1.0f, 1.0f, 1.0f ) );
    geometry->setColorArray( colour );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

  }

  //////////////////////////////////////////////////
  // Deciding How to draw triangle strips
  vw_out() << "Drawing triangle strips\n";
  {
    uint32 col_steps = point_image.impl().cols()/opt.step_size;

    for (uint32 r = 0;
         r < ( point_image.impl().rows()/opt.step_size - 1); ++r){

      bool add_direction_down = true;
      osg::DrawElementsUInt* dui = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP);

      for (uint32 c = 0; c < ( col_steps ); ++c){

        uint32 pointing_index = r*(col_steps) + c;

        //vw_out() << "V: " << vertices->at(pointing_index)[0] << " " << vertices->at(pointing_index)[1] << " " << vertices->at(pointing_index)[2] << std::endl;

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

    vw_out() << "Attaching texture data\n";

    osg::Image* textureImage = osgDB::readImageFile(tex_file.c_str());

    if ( textureImage ) {
      if ( textureImage->valid() ){
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(textureImage);
        osg::StateSet* stateset = geometry->getOrCreateStateSet();
        stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
      } else {
        vw_out() << "Failed to open texture data in " << tex_file << std::endl;
      }
    } else {
      vw_out() << "Failed to open texture data in " << tex_file << std::endl;
    }
  }

  mesh->addDrawable( geometry );

  return mesh;

}

// MAIN
// ---------------------------------------------------------

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("simplify-mesh", po::value(&opt.simplify_percent),
       "Run OSG Simplifier on mesh, 1.0 = 100%")
    ("smooth-mesh", po::bool_switch(&opt.smooth_mesh)->default_value(false),
     "Run OSG Smoother on mesh")
    ("use-delaunay", "Uses the delaunay triangulator to create a surface from the point cloud. This is not recommended for point clouds with serious noise issues.")
    ("step,s", po::value(&opt.step_size)->default_value(10),
     "Step size for mesher, sets the polygons size per point")
    ("output-prefix,o", po::value(&opt.output_prefix),
     "Specify the output prefix.")
    ("output-filetype,t",
     po::value(&opt.output_file_type)->default_value("osgb"),
     "Specify the output file")
    ("enable-lighting,l",
     po::bool_switch(&opt.enable_lighting)->default_value(false),
     "Enables shades and lighting on the mesh" )
    ("center", po::bool_switch(&opt.center)->default_value(false),
     "Center the model around the origin. Use this option if you are experiencing numerical precision issues.");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.pointcloud_filename),
       "Explicitly specify the input file")
    ("texture-file", po::value(&opt.texture_file_name),
       "Explicity specify the texture file");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);
  positional_desc.add("texture-file", 1);

  std::string usage("[options] <pointcloud> <texture-file>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.pointcloud_filename.empty() )
    vw_throw( ArgumentErr() << "Missing point cloud.\n"
              << usage << general_options );
  if ( opt.output_prefix.empty() )
    opt.output_prefix =
      asp::prefix_from_pointcloud_filename( opt.pointcloud_filename );

  // Create the output directory
  vw::create_out_dir(opt.output_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.output_prefix);

  opt.simplify_mesh = vm.count("simplify-mesh");

  // The purpose of this is to force ASP to link to the OSG libraries
  // at link-time, otherwise it fails to find them at run-time
  // due to peculiarities in OSG's functionality for library search.
  opt.osg_version = std::string("")
    + osgSimGetVersion()
    + osgFXGetVersion()
    + osgTerrainGetVersion();
    + osgVolumeGetVersion();
}

int main( int argc, char *argv[] ){

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    std::string input_file = opt.pointcloud_filename;
    int num_channels = get_num_channels(input_file);
    GeoReference georef;
    bool has_georef = read_georeference(georef, input_file);
    double nodata_val = -std::numeric_limits<double>::max();
    vw::read_nodata_val(input_file, nodata_val);

    // Loading point cloud
    ImageViewRef<Vector3> point_image;
    if (num_channels == 1 && has_georef) {
      // The input is a DEM. Convert it to a point cloud.
      DiskImageView<double> dem(input_file);
      point_image = geodetic_to_cartesian(dem_to_geodetic
                                          (create_mask(dem, nodata_val), georef),
                                          georef.datum());
    }else if (num_channels >= 3){
      // The input DEM is a point cloud
      point_image = asp::read_asp_point_cloud<3>(input_file);
    }else{
      vw_throw( ArgumentErr() << "The input must be a point cloud or a DEM.\n");
    }

    // Centering Option (helpful if you are experiencing round-off error...)
    if (opt.center) {
      bool is_geodetic = false; // raw xyz values
      BBox3 bbox = asp::pointcloud_bbox(point_image, is_geodetic);
      vw_out() << "\t--> Centering model around the origin.\n";
      vw_out() << "\t    Initial point image bounding box: " << bbox << "\n";
      Vector3 midpoint = (bbox.max() + bbox.min()) / 2.0;
      vw_out() << "\t    Midpoint: " << midpoint << "\n";
      point_image = asp::point_image_offset(point_image, -midpoint);
    }

    {
      vw_out() << "\nGenerating 3D mesh from point cloud:\n";
      opt.root->addChild(build_mesh(point_image, opt));

      if ( !opt.texture_file_name.empty() ) {
        // Turning off lighting and other likes
        osg::StateSet* stateSet = new osg::StateSet();
        if ( !opt.enable_lighting )
          stateSet->setMode( GL_LIGHTING , osg::StateAttribute::OFF );
        stateSet->setMode( GL_BLEND , osg::StateAttribute::ON );
        opt.root->setStateSet( stateSet );

      } else {
        vw_out() << "Adding contour coloring\n";
        osg::StateSet* stateSet =
          create1DTexture( opt.root.get() , opt.dataNormal );
        if ( !opt.enable_lighting )
          stateSet->setMode( GL_LIGHTING , osg::StateAttribute::OFF );
        stateSet->setMode( GL_BLEND , osg::StateAttribute::ON );
        opt.root->setStateSet( stateSet );
      }
    }

    if ( opt.smooth_mesh ) {
      vw_out() << "Smoothing data\n";
      osgUtil::SmoothingVisitor sv;
      opt.root->accept(sv);
    }

    if ( opt.simplify_mesh ) {
      if ( opt.simplify_percent == 0.0 )
        opt.simplify_percent = 1.0;

      vw_out() << "Simplifying data\n";
      osgUtil::Simplifier simple;
      simple.setSmoothing( opt.smooth_mesh );
      simple.setSampleRatio( opt.simplify_percent );
      opt.root->accept(simple);
    }

    {
      vw_out() << "Optimizing data\n";
      osgUtil::Optimizer optimizer;
      optimizer.optimize( opt.root.get() );
    }

    {
      std::ostringstream os;
      os << opt.output_prefix << "." << opt.output_file_type;
      vw_out() << "Writing: " << os.str() << "\n";
      osgDB::writeNodeFile( *opt.root.get() , os.str(),
                            new osgDB::Options("WriteImageHint=IncludeData Compressor=zlib"));
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
