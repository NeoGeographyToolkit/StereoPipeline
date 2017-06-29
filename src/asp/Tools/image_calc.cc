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


/// \file wv_correct.cc
///

// Correct CCD artifacts in WorldView 1 and 2 images with given TDI.

// The problem: A WV image is obtained by mosaicking from left to
// right image blocks which are as tall is the entire image (each
// block comes from an individual CCD image sensor). Blocks are
// slightly misplaced in respect to each other by some unknown
// subpixel offsets. We use tabulated values for the offsets and their
// locations to undo them.

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <ogrsf_frmts.h>

namespace po = boost::program_options;
using namespace vw;
using namespace asp;
using namespace vw::cartography;
using namespace std;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}


struct Options : vw::cartography::GdalWriteOptions {
  std::string camera_image_file, camera_model_file, output_image; 
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("camera-image", po::value(&opt.camera_image_file))
    ("camera-model", po::value(&opt.camera_model_file))
    ("output-image", po::value(&opt.output_image));
  
  po::positional_options_description positional_desc;
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-image",1);
  
  std::string usage("[options] <camera-image> <camera-model> <output-image>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( !vm.count("camera-image") || !vm.count("camera-model") ||
       !vm.count("output-image") )
    vw_throw( ArgumentErr() << "Requires <camera-image>, <camera-model> "
              << "and <output-image> in order to proceed.\n\n"
              << usage << general_options );

  vw::create_out_dir(opt.output_image);
  
}

int main( int argc, char *argv[] ) {

//   Options opt;
//   try {
//     handle_arguments( argc, argv, opt );

  GDALAllRegister();
  GDALDataset       *poDS;
  poDS = (GDALDataset*) GDALOpenEx( "point7.shp", GDAL_OF_VECTOR, NULL, NULL, NULL );
  if( poDS == NULL )
    {
      printf( "Open failed.\n" );
      exit( 1 );
    }
  OGRLayer  *poLayer;
  poLayer = poDS->GetLayerByName( "point7" );

  int nGeomFieldCount =
    poLayer->GetLayerDefn()->GetGeomFieldCount();
  std::cout << "geom field count " << nGeomFieldCount << std::endl;

  char    *pszWKT;
  if( nGeomFieldCount > 1 )
    {
      for(int iGeom = 0;iGeom < nGeomFieldCount; iGeom ++ )
        {
          OGRGeomFieldDefn* poGFldDefn =
            poLayer->GetLayerDefn()->GetGeomFieldDefn(iGeom);
          OGRSpatialReference* poSRS = poGFldDefn->GetSpatialRef();
          if( poSRS == NULL )
            pszWKT = CPLStrdup( "(unknown)" );
          else
            {
              poSRS->exportToPrettyWkt( &pszWKT );
            }
          
          printf( "SRS WKT (%s):\n%s\n",
                  poGFldDefn->GetNameRef(), pszWKT );
          CPLFree( pszWKT );
        }
    }
  else
    {
      if( poLayer->GetSpatialRef() == NULL )
        pszWKT = CPLStrdup( "(unknown)" );
      else
        {
          poLayer->GetSpatialRef()->exportToPrettyWkt( &pszWKT );
        }
      
      printf( "Layer SRS WKT:\n%s\n", pszWKT );
      CPLFree( pszWKT );
    }
  
  
  OGRFeature *poFeature;
  poLayer->ResetReading();
  while( (poFeature = poLayer->GetNextFeature()) != NULL )
    {
      OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
      int iField;
      for( iField = 0; iField < poFDefn->GetFieldCount(); iField++ )
        {
          OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn( iField );
          if( poFieldDefn->GetType() == OFTInteger )
            printf( "%d,", poFeature->GetFieldAsInteger( iField ) );
          else if( poFieldDefn->GetType() == OFTInteger64 )
            printf( CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64( iField ) );
          else if( poFieldDefn->GetType() == OFTReal )
            printf( "%.3f,", poFeature->GetFieldAsDouble(iField) );
          else if( poFieldDefn->GetType() == OFTString )
            printf( "%s,", poFeature->GetFieldAsString(iField) );
          else
            printf( "%s,", poFeature->GetFieldAsString(iField) );
        }
      OGRGeometry *poGeometry;
      poGeometry = poFeature->GetGeometryRef();
      if( poGeometry != NULL
          && wkbFlatten(poGeometry->getGeometryType()) == wkbPoint )
        {
          OGRPoint *poPoint = (OGRPoint *) poGeometry;
          printf( "%.3f,%3.f\n", poPoint->getX(), poPoint->getY() );
        }
      else if( poGeometry != NULL
               && wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon ){
        std::cout << "---polygon " << std::endl;
        //OGRPoint *poPoint = (OGRPoint *) poGeometry;
        //printf( "%.3f,%3.f\n", poPoint->getX(), poPoint->getY() );
        FILE * fp = NULL;
        char ** papszOptions = NULL;
        poGeometry->dumpReadable( fp, "", papszOptions );
        std::cout << "--end dumpreadable polygon" << std::endl;
      }else{
        printf( "no point geometry\n" );
      }
      OGRFeature::DestroyFeature( poFeature );
    }
  GDALClose( poDS );
    
  return 0;
}

