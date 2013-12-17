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

// Correct CCD artifacts in WorldView2 images with TDI 16.

// To do:
// 1. Rm unnecessary digits from x and y offset values.
// 2. Make the code multi-threaded.
// 3. Add documentation.

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/DG/XML.h>
#include <asp/Sessions/RPC/RPCModel.h>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>

namespace po = boost::program_options;
using namespace vw;
using namespace asp;
using namespace vw::cartography;
using namespace xercesc;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : asp::BaseOptions {
  // Input
  double xoffset, yoffset;
  double xoffset_forward, yoffset_forward;
  double xoffset_reverse, yoffset_reverse;
  
  std::string camera_image_file, camera_model_file, output_image; 
  Options(){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("");
  general_options.add_options()
    ("xoffset", po::value(&opt.xoffset), "Specify the CCD offset correction to apply in the x direction.")
    ("yoffset", po::value(&opt.yoffset), "Specify the CCD offset correction to apply in the y direction.");
  general_options.add( asp::BaseOptionsDescription(opt) );
  
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
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( !vm.count("camera-image") || !vm.count("camera-model") || !vm.count("output-image") )
    vw_throw( ArgumentErr() << "Requires <camera-image>, <camera-model> and <output-image> "
              << "in order to proceed.\n\n"
              << usage << general_options );


  double default_xoffset_forward = 0.284177215039376;
  double default_yoffset_forward = 0.23692707872382;
  double default_xoffset_reverse = 0.339580131804236;
  double default_yoffset_reverse = 0.372509828912177;
  if (vm.count("xoffset")){
    opt.xoffset_forward = opt.xoffset;
    opt.xoffset_reverse = opt.xoffset;
  }else{
    opt.xoffset_forward = default_xoffset_forward;
    opt.xoffset_reverse = default_xoffset_reverse;
  }
  if (vm.count("yoffset")){
    opt.yoffset_forward = opt.yoffset;
    opt.yoffset_reverse = opt.yoffset;
  }else{
    opt.yoffset_forward = default_yoffset_forward;
    opt.yoffset_reverse = default_yoffset_reverse;
  }
  
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    std::cout << "data is " << opt.camera_image_file << ' ' << opt.camera_model_file << ' ' << opt.output_image << std::endl;
    
    GeometricXML geo;
    AttitudeXML att;
    EphemerisXML eph;
    ImageXML img;
    RPCXML rpc;
    std::string scan_dir;
    try{
      XMLPlatformUtils::Initialize();
      read_xml( opt.camera_model_file, geo, att, eph, img, rpc );
      
      scan_dir = boost::to_lower_copy( img.scan_direction );
      if (scan_dir != "forward" && scan_dir != "reverse"){
        vw_throw( ArgumentErr() << "XML file \"" << opt.camera_model_file
                  << "\" is lacking a valid image scan direction.\n" );
      }
      if (geo.detector_pixel_pitch <= 0.0){
        vw_throw( ArgumentErr() << "XML file \"" << opt.camera_model_file
                  << "\" has a non-positive pixel pitch.\n" );
      }
    }catch(...){
      vw_throw( ArgumentErr() << "XML file \"" << opt.camera_model_file << "\" is invalid.\n" );
    }
      
    double period = 5.64/geo.detector_pixel_pitch;
    double shift  = -30.0;

    double xoffset, yoffset;
    if (scan_dir == "forward"){
      xoffset = opt.xoffset_forward;
      yoffset = opt.yoffset_forward;
    }else{
      xoffset = opt.xoffset_reverse;
      yoffset = opt.yoffset_reverse;
    }

    vw_out() << "Using xoffset: " << xoffset << std::endl;
    vw_out() << "Using yoffset: " << yoffset << std::endl;

    // Internal sign adjustments
    if (scan_dir == "forward"){
      yoffset = -yoffset;
    }else{
      xoffset = -xoffset;
    }
    std::cout << "final values: " << xoffset << ' ' << yoffset << std::endl;
    std::cout << "period is " << period << std::endl;

    ImageView<float> D = copy(DiskImageView<float>(opt.camera_image_file));
    ImageView<float> E = copy(DiskImageView<float>(opt.camera_image_file));
    
    InterpolationView<EdgeExtensionView< ImageView<float>, ValueEdgeExtension<float> >, BilinearInterpolation> interp_E
      = interpolate(E, BilinearInterpolation(),
                    ValueEdgeExtension<float>(0));
    
    int num_cols = D.cols();
    int num_rows = D.rows();
    
    for (int col = 0; col < num_cols; col++){
      int n = (int)floor((col - shift)/period);
      double valx = 0, valy = 0;
      if (n%2 == 1){
        valx = -xoffset;
        valy = -yoffset;
      }
      
      for (int row = 0; row < num_rows; row++){
        D(col, row) = interp_E(col + valx, row + valy);
      }
    }
    
    vw_out() << "Writing: " << opt.output_image << std::endl;
    asp::block_write_gdal_image(opt.output_image, D, opt,
                                TerminalProgressCallback("asp", "\t-->: "));
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
