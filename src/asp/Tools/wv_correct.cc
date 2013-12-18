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
// Verify that the input image satisfies the above assumptions.
// Add documentation.
// Print default offsets.

// The problem: A WV2 image is obtained by mosaicking from left to
// right image blocks which are as tall is the entire image (each
// block comes from an individual CCD image sensor). Blocks are
// slightly misplaced in respect to each other by some unknown
// subpixel offset. The goal of this tool is to locate these CCD
// artifact boundary locations, and undo the offsets.
// 

// Observations:
// 1. The CCD artifact locations are periodic, but the starting offset
//    is not positioned at exactly one period. It is less by one fixed
//    value which we name 'shift'.  
// 2. There are CCD offsets in both x and y at each location.
// 3. The magnitude of all CCD offsets in x is the same, but their sign
//    alternates. The same is true in y.
// 4. The period of CCD offsets is inversely proportional to the detector
//    pitch.
// 5. The CCD offsets are pretty consistent among all images of given
//    scan direction (forward or reverse).
// We used all these and a lot of images to heuristically find the
// period and offset of the artifacts, and the 'shift' value. We
// correct these below. We allow the user to override the value of CCD
// offsets if desired.

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
using namespace std;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : asp::BaseOptions {
  double xoffset, yoffset;
  double xoffset_forward, yoffset_forward;
  double xoffset_reverse, yoffset_reverse;
  std::string camera_image_file, camera_model_file, output_image; 
  Options(){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  // These quantities were heuristically obtained by averaging
  // over a large set of runs while removing outliers and giving
  // more weight to more reliable data. 
  double default_xoffset_forward = 0.2842;
  double default_yoffset_forward = 0.2369;
  double default_xoffset_reverse = 0.3396;
  double default_yoffset_reverse = 0.3725;

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

template <class ImageT>
class WVCorrectView: public ImageViewBase< WVCorrectView<ImageT> >{
  ImageT m_img;
  double m_shift, m_period, m_xoffset, m_yoffset;
  typedef typename ImageT::pixel_type PixelT;

public:
  WVCorrectView( ImageT const& img,
                 double shift, double period, double xoffset, double yoffset):
    m_img(img), m_shift(shift), m_period(period),
    m_xoffset(xoffset), m_yoffset(yoffset){}
  
  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef ProceduralPixelAccessor<WVCorrectView> pixel_accessor;

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "WVCorrectView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // Need to see a bit more of the input image for the purpose
    // of interpolation.
    int bias = (int)ceil(std::max(std::abs(m_xoffset), std::abs(m_yoffset)))
      + BilinearInterpolation::pixel_buffer + 1;
    BBox2i biased_box = bbox;
    biased_box.expand(bias);
    biased_box.crop(bounding_box(m_img));
    
    ImageView<result_type> cropped_img = crop(m_img, biased_box);
    InterpolationView<EdgeExtensionView< ImageView<result_type>, ValueEdgeExtension<result_type> >, BilinearInterpolation> interp_img
      = interpolate(cropped_img, BilinearInterpolation(),
                    ValueEdgeExtension<result_type>(result_type()));
    
    ImageView<result_type> tile(bbox.width(), bbox.height());
    for (int col = bbox.min().x(); col < bbox.max().x(); col++){

      // The sign of CCD offsets alternates as one moves along the image
      // columns. As such, at "even" blocks, the offsets accumulated so
      // far cancel each other, so we need to correct the "odd" blocks
      // only.
      int block_index = (int)floor((col - m_shift)/m_period);
      double valx = 0, valy = 0;
      if (block_index % 2 ){
        valx = -m_xoffset;
        valy = -m_yoffset;
      }
      
      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
        tile(col - bbox.min().x(), row - bbox.min().y() )
          = interp_img(col - biased_box.min().x() + valx,
                       row - biased_box.min().y() + valy);
      }
    }
    
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};
template <class ImageT>
WVCorrectView<ImageT> wv_correct(ImageT const& img, double shift,
                                 double period, double xoffset, double yoffset){
  return WVCorrectView<ImageT>(img, shift, period, xoffset, yoffset);
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

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
      
    // The first CCD artifact is at column period + shift,
    // then they repeat with given period.
    double shift  = -30.0;
    double period = 5.64/geo.detector_pixel_pitch;

    // The offsets at the first CCD artifact location.
    // The offsets keep the same magnitude but their sign
    // alternates as one moves along image columns.
    double xoffset, yoffset;
    if (scan_dir == "forward"){
      xoffset = opt.xoffset_forward;
      yoffset = opt.yoffset_forward;
    }else{
      xoffset = opt.xoffset_reverse;
      yoffset = opt.yoffset_reverse;
    }

    vw_out() << "Using x offset: " << xoffset << std::endl;
    vw_out() << "Using y offset: " << yoffset << std::endl;

    // Internal sign adjustments
    if (scan_dir == "forward"){
      yoffset = -yoffset;
    }else{
      xoffset = -xoffset;
    }

    DiskImageView<float> input_img(opt.camera_image_file);
    bool has_nodata = false;
    double nodata = numeric_limits<double>::quiet_NaN();
    boost::shared_ptr<DiskImageResource> img_rsrc
      ( new DiskImageResourceGDAL(opt.camera_image_file) );
    if (img_rsrc->has_nodata_read()){
      has_nodata = true;
      nodata = img_rsrc->nodata_read();
    }

    vw_out() << "Writing: " << opt.output_image << std::endl;
    if (has_nodata){
      asp::block_write_gdal_image(opt.output_image,
                                  apply_mask
                                  (wv_correct(create_mask(input_img,
                                                          nodata),
                                              shift, period, xoffset, yoffset),
                                   nodata),
                                  nodata, opt,
                                  TerminalProgressCallback("asp", "\t-->: "));
    }else{
      asp::block_write_gdal_image(opt.output_image,
                                  wv_correct(input_img, shift, period,
                                             xoffset, yoffset),
                                  opt,
                                  TerminalProgressCallback("asp", "\t-->: "));
    }
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
