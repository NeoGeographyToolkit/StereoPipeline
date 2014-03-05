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

// Correct CCD artifacts in WorldView 1 and 2 images with TDI 16.

// The problem: A WV image is obtained by mosaicking from left to
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
  std::string camera_image_file, camera_model_file, output_image; 
  double xoffset, yoffset, period, shift;
  Options(){
    xoffset = yoffset = period = shift = std::numeric_limits<double>::quiet_NaN();
  }
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("");
  general_options.add_options()
    ("xoffset", po::value(&opt.xoffset), "Specify the CCD offset correction to apply in the x direction (optional).")
    ("yoffset", po::value(&opt.yoffset), "Specify the CCD offset correction to apply in the y direction (optional).")
    ("period", po::value(&opt.period), "Specify the period of CCD artifacts (optional).")
    ("shift", po::value(&opt.shift), "Specify how much to add to the period to get the location of the first CCD artifact (optional).")
    ;
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

  asp::create_out_dir(opt.output_image);
  
}

void get_offsets(bool is_wv01, bool is_forward, std::vector<double> & off){

  // Get a sequence of CCD offsets (they will later be scaled).
  // We need this primarily for WV01 cameras, as there the offsets
  // don't follow a simple pattern.
  off.clear();
  if (!is_wv01){
    double o[] = {1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1};
    for (int i = 0; i < (int)(sizeof(o)/sizeof(double)); i++) off.push_back(o[i]);
  }else{
    //double o[] = {1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 0.5, -0.5, 0.5, -0.5, 0, 0, 0, 0, 0, 0, 0, 0.5, -0.5, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1}; // good!
    double o[] = {1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 0.5, -1, 0.5, -0.5, -0.5, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1};
    for (int i = 0; i < (int)(sizeof(o)/sizeof(double)); i++) off.push_back(o[i]);
  }
  
}

template <class ImageT>
class WVCorrectView: public ImageViewBase< WVCorrectView<ImageT> >{
  ImageT m_img;
  bool m_is_wv01, m_is_forward;
  double m_shift, m_period, m_xoffset, m_yoffset;
  typedef typename ImageT::pixel_type PixelT;

public:
  WVCorrectView( ImageT const& img, bool is_wv01, bool is_forward,
                 double shift, double period, double xoffset, double yoffset):
    m_img(img), m_is_wv01(is_wv01), m_is_forward(is_forward),
    m_shift(shift), m_period(period),
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

    vector<double> off;
    get_offsets(m_is_wv01, m_is_forward, off);
    
    // Need to see a bit more of the input image for the purpose
    // of interpolation.
    int bias = (int)ceil(std::max(std::abs(m_xoffset), std::abs(m_yoffset)))
      + BilinearInterpolation::pixel_buffer + 1;
    BBox2i biased_box = bbox;
    biased_box.expand(bias);
    biased_box.crop(bounding_box(m_img));
    
    ImageView<result_type> cropped_img = crop(m_img, biased_box);
    InterpolationView<EdgeExtensionView< ImageView<result_type>, ConstantEdgeExtension >, BilinearInterpolation> interp_img
      = interpolate(cropped_img, BilinearInterpolation(),
                    ConstantEdgeExtension());
    
    ImageView<result_type> tile(bbox.width(), bbox.height());
    for (int col = bbox.min().x(); col < bbox.max().x(); col++){

      // The sign of CCD offsets alternates as one moves along the image
      // columns. As such, at "even" blocks, the offsets accumulated so
      // far cancel each other, so we need to correct the "odd" blocks
      // only.
      int block_index = (int)floor((col - m_shift)/m_period);
      double valx = 0, valy = 0;
      if (block_index % 2 == 1){
        valx = -m_xoffset;
        valy = -m_yoffset;
      }

      // Special treatment for WV01
      if (m_is_wv01){
        if (!m_is_forward){
          // Use a list of tabulated values to find the y offsets
          double sum = 0;
          int noff = off.size();
          for (int k = 0; k < std::min(noff, block_index); k++) sum += off[k];
          valy = -m_yoffset*sum;
        }else{
          // Just set the early y offsets to 0
          double s = 8000;
          s = m_period*floor((s - m_shift)/m_period) + m_shift;
          if (col < s)
            valy = 0;
        }
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
WVCorrectView<ImageT> wv_correct(ImageT const& img,
                                 bool is_wv01, bool is_forward, double shift,
                                 double period, double xoffset, double yoffset){
  return WVCorrectView<ImageT>(img, is_wv01, is_forward, shift, period,
                               xoffset, yoffset);
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
    std::string scan_dir, sat_id;
    double det_pitch;
    try{
      XMLPlatformUtils::Initialize();
      read_xml( opt.camera_model_file, geo, att, eph, img, rpc );
      
      scan_dir = boost::to_lower_copy( img.scan_direction );
      if (scan_dir != "forward" && scan_dir != "reverse")
        vw_throw( ArgumentErr() << "XML file \"" << opt.camera_model_file
                  << "\" is lacking a valid image scan direction.\n" );

      sat_id = img.sat_id;
      if (sat_id != "WV01" && sat_id != "WV02")
        vw_throw( ArgumentErr() << "Can apply CCD artifacts corrections only "
                  << "for WV01 and WV02 camera images.\n" );
      
      det_pitch = geo.detector_pixel_pitch;
      if (det_pitch <= 0.0)
        vw_throw( ArgumentErr() << "XML file \"" << opt.camera_model_file
                  << "\" has a non-positive pixel pitch.\n" );

      if (img.tdi != 16)
        vw_throw( ArgumentErr() << "Can apply CCD artifacts corrections only for TDI 16.\n" );
      
    } catch ( const std::exception& e ) {                
      vw_throw( ArgumentErr() << e.what() );
    }

    bool is_forward = (scan_dir == "forward");
    
    // Defaults, depending on satellite and scan direction
    double xoffset, yoffset, period, shift;
    bool is_wv01 = (sat_id == "WV01");
    if (is_wv01){
      if (is_forward){
        period = 708;
        shift  = -119; 
        xoffset = 0.10;
        yoffset = -0.25;
      }else{
        period  = 709;
        shift   = -136; 
        xoffset = 0.17;
        yoffset = 0.20;
      }
    }else{
      period = 705;
      shift  = -35.0;
      if (is_forward){
        xoffset = 0.2842;
        yoffset = 0.2369;
      }else{
        xoffset = 0.3396;
        yoffset = 0.3725;
      }
    }

    // Apply user's overrides, if any
    if (!boost::math::isnan(opt.xoffset)) xoffset = opt.xoffset;
    if (!boost::math::isnan(opt.yoffset)) yoffset = opt.yoffset;
    if (!boost::math::isnan(opt.period))  period  = opt.period;
    if (!boost::math::isnan(opt.shift))   shift   = opt.shift;

    vw_out() << "Using x offset: " << xoffset << std::endl;
    vw_out() << "Using y offset: " << yoffset << std::endl;
    vw_out() << "Using period:   " << period  << std::endl;
    vw_out() << "Using shift:    " << shift   << std::endl;

    // Adjust for detector pitch
    period = period*(8.0e-3/det_pitch);
    
    // Internal sign adjustments
    if (is_forward){
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
                                              is_wv01, is_forward,
                                              shift, period, xoffset, yoffset),
                                   nodata),
                                  nodata, opt,
                                  TerminalProgressCallback("asp", "\t-->: "));
    }else{
      asp::block_write_gdal_image(opt.output_image,
                                  wv_correct(input_img,
                                             is_wv01, is_forward, shift, period,
                                             xoffset, yoffset),
                                  opt,
                                  TerminalProgressCallback("asp", "\t-->: "));
    }
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
