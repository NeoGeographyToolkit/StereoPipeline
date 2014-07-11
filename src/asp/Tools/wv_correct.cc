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
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("");
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

void arr_to_vec(double arr[], int len, vector<double> & vec){
  vec.clear();
  for (int i = 0; i < len; i++) vec.push_back(arr[i]);
}

void get_offsets(bool is_wv01, bool is_forward,
                 std::vector<double> & posx, std::vector<double> & ccdx,
                 std::vector<double> & posy, std::vector<double> & ccdy
                 ){

  // Here we tabulate all ccds offsets and their column pixel positions.
  if (!is_wv01){
  }else{
    double posx_arr[] = {6.2900000000000000e+02,1.3570000000000000e+03,2.0190000000000000e+03,2.7690000000000000e+03,3.4720000000000000e+03,4.1790000000000000e+03,4.8670000000000000e+03,5.5680000000000000e+03,6.2730000000000000e+03,6.9730000000000000e+03,7.6250000000000000e+03,8.3790000000000000e+03,9.0910000000000000e+03,9.7920000000000000e+03,1.0499000000000000e+04,1.1204000000000000e+04,1.1911000000000000e+04,1.2620000000000000e+04,1.3332000000000000e+04,1.4040000000000000e+04,1.4750000000000000e+04,1.5459000000000000e+04,1.6170000000000000e+04,1.6879000000000000e+04,1.7591000000000000e+04,1.8300000000000000e+04,1.9008000000000000e+04,1.9720000000000000e+04,2.0426000000000000e+04,2.1141000000000000e+04,2.1848000000000000e+04,2.2559000000000000e+04,2.3271000000000000e+04,2.3977000000000000e+04,2.4682000000000000e+04,2.5419000000000000e+04,2.6095000000000000e+04,2.6803000000000000e+04,2.7446000000000000e+04,2.8302000000000000e+04,2.8908000000000000e+04,2.9608000000000000e+04,3.0313000000000000e+04,3.1009000000000000e+04,3.1705000000000000e+04,3.2406000000000000e+04,3.3100000000000000e+04,3.3792000000000000e+04,3.4483000000000000e+04};
    arr_to_vec(posx_arr, sizeof(posx_arr)/sizeof(double), posx);
    
    double ccdx_arr[] = {6.7322551801454480e-02,5.5941279131803634e-02,-6.3757124794674186e-02,-6.8350222655965132e-02,1.7691659140427704e-01,-6.5546000611674385e-02,6.3907733085781446e-02,-9.4966625926268172e-02,9.4161850926099605e-02,-1.0542259530055144e-01,-8.1558061694499709e-02,-6.3956300595019416e-02,4.1564490770397769e-02,-6.0005436499402910e-02,2.7687811096602116e-01,-6.7650615566507560e-02,8.9531829438824359e-02,-7.5787800987529685e-02,6.0918450645685199e-02,-4.9435252430723518e-02,7.8217062959282352e-02,-9.4972312086564092e-02,6.8771948144368461e-02,-5.5916563953937647e-02,2.5576397858317801e-01,-6.2953016311800947e-02,4.5986274236245760e-02,-5.2151312380337433e-02,3.5461672386768096e-02,-7.8840968976700190e-02,5.8279888571948685e-02,-5.6318493405969880e-02,-2.3130788594668883e-02,-2.5727892445735229e-02,6.7618551931374907e-02,-3.2635798235672329e-02,-6.7421171442540700e-02,8.6289010551310996e-02,-4.2940282592809562e-02,-6.7487297331515511e-02,-1.6954569447987194e-01,1.5768945188280239e-01,-1.3197137444641416e-01,2.2044168881490209e-01,-1.7686345973293444e-01,2.8571073037673689e-01,-2.5204743971722710e-01,2.9477986650443111e-01,-3.1357641656875035e-01};
    arr_to_vec(ccdx_arr, sizeof(ccdx_arr)/sizeof(double), ccdx);

    double posy_arr[] ={5.5700000000000000e+02,1.3840000000000000e+03,2.0790000000000000e+03,2.7720000000000000e+03,4.9010000000000000e+03,5.6020000000000000e+03,6.8740000000000000e+03,7.6810000000000000e+03,8.3770000000000000e+03,9.0860000000000000e+03,9.7910000000000000e+03,1.0500000000000000e+04,1.1206000000000000e+04,1.1914000000000000e+04,1.2623000000000000e+04,1.3331000000000000e+04,1.4041000000000000e+04,1.4749000000000000e+04,1.5459000000000000e+04,1.6170000000000000e+04,1.6881000000000000e+04,1.7590000000000000e+04,1.8301000000000000e+04,1.9011000000000000e+04,1.9722000000000000e+04,2.0431000000000000e+04,2.1140000000000000e+04,2.1851000000000000e+04,2.2558000000000000e+04,2.3267000000000000e+04,2.3976000000000000e+04,2.4682000000000000e+04,2.5388000000000000e+04,2.6094000000000000e+04,2.6800000000000000e+04,2.7503000000000000e+04,2.8206000000000000e+04,2.8909000000000000e+04,2.9611000000000000e+04,3.0309000000000000e+04,3.1009000000000000e+04,3.1707000000000000e+04,3.2401000000000000e+04,3.3098000000000000e+04,3.3791000000000000e+04,3.4482000000000000e+04};
    arr_to_vec(posy_arr, sizeof(posy_arr)/sizeof(double), posy);

    double ccdy_arr[] = {5.2874163366164850e-02,1.4813114779561809e-01,-1.3228522060801995e-01,5.1229840789382905e-02,4.1734055216619194e-02,-3.4922349728371403e-02,2.3489411511399848e-02,8.0756339158257071e-02,-4.1430846518602682e-02,1.3825891634375403e-01,-1.0281921942927558e-01,1.8102049531293241e-01,-1.4505660180914340e-01,1.8186344173432878e-01,-1.8620316464758269e-01,1.9090650984864999e-01,-2.8086787970093174e-01,2.7066109799278149e-01,-2.2564204861733023e-01,3.2153128507658019e-01,-2.5708859257733496e-01,2.6364737894970347e-01,-2.8693208660602365e-01,2.7970516529790845e-01,-3.5311023659542107e-01,3.4701280163120907e-01,-3.1032664634310730e-01,3.5125263768893017e-01,-3.3322268782288833e-01,3.6955614181826035e-01,-3.3683473663931718e-01,3.2182908581461295e-01,-2.8261547868377229e-01,3.4742873160768417e-01,-2.9435215444692314e-01,3.0697747027458971e-01,-2.9547216462701609e-01,3.2267414829365171e-01,-3.4291155301037418e-01,2.8631768612226638e-01,-2.6625338683543259e-01,3.4282658200878574e-01,-2.5735046181265991e-01,2.6016063755950836e-01,-2.7861330621270186e-01,2.4248759221995142e-01};
    arr_to_vec(ccdy_arr, sizeof(ccdy_arr)/sizeof(double), ccdy);
  }
  
}

template <class ImageT>
class WVCorrectView: public ImageViewBase< WVCorrectView<ImageT> >{
  ImageT m_img;
  bool m_is_wv01, m_is_forward;
  double m_shift, m_period, m_xoffset, m_yoffset;
  std::vector<double> m_posx, m_ccdx, m_posy, m_ccdy;

  typedef typename ImageT::pixel_type PixelT;

public:
  WVCorrectView( ImageT const& img, bool is_wv01, bool is_forward,
                 double shift, double period, double xoffset, double yoffset):
    m_img(img), m_is_wv01(is_wv01), m_is_forward(is_forward),
    m_shift(shift), m_period(period),
    m_xoffset(xoffset), m_yoffset(yoffset){

    get_offsets(m_is_wv01, m_is_forward,  
                m_posx, m_ccdx, m_posy, m_ccdy);
    
  }
  
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
      if (m_is_wv01 && m_is_forward){
        
        valx = 0.0;
        for (size_t t = 0; t < m_ccdx.size(); t++){
          if (m_posx[t] < col){
            valx -= m_ccdx[t];
          }
        }
        
        valy = 0.0;
        for (size_t t = 0; t < m_ccdy.size(); t++){
          if (m_posy[t] < col){
            valy -= m_ccdy[t];
          }
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
