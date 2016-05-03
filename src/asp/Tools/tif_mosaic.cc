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

/// \file tif_mosaic.cc
///

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <vw/FileIO/DiskImageUtils.h>

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/InterestPointMatching.h>

using namespace vw;
namespace po = boost::program_options;

#include <limits>

namespace vw {
  template<> struct PixelFormatID<Vector<uint16, 1> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_1_CHANNEL; };
  template<> struct PixelFormatID<Vector<uint16, 4> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector<float, 1> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_1_CHANNEL; };
  template<> struct PixelFormatID<Vector<float, 4> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector<double, 1> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_1_CHANNEL; };
  template<> struct PixelFormatID<Vector<double, 4> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
}

struct ImageData{
  std::string src_file;
  int band;
  ImageViewRef<float> src_img;
  BBox2 src_box, dst_box;
  double nodata_value;
  AffineTransform transform; // Transform from src_box to dst_box.

  ImageData(std::string const& src_file_in, int band_in,
            BBox2 const& src_box_in, BBox2 const& dst_box_in,
            bool has_input_nodata_value, double input_nodata_value
            ):
    src_file(src_file_in), band(band_in), src_box(src_box_in), dst_box(dst_box_in),
    nodata_value(0.0),
    transform(AffineTransform(Matrix2x2(dst_box.width()/src_box.width(),0,
                                        0,dst_box.height()/src_box.height()),
                              dst_box.min() - src_box.min())) {

    int num_bands = get_num_channels(src_file);
    if (num_bands == 1){
      src_img = DiskImageView<float>(src_file);
    }else{
      // Multi-band image. Pick the desired band. In principle, this
      // block can handle the above case as well. We do it this way
      // because reading multi-band images is a fragile process in
      // ASP, so if we know for sure that just one band is present,
      // don't come here.
      int channel = band - 1;  // In VW, bands start from 0, not 1.
      src_img = select_channel(read_channels<1, float>(src_file, channel), 0);
    }

    // Read nodata-value from disk, if available. Overwrite with
    // user-provided nodata-value if given.
    DiskImageResourceGDAL in_rsrc(src_file);
    if ( in_rsrc.has_nodata_read() ) nodata_value = in_rsrc.nodata_read();
    if (has_input_nodata_value) nodata_value = input_nodata_value;

  }
};

// Fix seams by finding interest point matches and adjusting the
// transforms so that they map points from one image on top of
// matches from another image. We count on the fact
// that each image overlaps with the next one.
void fix_seams_using_ip(std::vector<ImageData> & img_data){

  for (int img_index = 0; img_index < int(img_data.size())-1; img_index++) {

    // The intersection of the image regions in the common
    // destination domain.
    BBox2 intersection_box = img_data[img_index].dst_box;
    intersection_box.crop(img_data[img_index+1].dst_box);

    // Pull into first image's pixel domain
    BBox2 box0 = img_data[img_index].transform.reverse_bbox(intersection_box);
    DiskImageView<float> img0(img_data[img_index].src_file);
    box0.crop(bounding_box(img0));
    ImageViewRef<float> crop0 = crop(img0, box0);

    // Pull into second image's pixel domain
    BBox2 box1 = img_data[img_index+1].transform.reverse_bbox(intersection_box);
    DiskImageView<float> img1(img_data[img_index+1].src_file);
    box1.crop(bounding_box(img1));
    ImageViewRef<float> crop1 = crop(img1, box1);

    // The transform from cropped image0 to cropped image1
    int ip_per_tile = 0; // auto-determination
    Matrix3x3 T = asp::translation_ip_matching(crop0, crop1,
                                               ip_per_tile,
                                               img_data[img_index].nodata_value,
                                               img_data[img_index+1].nodata_value);
    T = inverse(T); // originally it was going from image1 to image0

    // The transform from image0 to cropped image0
    Matrix3x3 T0;
    T0.set_identity();
    T0(0, 2) = -box0.min().x();
    T0(1, 2) = -box0.min().y();

    // The transform from cropped image1 to image1
    Matrix3x3 T1;
    T1.set_identity();
    T1(0, 2) = box1.min().x();
    T1(1, 2) = box1.min().y();

    // The transform from image0 to image 1
    T = T1*T*T0;

    // The transform from image1 to image 0
    Matrix3x3 TI = inverse(T);

    Matrix3x3 N = affine2mat(img_data[img_index].transform)*TI;
    Matrix3x3 N0 = affine2mat(img_data[img_index+1].transform);

    vw_out() << "Old transform for image " << img_index+1 << ": " << N0 << std::endl;
    vw_out() << "New transform for image " << img_index+1 << ": " << N  << std::endl;
    
    // Update the transform
    img_data[img_index+1].transform = mat2affine(N);

    // Update the dst box.
    // TODO: forward_bbox() always give results as an int box. Must revisit this using
    // a float box!
    img_data[img_index+1].dst_box
      = img_data[img_index+1].transform.forward_bbox(img_data[img_index+1].src_box);
  }
}

void parseImgData(std::string data, int band,
                  bool has_input_nodata_value, double input_nodata_value,
                  bool fix_seams,
                  int& dst_cols, int& dst_rows,
                  std::vector<ImageData> & img_data){

  // Extract the tif files to mosaic, their dimensions, and for each
  // of them the location to mosaic to in the output image.  The input
  // is comma-separated.

  dst_cols = 0; dst_rows = 0;
  img_data.clear();

  // Replace commas with spaces.
  std::string oldStr = ",", newStr = " ";
  size_t pos = 0;
  while((pos = data.find(oldStr, pos)) != std::string::npos){
    data.replace(pos, oldStr.length(), newStr);
    pos += newStr.length();
  }

  std::istringstream is(data);
  is >> dst_cols >> dst_rows;

  std::string src_file;
  double src_lenx, src_leny, dst_minx, dst_miny, dst_lenx, dst_leny;
  BBox2 src_box, dst_box;
  bool warned_about_bands = false;
  while( is >> src_file >> src_lenx >> src_leny >> dst_minx >> dst_miny
         >> dst_lenx >> dst_leny){
    src_box = BBox2(0,        0,        src_lenx, src_leny);
    dst_box = BBox2(dst_minx, dst_miny, dst_lenx, dst_leny);

    // If the user did not specify the band to use (band == 0), and there is
    // more than one band, use the first band but warn the user about it.
    if (!warned_about_bands){
      int num_bands = get_num_channels(src_file);
      if (num_bands > 1 && band <= 0)
        vw_out(vw::WarningMessage) << "Input images have " << num_bands
                                   << " bands. Will use the first band only.\n";
      band = std::max(band, 1);
      warned_about_bands = true;
    }

    img_data.push_back(ImageData(src_file, band, src_box, dst_box,
                                 has_input_nodata_value, input_nodata_value));
  }

  if (fix_seams)
    fix_seams_using_ip(img_data);

  for (int k = (int)img_data.size()-1; k >= 0; k--){

    for (int l = k - 1; l >= 0; l--){

      // Later images will be on top of earlier images. For that
      // reason, reduce each image to the part it does not overlap
      // with later images.
      img_data[l].dst_box.max().y() =
        std::min( img_data[l].dst_box.max().y(),
                  img_data[k].dst_box.min().y() );

      // Make sure min of box is <= max of box after the above
      // adjustment.
      img_data[l].dst_box.min().y() =
        std::min( img_data[l].dst_box.min().y(),
                  img_data[l].dst_box.max().y() );
    }

    // Adjust the source box as well. Expand the box slightly before
    // reversing as reverse_bbox casts its input to BBox2i which
    // is a problem if the box has floating point corners.
    // The ultimate references are always the destination box
    // and the affine transform.
    BBox2 box = img_data[k].dst_box;
    box.expand(1);
    img_data[k].src_box = img_data[k].transform.reverse_bbox(box);
  }

#if 0
  for (int k = 0; k < (int)img_data.size(); k++){
    std::cout << "boxes: " << img_data[k].src_file << " "
              << img_data[k].src_box << ' '
              << img_data[k].transform.reverse_bbox(img_data[k].dst_box)
              << ' ' << img_data[k].dst_box << std::endl;
  }
#endif

}

// A class to mosaic and rescale images using bilinear interpolation.

class TifMosaicView: public ImageViewBase<TifMosaicView>{
  int m_dst_cols, m_dst_rows;
  std::vector<ImageData> m_img_data;
  double m_scale;
  double m_output_nodata_value;

public:
  TifMosaicView(int dst_cols, int dst_rows, std::vector<ImageData> & img_data,
                double scale, double output_nodata_value):
    m_dst_cols((int)(scale*dst_cols)),
    m_dst_rows((int)(scale*dst_rows)),
    m_img_data(img_data), m_scale(scale),
    m_output_nodata_value(output_nodata_value){}

  typedef float pixel_type;
  typedef pixel_type result_type;
  typedef PixelMask<float> masked_pixel_type;
  typedef ProceduralPixelAccessor<TifMosaicView> pixel_accessor;

  inline int32 cols  () const { return m_dst_cols; }
  inline int32 rows  () const { return m_dst_rows; }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "TifMosaicView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // Scaled box
    Vector2i b = floor(bbox.min()/m_scale);
    Vector2i e = ceil(elem_diff(bbox.max(),1)/m_scale) + Vector2i(1, 1);
    BBox2i scaled_box(b[0], b[1], e[0] - b[0], e[1] - b[1]);

    // The scaled box can potentially intersect several of the images
    // to be mosaicked. So prepare to interpolate into all of them.
    // Note 1: the cropped sub-images we get below are non-overlapping
    // and no bigger than they need to be.
    // Note 2: We mask each image using its individual nodata-value.
    // The output mosaic uses the global m_output_nodata_value.
    typedef ImageView<masked_pixel_type> ImageT;
    typedef InterpolationView<ImageT, BilinearInterpolation> InterpT;

    std::vector<BBox2i>  src_vec(m_img_data.size());  // Effective area of image tile
    std::vector<InterpT> crop_vec(m_img_data.size(),
                                  InterpT(ImageT())); // Image data but expanded a bit for interpolation's sake
    int extra = BilinearInterpolation::pixel_buffer;
    // Loop through the input images
    for (int k = 0; k < (int)m_img_data.size(); k++){
      BBox2 box = m_img_data[k].dst_box;
      box.crop(scaled_box);
      if (box.empty())
        continue;
      box.expand(1); // since reverse_bbox will truncate input box to BBox2i
      box = m_img_data[k].transform.reverse_bbox(box);
      box = grow_bbox_to_int(box);
      box.crop(bounding_box(m_img_data[k].src_img));
      if (box.empty())
        continue;
      src_vec[k] = ( box ); // Recording active area of the tile
      box.expand( extra );  // Expanding to help interpolation
      crop_vec[k] =
        InterpT(create_mask_less_or_equal
                (crop(edge_extend(m_img_data[k].src_img, ConstantEdgeExtension()),
                      box),
                 m_img_data[k].nodata_value));
    }

    ImageView<pixel_type> tile(bbox.width(), bbox.height());
    fill( tile, m_output_nodata_value );

    // TODO: Replace this code with the resample_aa function??

    // TODO: This could possibly be performed entirely with a
    // TransformView and apply_mask.
    // -- or --
    // Since we have no rotations, we can assume whole lines will come
    // from a single image
    for (int row = 0; row < bbox.height(); row++){
      for (int col = 0; col < bbox.width(); col++){

        Vector2 dst_pix = Vector2(col + bbox.min().x(),
                                  row + bbox.min().y())/m_scale;

        // See which src image we end up in. Start from the later
        // images, as those are on top. Stop when we find an image
        // with a valid pixel at given location.
        for (int k = (int)m_img_data.size()-1; k >= 0; k--){
          Vector2 src_pix = m_img_data[k].transform.reverse(dst_pix);
          if (!src_vec[k].contains(src_pix))
            continue;

          // Go to the coordinate system of image crop_vec[k]. Note that
          // we add back the 'extra' number used in expanding the image earlier.
          src_pix += elem_diff(extra, src_vec[k].min());

          masked_pixel_type r = crop_vec[k](src_pix[0], src_pix[1] );
          if (is_valid(r)){
            tile(col, row) = r.child();
            break;
          }
        } // image stack iteration

      } // col iteration
    } // row iteration

    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

struct Options : vw::cartography::GdalWriteOptions {
  std::string img_data, output_image;
  int band;
  bool has_input_nodata_value, has_output_nodata_value, fix_seams;
  double percent, input_nodata_value, output_nodata_value;
  Options(): band(0), has_input_nodata_value(false), has_output_nodata_value(false),
             input_nodata_value (std::numeric_limits<double>::quiet_NaN()),
             output_nodata_value(std::numeric_limits<double>::quiet_NaN()){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  general_options.add_options()
    ("image-data", po::value(&opt.img_data)->default_value(""),
         "Information on the images to mosaic.")
    ("output-image,o", po::value(&opt.output_image)->default_value(""),
         "Specify the output image.")
    ("band", po::value(&opt.band), "Which band to use (for multi-spectral images).")
    ("input-nodata-value", po::value(&opt.input_nodata_value),
         "Nodata value to use on input; input pixel values less than or equal to this are considered invalid.")
    ("output-nodata-value", po::value(&opt.output_nodata_value),
         "Nodata value to use on output.")
    ("reduce-percent", po::value(&opt.percent)->default_value(100.0),
     "Reduce resolution using this percentage.")
    ("fix-seams",   po::bool_switch(&opt.fix_seams)->default_value(false),
     "Fix seams in the output mosaic due to inconsistencies between image and camera data using interest point matching.");

  po::options_description positional("");
  po::positional_options_description positional_desc;
  std::string usage("");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  opt.has_input_nodata_value  = vm.count("input-nodata-value" );
  opt.has_output_nodata_value = vm.count("output-nodata-value");

  if ( opt.img_data.empty() )
    vw_throw( ArgumentErr() << "No images to mosaic.\n" << usage << general_options );

  if ( opt.output_image.empty() )
    vw_throw( ArgumentErr() << "Missing output image name.\n" << usage << general_options );

  if ( opt.percent > 100.0 || opt.percent <= 0.0 )
    vw_throw( ArgumentErr() << "The percent amount must be between 0% and 100%.\n"
                            << usage << general_options );

}

int main( int argc, char *argv[] ) {

  Options opt;

  try {

    handle_arguments( argc, argv, opt );

    double scale = opt.percent/100.0;

    int dst_cols, dst_rows;
    std::vector<ImageData> img_data;
    parseImgData(opt.img_data, opt.band,
                 opt.has_input_nodata_value, opt.input_nodata_value,
                 opt.fix_seams, dst_cols, dst_rows, img_data);
    if ( dst_cols <= 0 || dst_rows <= 0 || img_data.empty() )
      vw_throw( ArgumentErr() << "Invalid input data.\n");

    // We can handle individual images having different nodata
    // values. Pick the one of the first image as the output nodata
    // value. Override with user's nodata value if provided.
    double output_nodata_value = img_data[0].nodata_value;
    if (opt.has_output_nodata_value)
      output_nodata_value = opt.output_nodata_value;

    vw_out() << "Writing: " << opt.output_image << std::endl;
    vw::cartography::block_write_gdal_image(opt.output_image,
                                TifMosaicView(dst_cols, dst_rows,
                                              img_data, scale,
                                              output_nodata_value),
                                output_nodata_value, opt,
                                TerminalProgressCallback("asp", "\t    Mosaic:"));

  } ASP_STANDARD_CATCHES;
  return 0;
}
