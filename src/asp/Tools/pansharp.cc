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


#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>

using std::endl;
using std::string;

using namespace vw;
using namespace vw::cartography;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Camera/RPC_XML.h>
namespace po = boost::program_options;
namespace fs = boost::filesystem;



// TODO: Move these to vw/PixelType.h ??
// - These don't follow the existing pixel color conventions!

/// Convert an RGB pixel to a YCbCr pixel
/// - So far this only works for unsigned char and unsigned short.
template <typename T>
T rgbToYCbCr(T const& rgb, double min_val, double max_val) {
  // Convert
  double mean_val = (min_val + max_val+1) / 2.0;
  double temp[3];
  temp[0] =            0.299   *rgb[0] + 0.587   *rgb[1] + 0.114   *rgb[2];
  temp[1] = mean_val - 0.168736*rgb[0] - 0.331264*rgb[1] + 0.5     *rgb[2];
  temp[2] = mean_val + 0.5     *rgb[0] - 0.418688*rgb[1] - 0.081312*rgb[2];
  // Copy and constrain
  T ycbcr;
  for (int i=0; i<3; ++i)
  {
    ycbcr[i] = temp[i];
    if (temp[i] < min_val) ycbcr[i] = min_val;
    if (temp[i] > max_val) ycbcr[i] = max_val;
  }
  return ycbcr;
}


/// Converts a single YCbCr pixel to RGB
/// - So far this only works for unsigned char and unsigned short.
template <typename T>
T ycbcrToRgb(T const& ycbcr, double min_val, double max_val) {
  // Convert
  double mean_val = (min_val + max_val+1) / 2.0;
  double temp[3];
  temp[0] = ycbcr[0]                                   + 1.402   * (ycbcr[2] - mean_val);
  temp[1] = ycbcr[0] - 0.34414 * (ycbcr[1] - mean_val) - 0.71414 * (ycbcr[2] - mean_val);
  temp[2] = ycbcr[0] + 1.772   * (ycbcr[1] - mean_val);

  // Copy and constrain
  T rgb;
  for (int i=0; i<3; ++i)
  {
    rgb[i] = temp[i];
    if (temp[i] < min_val) rgb[i] = min_val;
    if (temp[i] > max_val) rgb[i] = max_val;
  }
  return rgb;
}



/// Image view class which applies a pan sharp algorithm.
/// - This takes a gray and an RGB image as input and generates an RGB image as output.
/// - This operation is not particularly useful unless the gray image is higher
///   resolution than the RGB image.
template <class ImageGrayT, class ImageColorT, typename DataTypeT>
class PanSharpView : public ImageViewBase<PanSharpView<ImageGrayT, ImageColorT, DataTypeT> > {

public: // Definitions

  typedef PixelRGB<DataTypeT> pixel_type;  // This is what controls the type of image that is written to disk.
  typedef pixel_type          result_type;

private: // Variables

  // TODO: These should be Ref views and masked?
  ImageGrayT  const& m_gray_image;
  ImageColorT const& m_color_image;

  DataTypeT m_output_nodata;
  DataTypeT m_min_val;
  DataTypeT m_max_val;

  int m_num_rows;
  int m_num_cols;

public: // Functions

  // Constructor
  PanSharpView( ImageGrayT  const& gray_image,
                ImageColorT const& color_image,
                DataTypeT          outputNodata,
                DataTypeT          min_value,
                DataTypeT          max_value)
                  : m_gray_image(gray_image), m_color_image(color_image),
                    m_output_nodata(outputNodata),
                    m_min_val(min_value),
                    m_max_val(max_value) {
    // The images are assumed to be the same size (possibly due to transforms)
    m_num_rows     = m_gray_image.rows();
    m_num_cols     = m_gray_image.cols();
  }

  inline int32 cols  () const { return m_num_cols; }
  inline int32 rows  () const { return m_num_rows; }
  inline int32 planes() const { return 1; }

  inline result_type operator()( int32 i, int32 j, int32 p=0 ) const
  {
    return 0; // NOT IMPLEMENTED!
  }

  typedef ProceduralPixelAccessor<PanSharpView<ImageGrayT, ImageColorT, DataTypeT> > pixel_accessor;
  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }


  /// Apply pansharp algorithm to a single pair of pixels.
  /// - Any required interpolation etc. will have already happened by now.
  template <class P1, class P2>
  result_type convert_pixel(P1 const& gray_pixel, P2 const& color_pixel) const {
    // Convert RGB to YCbCr
    P2 temp = rgbToYCbCr(color_pixel, m_min_val, m_max_val);
    result_type ycbcr_pixel(temp[0], temp[1], temp[2]); // Extra step breaks any type dependency from the inputs

    // Replace Y channel with gray value
    ycbcr_pixel[0] = gray_pixel[0];

    // Convert YCbCr back to RGB
    return ycbcrToRgb(ycbcr_pixel, m_min_val, m_max_val);
  }


  typedef CropView<ImageView<result_type> > prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {

    // Set up the output image tile
    ImageView<result_type> tile(bbox.width(), bbox.height());

    // Loop through each output pixels and compute each output value
    for (int c = 0; c < bbox.width(); c++) {
      int source_c = c + bbox.min()[0];
      for (int r = 0; r < bbox.height(); r++) {
        int source_r = r + bbox.min()[1];

        // Check for a masked pixel
        if ( !is_valid(m_gray_image (source_c, source_r)) ||
             !is_valid(m_color_image(source_c, source_r))  ) {
          tile(c, r) = m_output_nodata;
          continue;
        }

        // Pass the two input pixels into the conversion function
        //result_type output_pixel = convert_pixel(m_gray_image(source_c, r), m_color_image(source_c, r));
        tile(c,r) = convert_pixel(m_gray_image (source_c, source_r),
                                  m_color_image(source_c, source_r));

      } // End row loop
    } // End column loop

    // Return the tile we created with fake borders to make it look the size of the entire output image
    return prerasterize_type(tile,
                             -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );

  } // End prerasterize function

 template <class DestT>
 inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
   vw::rasterize( prerasterize(bbox), dest, bbox );
 }

}; // End class PanSharpView


/// Convenience function
template <class ImageGrayT, class ImageColorT, typename DataTypeT>
PanSharpView<ImageGrayT, ImageColorT, DataTypeT>
inline pansharp_view( ImageGrayT  const& gray_image,
                      ImageColorT const& color_image,
                      DataTypeT          output_nodata,
                      DataTypeT          min_value,
                      DataTypeT          max_value ) {
  return PanSharpView<ImageGrayT, ImageColorT, DataTypeT>
    (gray_image, color_image, output_nodata, min_value, max_value);
}



//-------------------------------------------------------------------------------------

struct Options : vw::cartography::GdalWriteOptions {
  string gray_file,
         gray_xml_file,
         color_file,
         color_xml_file,
         output_file;
  double nodata_value,
         min_value,
         max_value;
  bool has_nodata;
};

const double DEFAULT_NODATA = -std::numeric_limits<double>::max();

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("min-value", po::value(&opt.min_value)->default_value(0),
             "Set this as the minimum legal image value, overriding the data type default.")
    ("max-value", po::value(&opt.max_value)->default_value(0),
             "Set this as the maximum legal image value, overriding the data type default.")

    ("gray-xml", po::value(&opt.gray_xml_file)->default_value(""),
             "Path to a WV XML file for the gray image.  Can be used to obtain the geo data.")
    ("color-xml", po::value(&opt.color_xml_file)->default_value(""),
             "Path to a WV XML file for the color image.  Can be used to obtain the geo data.")
    ("nodata-value", po::value(&opt.nodata_value)->default_value(DEFAULT_NODATA),
             "The no-data value to use, unless present in the color image header.");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("gray_file",   po::value(&opt.gray_file),   "The gray image file")
    ("color_file",  po::value(&opt.color_file),  "The color image file")
    ("output_file", po::value(&opt.output_file), "The output file");

  po::positional_options_description positional_desc;
  positional_desc.add("gray_file",   1);
  positional_desc.add("color_file",  1);
  positional_desc.add("output_file", 1);

  std::string usage("[options] <gray file> <color file> <output file>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.gray_file.empty() || opt.color_file.empty() || opt.output_file.empty())
    vw_throw( ArgumentErr()
         << "Requires <gray file>, <color file>, and <output file> in order to proceed.\n\n"
         << usage << general_options );

  if (opt.min_value > opt.max_value)
    vw_throw( ArgumentErr() << "The minimum value cannot be greater than the maximum value!\n\n");

  // Determine if the user entered a nodata value
  opt.has_nodata = vm.count("output-nodata-value");

  vw::create_out_dir(opt.output_file);
}


/// Load and process the images with the correct data type
template <typename T>
void load_inputs_and_process(Options           & opt,
                             GeoReference const& gray_georef,
                             GeoReference const& color_georef,
                             double       const& gray_nodata,
                             double       const& color_nodata) {

  // If the user left max_value at the default, make it the data type max.
  // - min_value defaults to zero so no need to change that.
  if (opt.max_value == 0)
    opt.max_value = std::numeric_limits<T>::max();

  // TODO: How to handle nonexistant nodata value?
  std::cout << "Min value: " << (double)opt.min_value << std::endl;
  std::cout << "Max value: " << (double)opt.max_value << std::endl;
  std::cout << "Gray  nodata: " << (double)gray_nodata << std::endl;
  std::cout << "Color nodata: " << (double)color_nodata << std::endl;
  std::cout << "Out   nodata: " << (double)opt.nodata_value << std::endl;

  // Set up file handles
  DiskImageResourceGDAL gray_rsrc(opt.gray_file),
                        color_rsrc(opt.color_file);
  DiskImageView<PixelGray<T> > gray_img (gray_rsrc);
  DiskImageView<PixelRGB <T> > color_img(color_rsrc);

  // Generate a bounding box that is the minimum of the two BBox areas
  BBox2 crop_box = bounding_box( gray_img );
  crop_box.crop(gray_georef.lonlat_to_pixel_bbox(color_georef.pixel_to_lonlat_bbox(bounding_box( color_img ))));

  // Processing is done on doubles to handle nodata values, then converted to the desired output type.

  // Generate a view of the color image from the pixel coordinate system of the gray image
  typedef PixelMask<PixelRGB<double> > PixelRGBMaskD;
  typedef PixelMask<PixelRGB<T     > > PixelRGBMask;
  ImageViewRef<PixelRGBMaskD> color_trans =
    crop(geo_transform( create_mask(pixel_cast<PixelRGBMaskD>(color_img),
                                                  color_nodata),
                        color_georef, gray_georef,
                        ValueEdgeExtension<PixelRGBMaskD>(PixelRGBMaskD()) ),
         crop_box );


  // WorldView convention is to mask <= a value, but this may not be a universal standard!
  // - create_mask_less_or_equal seems to break on PixelRGB types.

  vw_out() << "Writing: " << opt.output_file << std::endl;
  vw::cartography::block_write_gdal_image( opt.output_file,
                               // The final output image is set up in these few lines:
                               pixel_cast<PixelRGBMask>
                               (apply_mask(pansharp_view
                                           (crop(create_mask_less_or_equal
                                                 (pixel_cast<double>(gray_img),
                                                  gray_nodata),
                                                 crop_box),
                                            color_trans,
                                            opt.nodata_value,
                                            opt.min_value,
                                            opt.max_value),
                                           opt.nodata_value)
                                ),
                               true, gray_georef, // The output is written in the gray coordinate system
                               opt.has_nodata, opt.nodata_value,
                               opt,
                               TerminalProgressCallback("pansharp","\t--> Writing:"));
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    DiskImageResourceGDAL gray_rsrc(opt.gray_file),
                          color_rsrc(opt.color_file);

    double gray_nodata  = opt.nodata_value;
    double color_nodata = opt.nodata_value;
    if ( gray_rsrc.has_nodata_read() ) {
      gray_nodata = gray_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value for the gray image: " << gray_nodata << endl;
    }
    if ( color_rsrc.has_nodata_read() ) {
      color_nodata = color_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value for the color image: " << color_nodata << endl;
      // If we read in color nodata and the user did not provide nodata,
      //  set this as the output nodata value.
      if (opt.nodata_value == DEFAULT_NODATA) {
        opt.nodata_value = color_nodata;
        opt.has_nodata   = true;
      }
    }

    // Read in geo information
    GeoReference gray_georef, color_georef;
    if (!asp::read_wv_georeference(gray_georef,  opt.gray_file,  opt.gray_xml_file ) ||
        !asp::read_wv_georeference(color_georef, opt.color_file, opt.color_xml_file)   ) {
      vw_throw(ArgumentErr() << "Could not read a georeference from an input image!\n");
    }

    // Transform the color image into the same perspective as the grayscale image. However, we
    // don't support datum changes!
    if ( gray_georef.datum().proj4_str() != color_georef.datum().proj4_str() )
      vw_throw( NoImplErr() << "Pansharp can't operate on images which are on different datums!\n" );

    // Check the input data type
    ChannelTypeEnum input_data_type = gray_rsrc.channel_type();
    ChannelTypeEnum color_data_type = color_rsrc.channel_type();
    if (input_data_type != color_data_type)
      vw_throw( NoImplErr() << "Pansharp can't operate on images which are different data types!\n" );

    // Redirect to another function with the correct template type
    switch(input_data_type) {
      //case VW_CHANNEL_INT8   : load_inputs_and_process<vw::int8   >(opt);  break;
      case VW_CHANNEL_UINT8  : load_inputs_and_process<vw::uint8  >(opt, gray_georef, color_georef, gray_nodata, color_nodata);  break;
      //case VW_CHANNEL_INT16  : load_inputs_and_process<vw::int16  >(opt);  break;
      case VW_CHANNEL_UINT16 : load_inputs_and_process<vw::uint16 >(opt, gray_georef, color_georef, gray_nodata, color_nodata);  break;
      //case VW_CHANNEL_INT32  : load_inputs_and_process<vw::int32  >(opt);  break;
      //case VW_CHANNEL_UINT32 : load_inputs_and_process<vw::uint32 >(opt);  break;
      //case VW_CHANNEL_FLOAT32: load_inputs_and_process<vw::float32>(opt);  break;
      //case VW_CHANNEL_FLOAT64: load_inputs_and_process<vw::float64>(opt);  break;
      default : vw_throw(ArgumentErr() << "Input image format " << input_data_type << " is not supported!\n");
    };

  } ASP_STANDARD_CATCHES;

  return 0;
}
