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


#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography/GeoReference.h>

using namespace vw;

// Functors
template <class Arg1T, class Arg2T>
struct ReplaceChannelFunc: public ReturnFixedType<Arg1T> {
private:
  int m_channel;
public:
  ReplaceChannelFunc( int const& channel ) : m_channel(channel) {}

  inline Arg1T operator()( Arg1T const& arg1,
                           Arg2T const& arg2 ) const {
    Arg1T t( arg1 );
    t[m_channel] = arg2;
    return t;
  }
};

template <class Image1T, class Image2T>
inline BinaryPerPixelView<Image1T, Image2T, ReplaceChannelFunc<typename Image1T::pixel_type, typename Image2T::pixel_type> >
replace_channel( ImageViewBase<Image1T> const& image1,
                 int const& channel,
                 ImageViewBase<Image2T> const& image2 ) {
  typedef ReplaceChannelFunc<typename Image1T::pixel_type, typename Image2T::pixel_type> func_type;
  return BinaryPerPixelView<Image1T, Image2T, func_type >( image1.impl(), image2.impl(), func_type(channel));
}

// Standard Arguments
struct Options : public vw::cartography::GdalWriteOptions {
  std::string input_rgb, input_gray;
  std::string output_file;
};

// Image Operations
template <class ChannelT>
void do_merge(Options const& opt) {
  DiskImageView<PixelGray<ChannelT> > shaded_image( opt.input_gray );
  DiskImageView<PixelRGB<ChannelT> > rgb_image( opt.input_rgb );
  cartography::GeoReference georef;
  cartography::read_georeference(georef, opt.input_rgb);

  ImageViewRef<PixelRGB<ChannelT> > result =
    pixel_cast<PixelRGB<ChannelT> >(replace_channel(pixel_cast<PixelHSV<ChannelT> >(rgb_image),2,shaded_image));

  bool has_georef = true;
  bool has_nodata = false;
  double nodata = -std::numeric_limits<float>::max(); // smallest float

  block_write_gdal_image( opt.output_file, result, has_georef, georef, has_nodata, nodata, opt,
                          TerminalProgressCallback("tools.hsv_merge","Writing:") );
}

// Handle input
int main( int argc, char *argv[] ) {

  Options opt;

  try {
    po::options_description general_options("Description: Mimicks hsv_merge.py by Frank Warmerdam and Trent Hare. Use it to combine results from gdaldem.");
    general_options.add_options()
      ("output-file,o", po::value(&opt.output_file), "Specify the output file.");
    general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

    po::options_description positional_options("");
    positional_options.add_options()
      ("input-rgb", po::value(&opt.input_rgb), "Explicitly specify the input rgb image.")
      ("input-gray", po::value(&opt.input_gray), "Explicitly specify the input gray image.");

    po::positional_options_description positional_desc;
    positional_desc.add("input-rgb", 1 );
    positional_desc.add("input-gray", 1 );

    std::string usage("[options] <input rgb> <input gray>");
    bool allow_unregistered = false;
    std::vector<std::string> unregistered;
    po::variables_map vm =
      asp::check_command_line( argc, argv, opt, general_options, general_options,
                               positional_options, positional_desc, usage,
                               allow_unregistered, unregistered );

    if ( opt.input_rgb.empty() || opt.input_gray.empty() )
      vw_throw( ArgumentErr() << "Missing required input files.\n"
                << usage << general_options );

    // Get the input RGB's type
    boost::shared_ptr<DiskImageResource> rsrc = DiskImageResourcePtr(opt.input_rgb);
    ChannelTypeEnum channel_type = rsrc->channel_type();

    switch( channel_type ) {
    case VW_CHANNEL_UINT8: do_merge<uint8>( opt ); break;
    case VW_CHANNEL_INT16: do_merge<int16>( opt ); break;
    case VW_CHANNEL_UINT16: do_merge<uint16>( opt ); break;
    default: do_merge<float32>( opt );
    }

  } catch ( Exception const& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}
