// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


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
struct Options {
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

  ImageViewRef<PixelRGB<ChannelT> > result = pixel_cast<PixelRGB<ChannelT> >(replace_channel(pixel_cast<PixelHSV<ChannelT> >(rgb_image),2,shaded_image));

  cartography::write_georeferenced_image( opt.output_file, result, georef,
                                          TerminalProgressCallback("tools.hsv_merge","Writing:") );
}

// Handle input
int main( int argc, char *argv[] ) {

  Options opt;

  po::options_description desc("Description: Mimicks hsv_merge.py by Frank Warmerdam and Trent Hare. Use it to combine results from gdaldem.");
  desc.add_options()
    ("input-rgb", po::value<std::string>(&opt.input_rgb), "Explicitly specify the input rgb image.")
    ("input-gray", po::value<std::string>(&opt.input_gray), "Explicitly specify the input gray image.")
    ("output-file,o", po::value<std::string>(&opt.output_file), "Specify the output file.")
    ("help,h", "Display this help message");
  po::positional_options_description p;
  p.add("input-rgb", 1 );
  p.add("input-gray", 1 );

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
    po::notify( vm );
  } catch ( po::error & e ) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << desc << std::endl;
    return 1;
  }

  if( vm.count("help") ||
      (opt.input_rgb == "" && opt.input_gray == "" ) ) {
    std::cout << desc << std::endl;
    return 1;
  }

  try {
    // Get the input RGB's type
    DiskImageResource *rsrc = DiskImageResource::open(opt.input_rgb);
    ChannelTypeEnum channel_type = rsrc->channel_type();
    delete rsrc;

    switch( channel_type ) {
    case VW_CHANNEL_UINT8: do_merge<uint8>( opt ); break;
    case VW_CHANNEL_INT16: do_merge<int16>( opt ); break;
    case VW_CHANNEL_UINT16: do_merge<uint16>( opt ); break;
    default: do_merge<float32>( opt );
    }

  } catch ( Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}
