#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <stdlib.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Stereo.h>
using namespace vw;
using namespace vw::stereo;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
}

int main( int argc, char *argv[] ) {

  

  set_debug_level(VerboseDebugMessage+11);

  std::string input_file_name, output_prefix, output_file_type;
  unsigned cache_size;
  
  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1024), "Cache size, in megabytes")
    ("input-file", po::value<std::string>(&input_file_name), "Explicitly specify the input file")
    ("output-prefix,o", po::value<std::string>(&output_prefix)->default_value("disparity"), "Specify the output prefix")
    ("output-filetype,t", po::value<std::string>(&output_file_type)->default_value("jpg"), "Specify the output file");

  po::positional_options_description p;
  p.add("input-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  Cache::system_cache().resize( cache_size*1024*1024 ); 

  if( vm.count("help") ) {
    std::cout << desc << std::endl;
    return 1;
  }

  if( vm.count("input-file") != 1 ) {
    std::cout << "Error: Must specify exactly one input file!" << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  DiskImageView<PixelDisparity<float> > disk_disparity_map(input_file_name);

  std::cout << "Computing disparity range... " << std::flush;
  BBox2 disp_range = disparity::get_disparity_range(disk_disparity_map);
  std::cout << disp_range << "\n";
  write_image( output_prefix + "-H." + output_file_type, normalize(clamp(select_channel(disk_disparity_map,0), disp_range.min().x(), disp_range.max().x())), TerminalProgressCallback());
  write_image( output_prefix + "-V." + output_file_type, normalize(clamp(select_channel(disk_disparity_map,1), disp_range.min().y(), disp_range.max().y())), TerminalProgressCallback());

  return 0;
}
