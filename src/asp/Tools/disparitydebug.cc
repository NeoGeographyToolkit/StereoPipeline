// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file disparitydebug.cc
///

#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <stdlib.h>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Core/Macros.h>
using namespace vw;
using namespace vw::stereo;

struct Options {
  // Input
  std::string input_file_name;

  // Output
  bool float_output;
  std::string output_prefix, output_file_type;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o", po::value(&opt.output_prefix), "Specify the output prefix")
    ("output-filetype,t", po::value(&opt.output_file_type)->default_value("tif"), "Specify the output file")
    ("float-pixels", "Save the resulting debug images as 32 bit floating point files (if supported by the selected files type.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.input_file_name), "Input disparity map");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <input disparity map> \n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.input_file_name.empty() )
    vw_throw( ArgumentErr() << "Missing input file!\n"
              << usage.str() << general_options );
  if ( opt.output_prefix == "" )
    opt.output_prefix = fs::path(opt.input_file_name).stem();
  opt.float_output = vm.count("float-pixels");
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    vw_out() << "Opening " << opt.input_file_name << "\n";
    DiskImageView<PixelMask<Vector2f> > disk_disparity_map(opt.input_file_name);

    vw_out() << "\t--> Computing disparity range \n";
    BBox2 disp_range = get_disparity_range(disk_disparity_map);
    vw_out() << "\t    Horizontal - [" << disp_range.min().x()
             << " " << disp_range.max().x() << "]    Vertical: ["
             << disp_range.min().y() << " " << disp_range.max().y() << "]\n";

    ImageViewRef<float32> horizontal = apply_mask(copy_mask(clamp(normalize(select_channel(disk_disparity_map,0), disp_range.min().x(), disp_range.max().x(),0,1)),disk_disparity_map));
    ImageViewRef<float32> vertical = apply_mask(copy_mask(clamp(normalize(select_channel(disk_disparity_map,1), disp_range.min().y(), disp_range.max().y(),0,1)),disk_disparity_map));

    vw_out() << "\t--> Saving disparity debug images\n";
    if (opt.float_output) {
      write_image( opt.output_prefix + "-H." + opt.output_file_type, horizontal,
                   TerminalProgressCallback("asp","\t    Left  : "));
      write_image( opt.output_prefix + "-V." + opt.output_file_type, vertical,
                   TerminalProgressCallback("asp","\t    Right : "));
    } else {
      write_image( opt.output_prefix + "-H." + opt.output_file_type,
                   channel_cast_rescale<uint8>(horizontal),
                   TerminalProgressCallback("asp","\t    Left  : "));
      write_image( opt.output_prefix + "-V." + opt.output_file_type,
                   channel_cast_rescale<uint8>(vertical),
                   TerminalProgressCallback("asp","\t    Right : "));
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
