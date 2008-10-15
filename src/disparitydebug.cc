// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
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

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Stereo.h>
using namespace vw;
using namespace vw::stereo;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

int main( int argc, char *argv[] ) {
  std::string input_file_name, output_prefix = "", output_file_type;
  unsigned cache_size;
  int debug_level;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1024), "Cache size, in megabytes")
    ("input-file", po::value<std::string>(&input_file_name), "Explicitly specify the input file")
    ("output-prefix,o", po::value<std::string>(&output_prefix), "Specify the output prefix")
    ("output-filetype,t", po::value<std::string>(&output_file_type)->default_value("tif"), "Specify the output file")
    ("debug-level,d", po::value<int>(&debug_level)->default_value(vw::DebugMessage-1), "Set the debugging output level. (0-50+)")
    ("float-pixels", "Save the resulting debug images as 32 bit floating point files (if supported by the selected files type.");

  po::positional_options_description p;
  p.add("input-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  Cache::system_cache().resize( cache_size*1024*1024 ); 
  set_debug_level(debug_level);

  if( vm.count("help") ) {
    std::cout << desc << std::endl;
    return 1;
  }

  if( vm.count("input-file") != 1 ) {
    std::cout << "Error: Must specify exactly one input file!" << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  if( output_prefix == "" ) {
    output_prefix = prefix_from_filename(input_file_name);
  }

  std::cout << "Opening " << input_file_name << "\n";
  DiskImageView<PixelDisparity<float> > disk_disparity_map(input_file_name);

  std::cout << "Computing disparity range... \n";
  int num_good;
  BBox2 disp_range = disparity::get_disparity_range(disk_disparity_map,num_good,true,TerminalProgressCallback());
  if (vm.count("float-pixels")) {
    write_image( output_prefix + "-H." + output_file_type, clamp(normalize(select_channel(disk_disparity_map,0), disp_range.min().x(), disp_range.max().x(),0,1)), TerminalProgressCallback());
    write_image( output_prefix + "-V." + output_file_type, clamp(normalize(select_channel(disk_disparity_map,1), disp_range.min().y(), disp_range.max().y(),0,1)), TerminalProgressCallback());
  } else {
    write_image( output_prefix + "-H." + output_file_type, channel_cast_rescale<uint8>(clamp(normalize(select_channel(disk_disparity_map,0), disp_range.min().x(), disp_range.max().x(),0,1))), TerminalProgressCallback());
    write_image( output_prefix + "-V." + output_file_type, channel_cast_rescale<uint8>(clamp(normalize(select_channel(disk_disparity_map,1), disp_range.min().y(), disp_range.max().y(),0,1))), TerminalProgressCallback());
  }    

  return 0;
}
