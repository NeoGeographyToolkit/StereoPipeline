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


/// \file pc_merge.cc
///
/// A simple tool to merge multiple point cloud files into a single file. The clouds
/// can have 1 channel (plain raster images) or 3 to 6 channels.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/OrthoRasterizer.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/PointImageManipulation.h>

#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,4> Vector4;
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector4f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::vector<std::string> pointcloud_files;

  // Settings
  bool  write_double;  ///< If true, output file is double instead of float

  // Output
  std::string out_file;

  Options() : write_double(false) {}
};


void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("output-file,o",  po::value(&opt.out_file)->default_value(""),        "Specify the output file.")
    ("write-double,d", po::value(&opt.write_double)->default_value(false), "Write a double precision output file.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value< std::vector<std::string> >(), "Input files");

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <point-clouds> ");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if (vm.count("input-files") == 0)
    vw_throw( ArgumentErr() << "Missing input point clouds.\n"
                            << usage << general_options );
  opt.pointcloud_files = vm["input-files"].as< std::vector<std::string> >();

  if (opt.out_file == "")
    vw_throw( ArgumentErr() << "The output file must be specified!\n"
              << usage << general_options );

  vw::create_out_dir(opt.out_file);
}


/// Throws if the input point clouds do not have the same number of channels.
/// - Returns the number of channels.
int check_num_channels(std::vector<std::string> const& pc_files){
  VW_ASSERT(pc_files.size() >= 1,
            ArgumentErr() << "Expecting at least one file.\n");

  int target_num = get_num_channels(pc_files[0]);
  for (int i = 1; i < (int)pc_files.size(); ++i){
    int num_channels = get_num_channels(pc_files[i]);
    if (num_channels != target_num)
      vw_throw( ArgumentErr() << "Input point clouds must all have the same number of channels!.\n" );
  }
  return target_num;
}

/// Determine the common shift value to use for the output files
Vector3 determine_output_shift(std::vector<std::string> const& pc_files, Options const& opt){

  // If writing to double format, no shift is needed.
  if (opt.write_double)
    return Vector3(0,0,0);

  // As an approximation, compute the mean shift vector of the input files.
  // - If none of the input files have a shift, the output file will be written as a double.
  vw::Vector3 shift(0,0,0), shiftIn;
  double shift_count = 0;
  for (size_t i=0; i<pc_files.size(); ++i) {
    // Read in the shift from each cloud and accumulate them
    std::string shift_str;
    boost::shared_ptr<vw::DiskImageResource> rsrc( new vw::DiskImageResourceGDAL(pc_files[i]) );
    if (vw::cartography::read_header_string(*rsrc.get(), asp::ASP_POINT_OFFSET_TAG_STR, shift_str)){
      //std::cout << "shift string = " << shift_str << std::endl;
      shift += asp::str_to_vec<vw::Vector3>(shift_str);
      shift_count += 1.0;
    }
  }
  if (shift_count < 0.9) // If no shifts read, don't use a shift.
    return Vector3(0,0,0);

  // Compute the mean shift
  // - It would be more accurate to weight the shift according to the number of input points
  shift /= shift_count;
  return shift;
}


// Do the actual work of loading, merging, and saving the point clouds

// Case 1: Single-channel cloud.
template <class PixelT>
typename boost::enable_if<boost::is_same<PixelT, vw::PixelGray<float> >, void >::type
do_work(Vector3 const& shift, Options const& opt) {
  // The spacing is selected to be compatible with the point2dem convention.
  const int spacing = asp::OrthoRasterizerView::max_subblock_size();
  ImageViewRef<PixelT> merged_cloud = asp::form_point_cloud_composite<PixelT>(opt.pointcloud_files, spacing);

  vw_out() << "Writing image: " << opt.out_file << "\n";

  bool has_georef = false;
  bool has_nodata = false;
  double nodata = -std::numeric_limits<float>::max(); // smallest float
  GeoReference georef;
  vw::cartography::block_write_gdal_image(opt.out_file, merged_cloud, has_georef,
                              georef,  has_nodata, nodata, opt,
                              TerminalProgressCallback("asp", "\t--> Merging: "));
}

// Case 2: Multi-channel cloud.
template <class PixelT>
typename boost::disable_if<boost::is_same<PixelT, vw::PixelGray<float> >, void >::type
do_work(Vector3 const& shift, Options const& opt) {
  // The spacing is selected to be compatible with the point2dem convention.
  const int spacing = asp::OrthoRasterizerView::max_subblock_size();
  ImageViewRef<PixelT> merged_cloud = asp::form_point_cloud_composite<PixelT>(opt.pointcloud_files, spacing);

  // See if we can pull a georeference from somewhere. Of course it will be wrong
  // when applied to the merged cloud, but it will at least have the correct datum
  // and projection.
  bool has_georef = false;
  cartography::GeoReference georef;
  for (size_t i = 0; i < opt.pointcloud_files.size(); i++){
    cartography::GeoReference local_georef;

    if (read_georeference(local_georef, opt.pointcloud_files[i])){
      georef = local_georef;
      has_georef = true;
    }
  }

  bool has_nodata = false;
  double nodata = -std::numeric_limits<float>::max(); // smallest float

  vw_out() << "Writing point cloud: " << opt.out_file << "\n";

  // If shift != zero then this will cast the output data to type float.
  //  Otherwise it will keep its data type.
  double point_cloud_rounding_error = 0.0;
  asp::block_write_approx_gdal_image
    ( opt.out_file, shift,
      point_cloud_rounding_error,
      merged_cloud,
      has_georef, georef, has_nodata, nodata,
      opt, TerminalProgressCallback("asp", "\t--> Merging: "));
}

//-----------------------------------------------------------------------------------

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Determine the number of channels
    int num_channels = check_num_channels(opt.pointcloud_files);

    // Determine the output shift (if any)
    Vector3 shift = determine_output_shift(opt.pointcloud_files, opt);

    // The code has to branch here depending on the number of channels
    switch (num_channels)
    {
      // The input point clouds have their shift incorporated and are stored as doubles.
      // If the output file is stored as float, it needs to have a single shift value applied.
      case 1:  do_work< vw::PixelGray<float> >(shift, opt); break;
      case 3:  do_work<Vector3>(shift, opt); break;
      case 4:  do_work<Vector4>(shift, opt); break;
      case 6:  do_work<Vector6>(shift, opt); break;
      default: vw_throw( ArgumentErr() << "Unsupported number of channels!.\n" );
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
