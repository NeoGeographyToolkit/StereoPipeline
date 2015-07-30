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
/// A simple tool to merge multiple point cloud files into a single file.

#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/InpaintView.h>

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
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : asp::BaseOptions {
  // Input
  std::vector<std::string> pointcloud_files;

  // Settings
  //float nodata_value;
  bool  write_double;  ///< If true, output file is double instead of float

  // Output
  std::string out_path;

  // Defaults that the user doesn't need to see.
  Options() : //nodata_value(-std::numeric_limits<float>::max()), 
              write_double(false)
             {}
};


void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General Options");
  general_options.add_options()
    //("nodata-value",      po::value(&opt.nodata_value)->default_value(-std::numeric_limits<float>::max()),
    //         "Set the nodata value.")
    ("output-path,o",   po::value(&opt.out_path),                             "Specify the output path.")
    ("write-double,d", po::value(&opt.write_double)->default_value(false), "Write a double precision output file.");

  general_options.add( asp::BaseOptionsDescription(opt) );

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

  //// Turn on logging to file
  //asp::log_to_file(argc, argv, "", opt.out_prefix);


}


/// Read given files and form an image composite.
/// - Each input point cloud is a 2d arrangement of 3d points.
/// - This function combines them into a simple patchwork mosaic without
///   regard for the actual xyz values of the points.
template<class PixelT>
ImageViewRef<PixelT> form_composite(std::vector<std::string> const & files){

  VW_ASSERT(files.size() >= 1, ArgumentErr() << "Expecting at least one file.\n");

  // TODO: Is there any requirement for the spacing?
  int spacing = 1;

  vw::mosaic::ImageComposite<PixelT> composite_image;
  composite_image.set_draft_mode(true); // Images will be disjoint, no need for fancy stuff

  for (int i = 0; i < (int)files.size(); i++){

    ImageViewRef<PixelT> I = asp::read_cloud< math::VectorSize<PixelT>::value >(files[i]);

    // We will stack the images in the composite side by side. Images which
    // are wider than tall will be transposed.
    // To do: A more efficient approach would be to also stack one
    // image on top of each other, if some images are not too tall.
    // --> Does this code ever get images where rows != cols??
    if (I.rows() < I.cols())
      I = transpose(I);

    int start = composite_image.cols();
    if (i > 0){
      // Insert the spacing
      start = spacing*(int)ceil(double(start)/spacing) + spacing;
    }
    composite_image.insert(I, start, 0);

  } // End loop through files

  return composite_image;
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
  for (size_t i=0; i<pc_files.size(); ++i) {
    // Read in the shift from each cloud and accumulate them
    std::string shift_str;
    boost::shared_ptr<vw::DiskImageResource> rsrc( new vw::DiskImageResourceGDAL(pc_files[i]) );
    if (vw::cartography::read_header_string(*rsrc.get(), asp::POINT_OFFSET, shift_str)){
      //std::cout << "shift string = " << shift_str << std::endl;
      shift += asp::str_to_vec<vw::Vector3>(shift_str);
    }
  }
  // Compute the mean shift
  // - It would be more accurate to weight the shift according to the number of input points
  shift /= static_cast<double>(pc_files.size());
  return shift;
}

/// Write the merged point cloud to disk using the selected options.
template <class ImageT>
void save_point_cloud(Vector3     const& shift, 
                      ImageT      const& point_cloud,
                      std::string const& point_cloud_file,
                      Options     const& opt){

  vw_out() << "Writing point cloud: " << point_cloud_file << "\n";

  // If shift != zero then this will cast the output data to type float.
  //  Otherwise it will keep its data type.
  double point_cloud_rounding_error = 0.0;
  asp::block_write_approx_gdal_image
    ( point_cloud_file, shift,
      point_cloud_rounding_error,
      point_cloud, opt,
      TerminalProgressCallback("asp", "\t--> Merging: "));

}

// Do the actual work of loading, merging, and saving the point clouds
template <class PixelT>
void do_work(Vector3 const& shift, Options const& opt) {

  ImageViewRef<PixelT> merged_cloud = form_composite<PixelT>(opt.pointcloud_files);
  save_point_cloud(shift, merged_cloud, opt.out_path, opt);
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

    //std::cout << "shift = " << shift << std::endl;

    // The code has to branch here depending on the number of channels
    switch (num_channels)
    {
      // The input point clouds have their shift incorporated and are stored as doubles.
      // If the output file is stored as float, it needs to have a single shift value applied.
      case 3:  do_work<Vector3>(shift, opt); break;
      case 4:  do_work<Vector4>(shift, opt); break;
      case 6:  do_work<Vector6>(shift, opt); break;
      default: vw_throw( ArgumentErr() << "Unsupported number of channels!.\n" );
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}



