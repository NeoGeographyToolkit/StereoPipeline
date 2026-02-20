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


/// \file coverage_fraction.cc
///
/// This tool should provide the best estimate of how much coverage a stereo
/// call provided of the theoretically possible coverage.  Currently it is 
/// based off of the percentage of the left input image.

#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>

#include <vw/Core/StringUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/BlockImageOperator.h>
#include <vw/Image/Filter.h>

#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// Compute the error norm from a point cloud image file.
/// - This is a simpler version of the function in point2dem.cc
template<int num_ch>
ImageViewRef<double> error_norm(std::string const& pc_file){

  // Read the error channels from the point cloud, and take the norm

  const int beg_ech = 3; // errors start at this channel
  const int num_ech = num_ch - beg_ech; // number of error channels

  typedef Vector<double, num_ch > PixelInT;
  typedef Vector<double, num_ech> PixelErrT;
  vw::ImageViewRef<PixelInT> I = vw::DiskImageView<PixelInT>(pc_file);

  ImageViewRef<PixelErrT> error_channels =
    select_channels<num_ech, num_ch, double>(I, beg_ech);

  return per_pixel_filter(error_channels, asp::VectorNorm<PixelErrT>());
}


class Options : public vw::GdalWriteOptions {
public:
  std::string input_prefix;
  float error_cutoff;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("error-cutoff", po::value(&opt.error_cutoff)->default_value(0));

  po::options_description positional("");
  positional.add_options()
    ("input-prefix", po::value<std::string>(), "Input prefix");

  po::positional_options_description positional_desc;
  positional_desc.add("input-prefix", -1);

  std::string usage("[options] <input-prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if (vm.count("input-prefix") == 0)
    vw_throw( ArgumentErr() << "Missing input prefix.\n" << usage << general_options );
  opt.input_prefix = vm["input-prefix"].as<std::string>();

}

/// Count the number of pixels above a threshold on a per-block basis.
class ValidPixelCounterFunctor {
  
  double m_error_threshold;
  uint64 m_masked_count;
  uint64 m_invalid_count;
  Mutex  m_mutex;
  
public:
  
  ValidPixelCounterFunctor(double threshold)
    : m_error_threshold(threshold), m_masked_count(0), m_invalid_count(0) {}

  uint64 get_masked_count () const {return m_masked_count; }
  uint64 get_invalid_count() const {return m_invalid_count;}

  template <class T>
  void operator()(ImageView<T> const& image, BBox2i const& bbox) {
    // Count the pixels where the value is zero or >= the threshold.
    uint64 local_invalid_count = 0;
    uint64 local_masked_count  = 0;
    for (std::int64_t r=0; r<bbox.height(); ++r) {
      for (std::int64_t c=0; c<bbox.width(); ++c) {
        typename ImageView<T>::pixel_type value = image(c,r);
        if (value < 0.0) { // These pixels are outside the left mask.
          ++local_masked_count;
          continue;
        }
        if ((value == 0) || (value >= m_error_threshold)) {
          ++local_invalid_count; // Bad triangulation
        }
      }
    }
    // Safely add to the shared totals.
    m_mutex.lock();
    m_masked_count  += local_masked_count;
    m_invalid_count += local_invalid_count;
    m_mutex.unlock();
  }
};


//-----------------------------------------------------------------------------------

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Find the two required input files
    std::string lmask_file = opt.input_prefix + "-lMask.tif";
    std::string pc_file    = opt.input_prefix + "-PC.tif";

    if (!boost::filesystem::exists(lmask_file)) {
      vw_out() << "Required left mask file not found: "
               << lmask_file << std::endl;
      return 0;
    }
    if (!boost::filesystem::exists(pc_file)) {
      vw_out() << "Required point cloud file not found: "
               << pc_file << std::endl;
      return 0;
    }

    // Each thread will process a block of this tile size.
    std::int64_t tile_size = asp::ASPGlobalOptions::tri_tile_size();


    // Compute the error norm from the PC file, and apply a mask to it copied from the lmask file.
    // - Zero error values are a flag for a bad pixel, left over from stereo_tri.
    // - Values outside the left mask are set to a flag value of -1, they are not counted
    //   in the computations.
    std::int64_t num_pc_channels = get_num_channels(pc_file);
    ImageViewRef<double> masked_error_image;
    switch(num_pc_channels) {
      case 4:
        masked_error_image = 
          apply_mask(copy_mask(error_norm<4>(pc_file), 
                              create_mask(DiskImageView<uint8>(lmask_file))), -1.0);
        break;
      case 6:
        masked_error_image = 
          apply_mask(copy_mask(error_norm<6>(pc_file), 
                              create_mask(DiskImageView<uint8>(lmask_file))), -1.0);
        break;
      default:
        vw_out() << "Error: The point cloud file must have 4 or 6 channels!\n";
      return 0;
    };

    // Use BlockImageOperator to count invalid pixels in parallel threads.
    ValidPixelCounterFunctor functor(opt.error_cutoff);

    //write_image("debug.tif", masked_error_image);

    const Vector2i block_size(tile_size, tile_size);
    block_op_cache(masked_error_image, functor, block_size);

    std::int64_t num_masked  = functor.get_masked_count();
    std::int64_t num_invalid = functor.get_invalid_count();
    std::int64_t num_rows    = masked_error_image.rows();
    std::int64_t num_cols    = masked_error_image.cols();
    std::int64_t num_pixels  = num_rows * num_cols;
    std::int64_t num_unmasked_pixels = num_pixels - num_masked;
    std::int64_t num_valid   = num_unmasked_pixels - num_invalid;

    double percent_invalid = static_cast<double>(num_invalid) / static_cast<double>(num_unmasked_pixels);

    std::cout << "Total number of pixels:     " << num_pixels      << std::endl;
    std::cout << "Number of valid pixels:     " << num_valid       << std::endl;
    std::cout << "Percentage of valid pixels: " << 1.0-percent_invalid << std::endl;

  } ASP_STANDARD_CATCHES;

  return 0;
}
