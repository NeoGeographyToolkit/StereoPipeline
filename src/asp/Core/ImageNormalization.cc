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

#include <asp/Core/ImageNormalization.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/ImageUtils.h>

#include <vw/FileIO/DiskImageResource.h>
#include <vw/Core/Log.h>
#include <vw/Image/Algorithms.h>
#include <vw/FileIO/DiskImageUtils.h>

#include <boost/filesystem.hpp>

#include <limits>

using namespace vw;
namespace fs = boost::filesystem;

namespace asp {
  
/// Returns the correct nodata value from the input images or the input options.
/// The user_nodata value overrides the image no-data values if it is not NaN.
void get_nodata_values(boost::shared_ptr<vw::DiskImageResource> left_rsrc,
                       boost::shared_ptr<vw::DiskImageResource> right_rsrc,
                       float user_nodata,
                       float & left_nodata_value,
                       float & right_nodata_value){
  
  // The no-data value read from options overrides the value present in the image files.
  left_nodata_value  = std::numeric_limits<float>::quiet_NaN();
  right_nodata_value = std::numeric_limits<float>::quiet_NaN();
  if (left_rsrc->has_nodata_read ()) left_nodata_value  = left_rsrc->nodata_read();
  if (right_rsrc->has_nodata_read()) right_nodata_value = right_rsrc->nodata_read();
  
  if (!std::isnan(user_nodata)) {
    
    if (user_nodata < left_nodata_value)
      vw_out(WarningMessage) 
        << "It appears that the user-supplied no-data value is less than "
        << "the no-data value of left image. This may not be what was "
        << "intended.\n";
    if (user_nodata < right_nodata_value)
      vw_out(WarningMessage) 
        << "It appears that the user-supplied no-data value is less than "
        << "the no-data value of right image. This may not be what was "
        << "intended.\n";

    left_nodata_value  = user_nodata;
    right_nodata_value = user_nodata;
  }

  return;
}

// For OpenCV image detectors some things are a bit different. Used in a few places.
bool openCvDetectMethod() {
  return (asp::stereo_settings().ip_detect_method != DETECT_IP_METHOD_INTEGRAL);
}

// If it is allowed to exceed the min and max values when normalizing images.
// TODO(oalexan1): This now returns false. This must be switched to true, and
// must integrate the logic for ISIS cub files where it is always true.
bool doNotExceedMinMax() {
  return false;
}

// Calculate the min and max for all images to normalize, while respecting
// given options.
void calcImageSeqMinMax(bool force_use_entire_range,
                        bool individually_normalize,
                        bool use_percentile_stretch,
                        bool do_not_exceed_min_max,
                        std::vector<vw::Vector<float>> const& image_stats,
                        // Outputs
                        std::vector<double> & min_vals,
                        std::vector<double> & max_vals) {

  // TODO(oalexan1): Find global min and max in either case
  // and do not exceed these in either case. There's nothing
  // to gain by exceeding those for all images.
  
  // Some of these will need a global min and max
  double global_min = std::numeric_limits<double>::max();
  double global_max = -std::numeric_limits<double>::min();
  min_vals.resize(image_stats.size(), global_min);
  max_vals.resize(image_stats.size(), global_max);
  
  // Iterate through the images
  bool warning_printed = false;
  for (size_t i = 0; i < image_stats.size(); i++) {
    
    // Sanity check
    if (image_stats[i].size() != 6)
      vw::vw_throw(vw::ArgumentErr() 
        << "Expecting an image stats vector of size 6 in normalize_images().\n");
    
    if (force_use_entire_range) { 
                                
      if (individually_normalize) {
        // Use min and max values for each image
        min_vals[i] = image_stats[i][0];
        max_vals[i] = image_stats[i][1];
      } else { 
        // Normalize all using the same min and max
        global_min = std::min(global_min, (double)image_stats[i][0]);
        global_max = std::max(global_max, (double)image_stats[i][1]);
      }

    } else { 
            
      // Don't force the entire range
      if (use_percentile_stretch) {
        // Percentile stretch
        min_vals[i] = image_stats[i][4];
        max_vals[i] = image_stats[i][5];
      } else {
        // Two standard deviation stretch
        
        // Standard deviation must be positive
        if (image_stats[i][3] == 0) {
          // This can happen if the image is completely blank. It is better to
          // give a warning than fail.
          vw::vw_out(vw::WarningMessage)  
            << "The image stats do not have a positive stddev. "
            << "Try using the option --force-use-entire-range.\n";
        }
        
        // This is an important check for when images come from different
        // sensors and have vastly different standard deviations.
        if (!individually_normalize && i > 0 && !warning_printed) {
          double min_std = std::min(image_stats[i][3], image_stats[i-1][3]);
          double max_std = std::max(image_stats[i][3], image_stats[i-1][3]);
          double std_ratio = min_std / std::max(max_std, 1e-10);
          
          if (std_ratio < 0.2) {
            vw::vw_out(vw::WarningMessage) 
              << "The standard deviations of some images are very different. "
              << "Consider using the option --individually-normalize.\n";
            warning_printed = true;
          }
        }
        
        // Use mean  +/- 2 standard deviations. This is done in a few places in ASP.
        min_vals[i] = image_stats[i][2] - 2*image_stats[i][3];
        max_vals[i] = image_stats[i][2] + 2*image_stats[i][3];
      }
      
      if (!individually_normalize) {
        // Normalize all using the same min and max
        global_min = std::min(global_min, min_vals[i]);
        global_max = std::max(global_max, max_vals[i]);
      }
      
    } // end if (!force_use_entire_range)
  } // end for loop

  if (!individually_normalize) {
    // All get the same global min and max. 
    for (size_t i = 0; i < image_stats.size(); i++) {
      min_vals[i] = global_min;
      max_vals[i] = global_max;
    }
  } 
    
  if (do_not_exceed_min_max) {
    // This is important for ISIS which may have special pixels beyond the min and max
    for (size_t i = 0; i < image_stats.size(); i++) {
      min_vals[i] = std::max(min_vals[i], (double)image_stats[i][0]);
      max_vals[i] = std::min(max_vals[i], (double)image_stats[i][1]);
    }    
  }
  
  return;
}

/// Normalize the intensity of two images based on input statistics
void normalize_images(bool force_use_entire_range,
                      bool individually_normalize,
                      bool use_percentile_stretch,
                      bool do_not_exceed_min_max,
                      vw::Vector<float> const& left_stats,
                      vw::Vector<float> const& right_stats,
                      vw::ImageViewRef<vw::PixelMask<float>> & left_img,
                      vw::ImageViewRef<vw::PixelMask<float>> & right_img) {

  // Get the min and max values for the two images  
  std::vector<double> min_vals, max_vals;
  std::vector<vw::Vector<float>> image_stats = {left_stats, right_stats};
  calcImageSeqMinMax(force_use_entire_range, individually_normalize,
                     use_percentile_stretch, do_not_exceed_min_max,
                     image_stats,
                     // Outputs
                     min_vals, max_vals);
  
  if (individually_normalize > 0)
    vw::vw_out() << "\t--> Individually normalize images.\n";
  else // Normalize using the same stats
    vw::vw_out() << "\t--> Normalize images globally.\n";
  
  vw::vw_out() << "Left image value bounds: [" 
                 << min_vals[0] << " " << max_vals[0] << "]\n";
  vw::vw_out() << "Right image value bounds: [" 
                 << min_vals[1] << " " << max_vals[1] << "]\n";
  
  left_img = normalize(left_img, min_vals[0], max_vals[0], 0.0, 1.0);
  right_img = normalize(right_img, min_vals[1], max_vals[1], 0.0, 1.0);

  return;
}

// Normalize a single image to [0, 1] using mean and stddev-derived bounds.
// Do not exceed the min and max values present in the image stats. This may 
// need to be the default.
void normalizeImage(std::string const& image_file,
                    vw::ImageViewRef<vw::PixelMask<float>> & image) {

  // Read the nodata value
  float nodata_val = -32768.0; 
  vw::read_nodata_val(image_file, nodata_val);

  // Open the file and apply the mask
  image = vw::create_mask(vw::DiskImageView<float>(image_file), nodata_val);

  // Gather stats
  std::string tag = image_file;
  std::string prefix = ""; 
  bool reuse_cache = false;
  vw::Vector6f stats = asp::gather_stats(image, tag, prefix, image_file, reuse_cache);
  
  // Use mean +/- 2 standard deviations. This is done in a few places in ASP.
  float mean = stats[2];
  float stddev = stats[3];
  double min_val = std::max(stats[0], mean - 2*stddev);
  double max_val = std::min(stats[1], mean + 2*stddev);
  
  image = normalize(image, min_val, max_val, 0.0, 1.0);
}
   
// This is called by parallel_bundle_adjust just once to accumulate all stats that
// were done by individual processes.
void calcNormalizationBounds(std::string const& out_prefix, 
                             std::vector<std::string> const& image_files,
                             std::string const& boundsFile) { 

  int num_images = image_files.size();

  std::vector<vw::Vector<float>> image_stats(num_images); 
  // Assign the images which this instance should compute statistics for.
  for (int i = 0; i < num_images; i++) {

    std::string image_path = image_files[i];
    std::string stats_path 
      = out_prefix + '-' + fs::path(image_path).stem().string() + "-stats.tif";
    
    // Read the stats
    vw::read_vector(image_stats[i], stats_path);
  
    // Sanity check
    if (image_stats[i].size() != 6)
      vw::vw_throw(vw::ArgumentErr() 
        << "Expecting an image stats vector of size 6.\n");
  }

  std::vector<double> min_vals, max_vals;
  asp::calcImageSeqMinMax(asp::stereo_settings().force_use_entire_range, 
                          asp::stereo_settings().individually_normalize,
                          asp::openCvDetectMethod(),
                          asp::doNotExceedMinMax(),
                          image_stats,
                          // Outputs
                          min_vals, max_vals);

  vw::vw_out() << "Writing: " << boundsFile << "\n";
  std::ofstream out(boundsFile.c_str());
  out.precision(17);
  for (int i = 0; i < num_images; i++)
    out << image_files[i] << " " << min_vals[i] << " " << max_vals[i] << "\n";
  
} // end function calcNormalizationBounds()

// Read normalization bounds into std::map
void readNormalizationBounds(std::string const& boundsFile,
                             std::vector<std::string> const& image_files,
                             std::map<std::string, vw::Vector2> & bounds_map) {

  std::ifstream in(boundsFile.c_str());
  if (!in) {
    vw::vw_throw(vw::IOErr() << "Unable to open normalization bounds file: " 
      << boundsFile << "\n");
  }
  
  std::string image_file;
  double min_val, max_val;
  while (in >> image_file >> min_val >> max_val) {
    vw::Vector2 bounds(min_val, max_val);
    bounds_map[image_file] = bounds;
  }
  
  // Check that data for all images was read
  for (size_t i = 0; i < image_files.size(); i++) {
    std::string image_file = image_files[i];
    if (bounds_map.find(image_file) == bounds_map.end()) {
      vw::vw_throw(vw::IOErr() << "Normalization bounds not found for image: " 
        << image_file << "\n");
    }
  }
  
} // end function readNormalizationBounds()

} // end namespace asp
