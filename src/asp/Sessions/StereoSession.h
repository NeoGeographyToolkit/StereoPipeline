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


/// \file StereoSession.h
///

#ifndef __STEREO_SESSION_H__
#define __STEREO_SESSION_H__

#include <vw/Image/ImageViewBase.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Transform.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoModel.h>

#include <boost/shared_ptr.hpp>
#include <asp/Core/Common.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/CameraModelLoader.h>

namespace asp {

  typedef vw::Vector<vw::float32,6> Vector6f;

  // Forward declare this class for constructing StereoSession objects
  class StereoSessionFactory;

  /// Stereo Sessions define for different missions or satellites how to:
  ///   * Initialize, normalize, and align the input imagery
  ///   * Extract the camera model
  ///   * Custom code needed for correlation, filtering, and triangulation.
  class StereoSession {
    friend class StereoSessionFactory; // Needed so the factory can call initialize()

  public:
    virtual ~StereoSession() {}

    /// Simple typedef of a factory function that creates a StereoSession instance
    typedef StereoSession* (*construct_func)();

    /// General init function.
    void initialize (vw::cartography::GdalWriteOptions const& options,
                     std::string const& left_image_file,
                     std::string const& right_image_file,
                     std::string const& left_camera_file,
                     std::string const& right_camera_file,
                     std::string const& out_prefix,
                     std::string const& input_dem);

    // The next set of functions describe characteristics of the derived session class.
    // - These could be made in to some sort of static constant if needed.
    virtual bool isMapProjected() const { return false; } // TODO: Delete?
    virtual bool uses_map_projected_inputs() const {return isMapProjected();}
    virtual bool uses_rpc_map_projection  () const {return isMapProjected();} // Set to false if using another type
    virtual bool requires_input_dem       () const {return isMapProjected();}
    virtual bool supports_image_alignment () const {return !isMapProjected(); }
    virtual bool is_nadir_facing          () const {return !asp::stereo_settings().no_datum; }
    virtual bool supports_multi_threading () const {return true; }


    /// Helper function that retrieves both cameras.
    virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                               boost::shared_ptr<vw::camera::CameraModel> &cam2) const;

    /// Method that produces a Camera Model from input files.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "") const;

    /// Method to help determine what session we actually have
    virtual std::string name() const = 0;

    /// Specialization for how interest points are found
    bool ip_matching(std::string  const& input_file1,
                     std::string  const& input_file2,
                     vw::Vector2  const& uncropped_image_size,
                     Vector6f     const& stats1,
                     Vector6f     const& stats2,
                     int   ip_per_tile,
                     float nodata1, float nodata2,
                     vw::camera::CameraModel* cam1,
                     vw::camera::CameraModel* cam2,
                     std::string const& match_filename,
                     std::string const left_ip_file ="",
                     std::string const right_ip_file=""
                    );

    /// Compute the min, max, mean, and standard deviation of an image object and write them to a log.
    /// - "tag" is only used to make the log messages more descriptive.
    /// - If prefix and image_path is set, will cache the results to a file.
    template <class ViewT> static inline
    Vector6f gather_stats( vw::ImageViewBase<ViewT> const& view_base, std::string const& tag,
                           std::string const& prefix="", std::string const& image_path="");

    /// Normalize the intensity of two grayscale images based on input statistics
    template<class ImageT> static inline
    void normalize_images(bool force_use_entire_range,
                          bool individually_normalize,
                          bool use_percentile_stretch,
                          Vector6f const& left_stats,
                          Vector6f const& right_stats,
                          ImageT & Limg, ImageT & Rimg);

    // If both left-image-crop-win and right-image-crop win are specified,
    // we crop the images to these boxes, and hence the need to keep
    // the upper-left corners of the crop windows to handle the cameras correctly.
    static vw::Vector2 camera_pixel_offset(std::string const& input_dem,
                                           std::string const& left_image_file,
                                           std::string const& right_image_file,
                                           std::string const& curr_image_file);

    // If we have adjusted camera models, load them. The adjustment
    // may be in the rotation matrix, camera center, or pixel offset.
    static boost::shared_ptr<vw::camera::CameraModel>
    load_adjusted_model(boost::shared_ptr<vw::camera::CameraModel> cam,
                        std::string const& image_file,
                        std::string const& camera_file,
                        vw::Vector2 const& pixel_offset);


    /// Returns the target datum to use for a given camera model.
    virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam,
                                             bool use_sphere_for_datum) const {
      return vw::cartography::Datum(asp::stereo_settings().datum);
    }

    // Peek inside the images and camera models and return the datum and projection,
    // or at least the datum, packaged in a georef.
    virtual vw::cartography::GeoReference get_georef();

    /// Get the crop ROI applied to the two input images.
    void get_input_image_crops(vw::BBox2i &left_image_crop, vw::BBox2i &right_image_crop) const;

    // All Stereo Session children must define the following which are not defined in the the parent:
    //   typedef VWStereoModel stereo_model_type;
    typedef vw::stereo::StereoModel stereo_model_type; // Currently this is the only option!

    /// Transform from image coordinates on disk to original untransformed image pixels.
    typedef boost::shared_ptr<vw::Transform> tx_type;

    virtual tx_type tx_left () const {return tx_left_homography ();} // Default implementation
    virtual tx_type tx_right() const {return tx_right_homography();}

    // All of the "hook" functions below have default implementations that just copy the inputs to the outputs!


    /// Stage 1: Preprocessing
    ///
    /// Pre  file is a pair of images.   ( ImageView<PixelT> )
    /// Post file is a grayscale images. ( ImageView<PixelGray<float> > )
    virtual void pre_preprocessing_hook( bool adjust_left_image_size,
                                         std::string const& input_file1,
                                         std::string const& input_file2,
                                         std::string      & output_file1,
                                         std::string      & output_file2);
    virtual void post_preprocessing_hook(std::string const& input_file1,    // CURRENTLY NEVER USED!
                                         std::string const& input_file2,
                                         std::string      & output_file1,
                                         std::string      & output_file2);

    /// Stage 2: Correlation
    ///
    /// Pre  file is a pair of grayscale images. ( ImageView<PixelGray<float> > )
    /// Post file is a disparity map.            ( ImageView<PixelDisparity> )
    virtual void pre_correlation_hook( std::string const& input_file1,    // CURRENTLY NEVER USED!
                                       std::string const& input_file2,
                                       std::string      & output_file1,
                                       std::string      & output_file2);
    virtual void post_correlation_hook(std::string const& input_file,    // CURRENTLY NEVER USED!
                                       std::string      & output_file);

    /// Stage 3: Filtering
    ///
    /// Pre  file is a disparity map. ( ImageView<PixelDisparity<float> > )
    /// Post file is a disparity map. ( ImageView<PixelDisparity<float> > )
    virtual void pre_filtering_hook( std::string const& input_file,
                                     std::string      & output_file);
    virtual void post_filtering_hook(std::string const& input_file,    // CURRENTLY NEVER USED!
                                     std::string      & output_file);

    /// Stage 4: Point cloud generation
    ///
    /// Pre  file is a disparity map. ( ImageView<PixelDisparity<float> > )
    /// Post file is point image.     ( ImageView<Vector3> )
    virtual vw::ImageViewRef<vw::PixelMask<vw::Vector2f> >
		 pre_pointcloud_hook (std::string const& input_file);
    virtual void post_pointcloud_hook(std::string const& input_file,      // CURRENTLY NEVER USED!
                                      std::string      & output_file);

    /// Returns the correct nodata value from the input images or the input options.
    void get_nodata_values(boost::shared_ptr<vw::DiskImageResource> left_rsrc,
                           boost::shared_ptr<vw::DiskImageResource> right_rsrc,
                           float & left_nodata_value,
                           float & right_nodata_value);

  protected: // Variables

    vw::cartography::GdalWriteOptions m_options;
    std::string m_left_image_file,  m_right_image_file;
    std::string m_left_camera_file, m_right_camera_file;
    std::string m_out_prefix, m_input_dem;

    /// Object to help with camera model loading, mostly used by derived classes.
    CameraModelLoader m_camera_loader;

    /// Storage for the camera models used to map project the input images.
    /// - Not used in non map-projected sessions.
    boost::shared_ptr<vw::camera::CameraModel> m_left_map_proj_model, m_right_map_proj_model;

  private:

    /// Handles init required for map projected session types.
    void init_disk_transform();

  protected:

    // Factor out here all functionality shared among the preprocessing hooks
    // for various sessions. Return 'true' if we encounter cached images
    // and don't need to go through the motions again.
    bool shared_preprocessing_hook(vw::cartography::GdalWriteOptions & options,
                                   std::string const                 & left_input_file,
                                   std::string const                 & right_input_file,
                                   std::string                       & left_output_file,
                                   std::string                       & right_output_file,
                                   std::string                       & left_cropped_file,
                                   std::string                       & right_cropped_file,
                                   float                             & left_nodata_value,
                                   float                             & right_nodata_value,
                                   bool                              & has_left_georef,
                                   bool                              & has_right_georef,
                                   vw::cartography::GeoReference     & left_georef,
                                   vw::cartography::GeoReference     & right_georef);

    // These are all the currently supported transformation types
    tx_type tx_identity        () const; // Not left or right specific
    tx_type tx_left_homography () const;
    tx_type tx_right_homography() const;
    tx_type tx_left_map_trans  () const;
    tx_type tx_right_map_trans () const;

    /// Function to load a specific type of camera model with a pixel offset.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const = 0;

    /// Load an RPC camera model with a pixel offset
    /// - We define it here so it can be used for reading RPC map projection models and also
    ///   so it does not get duplicated in derived RPC sessions.
    boost::shared_ptr<vw::camera::CameraModel> load_rpc_camera_model(std::string const& image_file, 
                                                                     std::string const& camera_file,
                                                                     vw::Vector2 pixel_offset) const;
  };

// ===========================================================================
// --- Template function definitions ---

template <class ViewT>
Vector6f StereoSession::gather_stats( vw::ImageViewBase<ViewT> const& view_base, std::string const& tag,
                                      std::string const& prefix, std::string const& image_path) {
  using namespace vw;
  Vector6f result;

  vw_out(InfoMessage) << "\t--> Computing statistics for " + tag << std::endl;
  ViewT image = view_base.impl();

  const bool use_cache = ((prefix != "") && (image_path != ""));
  std::string cache_path = "";
  if (use_cache)
    cache_path = prefix + '-' + boost::filesystem::path(image_path).stem().string() + "-stats.tif";

  // Check if this stats file was computed after any image modifications.
  if ((use_cache && asp::is_latest_timestamp(cache_path, image_path)) ||
      (stereo_settings().force_reuse_match_files && boost::filesystem::exists(cache_path))) {
    vw_out(InfoMessage) << "\t--> Reading statistics from file " + cache_path << std::endl;
    Vector<float32> stats;
    read_vector(stats, cache_path); // Just fetch the stats from the file on disk.
    result = stats;

  } else { // Compute the results

    // Compute statistics at a reduced resolution
    const float TARGET_NUM_PIXELS = 1000000;
    float num_pixels = float(image.cols())*float(image.rows());
    int   stat_scale = int(ceil(sqrt(num_pixels / TARGET_NUM_PIXELS)));

    vw_out(InfoMessage) << " using downsample scale: " << stat_scale << std::endl;

    ChannelAccumulator<vw::math::CDFAccumulator<float> > accumulator;
    for_each_pixel( subsample( edge_extend(image, ConstantEdgeExtension()),
                              stat_scale ),
                    accumulator );

    // Compute the CDF in parallel.
    // -> Turned off because it makes the unit tests non-deterministic.
    //Vector2i block_size(256,256);
    //int buffer_size = 100000; // TODO: If this gets too large the CDFAccumulator class fails!
    //vw::math::CDFAccumulator<float> accumulator(buffer_size);
    //block_cdf_computation(image, accumulator, stat_scale, block_size);

    result[0] = accumulator.quantile(0); // Min
    result[1] = accumulator.quantile(1); // Max
    result[2] = accumulator.approximate_mean();
    result[3] = accumulator.approximate_stddev();
    result[4] = accumulator.quantile(0.02); // Percentile values
    result[5] = accumulator.quantile(0.98);

    // Cache the results to disk
    if (use_cache) {
      vw_out() << "\t    Writing stats file: " << cache_path << std::endl;
      Vector<float32> stats = result;  // cast
      write_vector(cache_path, stats);
    }

  } // Done computing the results

  vw_out(InfoMessage) << "\t    " << tag << ": [ lo: " << result[0] << " hi: " << result[1]
                      << " mean: " << result[2] << " std_dev: "  << result[3] << " ]\n";

  return result;
}


template<class ImageT>
void StereoSession::normalize_images(bool force_use_entire_range,
                                     bool individually_normalize,
                                     bool use_percentile_stretch,
                                     Vector6f const& left_stats,
                                     Vector6f const& right_stats,
                                     ImageT & Limg, ImageT & Rimg){

  // These arguments must contain: (min, max, mean, std)
  VW_ASSERT(left_stats.size() == 6 && right_stats.size() == 6,
    vw::ArgumentErr() << "Expecting a vector of size 6 in normalize_images()\n");

  // If the input stats don't contain the stddev, must use the entire range version.
  // - This should only happen when normalizing ISIS images for ip_matching purposes.
  if ((left_stats[3] == 0) || (right_stats[3] == 0))
    force_use_entire_range = true;

  if ( force_use_entire_range ) { // Stretch between the min and max values
    if ( individually_normalize ) {
      vw::vw_out() << "\t--> Individually normalize images to their respective min max\n";
      Limg = normalize( Limg, left_stats [0], left_stats [1], 0.0, 1.0 );
      Rimg = normalize( Rimg, right_stats[0], right_stats[1], 0.0, 1.0 );
    } else { // Normalize using the same stats
      float low = std::min(left_stats[0], right_stats[0]);
      float hi  = std::max(left_stats[1], right_stats[1]);
      vw::vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";
      Limg = normalize( Limg, low, hi, 0.0, 1.0 );
      Rimg = normalize( Rimg, low, hi, 0.0, 1.0 );
    }
  } else { // Don't force the entire range
    double left_min, left_max, right_min, right_max;
    if (use_percentile_stretch) {
      // Percentile stretch
      left_min  = left_stats [4];
      left_max  = left_stats [5];
      right_min = right_stats[4];
      right_max = right_stats[5];
    } else {
      // Two standard deviation stretch
      left_min  = left_stats [2] - 2*left_stats [3];
      left_max  = left_stats [2] + 2*left_stats [3];
      right_min = right_stats[2] - 2*right_stats[3];
      right_max = right_stats[2] + 2*right_stats[3];
    }

    // The images are normalized so most pixels fall into this range,
    // but the data is not clamped so some pixels can fall outside this range.
    if ( individually_normalize > 0 ) {
      vw::vw_out() << "\t--> Individually normalize images\n";
      Limg = normalize( Limg, left_min,  left_max,  0.0, 1.0 );
      Rimg = normalize( Rimg, right_min, right_max, 0.0, 1.0 );
    } else { // Normalize using the same stats
      float low = std::min(left_min, right_min);
      float hi  = std::max(left_max, right_max);
      vw::vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";
      Limg = normalize( Limg, low, hi, 0.0, 1.0 );
      Rimg = normalize( Rimg, low, hi, 0.0, 1.0 );
    }
  }
  return;
}

} // end namespace asp

#endif // __STEREO_SESSION_H__
