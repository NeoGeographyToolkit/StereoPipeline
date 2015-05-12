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
#include <vw/Camera/CameraModel.h>

#include <vw/Math/Functors.h>
#include <vw/Math/Geometry.h>
#include <vw/InterestPoint.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>

#include <asp/Core/Common.h>
#include <asp/Core/InterestPointMatching.h>

namespace asp {

  //TODO: Move this function!
  /// Compute the min, max, mean, and standard deviation of an image object and write them to a log.
  /// - "tag" is only used to make the log messages more descriptive.
  template <class ViewT>
  vw::Vector4f gather_stats( vw::ImageViewBase<ViewT> const& view_base, std::string const& tag) {
    using namespace vw;
    vw_out(InfoMessage) << "\t--> Computing statistics for the "+tag+" image\n";
    ViewT image = view_base.impl();
    int stat_scale = int(ceil(sqrt(float(image.cols())*float(image.rows()) / 1000000)));
    
    ChannelAccumulator<vw::math::CDFAccumulator<float> > accumulator;
    for_each_pixel( subsample( edge_extend(image, ConstantEdgeExtension()),
                               stat_scale ), accumulator );
    Vector4f result( accumulator.quantile(0),
                     accumulator.quantile(1),
                     accumulator.approximate_mean(),
                     accumulator.approximate_stddev() );
    vw_out(InfoMessage) << "\t  " << tag << ": [ lo: " << result[0] << " hi: " << result[1]
                                             << " m: " << result[2] << " s: "  << result[3] << " ]\n";
    return result;
  }

  //TODO: Move this function!
  /// Normalize the intensity of two grayscale images based on input statistics
  template<class ImageT, class VectorT>
  void normalize_images(bool force_use_entire_range,
                        bool individually_normalize,
                        VectorT const& left_stats, 
                        VectorT const& right_stats,
                        ImageT & Limg, ImageT & Rimg){

    // These arguments must contain: (min, max, mean, std)
    VW_ASSERT(left_stats.size() == 4 && right_stats.size() == 4,
                  vw::ArgumentErr() << "Expecting a vector of size 4 in normalize_images()\n");
    
    if ( force_use_entire_range > 0 ) {
      if ( individually_normalize > 0 ) {
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
      double left_min  = left_stats [2] - 2*left_stats [3];
      double left_max  = left_stats [2] + 2*left_stats [3];
      double right_min = right_stats[2] - 2*right_stats[3];
      double right_max = right_stats[2] + 2*right_stats[3];      
      if ( individually_normalize > 0 ) {
        vw::vw_out() << "\t--> Individually normalize images to their respective 4 std dev window\n";
        Limg = normalize( Limg, left_min,  left_max, 0.0, 1.0 );
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
  
  
   
  // Forward declare this class for constructing StereoSession objects
  class StereoSessionFactory;
  
  /// Stereo Sessions define for different missions or satellites how to:
  ///   * Initialize, normalize, and align the input imagery
  ///   * Extract the camera model
  ///   * Custom code needed for correlation, filtering, and triangulation.
  class StereoSession {
    friend class StereoSessionFactory; // Needed so the factory can call initialize()
  protected:
    asp::BaseOptions m_options;
    std::string      m_left_image_file, 
                     m_right_image_file,
                     m_left_camera_file, 
                     m_right_camera_file, 
                     m_out_prefix;
    std::string      m_input_dem;

    virtual void initialize (BaseOptions const& options,
                             std::string const& left_image_file,
                             std::string const& right_image_file,
                             std::string const& left_camera_file,
                             std::string const& right_camera_file,
                             std::string const& out_prefix,
                             std::string const& input_dem);

  public:
    virtual ~StereoSession() {}

    /// Simple typedef of a factory function that creates a StereoSession instance
    typedef StereoSession* (*construct_func)();

    /// Helper function that retrieves both cameras.
    virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                               boost::shared_ptr<vw::camera::CameraModel> &cam2);

    /// Method that produces a Camera Model from input files.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "") = 0;

    /// Method to help determine what session we actually have
    virtual std::string name() const = 0;

    /// Specialization for how interest points are found
    virtual bool ip_matching(std::string const& input_file1,
                             std::string const& input_file2,
                             float nodata1, float nodata2,
                             std::string const& match_filename,
                             vw::camera::CameraModel* cam1,
                             vw::camera::CameraModel* cam2);

    /// Returns the target datum to use for a given camera model
    virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam) const { return vw::cartography::Datum("WGS84"); }

    // All Stereo Session children must define the following which are not defined in the the parent:
    //   typedef VWStereoModel stereo_model_type;
    //   typedef VWTransform   tx_type; ///< Transform from image coordinates on disk to original untransformed image pixels.
    //   tx_type tx_left (void) const;  // For the left and right cameras
    //   tx_type tx_right(void) const;

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
  };


} // end namespace asp

#endif // __STEREO_SESSION_H__
