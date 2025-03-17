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

#include <boost/shared_ptr.hpp>
#include <asp/Core/ImageNormalization.h>
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
    void initialize(vw::GdalWriteOptions const& options,
                    std::string const& left_image_file,
                    std::string const& right_image_file,
                    std::string const& left_camera_file,
                    std::string const& right_camera_file,
                    std::string const& out_prefix,
                    std::string const& input_dem);

    // The next set of functions describe characteristics of the derived session class.
    // - These could be made in to some sort of static constant if needed.
    virtual bool isMapProjected() const { return false; }
    virtual bool have_datum() const;
    virtual bool supports_multi_threading () const {
      return true;
    }

    /// Helper function that retrieves both cameras.
    virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                               boost::shared_ptr<vw::camera::CameraModel> &cam2);

    /// Method that produces a Camera Model from input files.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "",
                 bool quiet = false);

    /// Method to help determine what session we actually have
    virtual std::string name() const = 0;

    /// Specialization for how interest points are found
    bool ip_matching(std::string  const& input_file1,
                     std::string  const& input_file2,
                     vw::Vector2  const& uncropped_image_size,
                     Vector6f     const& stats1,
                     Vector6f     const& stats2,
                     float nodata1, float nodata2,
                     vw::camera::CameraModel* cam1,
                     vw::camera::CameraModel* cam2,
                     std::string const& match_filename,
                     std::string const left_ip_file = "",
                     std::string const right_ip_file = "",
                     vw::BBox2i const& bbox1 = vw::BBox2i(),
                     vw::BBox2i const& bbox2 = vw::BBox2i());

    // Returns the target datum to use for a given camera model.
    // Can be overridden by derived classes.
    virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam,
                                             bool use_sphere_for_non_earth) const;

    // Peek inside the images and camera models and return the datum and projection,
    // or at least the datum, packaged in a georef.
    virtual vw::cartography::GeoReference get_georef();

    /// Get the crop ROI applied to the two input images.
    void get_input_image_crops(vw::BBox2i &left_image_crop, vw::BBox2i &right_image_crop) const;

    virtual vw::TransformPtr tx_left () const {return tx_left_homography ();} // Default implementation
    virtual vw::TransformPtr tx_right() const {return tx_right_homography();}

    // All of the "hook" functions below have default implementations
    // that just copy the inputs to the outputs!

    /// Stage 1: Preprocessing
    ///
    /// Pre  file is a pair of images.   ( ImageView<PixelT> )
    /// Post file is a grayscale images. ( ImageView<PixelGray<float> > )
    virtual void preprocessing_hook( bool adjust_left_image_size,
                                         std::string const& input_file1,
                                         std::string const& input_file2,
                                         std::string      & output_file1,
                                         std::string      & output_file2);

    /// Stage 3: Filtering
    ///
    /// Pre  file is a disparity map. ( ImageView<PixelDisparity<float> > )
    /// Post file is a disparity map. ( ImageView<PixelDisparity<float> > )
    virtual void pre_filtering_hook( std::string const& input_file,
                                     std::string      & output_file);

    /// Stage 4: Point cloud generation
    ///
    /// Pre  file is a disparity map. ( ImageView<PixelDisparity<float> > )
    /// Post file is point image.     ( ImageView<Vector3> )
    virtual vw::ImageViewRef<vw::PixelMask<vw::Vector2f>>
    pre_pointcloud_hook(std::string const& input_file);

    /// Returns the correct nodata value from the input images or the input options.
    void get_nodata_values(boost::shared_ptr<vw::DiskImageResource> left_rsrc,
                           boost::shared_ptr<vw::DiskImageResource> right_rsrc,
                           float & left_nodata_value,
                           float & right_nodata_value);

    bool do_bathymetry() const;

    // Return the left and right cropped images. These are the same as
    // the input images unless the cropping is on.
    std::string left_cropped_image() const;
    std::string right_cropped_image() const;

    std::string left_aligned_bathy_mask() const;
    std::string right_aligned_bathy_mask() const;

    void read_aligned_bathy_masks
    (vw::ImageViewRef< vw::PixelMask<float> > & left_aligned_bathy_mask_image,
     vw::ImageViewRef< vw::PixelMask<float> > & right_aligned_bathy_mask_image);

    // Align the bathy masks. This will be called in stereo_pprc and, if needed,
    // in stereo_tri
    void align_bathy_masks(vw::GdalWriteOptions const& options);
    
     // Read a camera used in mapprojection
     void read_mapproj_cam(std::string const& image_file, std::string const& cam_file,
                           std::string const& adj_prefix, std::string const& cam_type,
                           vw::CamPtr & map_proj_cam);

     // Read cameras used in mapprojection
     void read_mapproj_cams(std::string const& left_image_file,
                            std::string const& right_image_file,
                            std::string const& left_camera_file,
                            std::string const& right_camera_file, 
                            std::string const& input_dem,
                            std::string const& session_name, 
                            vw::CamPtr & left_map_proj_cam, 
                            vw::CamPtr & right_map_proj_cam);

    /// Function to load a specific type of camera model with a pixel offset.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix, 
                      vw::Vector2 pixel_offset) const = 0;

  protected: // Variables

    vw::GdalWriteOptions m_options;
    std::string m_left_image_file,  m_right_image_file;
    std::string m_left_camera_file, m_right_camera_file;
    std::string m_out_prefix, m_input_dem;

    /// Object to help with camera model loading, mostly used by derived classes.
    CameraModelLoader m_camera_loader;

    /// Storage for the camera models used to map project the input images.
    /// - Not used in non map-projected sessions.
    boost::shared_ptr<vw::camera::CameraModel> m_left_map_proj_model, m_right_map_proj_model;

  protected:

    // Factor out here all functionality shared among the preprocessing hooks
    // for various sessions. Return 'true' if we encounter cached images
    // and don't need to go through the motions again.
    bool shared_preprocessing_hook(vw::GdalWriteOptions & options,
                                   std::string const                 & left_input_file,
                                   std::string const                 & right_input_file,
                                   std::string                       & left_output_file,
                                   std::string                       & right_output_file,
                                   std::string                       & left_cropped_file,
                                   std::string                       & right_cropped_file,
                                   vw::ImageViewRef<float>           & left_cropped_image, 
                                   vw::ImageViewRef<float>           & right_cripped_image, 
                                   float                             & left_nodata_value,
                                   float                             & right_nodata_value,
                                   bool                              & has_left_georef,
                                   bool                              & has_right_georef,
                                   vw::cartography::GeoReference     & left_georef,
                                   vw::cartography::GeoReference     & right_georef);

    // These are all the currently supported transformation types
    vw::TransformPtr tx_identity        () const; // Not left or right specific
    vw::TransformPtr tx_left_homography () const;
    vw::TransformPtr tx_right_homography() const;
    vw::TransformPtr tx_left_map_trans  () const;
    vw::TransformPtr tx_right_map_trans () const;

    /// Load an RPC camera model with a pixel offset
    /// - We define it here so it can be used for reading RPC map projection models and also
    ///   so it does not get duplicated in derived RPC sessions.
    boost::shared_ptr<vw::camera::CameraModel>
    load_rpc_camera_model(std::string const& image_file, 
                          std::string const& camera_file,
                          std::string const& ba_prefix,
                          vw::Vector2 pixel_offset) const;
    
    void read_bathy_masks(float & left_bathy_nodata, 
                          float & right_bathy_nodata, 
                          vw::ImageViewRef< vw::PixelMask<float> > & left_bathy_mask,
                          vw::ImageViewRef< vw::PixelMask<float> > & right_bathy_mask);

    std::string left_cropped_bathy_mask() const;
    std::string right_cropped_bathy_mask() const;

    // Apply epipolar alignment to images, if the camera models are pinhole. This will
    // be reimplemented in StereoSessionPinhole.
    virtual void epipolar_alignment(vw::ImageViewRef<vw::PixelMask<float>> left_masked_image,
                                    vw::ImageViewRef<vw::PixelMask<float>> right_masked_image,
                                    vw::ValueEdgeExtension<vw::PixelMask<float>> ext_nodata,
                                    // Outputs
                                    vw::ImageViewRef<vw::PixelMask<float>> & Limg, 
                                    vw::ImageViewRef<vw::PixelMask<float>> & Rimg);
    
    // Find ip matches and determine the alignment matrices
    void determine_image_alignment(// Inputs
                                   std::string  const& out_prefix,
                                   std::string  const& left_cropped_file,
                                   std::string  const& right_cropped_file,
                                   std::string  const& left_uncropped_file,
                                   vw::Vector6f const& left_stats,
                                   vw::Vector6f const& right_stats,
                                   float left_nodata_value,
                                   float right_nodata_value,
                                   boost::shared_ptr<vw::camera::CameraModel> left_cam, 
                                   boost::shared_ptr<vw::camera::CameraModel> right_cam,
                                   bool adjust_left_image_size,
                                   // In-out
                                   vw::Matrix<double> & align_left_matrix,
                                   vw::Matrix<double> & align_right_matrix,
                                   vw::Vector2i & left_size,
                                   vw::Vector2i & right_size);
    
    // Cache here the camera when loaded. Use a mutex to protect the cache.
    vw::Mutex m_camera_mutex;
    std::map<std::pair<std::string, std::string>, vw::CamPtr> m_camera_model;
    
  };

// Compute the min, max, mean, and standard deviation of an image object and
// write them to a file. This is not a member function.
// - "tag" is only used to make the log messages more descriptive.
// - If prefix and image_path is set, will cache the results to a file.
vw::Vector6f gather_stats(vw::ImageViewRef<vw::PixelMask<float>> image, 
                          std::string const& tag,
                          std::string const& prefix, 
                          std::string const& image_path);

  typedef boost::shared_ptr<StereoSession> SessionPtr;

// Find the median angle in degrees at which rays emanating from
// matching points meet
void estimate_convergence_angle(ASPGlobalOptions const& opt);

} // end namespace asp

#endif // __STEREO_SESSION_H__
