// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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
#include <vw/Math/RANSAC.h>
#include <vw/InterestPoint.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>

#include <asp/Core/Common.h>

namespace asp {

  class StereoSession {
  protected:
    asp::BaseOptions m_options;
    std::string m_left_image_file, m_right_image_file,
      m_left_camera_file, m_right_camera_file, m_out_prefix;
    std::string m_extra_argument1, m_extra_argument2,
      m_extra_argument3, m_extra_argument4;

    // Helper function for determing image alignment.
    //
    // This expect image intensities to be between 0 and 1.0. If you
    // know the range before hand will be 0.25 to 0.75, you can give a
    // helping hand by setting the gain_guess parameter to 2.0 in that
    // case.
    template <class ImageT1, class ImageT2>
    vw::Matrix3x3
    determine_image_align( std::string const& input_file1,
                           std::string const& input_file2,
                           vw::ImageViewBase<ImageT1> const& input1,
                           vw::ImageViewBase<ImageT2> const& input2,
                           float gain_guess = 1.0 ) {
      namespace fs = boost::filesystem;
      using namespace vw;

      ImageT1 const& image1 = input1.impl();
      ImageT2 const& image2 = input2.impl();
      std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
      std::string match_filename =
        fs::path( input_file1 ).replace_extension("").string() + "__" +
        fs::path( input_file2 ).stem().string() + ".match";

      if ( fs::exists( match_filename ) ) {
        // Is there a match file linking these 2 image?

        vw_out() << "\t--> Found cached interest point match file: "
                 << match_filename << "\n";
        read_binary_match_file( match_filename,
                                matched_ip1, matched_ip2 );

        // Fitting a matrix immediately (no RANSAC)
        vw::math::HomographyFittingFunctor fitting;
        remove_duplicates( matched_ip1, matched_ip2 );
        std::vector<Vector3> list1 = iplist_to_vectorlist(matched_ip1);
        std::vector<Vector3> list2 = iplist_to_vectorlist(matched_ip2);
        vw::math::AffineFittingFunctor aff_fit;
        Matrix<double> seed = aff_fit( list2, list1 );
        Matrix<double> align = fitting( list2, list1, seed );  // LMA optimization second
        vw_out() << "\tFit = " << align << std::endl;
        return align;

      } else {

        // Next best thing.. VWIPs?
        std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
        std::string ip1_filename =
          fs::path( input_file1 ).replace_extension("vwip").string();
        std::string ip2_filename =
          fs::path( input_file2 ).replace_extension("vwip").string();
        if ( fs::exists( ip1_filename ) &&
             fs::exists( ip2_filename ) ) {
          // Found VWIPs already done before
          vw_out() << "\t--> Found cached interest point files: "
                   << ip1_filename << "\n"
                   << "\t                                       "
                   << ip2_filename << "\n";
          ip1_copy = ip::read_binary_ip_file( ip1_filename );
          ip2_copy = ip::read_binary_ip_file( ip2_filename );

        } else {
          // Worst case, no interest point operations have been performed before
          vw_out() << "\t--> Locating Interest Points\n";
          ip::InterestPointList ip1, ip2;

          // Interest Point module detector code.
          float ipgain = 0.08 / gain_guess;
          while ( ip1.size() < 1500 || ip2.size() < 1500 ) {
            ip1.clear(); ip2.clear();
            ip::OBALoGInterestOperator interest_operator( ipgain );
            ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator, 0 );
            vw_out() << "\t    Processing " << input_file1 << "\n";
            ip1 = detect_interest_points( image1, detector );
            vw_out() << "\t    Processing " << input_file2 << "\n";
            ip2 = detect_interest_points( image2, detector );

            ipgain *= 0.75;
          }
          if ( ip1.size() > 10000 ) {
            ip1.sort(); ip1.resize(10000);
          }
          if ( ip2.size() > 10000 ) {
            ip2.sort(); ip2.resize(10000);
          }
          vw_out() << "\t    Located " << ip1.size() << " points.\n";
          vw_out() << "\t    Located " << ip2.size() << " points.\n";

          vw_out() << "\t    Generating descriptors...\n";
          ip::SGradDescriptorGenerator descriptor;
          descriptor( image1, ip1 );
          descriptor( image2, ip2 );
          vw_out() << "\t    done.\n";

          // Writing out the results
          vw_out() << "\t    Caching interest points: "
                   << ip1_filename << ", "
                   << ip2_filename << "\n";
          ip::write_binary_ip_file(ip1_filename, ip1);
          ip::write_binary_ip_file(ip2_filename, ip2);

          // Reading back into the vector interestpoint format
          ip1_copy = ip::read_binary_ip_file( ip1_filename );
          ip2_copy = ip::read_binary_ip_file( ip2_filename );
        }

        vw_out() << "\t--> Matching interest points\n";
        ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.5);

        matcher(ip1_copy, ip2_copy,
                matched_ip1, matched_ip2,
                false,
                TerminalProgressCallback( "asp", "\t    Matching: "));

      } // End matching

      vw_out(InfoMessage) << "\t--> " << matched_ip1.size()
                          << " putative matches.\n";

      vw_out() << "\t--> Rejecting outliers using RANSAC.\n";
      remove_duplicates(matched_ip1, matched_ip2);
      std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
      std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
      vw_out(DebugMessage) << "\t--> Removed "
                           << matched_ip1.size() - ransac_ip1.size()
                           << " duplicate matches.\n";

      Matrix<double> T;
      std::vector<size_t> indices;
      try {

        vw::math::RandomSampleConsensus<vw::math::HomographyFittingFunctor, vw::math::InterestPointErrorMetric> ransac( vw::math::HomographyFittingFunctor(), vw::math::InterestPointErrorMetric(), 10 );
        T = ransac( ransac_ip2, ransac_ip1 );
        indices = ransac.inlier_indices(T, ransac_ip2, ransac_ip1 );
        vw_out(DebugMessage) << "\t--> AlignMatrix: " << T << std::endl;

      } catch (...) {
        vw_out(WarningMessage,"console") << "Automatic Alignment Failed! Proceed with caution...\n";
        T = vw::math::identity_matrix<3>();
      }

      vw_out() << "\t    Caching matches: "
               << match_filename << "\n";

      { // Keeping only inliers
        std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
        for ( size_t i = 0; i < indices.size(); i++ ) {
          inlier_ip1.push_back( matched_ip1[indices[i]] );
          inlier_ip2.push_back( matched_ip2[indices[i]] );
        }
        matched_ip1 = inlier_ip1;
        matched_ip2 = inlier_ip2;
      }

      write_binary_match_file( match_filename,
                               matched_ip1, matched_ip2);
      return T;
    }

  public:
    virtual void initialize (BaseOptions const& options,
                             std::string const& left_image_file,
                             std::string const& right_image_file,
                             std::string const& left_camera_file,
                             std::string const& right_camera_file,
                             std::string const& out_prefix,
                             std::string const& extra_argument1,
                             std::string const& extra_argument2,
                             std::string const& extra_argument3,
                             std::string const& extra_argument4);

    virtual ~StereoSession() {}

    // Methods (mostly static) for registering and creating stereo sessions.
    static StereoSession* create( std::string const& session_type );
    typedef StereoSession* (*construct_func)();
    static void register_session_type( std::string const& id, construct_func func);

    // Helper function that retrieves both cameras.
    virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                               boost::shared_ptr<vw::camera::CameraModel> &cam2);

    // Method that actual produces a Camera Model.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "") = 0;

    // Does this project utilize look up tables to determine pixel
    // location in camera model? (A map projected image or
    // epipolar resampled image would use this function.)
    virtual bool has_lut_images() const;

    // Provide the LUT images for the session
    virtual vw::ImageViewRef<vw::Vector2f> lut_image_left() const;
    virtual vw::ImageViewRef<vw::Vector2f> lut_image_right() const;

    // Stage 1: Preprocessing
    //
    // Pre file is a pair of images.            ( ImageView<PixelT> )
    // Post file is a grayscale images.         ( ImageView<PixelGray<flaot> > )
    virtual void pre_preprocessing_hook(std::string const& input_file1,
                                        std::string const& input_file2,
                                        std::string &output_file1,
                                        std::string &output_file2);
    virtual void post_preprocessing_hook(std::string const& input_file1,
                                         std::string const& input_file2,
                                         std::string &output_file1,
                                         std::string &output_file2);

    // Stage 2: Correlation
    //
    // Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
    // Post file is a disparity map.            ( ImageView<PixelDisparity> )
    virtual void pre_correlation_hook(std::string const& input_file1,
                                      std::string const& input_file2,
                                      std::string &output_file1,
                                      std::string &output_file2);
    virtual void post_correlation_hook(std::string const& input_file,
                                       std::string & output_file);

    // Stage 3: Filtering
    //
    // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
    // Post file is a disparity map. ( ImageView<PixelDisparity<float> > )
    virtual void pre_filtering_hook(std::string const& input_file,
                                    std::string & output_file);
    virtual void post_filtering_hook(std::string const& input_file,
                                     std::string & output_file);

    // Stage 4: Point cloud generation
    //
    // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
    // Post file is point image.     ( ImageView<Vector3> )
    virtual vw::ImageViewRef<vw::PixelMask<vw::Vector2f> >
    pre_pointcloud_hook(std::string const& input_file);
    virtual void post_pointcloud_hook(std::string const& input_file,
                                      std::string & output_file);
  };

} // end namespace asp

#endif // __STEREO_SESSION_H__
