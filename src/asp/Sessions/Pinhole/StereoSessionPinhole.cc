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


/// \file StereoSessionPinhole.cc
///

#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

#include <vw/Math/BBox.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/Vector.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/MaskViews.h>
#include <vw/Camera.h>
#include <vw/Stereo/DisparityMap.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::ip;
using namespace vw::camera;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

// Helper function for determing image alignment.
//
// This expects image intensities to be between 0 and 1.0. If you
// know the range before hand will be 0.25 to 0.75, you can give a
// helping hand by setting the gain_guess parameter to 2.0 in that
// case.
template <class ImageT1, class ImageT2>
vw::Matrix3x3
determine_image_align( std::string const& out_prefix,
                       std::string const& input_file1,
                       std::string const& input_file2,
                       vw::ImageViewBase<ImageT1> const& input1,
                       vw::ImageViewBase<ImageT2> const& input2,
                       double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                       double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                       float gain_guess = 1.0 ) {
  namespace fs = boost::filesystem;
  using namespace vw;

  ImageT1 const& image1 = input1.impl();
  ImageT2 const& image2 = input2.impl();
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  std::string match_filename
    = ip::match_filename(out_prefix, input_file1, input_file2);

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
    std::string ip1_filename, ip2_filename;
    ip::ip_filenames(out_prefix, input_file1, input_file2,
                     ip1_filename, ip2_filename);
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
      asp::detect_ip( ip1, ip2, image1, image2,
                      nodata1, nodata2 );

      if ( ip1.size() > 10000 ) {
        ip1.sort(); ip1.resize(10000);
      }
      if ( ip2.size() > 10000 ) {
        ip2.sort(); ip2.resize(10000);
      }

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
            TerminalProgressCallback( "asp", "\t    Matching: "));

  } // End matching

  vw_out(InfoMessage) << "\t--> " << matched_ip1.size()
                      << " putative matches.\n";

  vw_out() << "\t--> Rejecting outliers using RANSAC.\n";
  remove_duplicates(matched_ip1, matched_ip2);
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
  vw_out(DebugMessage,"asp") << "\t--> Removed "
                             << matched_ip1.size() - ransac_ip1.size()
                             << " duplicate matches.\n";

  Matrix<double> T;
  std::vector<size_t> indices;
  try {

    vw::math::RandomSampleConsensus<vw::math::HomographyFittingFunctor, vw::math::InterestPointErrorMetric> ransac( vw::math::HomographyFittingFunctor(), vw::math::InterestPointErrorMetric(), 100, 10, ransac_ip1.size()/2, true);
    T = ransac( ransac_ip2, ransac_ip1 );
    indices = ransac.inlier_indices(T, ransac_ip2, ransac_ip1 );
    vw_out(DebugMessage,"asp") << "\t--> AlignMatrix: " << T << std::endl;

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

boost::shared_ptr<vw::camera::CameraModel>
asp::StereoSessionPinhole::camera_model(std::string const& /*image_file*/,
                                        std::string const& camera_file) {
  if ( stereo_settings().alignment_method == "epipolar" ) {
    // Load the image
    DiskImageView<float> left_image(m_left_image_file);
    DiskImageView<float> right_image(m_right_image_file);

    Vector2i left_image_size( left_image.cols(), left_image.rows() ),
      right_image_size( right_image.cols(), right_image.rows() );

    bool is_left_camera = true;
    if (camera_file == m_left_camera_file)
      is_left_camera = true;
    else if (camera_file == m_right_camera_file)
      is_left_camera = false;
    else
      (ArgumentErr() << "StereoSessionPinhole: supplied camera model filename does not match the name supplied in the constructor.");

    // Return the appropriate camera model object
    std::string lcase_file = boost::to_lower_copy(m_left_camera_file);
    CAHVModel left_cahv, right_cahv;
    if (boost::ends_with(lcase_file, ".cahvore") ) {
      CAHVOREModel left_cahvore(m_left_camera_file);
      CAHVOREModel right_cahvore(m_right_camera_file);
      left_cahv =
        linearize_camera(left_cahvore, left_image_size, left_image_size);
      right_cahv =
        linearize_camera(right_cahvore, right_image_size, right_image_size);
    } else if (boost::ends_with(lcase_file, ".cahvor")  ||
               boost::ends_with(lcase_file, ".cmod") ) {
      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      left_cahv =
        linearize_camera(left_cahvor, left_image_size, left_image_size);
      right_cahv =
        linearize_camera(right_cahvor, right_image_size, right_image_size);
    } else if ( boost::ends_with(lcase_file, ".cahv") ||
                boost::ends_with(lcase_file, ".pin" )) {
      left_cahv = CAHVModel(m_left_camera_file);
      right_cahv = CAHVModel(m_right_camera_file);

    } else if ( boost::ends_with(lcase_file, ".pinhole") ||
                boost::ends_with(lcase_file, ".tsai") ) {
      PinholeModel left_pin(m_left_camera_file);
      PinholeModel right_pin(m_right_camera_file);
      left_cahv = linearize_camera(left_pin);
      right_cahv = linearize_camera(right_pin);
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }

    // Create epipolar recitified camera views
    boost::shared_ptr<CAHVModel> epipolar_left_cahv(new CAHVModel);
    boost::shared_ptr<CAHVModel> epipolar_right_cahv(new CAHVModel);
    epipolar(left_cahv, right_cahv, *epipolar_left_cahv, *epipolar_right_cahv);

    if (is_left_camera)
      return epipolar_left_cahv;
    else
      return epipolar_right_cahv;
  } else {
    // Keypoint alignment and everything else just gets camera models
    std::string lcase_file = boost::to_lower_copy(camera_file);
    if (boost::ends_with(lcase_file,".cahvore") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVOREModel(camera_file) );
    } else if (boost::ends_with(lcase_file,".cahvor") ||
               boost::ends_with(lcase_file,".cmod") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVORModel(camera_file) );
    } else if ( boost::ends_with(lcase_file,".cahv") ||
                boost::ends_with(lcase_file,".pin") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVModel(camera_file) );
    } else if ( boost::ends_with(lcase_file,".pinhole") ||
                boost::ends_with(lcase_file,".tsai") ) {
      return boost::shared_ptr<vw::camera::CameraModel> ( new PinholeModel(camera_file) );
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }


  }
  return boost::shared_ptr<vw::camera::CameraModel>(); // Never reached
}

asp::StereoSessionPinhole::left_tx_type
asp::StereoSessionPinhole::tx_left() const {
  return left_tx_type( math::identity_matrix<3>() );
}

asp::StereoSessionPinhole::right_tx_type
asp::StereoSessionPinhole::tx_right() const {
  if ( stereo_settings().alignment_method == "homography" ) {
    Matrix<double> align_matrix;
    read_matrix( align_matrix, m_out_prefix + "-align-R.exr" );
    return right_tx_type( align_matrix );
  }
  return right_tx_type( math::identity_matrix<3>() );
}

void asp::StereoSessionPinhole::pre_preprocessing_hook(std::string const& left_input_file,
                                                       std::string const& right_input_file,
                                                       std::string &left_output_file,
                                                       std::string &right_output_file) {


  boost::shared_ptr<DiskImageResource>
    left_rsrc( DiskImageResource::open(m_left_image_file) ),
    right_rsrc( DiskImageResource::open(m_right_image_file) );

  float left_nodata_value, right_nodata_value;
  get_nodata_values(left_rsrc, right_rsrc, left_nodata_value, right_nodata_value);

  // Load the unmodified images
  DiskImageView<float> left_disk_image( left_rsrc ), right_disk_image( right_rsrc );

  ImageViewRef< PixelMask<float> > left_masked_image
    = create_mask_less_or_equal(left_disk_image, left_nodata_value);
  ImageViewRef< PixelMask<float> > right_masked_image
    = create_mask_less_or_equal(right_disk_image, right_nodata_value);

  Vector4f left_stats  = gather_stats( left_masked_image,  "left" );
  Vector4f right_stats = gather_stats( right_masked_image, "right" );

  ImageViewRef< PixelMask<float> > Limg, Rimg;
  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);

  if ( stereo_settings().alignment_method == "epipolar" ) {

    vw_out() << "\t--> Performing epipolar alignment\n";

    // Load the two images and fetch the two camera models
    boost::shared_ptr<camera::CameraModel> left_camera =
      this->camera_model(left_input_file, m_left_camera_file);
    boost::shared_ptr<camera::CameraModel> right_camera =
      this->camera_model(right_input_file, m_right_camera_file);
    CAHVModel* left_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*left_camera));
    CAHVModel* right_epipolar_cahv = dynamic_cast<CAHVModel*>(&(*right_camera));

    // Remove lens distortion and create epipolar rectified images.
    if (boost::ends_with(lcase_file, ".cahvore")) {
      CAHVOREModel left_cahvore(m_left_camera_file);
      CAHVOREModel right_cahvore(m_right_camera_file);
      Limg = transform(left_masked_image, CameraTransform<CAHVOREModel, CAHVModel>(left_cahvore, *left_epipolar_cahv));

      Rimg = transform(right_masked_image, CameraTransform<CAHVOREModel, CAHVModel>(right_cahvore, *right_epipolar_cahv));
    } else if (boost::ends_with(lcase_file, ".cahvor") ||
               boost::ends_with(lcase_file, ".cmod") ) {
      CAHVORModel left_cahvor(m_left_camera_file);
      CAHVORModel right_cahvor(m_right_camera_file);
      Limg = transform(left_masked_image, CameraTransform<CAHVORModel, CAHVModel>(left_cahvor, *left_epipolar_cahv));
      Rimg = transform(right_masked_image, CameraTransform<CAHVORModel, CAHVModel>(right_cahvor, *right_epipolar_cahv));

    } else if ( boost::ends_with(lcase_file, ".cahv") ||
                boost::ends_with(lcase_file, ".pin" )) {
      CAHVModel left_cahv(m_left_camera_file);
      CAHVModel right_cahv(m_right_camera_file);
      Limg = transform(left_masked_image, CameraTransform<CAHVModel, CAHVModel>(left_cahv, *left_epipolar_cahv));
      Rimg = transform(right_masked_image, CameraTransform<CAHVModel, CAHVModel>(right_cahv, *right_epipolar_cahv));

    } else if ( boost::ends_with(lcase_file, ".pinhole") ||
                boost::ends_with(lcase_file, ".tsai") ) {
      PinholeModel left_pin(m_left_camera_file);
      PinholeModel right_pin(m_right_camera_file);
      Limg = transform(left_masked_image, CameraTransform<PinholeModel, CAHVModel>(left_pin, *left_epipolar_cahv));
      Rimg = transform(right_masked_image, CameraTransform<PinholeModel, CAHVModel>(right_pin, *right_epipolar_cahv));

    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }

  } else if ( stereo_settings().alignment_method == "homography" ) {

    float low = std::min(left_stats[0], right_stats[0]);
    float hi  = std::max(left_stats[1], right_stats[1]);
    float gain_guess = 1.0f / (hi - low);
    if ( gain_guess < 1.0f )
      gain_guess = 1.0f;

    // Note: Here we use the original images, without mask
    Matrix<double> align_matrix =
      determine_image_align( m_out_prefix,
                             left_input_file, right_input_file,
                             left_disk_image, right_disk_image,
                             left_nodata_value, right_nodata_value,
                             gain_guess );
    write_matrix( m_out_prefix + "-align-R.exr", align_matrix );

    // Applying alignment transform
    Limg = left_masked_image;
    Rimg = transform(right_masked_image,
                     HomographyTransform(align_matrix),
                     left_masked_image.cols(), left_masked_image.rows());

  } else {
    // Do nothing just provide the original files.
    Limg = left_masked_image;
    Rimg = right_masked_image;
  }

  // Apply our normalization options.
  normalize_images(stereo_settings().force_use_entire_range,
                   stereo_settings().individually_normalize,
                   left_stats, right_stats, Limg, Rimg);

  // The output no-data value must be < 0 as we scale the images to [0, 1].
  float output_nodata = -32768.0;

  // Enforce no predictor in compression, it works badly with L.tif and R.tif.
  asp::BaseOptions options = m_options;
  options.gdal_options["PREDICTOR"] = "1";

  left_output_file = m_out_prefix + "-L.tif";
  right_output_file = m_out_prefix + "-R.tif";
  vw_out() << "\t--> Writing pre-aligned images.\n";
  block_write_gdal_image( left_output_file, apply_mask(Limg, output_nodata),
                          output_nodata, options,
                          TerminalProgressCallback("asp","\t  L:  ") );
  block_write_gdal_image( right_output_file, apply_mask(crop(edge_extend(Rimg,ConstantEdgeExtension()),bounding_box(Limg)), output_nodata),
                          output_nodata, options,
                          TerminalProgressCallback("asp","\t  R:  ") );
}
