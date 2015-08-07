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

#include <asp/Sessions/StereoSessionPinhole.h>

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

//TODO: There is a lot of duplicate code here with the Pinhole
//class. Common functionality must be factored out.

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

// TODO: Need to make this session use the same IP matching code as
// the other sessions.
bool asp::StereoSessionPinhole::ip_matching(std::string const& input_file1,
                                            std::string const& input_file2,
                                            int ip_per_tile,
                                            float nodata1, float nodata2,
                                            std::string const& match_filename,
                                            vw::camera::CameraModel* cam1,
                                            vw::camera::CameraModel* cam2){

  // If we crop the images, we must regenerate each time the match
  // files.
  bool crop_left_and_right =
    ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)) &&
    ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) );

  if ( !crop_left_and_right && fs::exists( match_filename ) ) {
    vw_out() << "\t--> Using cached match file: " << match_filename << "\n";
    return true;
  }

  // Load the unmodified images
  DiskImageView<float> image1( input_file1 ), image2( input_file2 );

  // Worst case, no interest point operations have been performed before
  vw_out() << "\t--> Locating Interest Points\n";
  ip::InterestPointList ip1, ip2;
  asp::detect_ip( ip1, ip2, image1, image2, ip_per_tile,
                  nodata1, nodata2 );

  if ( ip1.size() > 10000 ) {
    ip1.sort(); ip1.resize(10000);
  }
  if ( ip2.size() > 10000 ) {
    ip2.sort(); ip2.resize(10000);
  }

  std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
  for (InterestPointList::iterator iter = ip1.begin(); iter != ip1.end(); ++iter)
    ip1_copy.push_back(*iter);
  for (InterestPointList::iterator iter = ip2.begin(); iter != ip2.end(); ++iter)
    ip2_copy.push_back(*iter);

  vw_out() << "\t--> Matching interest points\n";
  ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.5);
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;

  matcher(ip1_copy, ip2_copy,
          matched_ip1, matched_ip2,
          TerminalProgressCallback( "asp", "\t    Matching: "));


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

  return true;
}

// Helper function for determining image alignment.
vw::Matrix3x3
asp::StereoSessionPinhole::determine_image_align(std::string const& out_prefix,
                                                 std::string const& input_file1,
                                                 std::string const& input_file2,
                                                 float nodata1, float nodata2) {
  namespace fs = boost::filesystem;
  using namespace vw;

  std::string match_filename
    = ip::match_filename(out_prefix, input_file1, input_file2);
  ip_matching(input_file1, input_file2,
              stereo_settings().ip_per_tile,
              nodata1, nodata2,
              match_filename,
              NULL, NULL);

  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  read_binary_match_file( match_filename,
                          matched_ip1, matched_ip2 );

  // Get the matrix using RANSAC
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
  Matrix<double> T;
  try {

    vw::math::RandomSampleConsensus<vw::math::HomographyFittingFunctor, vw::math::InterestPointErrorMetric> ransac( vw::math::HomographyFittingFunctor(), vw::math::InterestPointErrorMetric(), 100, 10, ransac_ip1.size()/2, true);
    T = ransac( ransac_ip2, ransac_ip1 );
    std::vector<size_t> indices = ransac.inlier_indices(T, ransac_ip2, ransac_ip1 );
    vw_out(DebugMessage,"asp") << "\t--> AlignMatrix: " << T << std::endl;

  } catch (...) {
    vw_out(WarningMessage,"console") << "Automatic Alignment Failed! Proceed with caution...\n";
    T = vw::math::identity_matrix<3>();
  }

  return T;
}

void asp::StereoSessionPinhole::pre_preprocessing_hook
(bool adjust_left_image_size,
 std::string const& left_input_file,
 std::string const& right_input_file,
 std::string      & left_output_file,
 std::string      & right_output_file) {

  left_output_file  = m_out_prefix + "-L.tif";
  right_output_file = m_out_prefix + "-R.tif";

  bool crop_left_and_right =
    ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)) &&
    ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) );

  // If the output files already exist, and we don't crop both left
  // and right images, then there is nothing to do here.
  if ( boost::filesystem::exists(left_output_file)  &&
       boost::filesystem::exists(right_output_file) &&
       (!crop_left_and_right)) {
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelGray<float32> > out_left (left_output_file );
      DiskImageView<PixelGray<float32> > out_right(right_output_file);
      vw_out(InfoMessage) << "\t--> Using cached normalized input images.\n";
      vw_settings().reload_config();
      return;
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  // Retrieve nodata values
  float left_nodata_value, right_nodata_value;
  {
    // For this to work the ISIS type must be registered with the
    // DiskImageResource class.  - This happens in "stereo.cc", so
    // these calls will create DiskImageResourceIsis objects.
    boost::shared_ptr<DiskImageResource>
      left_rsrc (DiskImageResource::open(left_input_file )),
      right_rsrc(DiskImageResource::open(right_input_file));
    this->get_nodata_values(left_rsrc, right_rsrc,
                            left_nodata_value, right_nodata_value);
  }

  // Enforce no predictor in compression, it works badly with L.tif and R.tif.
  asp::BaseOptions options = m_options;
  options.gdal_options["PREDICTOR"] = "1";

  std::string left_cropped_file = left_input_file,
    right_cropped_file = right_input_file;

  // See if to crop the images
  if (crop_left_and_right) {
    // Crop the images, will use them from now on
    left_cropped_file  = this->m_out_prefix + "-L-cropped.tif";
    right_cropped_file = this->m_out_prefix + "-R-cropped.tif";

    DiskImageView<float> left_orig_image(left_input_file);
    stereo_settings().left_image_crop_win.crop(bounding_box(left_orig_image));
    vw_out() << "\t--> Writing cropped image: " << left_cropped_file << "\n";
    block_write_gdal_image(left_cropped_file,
                           crop(left_orig_image,
                                stereo_settings().left_image_crop_win),
                           left_nodata_value, options,
                           TerminalProgressCallback("asp", "\t:  "));

    DiskImageView<float> right_orig_image(right_input_file);
    stereo_settings().right_image_crop_win.crop(bounding_box(right_orig_image));
    vw_out() << "\t--> Writing cropped image: " << right_cropped_file << "\n";
    block_write_gdal_image(right_cropped_file,
                           crop(right_orig_image,
                                stereo_settings().right_image_crop_win),
                           right_nodata_value, options,
                           TerminalProgressCallback("asp", "\t:  "));
  }

  // Load the cropped images
  DiskImageView<float> left_disk_image(left_cropped_file),
    right_disk_image(right_cropped_file);

  ImageViewRef< PixelMask<float> > left_masked_image  = create_mask_less_or_equal(left_disk_image,  left_nodata_value);
  ImageViewRef< PixelMask<float> > right_masked_image = create_mask_less_or_equal(right_disk_image, right_nodata_value);

  Vector4f left_stats  = gather_stats(left_masked_image,  "left" );
  Vector4f right_stats = gather_stats(right_masked_image, "right");

  ImageViewRef< PixelMask<float> > Limg, Rimg;
  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);

  if ( stereo_settings().alignment_method == "epipolar" ) {

    vw_out() << "\t--> Performing epipolar alignment\n";

    // Load the two images and fetch the two camera models
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    camera_models( left_cam, right_cam );

    CAHVModel* left_epipolar_cahv  = dynamic_cast<CAHVModel*>(vw::camera::unadjusted_model(&(*left_cam)));
    CAHVModel* right_epipolar_cahv = dynamic_cast<CAHVModel*>(vw::camera::unadjusted_model(&(*right_cam)));

    // Remove lens distortion and create epipolar rectified images.
    if (boost::ends_with(lcase_file, ".cahvore")) {
      CAHVOREModel left_cahvore(m_left_camera_file );
      CAHVOREModel right_cahvore(m_right_camera_file);
      Limg = transform(left_masked_image,  CameraTransform<CAHVOREModel, CAHVModel>(left_cahvore,  *left_epipolar_cahv ));
      Rimg = transform(right_masked_image, CameraTransform<CAHVOREModel, CAHVModel>(right_cahvore, *right_epipolar_cahv));
    } else if (boost::ends_with(lcase_file, ".cahvor") ||
               boost::ends_with(lcase_file, ".cmod") ) {
      CAHVORModel left_cahvor (m_left_camera_file );
      CAHVORModel right_cahvor(m_right_camera_file);
      Limg = transform(left_masked_image,  CameraTransform<CAHVORModel, CAHVModel>(left_cahvor,  *left_epipolar_cahv ));
      Rimg = transform(right_masked_image, CameraTransform<CAHVORModel, CAHVModel>(right_cahvor, *right_epipolar_cahv));

    } else if ( boost::ends_with(lcase_file, ".cahv") ||
                boost::ends_with(lcase_file, ".pin" )) {
      CAHVModel left_cahv (m_left_camera_file );
      CAHVModel right_cahv(m_right_camera_file);
      Limg = transform(left_masked_image,  CameraTransform<CAHVModel, CAHVModel>(left_cahv,  *left_epipolar_cahv ));
      Rimg = transform(right_masked_image, CameraTransform<CAHVModel, CAHVModel>(right_cahv, *right_epipolar_cahv));

    } else if ( boost::ends_with(lcase_file, ".pinhole") ||
                boost::ends_with(lcase_file, ".tsai") ) {
      PinholeModel left_pin (m_left_camera_file );
      PinholeModel right_pin(m_right_camera_file);
      Limg = transform(left_masked_image,  CameraTransform<PinholeModel, CAHVModel>(left_pin,  *left_epipolar_cahv ));
      Rimg = transform(right_masked_image, CameraTransform<PinholeModel, CAHVModel>(right_pin, *right_epipolar_cahv));

    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }

  } else if ( stereo_settings().alignment_method == "homography" ) {

    vw_out() << "\t--> Performing homography alignment\n";

    Matrix<double> align_matrix
      = determine_image_align(m_out_prefix,
                              left_cropped_file,   right_cropped_file,
                              left_nodata_value, right_nodata_value);
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

  vw_out() << "\t--> Writing pre-aligned images.\n";
  block_write_gdal_image( left_output_file, apply_mask(Limg, output_nodata),
                          output_nodata, options,
                          TerminalProgressCallback("asp","\t  L:  ") );
  block_write_gdal_image( right_output_file, apply_mask(crop(edge_extend(Rimg,ConstantEdgeExtension()),bounding_box(Limg)), output_nodata),
                          output_nodata, options,
                          TerminalProgressCallback("asp","\t  R:  ") );
}
