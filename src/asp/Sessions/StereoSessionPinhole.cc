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
#include <asp/Core/InterestPointMatching.h>

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

// TODO: The other Session classes use functions like affine_epipolar_rectification
//        and homograhy_rectification (in the IP file) to perform this task.
//       Why does the Pinhole Session need to use something different?


// Helper function for determining image alignment.
vw::Matrix3x3
asp::StereoSessionPinhole::determine_image_align(std::string const& out_prefix,
                                                 std::string const& input_file1,
                                                 std::string const& input_file2,
                                                 vw::Vector2 const& uncropped_image_size,
                                                 Vector6f    const& stats1,
                                                 Vector6f    const& stats2,
                                                 float nodata1, float nodata2) {
  namespace fs = boost::filesystem;
  using namespace vw;

  std::string match_filename = ip::match_filename(out_prefix, input_file1, input_file2);
  vw::camera::CameraModel* null_camera_model = 0;
  this->ip_matching(input_file1, input_file2,
                    uncropped_image_size,
                    stats1, stats2,
                    stereo_settings().ip_per_tile,
                    nodata1, nodata2,
                    match_filename,
                    null_camera_model, null_camera_model);

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

void asp::StereoSessionPinhole::pre_preprocessing_hook(bool adjust_left_image_size,
                                                       std::string const& left_input_file,
                                                       std::string const& right_input_file,
                                                       std::string      & left_output_file,
                                                       std::string      & right_output_file) {


  std::string left_cropped_file, right_cropped_file;
  asp::BaseOptions options;
  float left_nodata_value, right_nodata_value;
  bool has_left_georef, has_right_georef;
  vw::cartography::GeoReference left_georef, right_georef;
  bool exit_early =
    StereoSession::shared_preprocessing_hook(options,
					     left_input_file,   right_input_file,
					     left_output_file,  right_output_file,
					     left_cropped_file, right_cropped_file,
					     left_nodata_value, right_nodata_value,
					     has_left_georef,   has_right_georef,
					     left_georef,       right_georef);
  if (exit_early) return;

  // Load the cropped images
  DiskImageView<float> left_disk_image(left_cropped_file),
                       right_disk_image(right_cropped_file);

  ImageViewRef< PixelMask<float> > left_masked_image  = create_mask_less_or_equal(left_disk_image,  left_nodata_value);
  ImageViewRef< PixelMask<float> > right_masked_image = create_mask_less_or_equal(right_disk_image, right_nodata_value);

  Vector6f left_stats  = gather_stats(left_masked_image,  "left" );
  Vector6f right_stats = gather_stats(right_masked_image, "right");

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
      CAHVOREModel left_cahvore (m_left_camera_file );
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

    DiskImageView<float> left_orig_image(left_input_file);
    Matrix<double> align_matrix
      = determine_image_align(m_out_prefix,
                              left_cropped_file, right_cropped_file,
                              bounding_box(left_orig_image).size(),
                              left_stats,        right_stats,
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
                   false, // Use std stretch
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

namespace asp {

// TODO: Move all these functions to some Pinhole camera class.

/// Load a pinhole camera model
boost::shared_ptr<vw::camera::CameraModel>
load_pinhole_camera_model(std::string const& path){
  // Keypoint alignment and everything else just gets camera models
  std::string lcase_file = boost::to_lower_copy(path);
  if (boost::ends_with(lcase_file,".cahvore") ) {
    return boost::shared_ptr<vw::camera::CameraModel>( new vw::camera::CAHVOREModel(path) );
  } else if (boost::ends_with(lcase_file,".cahvor") ||
             boost::ends_with(lcase_file,".cmod"  )   ) {
    return boost::shared_ptr<vw::camera::CameraModel>( new vw::camera::CAHVORModel(path) );
  } else if ( boost::ends_with(lcase_file,".cahv") ||
              boost::ends_with(lcase_file,".pin" )   ) {
    return boost::shared_ptr<vw::camera::CameraModel>( new vw::camera::CAHVModel(path) );
  } else if ( boost::ends_with(lcase_file,".pinhole") ||
              boost::ends_with(lcase_file,".tsai"   )   ) {
    return boost::shared_ptr<vw::camera::CameraModel>( new vw::camera::PinholeModel(path) );
  } else {
    vw::vw_throw(vw::ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
  }
}

boost::shared_ptr<vw::camera::CAHVModel>
load_cahv_pinhole_camera_model(std::string const& image_path,
                               std::string const& camera_path){
  // Get the image size
  vw::DiskImageView<float> disk_image(image_path);
  vw::Vector2i image_size(disk_image.cols(), disk_image.rows());

  // Load the appropriate camera model object and if necessary
  // convert it to the CAHVModel type.
  std::string lcase_file = boost::to_lower_copy(camera_path);
  boost::shared_ptr<vw::camera::CAHVModel> cahv(new vw::camera::CAHVModel);
  if (boost::ends_with(lcase_file, ".cahvore") ) {
    vw::camera::CAHVOREModel cahvore(camera_path);
    *(cahv.get()) = vw::camera::linearize_camera(cahvore, image_size, image_size);
  } else if (boost::ends_with(lcase_file, ".cahvor")  ||
             boost::ends_with(lcase_file, ".cmod"  )   ) {
    vw::camera::CAHVORModel cahvor(camera_path);
    *(cahv.get()) = vw::camera::linearize_camera(cahvor, image_size, image_size);

  } else if ( boost::ends_with(lcase_file, ".cahv") ||
              boost::ends_with(lcase_file, ".pin" )) {
    *(cahv.get()) = vw::camera::CAHVModel(camera_path);

  } else if ( boost::ends_with(lcase_file, ".pinhole") ||
              boost::ends_with(lcase_file, ".tsai"   )   ) {
    vw::camera::PinholeModel left_pin(camera_path);
    *(cahv.get()) = vw::camera::linearize_camera(left_pin);

  } else {
    vw_throw(vw::ArgumentErr() << "CameraModelLoader::load_cahv_pinhole_camera_model - unsupported camera file type.\n");
  }

  return cahv;
}


boost::shared_ptr<vw::camera::CameraModel>
load_adj_pinhole_model(std::string const& image_file, std::string const& camera_file,
                       std::string const& left_image_file, std::string const& right_image_file,
                       std::string const& left_camera_file, std::string const& right_camera_file,
                       std::string const& input_dem){

  // Unfortunately the pinhole case is more complicated since the left and right files are inter-dependent.

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 pixel_offset = camera_pixel_offset(input_dem,
                                                 left_image_file,
                                                 right_image_file,
                                                 image_file);

  if ( stereo_settings().alignment_method != "epipolar" ) {
    // Not epipolar, just load the camera model.
    return load_adjusted_model(load_pinhole_camera_model(camera_file),
                               image_file, camera_file, pixel_offset);
  }
  // Otherwise handle the epipolar case

  bool is_left_camera = true;
  if (image_file == left_image_file)
    is_left_camera = true;
  else if (image_file == right_image_file)
    is_left_camera = false;
  else
    (ArgumentErr() << "StereoSessionPinhole: supplied camera model filename does not match the name supplied in the constructor.");

  // Fetch CAHV version of the two input pinhole files
  boost::shared_ptr<CAHVModel> left_cahv
     = load_cahv_pinhole_camera_model(left_image_file,  left_camera_file);
  boost::shared_ptr<CAHVModel> right_cahv
     = load_cahv_pinhole_camera_model(right_image_file, right_camera_file);

  // Create epipolar rectified camera views
  boost::shared_ptr<CAHVModel> epipolar_left_cahv (new CAHVModel);
  boost::shared_ptr<CAHVModel> epipolar_right_cahv(new CAHVModel);
  epipolar(*(left_cahv.get()),  *(right_cahv.get()),
           *epipolar_left_cahv, *epipolar_right_cahv);

  if (is_left_camera)
    return load_adjusted_model(epipolar_left_cahv, image_file, camera_file, pixel_offset);

  // Right camera
  return load_adjusted_model(epipolar_right_cahv, image_file, camera_file, pixel_offset);
}
}

// Redirect to the correct function depending on the template parameters
boost::shared_ptr<vw::camera::CameraModel>
asp::StereoSessionPinhole::camera_model(std::string const& image_file,
                                    std::string const& camera_file) {
  return asp::load_adj_pinhole_model(image_file, camera_file,
                                m_left_image_file, m_right_image_file,
                                m_left_camera_file, m_right_camera_file,
                                m_input_dem);

}


asp::StereoSessionPinhole::tx_type
asp::StereoSessionPinhole::tx_left() const {
  Matrix<double> tx = math::identity_matrix<3>();
  return tx_type( tx );
}
asp::StereoSessionPinhole::tx_type
asp::StereoSessionPinhole::tx_right() const {
  if ( stereo_settings().alignment_method == "homography" ) {
    Matrix<double> align_matrix;
    read_matrix( align_matrix, m_out_prefix + "-align-R.exr" );
    return tx_type( align_matrix );
  }
  return tx_type( math::identity_matrix<3>() );
}
