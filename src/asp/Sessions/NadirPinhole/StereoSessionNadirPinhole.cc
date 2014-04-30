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


/// \file StereoSessionNadirPinhole.cc
///

#include <asp/Core/StereoSettings.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Sessions/NadirPinhole/StereoSessionNadirPinhole.h>

#include <vw/Camera.h>
#include <vw/Image/Transform.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace asp;
using namespace vw::camera;

StereoSessionNadirPinhole::left_tx_type
StereoSessionNadirPinhole::tx_left() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-L.exr" );
  }
  return left_tx_type( tx );
}

StereoSessionNadirPinhole::right_tx_type
StereoSessionNadirPinhole::tx_right() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-R.exr" );
  }
  return right_tx_type( tx );
}

void asp::StereoSessionNadirPinhole::pre_preprocessing_hook(
                                                            std::string const& left_input_file,
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

  } else if ( stereo_settings().alignment_method == "homography" ||
              stereo_settings().alignment_method == "affineepipolar" ) {
    // Getting left image size. Later alignment options can choose to
    // change this parameters. (Affine Epipolar).
    Vector2i left_size = file_image_size( left_input_file ),
      right_size = file_image_size( right_input_file );

    std::string match_filename
      = ip::match_filename(m_out_prefix, left_input_file, right_input_file);

    if (!fs::exists(match_filename)) {
      boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
      camera_models( left_cam, right_cam );

      bool inlier =
        ip_matching_w_alignment( left_cam.get(), right_cam.get(),
                                 left_disk_image, right_disk_image,
                                 cartography::Datum("WGS84"), match_filename,
                                 left_nodata_value, right_nodata_value );
      if ( !inlier ) {
        fs::remove( match_filename );
        vw_throw( IOErr() << "Unable to match left and right images." );
      }
    } else{
      vw_out() << "\t--> Using cached match file: " << match_filename << "\n";
    }

    std::vector<ip::InterestPoint> left_ip, right_ip;
    ip::read_binary_match_file( match_filename, left_ip, right_ip  );

    Matrix<double> align_left_matrix = math::identity_matrix<3>(),
      align_right_matrix = math::identity_matrix<3>();
    if ( stereo_settings().alignment_method == "homography" ) {
      left_size =
        homography_rectification( left_size, right_size, left_ip, right_ip,
                                  align_left_matrix, align_right_matrix );
      vw_out() << "\t--> Aligning right image to left using matrices:\n"
               << "\t      " << align_left_matrix << "\n"
               << "\t      " << align_right_matrix << "\n";
    } else {
      left_size =
        affine_epipolar_rectification( left_size, right_size,
                                       left_ip, right_ip,
                                       align_left_matrix,
                                       align_right_matrix );

      vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
               << "\t      " << submatrix(align_left_matrix,0,0,2,3) << "\n"
               << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
    }
    write_matrix( m_out_prefix + "-align-L.exr", align_left_matrix );
    write_matrix( m_out_prefix + "-align-R.exr", align_right_matrix );
    right_size = left_size; // Because the images are now aligned
                            // .. they are the same size.

    // Applying alignment transform
    Limg = transform(left_masked_image,
                     HomographyTransform(align_left_matrix),
                     left_size.x(), left_size.y() );
    Rimg = transform(right_masked_image,
                     HomographyTransform(align_right_matrix),
                     right_size.x(), right_size.y() );
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
  block_write_gdal_image( right_output_file,
                          apply_mask(crop(edge_extend(Rimg,ZeroEdgeExtension()),bounding_box(Limg)), output_nodata),
                          output_nodata, options,
                          TerminalProgressCallback("asp","\t  R:  ") );

}
