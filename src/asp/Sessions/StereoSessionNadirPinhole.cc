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

//#include <asp/Core/StereoSettings.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>

#include <vw/Camera.h>
#include <vw/Image/Transform.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace asp;
using namespace vw::camera;

//TODO: There is a lot of duplicate code here with the Pinhole
//class. Common functionality must be factored out.

void asp::StereoSessionNadirPinhole::pre_preprocessing_hook(bool adjust_left_image_size,
                                                            std::string const& left_input_file,
                                                            std::string const& right_input_file,
                                                            std::string      & left_output_file,
                                                            std::string      & right_output_file) {

  std::string left_cropped_file, right_cropped_file;
  vw::cartography::GdalWriteOptions options;
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
  if (exit_early) 
    return;

  // Load the cropped images
  DiskImageView<float> left_disk_image(left_cropped_file),
    right_disk_image(right_cropped_file);

  ImageViewRef< PixelMask<float> > left_masked_image
    = create_mask_less_or_equal(left_disk_image,  left_nodata_value);
  ImageViewRef< PixelMask<float> > right_masked_image
    = create_mask_less_or_equal(right_disk_image, right_nodata_value);

  Vector6f left_stats  = gather_stats(left_masked_image,  "left" );
  Vector6f right_stats = gather_stats(right_masked_image, "right");

  ImageViewRef< PixelMask<float> > Limg, Rimg;
  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);

  // Use no-data in interpolation and edge extension.
  PixelMask<float> nodata_pix(0);
  nodata_pix.invalidate();
  ValueEdgeExtension< PixelMask<float> > ext(nodata_pix); 

  if ( stereo_settings().alignment_method == "epipolar" ) {

    vw_out() << "\t--> Performing epipolar alignment\n";

    // Load the two images and fetch the two camera models
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;

    if ( boost::ends_with(lcase_file, ".pinhole") ||
         boost::ends_with(lcase_file, ".tsai"   )   ) {

      Vector2i left_out_size, right_out_size;
      load_camera_models( left_cam, right_cam, left_out_size, right_out_size );
      
      // Write out the camera models used to generate the aligned images.
      // - Currently this won't work if we used .adjust files from bundle_adjust!
      PinholeModel* left_pin_model  = dynamic_cast<PinholeModel*>(left_cam.get ());
      PinholeModel* right_pin_model = dynamic_cast<PinholeModel*>(right_cam.get());
      if (left_pin_model)
        left_pin_model->write(m_out_prefix + "-L.tsai");
      if (right_pin_model)
        right_pin_model->write(m_out_prefix + "-R.tsai");
      
      get_epipolar_transformed_pinhole_images(m_left_camera_file, m_right_camera_file,
                                              left_cam, right_cam,
                                              left_masked_image, right_masked_image,
                                              left_out_size, right_out_size,
                                              Limg, Rimg,
                                              ext,
                                              BilinearInterpolation());
    } else { // Handle CAHV derived models
      
      camera_models( left_cam, right_cam );
      
      get_epipolar_transformed_images(m_left_camera_file, m_right_camera_file,
                                      left_cam, right_cam,
                                      left_masked_image, right_masked_image,
                                      Limg, Rimg, ext);
    }                                    
    
  } else if ( stereo_settings().alignment_method == "homography" ||
              stereo_settings().alignment_method == "affineepipolar" ) {
    // Getting left image size. Later alignment options can choose to
    // change this parameters. (Affine Epipolar).
    Vector2i left_size  = file_image_size(left_cropped_file ),
      right_size = file_image_size(right_cropped_file);
    
    // Define the file name containing IP match information.
    std::string match_filename = ip::match_filename(this->m_out_prefix,
                                                    left_cropped_file,
                                                    right_cropped_file);
    
    DiskImageView<float> left_orig_image(left_input_file);
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    camera_models( left_cam, right_cam );
    this->ip_matching(left_cropped_file,   right_cropped_file,
                      bounding_box(left_orig_image).size(),
                      left_stats, right_stats,
                      stereo_settings().ip_per_tile,
                      left_nodata_value, right_nodata_value, match_filename,
                      left_cam.get(), right_cam.get() );

    std::vector<ip::InterestPoint> left_ip, right_ip;
    ip::read_binary_match_file( match_filename, left_ip, right_ip  );

    Matrix<double> align_left_matrix  = math::identity_matrix<3>(),
                   align_right_matrix = math::identity_matrix<3>();

    if ( stereo_settings().alignment_method == "homography" ) {
      left_size = homography_rectification(adjust_left_image_size,
                                           left_size, right_size, left_ip, right_ip,
                                           align_left_matrix, align_right_matrix );
      vw_out() << "\t--> Aligning right image to left using matrices:\n"
               << "\t      " << align_left_matrix  << "\n"
               << "\t      " << align_right_matrix << "\n";
    } else {
      left_size = affine_epipolar_rectification(left_size, right_size,
                                                left_ip,   right_ip,
                                                align_left_matrix,
                                                align_right_matrix );
      
      vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
               << "\t      " << submatrix(align_left_matrix, 0,0,2,3) << "\n"
               << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
    }
    write_matrix(m_out_prefix + "-align-L.exr", align_left_matrix );
    write_matrix(m_out_prefix + "-align-R.exr", align_right_matrix);
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
                   false, // Use std stretch
                   left_stats, right_stats, Limg, Rimg);

  // The output no-data value must be < 0 as we scale the images to [0, 1].
  bool has_nodata = true;
  float output_nodata = -32768.0;

  vw_out() << "\t--> Writing pre-aligned images.\n";
  vw_out() << "\t--> Writing: " << left_output_file << ".\n";
  block_write_gdal_image( left_output_file, apply_mask(Limg, output_nodata),
                          has_left_georef, left_georef,
                          has_nodata, output_nodata,
                          options,
                          TerminalProgressCallback("asp","\t  L:  ") );
  vw_out() << "\t--> Writing: " << right_output_file << ".\n";
  block_write_gdal_image( right_output_file,
                          apply_mask(crop(edge_extend(Rimg, ext),
                                          bounding_box(Limg)), output_nodata),
                          has_right_georef, right_georef,
                          has_nodata, output_nodata,
                          options,
                          TerminalProgressCallback("asp","\t  R:  ") );

}

asp::StereoSessionNadirPinhole::tx_type
asp::StereoSessionNadirPinhole::tx_left() const {
  if ( (stereo_settings().alignment_method == "homography"    ) ||
       (stereo_settings().alignment_method == "affineepipolar")   ) {
    Matrix<double> align_matrix;
    read_matrix( align_matrix, m_out_prefix + "-align-L.exr" );
    return tx_type( align_matrix );
  }
  return tx_type( math::identity_matrix<3>() );
}
asp::StereoSessionNadirPinhole::tx_type
asp::StereoSessionNadirPinhole::tx_right() const {
  if ( (stereo_settings().alignment_method == "homography"    ) ||
       (stereo_settings().alignment_method == "affineepipolar")   ) {
    Matrix<double> align_matrix;
    read_matrix( align_matrix, m_out_prefix + "-align-R.exr" );
    return tx_type( align_matrix );
  }
  return tx_type( math::identity_matrix<3>() );
}


