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


/// \file StereoSessionSpot.cc
///
#include <vw/Image/ImageMath.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Transform.h>

//#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Sessions/StereoSessionSpot.h>


#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace vw;
using namespace asp;


namespace pt = boost::posix_time;
namespace fs = boost::filesystem;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector2f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
}


namespace asp {

  vw::cartography::GeoReference StereoSessionSpot::get_georef() {

    // SPOT5 images never have georef information!
    // The best we can do is to set the datum to what SPOT5 always uses and
    // stick it inside a fake georef object.

    vw::cartography::GeoReference georef = vw::cartography::GeoReference();
    Matrix3x3 transform = georef.transform();
    // assume these are degrees, does not mater much, but it needs be small enough
    double small = 1e-8;
    transform(0,0) = small;
    transform(1,1) = small;
    transform(0,2) = small;
    transform(1,2) = small;
    georef.set_transform(transform);
    georef.set_geographic();

    georef.set_datum(vw::cartography::Datum("WGS84"));

    return georef;
  }



  void StereoSessionSpot::
  pre_preprocessing_hook(bool adjust_left_image_size,
			 std::string const& left_input_file,
			 std::string const& right_input_file,
			 std::string      & left_output_file,
			 std::string      & right_output_file) {

    std::string left_cropped_file, right_cropped_file;
    vw::cartography::GdalWriteOptions options;
    float left_nodata_value, right_nodata_value;
    bool  has_left_georef,   has_right_georef;
    vw::cartography::GeoReference left_georef, right_georef;

    // Normally this is where we would call shared_preprocessing_hook()
    //  but it runs into some problems with the Spot5 image type.
    // - A significant refactoring would address this problem.
    bool exit_early =
      unshared_preprocessing_hook(options,
                                  left_input_file,   right_input_file,
                                  left_output_file,  right_output_file,
                                  left_cropped_file, right_cropped_file,
                                  left_nodata_value, right_nodata_value,
                                  has_left_georef,   has_right_georef,
                                  left_georef,       right_georef);
    
    if (exit_early) return;

    const bool left_is_cropped  = (left_cropped_file  != left_input_file);
    const bool right_is_cropped = (right_cropped_file != right_input_file);

    // Load the cropped images
    // - These can be either SPOT5 images (no crop) or GDAL images (cropped)
    // - In the cropped case we don't have a camera model file.
    boost::shared_ptr<DiskImageResource> left_rsrc, right_rsrc, left_input_rsrc;
    left_input_rsrc = vw::DiskImageResourcePtr(left_input_file);
    if (left_is_cropped) // GDAL format
      left_rsrc = vw::DiskImageResourcePtr(left_cropped_file);
    else // SPOT format
      left_rsrc = left_input_rsrc; // Use the resource we already loaded.   
    if (right_is_cropped) // GDAL format
      right_rsrc = vw::DiskImageResourcePtr(right_cropped_file);
    else // SPOT format
      right_rsrc = vw::DiskImageResourcePtr(right_input_file);

    // Now load the DiskImageResource objects into DiskImageViews.
    DiskImageView<float> left_disk_image (left_rsrc ),
                         right_disk_image(right_rsrc),
                         left_orig_image(left_input_rsrc);

    // Set up image masks
    ImageViewRef< PixelMask<float> > left_masked_image
      = create_mask_less_or_equal(left_disk_image,  left_nodata_value);
    ImageViewRef< PixelMask<float> > right_masked_image
      = create_mask_less_or_equal(right_disk_image, right_nodata_value);

    // Compute input image statistics
    Vector6f left_stats  = gather_stats(left_masked_image,  "left" );
    Vector6f right_stats = gather_stats(right_masked_image, "right");

    ImageViewRef< PixelMask<float> > Limg, Rimg;
    std::string lcase_file = boost::to_lower_copy(this->m_left_camera_file);

    // Image alignment block - Generate aligned versions of the input
    // images according to the options.
    if ( stereo_settings().alignment_method == "homography" ||
	 stereo_settings().alignment_method == "affineepipolar" ) {
      // Define the file name containing IP match information.
      std::string match_filename = ip::match_filename(this->m_out_prefix,
						      left_cropped_file,
						      right_cropped_file);

      // Detect matching interest points between the left and right input images.
      // - The output is written directly to file!
      boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
      this->camera_models(left_cam, right_cam); // Fetch the camera models.
      this->ip_matching(left_cropped_file,   right_cropped_file,
			bounding_box(left_orig_image).size(),
			left_stats, right_stats,
			stereo_settings().ip_per_tile,
			left_nodata_value, right_nodata_value, match_filename,
			left_cam.get(),    right_cam.get() );

      // Load the interest points results from the file we just wrote.
      std::vector<ip::InterestPoint> left_ip, right_ip;
      ip::read_binary_match_file(match_filename, left_ip, right_ip);

      // Initialize alignment matrices and get the input image sizes.
      Matrix<double> align_left_matrix  = math::identity_matrix<3>(),
                     align_right_matrix = math::identity_matrix<3>();
      Vector2i left_size, right_size;
      if (left_is_cropped) // GDAL format
        left_size  = file_image_size(left_cropped_file);
      else // SPOT5 format
        left_size  = file_image_size(left_cropped_file);

      if (right_is_cropped) // GDAL format
        right_size  = file_image_size(right_cropped_file);
      else // SPOT5 format
        right_size  = file_image_size(right_cropped_file);

      // Compute the appropriate alignment matrix based on the input points
      if ( stereo_settings().alignment_method == "homography" ) {
        left_size = homography_rectification(adjust_left_image_size,
                                             left_size,         right_size,
                                             left_ip,           right_ip,
                                             align_left_matrix, align_right_matrix);
        vw_out() << "\t--> Aligning right image to left using matrices:\n"
	         << "\t      " << align_left_matrix  << "\n"
	         << "\t      " << align_right_matrix << "\n";
      } else {
        left_size = affine_epipolar_rectification(left_size,         right_size,
                                                  left_ip,           right_ip,
                                                  align_left_matrix, align_right_matrix);
        vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
	         << "\t      " << submatrix(align_left_matrix, 0,0,2,3) << "\n"
	         << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
      }
      // Write out both computed matrices to disk
      write_matrix(this->m_out_prefix + "-align-L.exr", align_left_matrix );
      write_matrix(this->m_out_prefix + "-align-R.exr", align_right_matrix);

      // Apply the alignment transform to both input images
      Limg = transform(left_masked_image,
                       HomographyTransform(align_left_matrix),
                       left_size.x(), left_size.y() );
      Rimg = transform(right_masked_image,
                       HomographyTransform(align_right_matrix),
                       left_size.x(), left_size.y() );
    } else if ( stereo_settings().alignment_method == "epipolar" ) {
      vw_throw( NoImplErr() << "StereoSessionGdal does not support epipolar rectification" );
    } else {
      // No alignment, just provide the original files.
      Limg = left_masked_image;
      Rimg = right_masked_image;
    } // End of image alignment block

    // Apply our normalization options.
    normalize_images(stereo_settings().force_use_entire_range,
		     stereo_settings().individually_normalize,
		     false, // Use std stretch
		     left_stats, right_stats, Limg, Rimg);

    // The output no-data value must be < 0 as we scale the images to [0, 1].
    bool  has_nodata    = true;
    float output_nodata = -32768.0;

    // The left image is written out with no alignment warping.
    vw_out() << "\t--> Writing pre-aligned images.\n";
    vw_out() << "\t--> Writing: " << left_output_file << ".\n";
    block_write_gdal_image( left_output_file, apply_mask(Limg, output_nodata),
                            has_left_georef, left_georef,
                            has_nodata, output_nodata, options,
                            TerminalProgressCallback("asp","\t  L:  ") );

    vw_out() << "\t--> Writing: " << right_output_file << ".\n";
    if ( stereo_settings().alignment_method == "none" )
      block_write_gdal_image( right_output_file, apply_mask(Rimg, output_nodata),
                              has_right_georef, right_georef,
                              has_nodata, output_nodata, options,
                              TerminalProgressCallback("asp","\t  R:  ") );
    else // Write out the right image cropped to align with the left image.
      block_write_gdal_image( right_output_file,
                              apply_mask(crop(edge_extend(Rimg, ConstantEdgeExtension()),
                              bounding_box(Limg)), output_nodata),
                              has_right_georef, right_georef,
                              has_nodata, output_nodata, options,
                              TerminalProgressCallback("asp","\t  R:  ") );
  } // End function pre_preprocessing_hook




bool StereoSessionSpot::
unshared_preprocessing_hook(vw::cartography::GdalWriteOptions              & options,
                            std::string const             & left_input_file,
                            std::string const             & right_input_file,
                            std::string                   & left_output_file,
                            std::string                   & right_output_file,
                            std::string                   & left_cropped_file,
                            std::string                   & right_cropped_file,
                            float                         & left_nodata_value,
                            float                         & right_nodata_value,
                            bool                          & has_left_georef,
                            bool                          & has_right_georef,
                            vw::cartography::GeoReference & left_georef,
                            vw::cartography::GeoReference & right_georef){

  // To simplify reintegration later, most of this function has been left the same
  //  as the general purpose function even if it could be simplified for the SPOT5 case.
 

  // Retrieve nodata values and let the handles go out of scope right away.
  // - SPOT5 does not support nodata values, but this will handle user-provided values.
  boost::shared_ptr<DiskImageResource>
    left_rsrc (vw::DiskImageResourcePtr(left_input_file)),
    right_rsrc(vw::DiskImageResourcePtr(right_input_file));
  this->get_nodata_values(left_rsrc,         right_rsrc,
                          left_nodata_value, right_nodata_value);

  // Set output file paths
  left_output_file  = this->m_out_prefix + "-L.tif";
  right_output_file = this->m_out_prefix + "-R.tif";

  left_cropped_file  = left_input_file;
  right_cropped_file = right_input_file;

  // Enforce no predictor in compression, it works badly with L.tif and R.tif.
  options = this->m_options;
  options.gdal_options["PREDICTOR"] = "1";

  // SPOT5 images do not have a georef.  Will have to change this to support map projection.
  has_left_georef  = false;
  has_right_georef = false;
  if ( stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef = false;
    has_right_georef = false;
  }

  bool crop_left  = ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  // If the output files already exist, and we don't crop both left
  // and right images, then there is nothing to do here.
  // Note: Must make sure all outputs are initialized before we
  // get to this part where we exit early.
  if ( boost::filesystem::exists(left_output_file)  &&
       boost::filesystem::exists(right_output_file) &&
       !crop_left && !crop_right) {
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelGray<float32> > out_left (left_output_file );
      DiskImageView<PixelGray<float32> > out_right(right_output_file);
      vw_out(InfoMessage) << "\t--> Using cached normalized input images.\n";
      vw_settings().reload_config();
      return true; // Return true if we exist early since the images exist
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  

  // See if to crop the images
  if (crop_left) {
    // Crop the images, will use them from now on. Crop the georef as well, if available.
    left_cropped_file  = this->m_out_prefix + "-L-cropped.tif";

    // SPOT5 images do not have a georef.  Will have to change this to support map projection.
    has_left_georef  = false;
    bool has_nodata = true;

    DiskImageView<float> left_orig_image(left_rsrc);
    BBox2i left_win  = stereo_settings().left_image_crop_win;
    left_win.crop (bounding_box(left_orig_image ));

    vw_out() << "\t--> Writing cropped image: " << left_cropped_file << "\n";
    block_write_gdal_image(left_cropped_file,
			   crop(left_orig_image, left_win),
			   has_left_georef, crop(left_georef, left_win),
			   has_nodata, left_nodata_value,
			   options,
			   TerminalProgressCallback("asp", "\t:  "));
  }
  if (crop_right) {
    // Crop the images, will use them from now on. Crop the georef as well, if available.
    right_cropped_file = this->m_out_prefix + "-R-cropped.tif";

    // SPOT5 images do not have a georef.  Will have to change this to support map projection.
    has_right_georef = false;
    bool has_nodata = true;

    DiskImageView<float> right_orig_image(right_rsrc);
    BBox2i right_win = stereo_settings().right_image_crop_win;
    right_win.crop(bounding_box(right_orig_image));

    vw_out() << "\t--> Writing cropped image: " << right_cropped_file << "\n";
    block_write_gdal_image(right_cropped_file,
			   crop(right_orig_image, right_win),
			   has_right_georef,
			   crop(right_georef, right_win),
			   has_nodata, right_nodata_value,
			   options,
			   TerminalProgressCallback("asp", "\t:  "));
  }


  // Re-read the georef, since it changed above.
  // - SPOT5 images do not have a georef.  Will have to change this to support map projection.
  has_left_georef  = false;
  has_right_georef = false;
  if ( stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef = false;
    has_right_georef = false;
  }

  return false; // don't exit early
}



} // End namespace asp
