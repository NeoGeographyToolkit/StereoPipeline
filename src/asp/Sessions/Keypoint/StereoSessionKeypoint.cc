// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionKeypoint.cc
///

#include <boost/shared_ptr.hpp>

#include <iostream>                        // debugging
#include <algorithm>

#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/Keypoint/StereoSessionKeypoint.h>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Math.h>

using namespace vw;
using namespace vw::ip;

std::string StereoSessionKeypoint::create_subsampled_align_image(std::string const& image_file, std::string const& suffix) {
  std::string align_image_file(m_out_prefix + std::string("-normalized-align-sub-") + suffix);

  DiskImageView<PixelGray<float> > disk_image(image_file);

  std::cout << "StereoSessionKeypoint::create_subsampled_align_image():\n";
  double sub_sampling = stereo_settings().keypoint_align_subsampling;
  write_image(align_image_file, channel_cast_rescale<uint8>(resample(disk_image, 1.0 / double(sub_sampling))), TerminalProgressCallback("asp","Subsampling:"));

  return align_image_file;
}

void StereoSessionKeypoint::scale_align_matrix(Matrix<double> & align_matrix) {
  Matrix<double> scale_matrix = vw::math::identity_matrix(3);
  Matrix<double> inv_scale_matrix = vw::math::identity_matrix(3);
  double sub_sampling = stereo_settings().keypoint_align_subsampling;
  scale_matrix(0, 0) = 1.0 / double(sub_sampling);
  scale_matrix(1, 1) = scale_matrix(0, 0);
  inv_scale_matrix(0, 0) = double(sub_sampling);
  inv_scale_matrix(1, 1) = inv_scale_matrix(0, 0);
  align_matrix = align_matrix * scale_matrix;
  align_matrix = inv_scale_matrix * align_matrix;
  vw_out() << "StereoSessionKeypoint::adjust_align_matrix(): scaled alignment matrix:\n";
  vw_out() << align_matrix << std::endl;
}

void
StereoSessionKeypoint::pre_preprocessing_hook(std::string const&/*input_file1*/,
                                              std::string const&/*input_file2*/,
                                              std::string & output_file1,
                                              std::string & output_file2) {

  std::string left_align_image_file(m_left_image_file),
    right_align_image_file(m_right_image_file);

  double sub_sampling = stereo_settings().keypoint_align_subsampling;
  if (sub_sampling > 1) {
    left_align_image_file =
      create_subsampled_align_image(m_left_image_file, "L.tif");
    right_align_image_file =
      create_subsampled_align_image(m_right_image_file, "R.tif");
  }

  // Load the two images
  DiskImageView<PixelGray<float> > left_align_image(left_align_image_file);
  DiskImageView<PixelGray<float> > right_align_image(right_align_image_file);

  // Determine the alignment matrix using keypoint matching techniques.
  Matrix<double> align_matrix = determine_image_align(left_align_image_file,
                                                      right_align_image_file,
                                                      left_align_image,
                                                      right_align_image);
  if (sub_sampling > 1)
    scale_align_matrix(align_matrix);
  ::write_matrix(m_out_prefix + "-align.exr", align_matrix);

  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  ImageViewRef<PixelGray<float> > Limg = left_disk_image;
  ImageViewRef<PixelGray<float> > Rimg = transform(right_disk_image, HomographyTransform(align_matrix),
                                                   left_disk_image.cols(), left_disk_image.rows());

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  write_image(output_file1, channel_cast_rescale<uint8>(Limg),
              TerminalProgressCallback("asp",""));
  write_image(output_file2, channel_cast_rescale<uint8>(Rimg),
              TerminalProgressCallback("asp",""));
}

void StereoSessionKeypoint::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {
  //  output_file = input_file;
  output_file = m_out_prefix + "-F-corrected.exr";
  vw_out() << "Processing disparity map to remove the earlier effects of interest point alignment.\n";

  DiskImageView<PixelMask<Vector2f> > disparity_map(input_file);

  // We used a homography to line up the images, we may want
  // to generate pre-alignment disparities before passing this information
  // onto the camera model in the next stage of the stereo pipeline.
  vw::Matrix<double> align_matrix;
  try {
    ::read_matrix(align_matrix, m_out_prefix + "-align.exr");
    std::cout << "Alignment Matrix: " << align_matrix << "\n";
  } catch (vw::IOErr &e) {
    std::cout << "Could not read in aligment matrix: " << m_out_prefix << "-align.exr.  Exiting. \n\n";
    exit(1);
  }

  ImageViewRef<PixelMask<Vector2f> > result = stereo::transform_disparities(disparity_map, HomographyTransform(align_matrix));

  // Remove pixels that are outside the bounds of the secondary image.
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  result = stereo::disparity_range_mask(result, right_disk_image.cols(), right_disk_image.rows());

  write_image(output_file, result,
              TerminalProgressCallback("asp","") );
}
