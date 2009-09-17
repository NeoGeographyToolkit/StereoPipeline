// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file StereoSessionKeypoint.cc
///

#include <boost/shared_ptr.hpp>

#include <iostream>			   // debugging
#include <algorithm>

#include "StereoSettings.h"
#include "StereoSessionKeypoint.h"
#include "stereo.h"

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Math.h>

using namespace vw;
using namespace vw::ip;

// Duplicate matches for any given interest point probably indicate a
// poor match, so we cull those out here.
static void remove_duplicates(std::vector<Vector3> &ip1, std::vector<Vector3> &ip2) {
  std::vector<Vector3> new_ip1, new_ip2;
  
  for (unsigned i = 0; i < ip1.size(); ++i) {
    bool bad_entry = false;
    for (unsigned j = 0; j < ip1.size(); ++j) {
      if (i != j && 
          (ip1[i] == ip1[j] || ip2[i] == ip2[j])) {
        bad_entry = true;
      }
    }
    if (!bad_entry) {
      new_ip1.push_back(ip1[i]);
      new_ip2.push_back(ip2[i]);
    }
  }
  
  ip1 = new_ip1;
  ip2 = new_ip2;
}

std::string StereoSessionKeypoint::create_subsampled_align_image(std::string const& image_file, std::string const& suffix) {
  std::string align_image_file(m_out_prefix + std::string("-normalized-align-sub-") + suffix);

  DiskImageView<PixelGray<float> > disk_image(image_file);
  
  std::cout << "StereoSessionKeypoint::create_subsampled_align_image(): subsampling... \n";
  double sub_sampling = stereo_settings().keypoint_align_subsampling;
  write_image(align_image_file, channel_cast_rescale<uint8>(resample(disk_image, 1.0 / double(sub_sampling))), TerminalProgressCallback());
  
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
  vw_out(0) << "StereoSessionKeypoint::adjust_align_matrix(): scaled alignment matrix:\n";
  vw_out(0) << align_matrix << std::endl;
}

vw::math::Matrix<double>
StereoSessionKeypoint::determine_image_alignment(std::string const& input_file1, std::string const& input_file2) {
  std::string left_align_image_file(input_file1), right_align_image_file(input_file2);

  double sub_sampling = stereo_settings().keypoint_align_subsampling;
  if (sub_sampling > 1) {
    left_align_image_file = create_subsampled_align_image(input_file1, "L.tif");
    right_align_image_file = create_subsampled_align_image(input_file2, "R.tif");
  }

  // Load the two images
  DiskImageView<PixelGray<float> > left_disk_image(left_align_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(right_align_image_file);

  std::cout << "StereoSessionKeypoint::determine_image_alignment(): aligning "
            << left_align_image_file << " and " << right_align_image_file << std::endl;

  // Image Alignment
  //
  // Images are aligned by computing interest points, matching
  // them using a standard 2-Norm nearest-neighor metric, and then
  // rejecting outliers by fitting a similarity between the
  // putative matches using RANSAC.  

  // Interest points are matched in image chunk of <= 2048x2048
  // pixels to conserve memory.
  vw_out(InfoMessage) << "\nInterest Point Detection:\n";

  // Interest Point module detector code.
  ScaledInterestPointDetector<LogInterestOperator> detector;
  InterestPointList ip1 = detect_interest_points(left_disk_image, detector);
  InterestPointList ip2 = detect_interest_points(right_disk_image, detector);
  vw_out(InfoMessage) << "Left image: " << ip1.size() << " points.  Right image: " << ip2.size() << "\n"; 

  // Generate descriptors for interest points.
  // TODO: Switch to a more sophisticated descriptor
  vw_out(InfoMessage) << "\tGenerating descriptors... " << std::flush;
  PatchDescriptorGenerator descriptor;
  descriptor(left_disk_image, ip1);
  descriptor(right_disk_image, ip2);
  vw_out(InfoMessage) << "done.\n";
 
  // The basic interest point matcher does not impose any
  // constraints on the matched interest points.
  vw_out(InfoMessage) << "\nInterest Point Matching:\n";
  double matcher_threshold = 0.8;

  // RANSAC needs the matches as a vector, and so does the matcher.
  // this is messy, but for now we simply make a copy.
  std::vector<InterestPoint> ip1_copy(ip1.size()), ip2_copy(ip2.size());
  std::copy(ip1.begin(), ip1.end(), ip1_copy.begin());
  std::copy(ip2.begin(), ip2.end(), ip2_copy.begin());

  InterestPointMatcher<L2NormMetric,NullConstraint> matcher(matcher_threshold);
  std::vector<InterestPoint> matched_ip1, matched_ip2;
  matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2, false, TerminalProgressCallback());
  vw_out(InfoMessage) << "Found " << matched_ip1.size() << " putative matches.\n";
       
  std::vector<Vector3> ransac_ip1(matched_ip1.size());
  std::vector<Vector3> ransac_ip2(matched_ip2.size());
  for (unsigned i = 0; i < matched_ip1.size();++i ) {
    ransac_ip1[i] = Vector3(matched_ip1[i].x, matched_ip1[i].y,1);
    ransac_ip2[i] = Vector3(matched_ip2[i].x, matched_ip2[i].y,1);
  }
  
  remove_duplicates(ransac_ip1, ransac_ip2);

  // RANSAC is used to fit a similarity transform between the
  // matched sets of points  
  vw::math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> ransac( vw::math::HomographyFittingFunctor(),
													  vw::math::InterestPointErrorMetric(), 
													  10 ); // inlier_threshold

  std::vector<Vector3> result_ip1;
  std::vector<Vector3> result_ip2;
  vw_out(InfoMessage) << "\nRunning RANSAC:\n";
  Matrix<double> align_matrix(ransac(ransac_ip2,ransac_ip1));

  if (sub_sampling > 1)
    scale_align_matrix(align_matrix);

  return align_matrix;
}

void StereoSessionKeypoint::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                   std::string & output_file1, std::string & output_file2) {

  // Determine the alignment matrix using keypoint matching techniques.
  Matrix<double> align_matrix = determine_image_alignment(m_left_image_file, m_right_image_file);
  ::write_matrix(m_out_prefix + "-align.exr", align_matrix);

  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  ImageViewRef<PixelGray<float> > Limg = left_disk_image;
  ImageViewRef<PixelGray<float> > Rimg = transform(right_disk_image, HomographyTransform(align_matrix),
                                                   left_disk_image.cols(), left_disk_image.rows());

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  write_image(output_file1, channel_cast_rescale<uint8>(Limg), TerminalProgressCallback());
  write_image(output_file2, channel_cast_rescale<uint8>(Rimg), TerminalProgressCallback()); 
}

void StereoSessionKeypoint::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {
  //  output_file = input_file;
  output_file = m_out_prefix + "-F-corrected.exr";
  vw_out(0) << "Processing disparity map to remove the earlier effects of interest point alignment.\n";
  
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

  write_image(output_file, result, TerminalProgressCallback() );
}
