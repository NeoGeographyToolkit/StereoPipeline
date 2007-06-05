#include <boost/shared_ptr.hpp>

#include "StereoSessionKeypoint.h"

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>

using namespace vw;
using namespace vw::ip;

#include "file_lib.h"
#include "SIFT.h"

using namespace vw;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
}

void StereoSessionKeypoint::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                   std::string & output_file1, std::string & output_file2) {

  // Load the two images
  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);

  // Image Alignment
  //
  // Images are aligned by computing interest points, matching
  // them using a standard 2-Norm nearest-neighor metric, and then
  // rejecting outliers by fitting a similarity between the
  // putative matches using RANSAC.  

  // Interest points are matched in image chunk of <= 2048x2048
  // pixels to conserve memory.
  vw_out(InfoMessage) << "\nInterest Point Detection:\n";
  static const int MAX_KEYPOINT_IMAGE_DIMENSION = 2048;

  // Interest Point module detector code.
  ScaledInterestPointDetector<LoGInterest> detector;
  ImageView<float> left = channels_to_planes(left_disk_image);
  ImageView<float> right = channels_to_planes(right_disk_image);
  KeypointList ip1 = interest_points(left, detector, MAX_KEYPOINT_IMAGE_DIMENSION);
  KeypointList ip2 = interest_points(right, detector, MAX_KEYPOINT_IMAGE_DIMENSION);

  // Old SIFT detector code.  Comment out the lines above and
  // uncomment these lines to enable. -mbroxton
  //   KeypointList ip1 = interest_points(channels_to_planes(left_disk_image), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
  //   KeypointList ip2 = interest_points(channels_to_planes(right_disk_image), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);

  // Discard points beyond some number to keep matching time within reason.
  // Currently this is limited by the use of the patch descriptor.
  static const int NUM_POINTS = 1000;
  vw_out(InfoMessage) << "\tTruncating to " << NUM_POINTS << " points\n";
  cull_interest_points(ip1, NUM_POINTS);
  cull_interest_points(ip2, NUM_POINTS);

  // Generate descriptors for interest points.
  // TODO: Switch to SIFT descriptor
  vw_out(InfoMessage) << "\tGenerating descriptors... ";
  compute_descriptors(left_disk_image, ip1, PatchDescriptor() );
  compute_descriptors(right_disk_image, ip2, PatchDescriptor() );
  vw_out(InfoMessage) << "done.\n";
    
  // The basic interest point matcher does not impose any
  // constraints on the matched interest points.
  vw_out(InfoMessage) << "\nInterest Point Matching:\n";
  double matcher_threshold = 0.8;
  InterestPointMatcher<L2NormMetric,NullConstraint> matcher(matcher_threshold);
  std::vector<InterestPoint> matched_ip1, matched_ip2;
  matcher.match(ip1, ip2, matched_ip1, matched_ip2);
  vw_out(InfoMessage) << "Found " << matched_ip1.size() << " putative matches.\n";
  
  // RANSAC is used to fit a similarity transform between the
  // matched sets of points
  Matrix<double> align_matrix = ransac(matched_ip2, matched_ip1, 
                                       vw::math::SimilarityFittingFunctor(),
                                       KeypointErrorMetric());
  write_matrix(m_out_prefix + "-align.exr", align_matrix);

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
  
  DiskImageView<PixelDisparity<float> > disparity_map(input_file);

  // We used a homography to line up the images, we may want 
  // to generate pre-alignment disparities before passing this information
  // onto the camera model in the next stage of the stereo pipeline.
  vw::Matrix<double> align_matrix;
  try {
    read_matrix(align_matrix, m_out_prefix + "-align.exr");
    std::cout << "Alignment Matrix: " << align_matrix << "\n";
  } catch (vw::IOErr &e) {
    std::cout << "Could not read in aligment matrix: " << m_out_prefix << "-align.exr.  Exiting. \n\n";
    exit(1);
  }
  
  vw::Matrix<double> inv_align_matrix = inverse(align_matrix);
  ImageViewRef<PixelDisparity<float> > result = stereo::disparity::disparity_linear_transform(disparity_map, inv_align_matrix);

  // Remove pixels that are outside the bounds of the secondary image.
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  result = stereo::disparity::remove_invalid_pixels(result, right_disk_image.cols(), right_disk_image.rows());

  write_image(output_file, result, TerminalProgressCallback() );
}
