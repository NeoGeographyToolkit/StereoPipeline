#include <boost/shared_ptr.hpp>

#include "StereoSessionKeypoint.h"

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/InterestPoint.h>

using namespace vw;
using namespace vw::ip;

#include "file_lib.h"

using namespace vw;

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
  
  HarrisInterest<float> harris;
  LoGInterest<float> log;
  InterestThreshold<float> thresholder(0.0);
  ScaledInterestPointDetector<float> detector(&log, &thresholder);

  // Interest points are matched in image chunk of <= 2048x2048
  // pixels to conserve memory.
  std::cout << "\nInterest Point Detection:\n";
  static const int MAX_KEYPOINT_IMAGE_DIMENSION = 2048;
  std::vector<InterestPoint> ip1 = interest_points(channels_to_planes(left_disk_image), detector, MAX_KEYPOINT_IMAGE_DIMENSION);
  std::vector<InterestPoint> ip2 = interest_points(channels_to_planes(right_disk_image), detector, MAX_KEYPOINT_IMAGE_DIMENSION);
    
  // The basic interest point matcher does not impose any
  // constraints on the matched interest points.
  std::cout << "\nInterest Point Matching:\n";
  InterestPointMatcher<L2NormMetric,NullConstraint> matcher;
  std::vector<InterestPoint> matched_ip1, matched_ip2;
  matcher.match(ip1, ip2, matched_ip1, matched_ip2);
  std::cout << "Found " << matched_ip1.size() << " putative matches.\n";
  
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

  write_image(output_file1, channel_cast_rescale<uint8>(Limg));
  write_image(output_file2, channel_cast_rescale<uint8>(Rimg)); 
}
