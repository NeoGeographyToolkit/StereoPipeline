#include <boost/shared_ptr.hpp>

#include <iostream>			   // debugging
#include <algorithm>

#include "StereoSessionKeypoint.h"
#include "stereo.h"

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Math.h>

using namespace vw;
using namespace vw::ip;

#include "file_lib.h"
#include "SIFT.h"

using namespace vw;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
}

enum { eHistogramSize = 32768 };

struct fill_histogram
{
  fill_histogram(unsigned int *counts) { m_counts = counts; }
  void operator()(float x)
  {
    m_counts[int(float(eHistogramSize - 1) * x)] += 1;
  }
  private:
    unsigned int *m_counts;
};


void
StereoSessionKeypoint::initialize(DFT_F& stereo_defaults) {
  std::cout << "StereoSessionKeypoint::initialize(DFT_F &): setting image sub-sampling factor to "
	    << stereo_defaults.keypoint_align_subsampling << std::endl;
  set_sub_sampling(stereo_defaults.keypoint_align_subsampling);
}


float
StereoSessionKeypoint::calculate_stretch(ImageView<PixelGray<float> > image)
{
  // For some reason it seems that we often have a bunch of random
  // pixels at the high end which seems to be noise... we calculate a
  // scale factor so valid pixel values fill the range

  unsigned int histogram[eHistogramSize];
  std::fill(histogram, histogram + eHistogramSize, 0);

  std::for_each(image.begin(), image.end(), fill_histogram(histogram));

  // find the peak non-zero value
  unsigned int max_count = 0;
  unsigned index_peak = 0;
  // We start the following at 1 since there typically are a bunch of
  // black pixels, and that kind of skews things
  for (int i = 1; i < eHistogramSize; i++)
    if (histogram[i] > max_count)
    {
      index_peak = i;
      max_count = histogram[i];
    }

  // discard the top 0.0005%
  unsigned int empty_threshold = int(float(max_count) * 0.000005);
  unsigned int max_value_index = 0;

  for (int i = eHistogramSize - 1; i > 0; --i)
  {
    if (histogram[i] > empty_threshold)
    {
      max_value_index = i;
      break;
    }
  }

  float max_value = float(max_value_index) / float(eHistogramSize - 1);
  float scale_factor = 1.0/max_value;

  // -- begin debugging
  std::cout << "\nIndex of peak = " << index_peak << std::endl;
  std::cout << "Count at peak = " << max_count << std::endl;
  std::cout << "Empty bin threshold = " << empty_threshold << std::endl;
  std::cout << "Max pixel value = " << max_value << std::endl;
  std::cout << "Scale factor = " << scale_factor << std::endl;
  // -- end debugging

  return scale_factor;
}


std::string
StereoSessionKeypoint::create_subsampled_align_image(std::string const& image_file, std::string const& suffix) {
  std::string align_image_file(m_out_prefix +
			       std::string("-normalized-align-sub-") + suffix);

  DiskImageView<PixelGray<float> > disk_image(image_file);

  std::cout << "StereoSessionKeypoint::create_subsampled_align_image(): "
	    << "subsampling and calculating contrast stretch... "
	    << std::flush;
  ImageViewRef<PixelGray<float> > subsampled_image =
    clamp(resample(disk_image, 1.0 / double(m_sub_sampling)), 0.0, 1.0);
  double contrast_stretch = calculate_stretch(subsampled_image);
  std::cout << "done." << std::endl;

  write_image(align_image_file,
	      channel_cast_rescale<uint8>(clamp(contrast_stretch *
						subsampled_image, 0.0, 1.0)));

  return align_image_file;
}

void
StereoSessionKeypoint::scale_align_matrix(Matrix<double> & align_matrix) {
  Matrix<double> scale_matrix = vw::math::identity_matrix(3);
  Matrix<double> inv_scale_matrix = vw::math::identity_matrix(3);
  scale_matrix(0, 0) = 1.0 / double(m_sub_sampling);
  scale_matrix(1, 1) = scale_matrix(0, 0);
  inv_scale_matrix(0, 0) = double(m_sub_sampling);
  inv_scale_matrix(1, 1) = inv_scale_matrix(0, 0);
  align_matrix = align_matrix * scale_matrix;
  align_matrix = inv_scale_matrix * align_matrix;
  vw_out(0) << "StereoSessionKeypoint::adjust_align_matrix(): scaled alignment matrix:\n";
  vw_out(0) << align_matrix << std::endl;
}

vw::math::Matrix<double>
StereoSessionKeypoint::determine_image_alignment(std::string const& input_file1, std::string const& input_file2) {
  std::string left_align_image_file(input_file1), right_align_image_file(input_file2);

  if (m_sub_sampling > 1)
  {
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
  static const int MAX_KEYPOINT_IMAGE_DIMENSION = 2048;

  // Interest Point module detector code.
  //
  // For some reason the interest point detector crashes sometimes
  // under linux so I've reverted to lowe style interest points for
  // now on that platform.
#ifdef __APPLE__
  ScaledInterestPointDetector<LoGInterest> detector;
  KeypointList ip1 = interest_points(channels_to_planes(left_disk_image), detector, MAX_KEYPOINT_IMAGE_DIMENSION);
  KeypointList ip2 = interest_points(channels_to_planes(right_disk_image), detector, MAX_KEYPOINT_IMAGE_DIMENSION);
#else
  // Old SIFT detector code.  Comment out the lines above and
  // uncomment these lines to enable. -mbroxton
  KeypointList ip1 = interest_points(channels_to_planes(left_disk_image), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
  KeypointList ip2 = interest_points(channels_to_planes(right_disk_image), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
#endif 

  // Discard points beyond some number to keep matching time within reason.
  // Currently this is limited by the use of the patch descriptor.
#ifdef __APPLE__			   // don't truncate if using Lowe-SIFT
  static const int NUM_POINTS = 1000;
  vw_out(InfoMessage) << "\tTruncating to " << NUM_POINTS << " points\n";
  cull_interest_points(ip1, NUM_POINTS);
  cull_interest_points(ip2, NUM_POINTS);
#endif

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

  if (m_sub_sampling > 1)
    scale_align_matrix(align_matrix);

  return align_matrix;
}

void StereoSessionKeypoint::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                   std::string & output_file1, std::string & output_file2) {
  Matrix<double> align_matrix = determine_image_alignment(m_left_image_file, m_right_image_file);

  write_matrix(m_out_prefix + "-align.exr", align_matrix);

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
