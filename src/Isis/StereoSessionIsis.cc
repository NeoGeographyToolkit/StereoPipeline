#include <boost/shared_ptr.hpp>

#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
#include "Isis/StereoSessionIsis.h"
#include "Isis/IsisCameraModel.h"

// Isis Headers
#include <Cube.h>
#include <Camera.h>
#include <CameraDetectorMap.h>
#include <SpecialPixel.h>

// Boost
#include "boost/filesystem.hpp"   
using namespace boost::filesystem; 

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
using namespace vw::ip;


template <class ViewT>
class IsisMinMaxChannelAccumulator {
  typedef typename ViewT::pixel_type pixel_type;
  typedef typename CompoundChannelType<typename ViewT::pixel_type>::type channel_type;
  int num_channels;
  channel_type minval, maxval;
  bool valid;
public:
  IsisMinMaxChannelAccumulator()
    : num_channels( PixelNumChannels<pixel_type>::value - (PixelHasAlpha<pixel_type>::value ? 1 : 0) ), valid(false) {}

  IsisMinMaxChannelAccumulator(channel_type no_data_value)
    : num_channels( PixelNumChannels<pixel_type>::value - (PixelHasAlpha<pixel_type>::value ? 1 : 0) ), valid(false) {}
  
  void operator()( pixel_type const& pix ) {
    if (!valid && !Isis::IsSpecial(compound_select_channel<channel_type>(pix,0)) ) {
      minval = maxval = compound_select_channel<channel_type>(pix,0);
      valid = true;
    }
    for (int channel = 0; channel < num_channels; channel++) {
      channel_type channel_value = compound_select_channel<channel_type>(pix,channel);
      if( Isis::IsSpecial(channel_value) ) continue;
      if( channel_value < minval ) minval = channel_value;
      if( channel_value > maxval ) maxval = channel_value;
    }
  }

  channel_type minimum() const { 
    VW_ASSERT(valid, ArgumentErr() << "MinMaxChannelAccumulator: no valid pixels");
    return minval; 
  }

  channel_type maximum() const {
    VW_ASSERT(valid, ArgumentErr() << "MinMaxChannelAccumulator: no valid pixels");
    return maxval;
  }
};

template <class ViewT>
void isis_min_max_channel_values( const ImageViewBase<ViewT> &view, 
                                  typename CompoundChannelType<typename ViewT::pixel_type>::type &min, 
                                  typename CompoundChannelType<typename ViewT::pixel_type>::type &max )
{
  IsisMinMaxChannelAccumulator<ViewT> accumulator;
  for_each_pixel( view, accumulator );
  min = accumulator.minimum();
  max = accumulator.maximum();
}

//  IsisSpecialPixelFunc
//
/// Replace ISIS missing data values with a pixel value of your
/// choice.

template <class PixelT>
class IsisSpecialPixelFunc: public vw::UnaryReturnSameType {
  PixelT m_replacement_value;
public:
  IsisSpecialPixelFunc(PixelT const& pix) : m_replacement_value(pix) {}
  
  PixelT operator() (PixelT const& pix) const {
    typedef typename CompoundChannelType<PixelT>::type channel_type;
    for (int n = 0; n < CompoundNumChannels<PixelT>::value; ++n) {
      // Check to see if this is an Isis special value.  If it is,
      // return 0 for now.
      if (Isis::IsSpecial(compound_select_channel<const channel_type&>(pix,n))) 
        return m_replacement_value;
    }
    return pix;
  }
};
    
template <class ViewT>
UnaryPerPixelView<ViewT, IsisSpecialPixelFunc<typename ViewT::pixel_type> > 
remove_isis_special_pixels(ImageViewBase<ViewT> &image, typename ViewT::pixel_type replacement_value = typename ViewT::pixel_type() ) {
  return per_pixel_filter(image.impl(), IsisSpecialPixelFunc<typename ViewT::pixel_type>(replacement_value));
}



static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

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

vw::math::Matrix<double> StereoSessionIsis::determine_image_alignment(std::string const& input_file1, std::string const& input_file2, float lo, float hi) {
  std::string left_align_image_file(input_file1), right_align_image_file(input_file2);

  // Load the two images
  DiskImageView<PixelGray<float> > left_disk_image(input_file1);
  DiskImageView<PixelGray<float> > right_disk_image(input_file2);

  // Image Alignment
  //
  // Images are aligned by computing interest points, matching
  // them using a standard 2-Norm nearest-neighor metric, and then
  // rejecting outliers by fitting a similarity between the
  // putative matches using RANSAC.  

  // Interest points are matched in image chunk of <= 2048x2048
  // pixels to conserve memory.
  vw_out(InfoMessage) << "\nInterest Point Detection:\n";
  InterestPointList ip1, ip2;
  
  // If the interest points are cached in a file, read that file in here.
  if ( boost::filesystem::exists( prefix_from_filename(input_file1) + ".vwip" ) && 
       boost::filesystem::exists( prefix_from_filename(input_file2) + ".vwip" )) {
    std::cout << "Found cached interest point files: \n";
    std::cout << "\t" << (prefix_from_filename(input_file1) + ".vwip") << "\n";
    std::cout << "\t" << (prefix_from_filename(input_file2) + ".vwip") << "\n";
    std::cout << "Skipping interest point detection step.\n";
    
  } else {

    ImageViewRef<PixelGray<float> > left_image = normalize(remove_isis_special_pixels(left_disk_image, lo), lo, hi, 0, 1.0);
    ImageViewRef<PixelGray<float> > right_image = normalize(remove_isis_special_pixels(right_disk_image, lo), lo, hi, 0, 1.0);
    
    // Interest Point module detector code.
    LogInterestOperator log_detector(0.1);
    ScaledInterestPointDetector<LogInterestOperator> detector(log_detector, 100);
    //    ScaledInterestPointDetector<LogInterestOperator> detector;
    std::cout << "Processing " << input_file1 << "\n";
    ip1 = detect_interest_points(left_image, detector);
    std::cout << "\nProcessing " << input_file2 << "\n";
    ip2 = detect_interest_points(right_image, detector);
    vw_out(InfoMessage) << "\nLeft image: " << ip1.size() << " points.  Right image: " << ip2.size() << "\n"; 

    // Generate descriptors for interest points.
    // TODO: Switch to a more sophisticated descriptor
    vw_out(InfoMessage) << "\tGenerating descriptors... " << std::flush;
    PatchDescriptorGenerator descriptor;
    descriptor(left_image, ip1);
    descriptor(right_image, ip2);
    vw_out(InfoMessage) << "done.\n";
  
    // Write out the results
    vw_out(InfoMessage) << "\tWriting interest points to disk... " << std::flush;
    write_binary_ip_file(prefix_from_filename(input_file1)+".vwip", ip1);
    write_binary_ip_file(prefix_from_filename(input_file2)+".vwip", ip2);
    vw_out(InfoMessage) << "done.\n";
  }

  // The basic interest point matcher does not impose any
  // constraints on the matched interest points.
  vw_out(InfoMessage) << "\nInterest Point Matching:\n";
  double matcher_threshold = 0.8;

  // RANSAC needs the matches as a vector, and so does the matcher.
  // this is messy, but for now we simply make a copy.
  std::vector<InterestPoint> ip1_copy, ip2_copy;
  ip1_copy = read_binary_ip_file(prefix_from_filename(input_file1)+".vwip");
  ip2_copy = read_binary_ip_file(prefix_from_filename(input_file2)+".vwip");

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
  RandomSampleConsensus<math::HomographyFittingFunctor, InterestPointErrorMetric> ransac( vw::math::HomographyFittingFunctor(),
                                                                                          InterestPointErrorMetric(), 
                                                                                          10 ); // inlier_threshold

  std::vector<Vector3> result_ip1;
  std::vector<Vector3> result_ip2;
  vw_out(InfoMessage) << "\nRunning RANSAC:\n";
  Matrix<double> T;
  try {
    T = ransac(ransac_ip2,ransac_ip1);
  } catch (vw::ip::RANSACErr &e) {
    std::cout << "\nWARNING: Automatic Alignment Failed!  Proceed with caution...\n\n";
    T.set_size(3,3);
    T.set_identity();
  }
  return T;
}

// Pre-align the ISIS images.  If the ISIS images are map projected,
// we can perform pre-alignment by transforming them both into a
// common map projection.  Otherwise, we resort to feature-based image
// matching techniques to align the right image to the left image.
void StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                               std::string & output_file1, std::string & output_file2) {

  DiskImageView<PixelGray<float> > left_disk_image(input_file1);
  DiskImageView<PixelGray<float> > right_disk_image(input_file2);
  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  GeoReference input_georef1, input_georef2;
  try {
    // Read georeferencing information (if it exists...)
    DiskImageResourceGDAL file_resource1( input_file1 );
    DiskImageResourceGDAL file_resource2( input_file2 );
    read_georeference( input_georef1, file_resource1 );
    read_georeference( input_georef2, file_resource2 );
  } catch (ArgumentErr &e) {
    std::cout << "Warning: Couldn't read georeference data from input images using the GDAL driver.\n";
  }

  // Make sure the images are normalized
  vw_out(InfoMessage) << "Computing min/max values for normalization.  " << std::flush;
  float left_lo, left_hi, right_lo, right_hi;
  isis_min_max_channel_values(left_disk_image, left_lo, left_hi);
  vw_out(InfoMessage) << "Left: [" << left_lo << " " << left_hi << "]    " << std::flush;
  isis_min_max_channel_values(right_disk_image, right_lo, right_hi);
  vw_out(InfoMessage) << "Right: [" << right_lo << " " << right_hi << "]\n\n";
  float lo = std::min (left_lo, right_lo);
  float hi = std::min (left_hi, right_hi);

  // If this is a map projected cube, we skip the step of aligning the
  // images, because the map projected images are probable very nearly
  // aligned already.  For unprojected cubes, we align in the "usual"
  // way using interest points.
  if (0) {
//   if (input_georef1.transform() != math::identity_matrix<3>() && 
//       input_georef2.transform() != math::identity_matrix<3>() ) {
    std::cout << "\nMap projected ISIS cubes detected.  Placing both images into the same map projection.\n\n";
    
    // If we are using map-projected cubes, we need to put them into a
    // common projection.  We adopt the projection of the first image.
    GeoReference common_georef = input_georef1;
    BBox2i common_bbox(0,0,left_disk_image.cols(), left_disk_image.rows());

    // Create the geotransform objects and determine the common size
    // of the output images and apply it to the right image.
    GeoTransform trans2(input_georef2, common_georef);
    ImageViewRef<PixelGray<float> > Limg = normalize(remove_isis_special_pixels(left_disk_image, lo), lo, hi, 0, 1.0);
    ImageViewRef<PixelGray<float> > Rimg = crop(transform(normalize(remove_isis_special_pixels(right_disk_image, lo),lo,hi,0.0,1.0),trans2),common_bbox);

    // Write the results to disk.
    write_image(output_file1, channel_cast_rescale<uint8>(Limg), TerminalProgressCallback());
    write_image(output_file2, channel_cast_rescale<uint8>(Rimg), TerminalProgressCallback()); 
  
  } else {
    // For unprojected ISIS images, we resort to the "old style" of
    // image alignment: determine the alignment matrix using keypoint
    // matching techniques.
    std::cout << "\nUnprojected ISIS cubes detected.  Aligning images using feature-based matching techniques.\n\n";

    Matrix<double> align_matrix(3,3);
    align_matrix.set_identity();
    //    align_matrix = determine_image_alignment(input_file1, input_file2, lo, hi);
    ::write_matrix(m_out_prefix + "-align.exr", align_matrix);

    // Apply the alignment transformation to the right image.
    ImageViewRef<PixelGray<float> > Limg = normalize(remove_isis_special_pixels(left_disk_image, lo),lo,hi,0.0,1.0);
    ImageViewRef<PixelGray<float> > Rimg = transform(normalize(remove_isis_special_pixels(right_disk_image,lo),lo,hi,0.0,1.0), 
                                                     HomographyTransform(align_matrix),
                                                     left_disk_image.cols(), left_disk_image.rows());

    // Write the results to disk.
    write_image(output_file1, channel_cast_rescale<uint8>(Limg), TerminalProgressCallback());
    write_image(output_file2, channel_cast_rescale<uint8>(Rimg), TerminalProgressCallback()); 
  }
}

// Reverse any pre-alignment that was done to the images.
void StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {

  DiskImageView<PixelDisparity<float> > disparity_map(input_file);
  output_file = m_out_prefix + "-F-corrected.exr";
  ImageViewRef<PixelDisparity<float> > result;

  // Read georeferencing information (if it exists...)
  GeoReference input_georef1, input_georef2;
  try {
    DiskImageResourceGDAL file_resource1( m_left_image_file );
    DiskImageResourceGDAL file_resource2( m_right_image_file );
    read_georeference( input_georef1, file_resource1 );
    read_georeference( input_georef2, file_resource2 );
  } catch (ArgumentErr &e) {
    std::cout << "Warning: Couldn't read georeference data from input images using the GDAL driver.\n";
  }
  
  // If this is a map projected cube, we skip the step of aligning the
  // images, because the map projected images are probable very nearly
  // aligned already.  For unprojected cubes, we align in the "usual"
  // way using interest points.
  if (input_georef1.transform() != math::identity_matrix<3>() && 
      input_georef2.transform() != math::identity_matrix<3>() ) {
    vw_out(0) << "\nMap projected ISIS cubes detected.  Placing both images into the same map projection.\n\n";

    // If we are using map-projected cubes, we need to put them into a
    // common projection.  We adopt the projection of the first image.
    result = stereo::disparity::transform_disparities(disparity_map, GeoTransform(input_georef2, input_georef1));

  } else {

    vw_out(0) << "Unprojected ISIS cubes detected.  Processing disparity map to remove the earlier effects of interest point alignment.\n";  

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
    
    result = stereo::disparity::transform_disparities(disparity_map, HomographyTransform(align_matrix));

    // Remove pixels that are outside the bounds of the secondary image.
    DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
    result = stereo::disparity::remove_invalid_pixels(result, right_disk_image.cols(), right_disk_image.rows());
  }
  
  write_image(output_file, result, TerminalProgressCallback() );
}


boost::shared_ptr<vw::camera::CameraModel> StereoSessionIsis::camera_model(std::string image_file, 
                                                                           std::string camera_file) {
  return boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(image_file));
}

