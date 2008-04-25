#include <boost/shared_ptr.hpp>

#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
#include "Isis/StereoSessionIsis.h"
#include "Isis/IsisCameraModel.h"
#include "Isis/DiskImageResourceIsis.h"

// Isis Headers
#include <Cube.h>
#include <Camera.h>
#include <CameraDetectorMap.h>


using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
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

vw::math::Matrix<double> StereoSessionIsis::determine_image_alignment(std::string const& input_file1, std::string const& input_file2) {
  std::string left_align_image_file(input_file1), right_align_image_file(input_file2);

  // Load the two images
  DiskImageView<PixelGray<float> > left_disk_image(input_file1);
  DiskImageView<PixelGray<float> > right_disk_image(input_file2);

  vw_out(InfoMessage) << "Computing min/max values.\n";
  ImageViewRef<PixelGray<float> > left_image = normalize(remove_isis_special_pixels(left_disk_image));
  ImageViewRef<PixelGray<float> > right_image = normalize(remove_isis_special_pixels(right_disk_image));

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
  InterestPointList ip1 = detect_interest_points(left_image, detector);
  InterestPointList ip2 = detect_interest_points(right_image, detector);
  vw_out(InfoMessage) << "Left image: " << ip1.size() << " points.  Right image: " << ip2.size() << "\n"; 

  // Generate descriptors for interest points.
  // TODO: Switch to a more sophisticated descriptor
  vw_out(InfoMessage) << "\tGenerating descriptors... " << std::flush;
  PatchDescriptorGenerator descriptor;
  descriptor(left_image, ip1);
  descriptor(right_image, ip2);
  vw_out(InfoMessage) << "done.\n";

  // Write out the results
  vw_out(InfoMessage) << "\tWriting interest points to disk... " << std::flush;
  write_binary_ip_file(input_file1+".vwip", ip1);
  write_binary_ip_file(input_file2+".vwip", ip2);
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

void StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                               std::string & output_file1, std::string & output_file2) {

  // Determine the alignment matrix using keypoint matching techniques.
  Matrix<double> align_matrix(3,3);
  align_matrix = determine_image_alignment(m_left_image_file, m_right_image_file);
  ::write_matrix(m_out_prefix + "-align.exr", align_matrix);

  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  ImageViewRef<PixelGray<float> > Limg = remove_isis_special_pixels(left_disk_image);
  ImageViewRef<PixelGray<float> > Rimg = transform(remove_isis_special_pixels(right_disk_image), HomographyTransform(align_matrix),
                                                   left_disk_image.cols(), left_disk_image.rows());

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  write_image(output_file1, channel_cast_rescale<uint8>(normalize(Limg)), TerminalProgressCallback());
  write_image(output_file2, channel_cast_rescale<uint8>(normalize(Rimg)), TerminalProgressCallback()); 
}

void StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {
  //  output_file = input_file;
  output_file = m_out_prefix + "-F-corrected.exr";
  vw_out(0) << "Processing disparity map to remove the earlier effects of interest point alignment.\n";
  
  DiskImageView<PixelDisparity<float> > disparity_map(input_file);

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
  
  vw::Matrix<double> inv_align_matrix = inverse(align_matrix);
  ImageViewRef<PixelDisparity<float> > result = stereo::disparity::disparity_linear_transform(disparity_map, inv_align_matrix);

  // Remove pixels that are outside the bounds of the secondary image.
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  result = stereo::disparity::remove_invalid_pixels(result, right_disk_image.cols(), right_disk_image.rows());

  write_image(output_file, result, TerminalProgressCallback() );
}


// void StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
//                                                    std::string & output_file1, std::string & output_file2) {

//   DiskImageView<PixelGray<uint8> > left_disk_image(m_left_image_file);
//   DiskImageView<PixelGray<uint8> > right_disk_image(m_right_image_file);

//   output_file1 = m_out_prefix + "-L.tif";
//   output_file2 = m_out_prefix + "-R.tif";

//   boost::shared_ptr<camera::CameraModel> cam1, cam2;
//   this->camera_models(cam1, cam2);

//   const double MOLA_PEDR_EQUATORIAL_RADIUS =  3396000.0;
//   vw::cartography::Datum mars_datum;
//   mars_datum.name() = "IAU2000 Mars Spheroid";
//   mars_datum.spheroid_name() = "IAU2000 Mars Spheroid";
//   mars_datum.meridian_name() = "Mars Prime Meridian";
//   mars_datum.set_semi_major_axis(MOLA_PEDR_EQUATORIAL_RADIUS);
//   mars_datum.set_semi_minor_axis(MOLA_PEDR_EQUATORIAL_RADIUS);
//   GeoReference terrain_ref(mars_datum);

// //   BBox2 bbox1 = camera_bbox(terrain_ref, cam1,left_disk_image.cols(), left_disk_image.rows());
// //   std::cout << "Bounding Box 1: " << bbox1 << "\n";
// //   BBox2 bbox2 = camera_bbox(terrain_ref, cam2,right_disk_image.cols(), right_disk_image.rows());
// //   std::cout << "Bounding Box 2: " << bbox2 << "\n";

// //   BBox2 joint_bbox = bbox1;
// //   joint_bbox.grow(bbox2);
// //   std::cout << "Combined Bounding Box: " << joint_bbox << "\n";
  
// //   Matrix3x3 affine;
// //   affine.set_identity();
// //   affine(0,0) = joint_bbox.width() / left_disk_image.cols()*2;
// //   affine(1,1) = joint_bbox.width() / left_disk_image.cols()*2;
// //   affine(0,2) = joint_bbox.min().x();
// //   affine(1,2) = joint_bbox.min().y();
// //   terrain_ref.set_transform(affine);
// //   std::cout << "AFFINE: " << affine << "\n";
  
// //   ConstantView<float> terrain(0, joint_bbox.width()/affine(0,0), joint_bbox.height()/affine(1,1));
  
// //   ImageViewRef<PixelGray<uint8> > ortho1 = orthoproject(terrain ,terrain_ref,
// //                                                         left_disk_image,cam1,
// //                                                         BilinearInterpolation(),
// //                                                         ZeroEdgeExtension());
                                                        
// //   ImageViewRef<PixelGray<uint8> > ortho2 = orthoproject(terrain ,terrain_ref,
// //                                                         right_disk_image,cam2,
// //                                                         BilinearInterpolation(),
// //                                                         ZeroEdgeExtension());
                                                        
// //   write_image(output_file1, ortho1, TerminalProgressCallback());
// //   write_image(output_file2, ortho2, TerminalProgressCallback());

//   ImageViewRef<PixelGray<uint8> > reproj2 = crop(transform(right_disk_image, 
//                                                            CamToCamTransform(cam2, cam1, terrain_ref), ZeroEdgeExtension()),
//                                                  0,0,left_disk_image.cols(),left_disk_image.rows());
//   write_image(output_file1, left_disk_image, TerminalProgressCallback());
//   write_image(output_file2, reproj2, TerminalProgressCallback());

//   exit(0);
// }

// // /// Erases a file suffix if one exists and returns the base string
// // static std::string prefix_from_filename(std::string const& filename) {
// //   std::string result = filename;
// //   int index = result.rfind(".");
// //   if (index != -1) 
// //     result.erase(index, result.size());
// //   return result;
// // }

// /// Cam to cam image transform functor
// //
// // TODO : Add in support for terrain
// class CamToCamTransform : public TransformHelper<CamToCamTransform,ConvexFunction,ConvexFunction> {
//   boost::shared_ptr<CameraModel> m_src_cam;
//   boost::shared_ptr<CameraModel> m_dst_cam;
//   GeoReference m_georef;
  
//   double semi_major_axis;
//   double semi_minor_axis;
//   double z_scale;
//   double radius;
//   double radius_sqr;

// public:    
//   CamToCamTransform( boost::shared_ptr<CameraModel> src_cam,
//                      boost::shared_ptr<CameraModel> dst_cam,
//                      GeoReference georef):
//     m_src_cam(src_cam), m_dst_cam(dst_cam), m_georef(georef) {
//     semi_major_axis = georef.datum().semi_major_axis();
//     semi_minor_axis = georef.datum().semi_minor_axis();
//     z_scale = semi_major_axis / semi_minor_axis;
//     radius = semi_major_axis;
//     radius_sqr = radius*radius;
//   }

//   Vector3 compute_intersection(boost::shared_ptr<CameraModel> camera_model,
//                                Vector2 pix) const {
    
//     Vector3 center = camera_model->camera_center( pix );
//     Vector3 vector = camera_model->pixel_to_vector( pix );
//     center.z() *= z_scale;
//     vector.z() *= z_scale;
//     vector = normalize( vector );

//     double alpha = - dot_prod(center,vector);
//     Vector3 projection = center + alpha * vector;
//     if( norm_2_sqr(projection) > radius_sqr )
//       vw_throw(LogicErr() << "Error in CamToCamTransform: Ray does not intersect geoid.");
//     alpha -= sqrt( radius_sqr - norm_2_sqr(projection) );
//     Vector3 intersection = center + alpha * vector;
//     intersection.z() /= z_scale;
//     return intersection;
//   }

//   inline Vector2 reverse( Vector2 const& p ) const {
//     Vector3 xyz = compute_intersection(m_dst_cam, p);
//     return m_src_cam->point_to_pixel(xyz);
//     }
  
//   inline Vector2 forward( Vector2 const& p ) const {
//     Vector3 xyz = compute_intersection(m_src_cam, p);
//     return m_dst_cam->point_to_pixel(xyz);
//   }
// };

void StereoSessionIsis::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                      boost::shared_ptr<camera::CameraModel> &cam2) {

  cam1 = boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(m_left_image_file));
  cam2 = boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(m_right_image_file));
//   std::cout << "Starting camera model test...\n";
//   Isis::Cube* cube_ptr = new Isis::Cube;
//   cube_ptr->Open(m_left_image_file);
//   if ( !(cube_ptr->IsOpen()) ) 
//       vw_throw(IOErr() << "IsisCameraModel: Could not open cube file: \"" << m_left_image_file << "\".");
//   Isis::Camera* isis_camera_ptr;
//   std::cout << "Accessing camera model\n";
//   isis_camera_ptr = cube_ptr->Camera();

//   unsigned samples = cube_ptr->Samples();
//   unsigned lines = cube_ptr->Lines();

//   for (unsigned int j=0; j < 1000; ++j) {
//     if (j % 50 == 0)
//       std::cout << "Processing Line " << j << "     \r" << std::flush;
//     for (unsigned int i=0; i < samples; ++i) {
//       isis_camera_ptr->SetImage(i,j);
//     }
//   }

//   for (unsigned int j=0; j < 1000; ++j) {
//     if (j % 50 == 0)
//       std::cout << "Processing Line " << j << "     \r" << std::flush;
//     for (unsigned int i=0; i < samples; ++i) {
//       isis_camera_ptr->DetectorMap()->SetParent(i,j);
//     }
//   }

//   for (unsigned int j=0; j < 1000; ++j) {
//     if (j % 50 == 0)
//       std::cout << "Processing Line " << j << "     \r" << std::flush;
//     for (unsigned int i=0; i < samples; ++i) {
//       isis_camera_ptr->SetImage(i,j);
//       double pos1[3];
//       isis_camera_ptr->InstrumentPosition(pos1);

//       isis_camera_ptr->DetectorMap()->SetParent(i,j);
//       double pos2[3];
//       isis_camera_ptr->InstrumentPosition(pos2);

//       if (pos1[0] != pos2[0] ||
//           pos1[1] != pos2[1] ||
//           pos1[2] != pos2[2]) {
//         std::cout << "hmmm...\n.";
//       }

//       if (j == 500 && i == 250)
//         std::cout << pos1[0] << " " << pos1[1] << " " << pos1[2] << "\n";
//     }
//   }


//   exit(0);
}

