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


// --> This file is only included by InterestPointMatching.h


/// Remove points in/out of a bounding box depending on "remove_outside".
/// - Returns the number of points removed.
/// TODO: MOVE THIS FUNCTION!
inline size_t remove_ip_bbox(vw::BBox2i const& roi, vw::ip::InterestPointList & ip_list,
			     bool remove_outside){
  // Loop through all the points
  size_t num_removed = 0;
  vw::ip::InterestPointList::iterator ip;
  for (ip = ip_list.begin(); ip != ip_list.end(); ++ip) {
    
    if (roi.contains(vw::Vector2i(ip->ix,ip->iy)) xor remove_outside) {
      ip = ip_list.erase(ip);
      ++num_removed;
      --ip;
    }
  }
  return num_removed;
} // End function remove_ip_bbox



template <class Image1T>
void detect_ip(vw::ip::InterestPointList& ip,
	       vw::ImageViewBase<Image1T> const& image,
	       int ip_per_tile, std::string const file_path, double nodata) {
  using namespace vw;
  ip.clear();

  // If a valid file_path was provided, just try to read in the IP's from that file.
  if ((file_path != "") && (boost::filesystem::exists(file_path))) {
    vw_out() << "\t    Reading interest points from file: " << file_path << std::endl;
    ip = ip::read_binary_ip_file_list(file_path);
    vw_out() << "\t    Found interest points: " << ip.size() << std::endl;
    return;
  }
  
  Stopwatch sw;
  sw.start();

  // Automatically determine how many ip we need
  const float  tile_size = 1024;
  BBox2i box = bounding_box(image.impl());
  float  number_tiles    = (box.width() / tile_size) * (box.height() / tile_size);
  size_t points_per_tile = 5000.f / number_tiles;
  if (points_per_tile > 5000) points_per_tile = 5000;
  if (points_per_tile < 50  ) points_per_tile = 50;

  // See if to override with manual value
  if (ip_per_tile != 0)
    points_per_tile = ip_per_tile;

  const bool has_nodata = !boost::math::isnan(nodata);

  vw_out() << "\t    Using " << points_per_tile << " interest points per tile (1024^2 px).\n";

  // Load the detection method from stereo_settings.
  // - This relies on a direct match in the enum integer value.
  DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

  // Detect Interest Points
  // - Due to templated types we need to duplicate a bunch of code here
  if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
    // Zack's custom detector
    int num_scales = stereo_settings().num_scales;
    if (num_scales <= 0) 
      num_scales = vw::ip::IntegralInterestPointDetector
        <vw::ip::OBALoGInterestOperator>::IP_DEFAULT_SCALES;
    else
      vw_out() << "\t    Using " << num_scales << " scales in OBALoG interest point detection.\n";

    vw::ip::IntegralAutoGainDetector detector(points_per_tile, num_scales);

    // This detector can't handle a mask so if there is nodata just set those pixels to zero.

    vw_out() << "\t    Detecting IP\n";
    if (!has_nodata)
      ip = detect_interest_points(image.impl(), detector, points_per_tile);
    else
      ip = detect_interest_points(apply_mask(create_mask_less_or_equal(image.impl(),nodata)), detector, points_per_tile);
  } else {

    // Initialize the OpenCV detector.  Conveniently we can just pass in the type argument.
    // - If VW was not build with OpenCV, this call will just throw an exception.
    vw::ip::OpenCvIpDetectorType cv_method = vw::ip::OPENCV_IP_DETECTOR_TYPE_SIFT;
    if (detect_method == DETECT_IP_METHOD_ORB)
      cv_method = vw::ip::OPENCV_IP_DETECTOR_TYPE_ORB;

    // The opencv detector only works if the inputs are normalized, so do it here if it was not done before.
    // - If the images are already normalized most of the data will be in the 0-1 range.
    // - This normalize option will normalize PER-TILE which has different results than
    //   the whole-image normalization that ASP normally uses.
    bool opencv_normalize = stereo_settings().skip_image_normalization;
    if (stereo_settings().ip_normalize_tiles)
      opencv_normalize = true;
    if (opencv_normalize)
      vw_out() << "\t    Using per-tile image normalization for IP detection...\n";

    bool build_opencv_descriptors = true;
    vw::ip::OpenCvInterestPointDetector detector(cv_method, opencv_normalize, build_opencv_descriptors, points_per_tile);

    // These detectors do accept a mask so use one if applicable.

    vw_out() << "\t    Detecting IP\n";
    if (!has_nodata)
      ip = detect_interest_points(image.impl(), detector, points_per_tile);
    else
      ip = detect_interest_points(create_mask_less_or_equal(image.impl(),nodata), detector, points_per_tile);
  } // End OpenCV case

  sw.stop();
  vw_out(DebugMessage,"asp") << "Detect interest points elapsed time: "
			     << sw.elapsed_seconds() << " s." << std::endl;

  if (!boost::math::isnan(nodata)) {
    vw_out() << "\t    Removing IP near nodata with radius "
             << stereo_settings().ip_nodata_radius << std::endl;
    remove_ip_near_nodata(image.impl(), nodata, ip, stereo_settings().ip_nodata_radius);
  }

  // For the two OpenCV options we already built the descriptors, so only do this for the integral method.
  if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
    sw.start();
    vw_out() << "\t    Building descriptors" << std::endl;
    ip::SGradDescriptorGenerator descriptor;
    if (!has_nodata)
      describe_interest_points(image.impl(), descriptor, ip);
    else
      describe_interest_points(apply_mask(create_mask_less_or_equal(image.impl(),nodata)), descriptor, ip);

    vw_out(DebugMessage,"asp") << "Building descriptors elapsed time: "
                               << sw.elapsed_seconds() << " s." << std::endl;
  }

  vw_out() << "\t    Found interest points: " << ip.size() << std::endl;

  // If a file path was provided, record the IP to disk.
  if (file_path != "") {
    vw_out() << "\t    Recording interest points to file: " << file_path << std::endl;
    ip::write_binary_ip_file(file_path, ip);
  }
}

template <class Image1T, class Image2T>
bool detect_ip_pair(vw::ip::InterestPointList& ip1, 
		    vw::ip::InterestPointList& ip2,  
		    vw::ImageViewBase<Image1T> const& image1,
		    vw::ImageViewBase<Image2T> const& image2,
		    int ip_per_tile,
		    std::string const left_file_path,
		    std::string const right_file_path,
		    double nodata1, double nodata2) {
  using namespace vw;

  // Detect Interest Points
  vw_out() << "\t    Looking for IP in left image...\n";
  detect_ip(ip1, image1.impl(), ip_per_tile, left_file_path, nodata1);
  vw_out() << "\t    Looking for IP in right image...\n";
  detect_ip(ip2, image2.impl(), ip_per_tile, right_file_path, nodata2);
  
  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing detected IP debug images. " << std::endl;
    write_ip_debug_image("ASP_IP_detect_debug1.tif", image1, ip1, !boost::math::isnan(nodata1), nodata1);
    write_ip_debug_image("ASP_IP_detect_debug2.tif", image2, ip2, !boost::math::isnan(nodata2), nodata2);
  }
  
  side_ip_filtering(ip1, ip2, bounding_box(image1), bounding_box(image2));
  
  return ((ip1.size() > 0) && (ip2.size() > 0));
}

template <class Image1T, class Image2T>
bool detect_ip_aligned_pair(vw::camera::CameraModel* cam1,
			    vw::camera::CameraModel* cam2,
			    vw::ImageViewBase<Image1T> const& image1,
			    vw::ImageViewBase<Image2T> const& image2,
			    int ip_per_tile,
			    vw::cartography::Datum const& datum,
			    vw::ip::InterestPointList& ip1,
			    vw::ip::InterestPointList& ip2,
			    vw::Matrix<double> &rough_homography,
			    std::string const left_file_path,
			    double nodata1,
			    double nodata2) {

  using namespace vw;

  // No longer supporting input transforms, set them to identity.
  const TransformRef left_tx (TranslateTransform(Vector2(0,0)));
  const TransformRef right_tx(TranslateTransform(Vector2(0,0)));

  BBox2i box1 = bounding_box(image1.impl()),
    box2 = bounding_box(image2.impl());

  try {
    // Homography is defined in the original camera coordinates
    rough_homography =  rough_homography_fit(cam1, cam2, left_tx.reverse_bbox(box1),
					     right_tx.reverse_bbox(box2), datum);
  } catch(...) {
    vw_out() << "Rough homography fit failed, trying with identity transform. " << std::endl;
    rough_homography.set_identity(3);
  }

  // Remove the main translation and solve for BBox that fits the
  // image. If we used the translation from the solved homography with
  // poorly position cameras, the right image might be moved out of frame.
  rough_homography(0,2) = rough_homography(1,2) = 0;
  vw_out() << "Aligning right to left for IP capture using rough homography: " 
	   << rough_homography << std::endl;
  
  { // Check to see if this rough homography works
    HomographyTransform func(rough_homography);
    VW_ASSERT(box1.intersects(func.forward_bbox(box2)),
	      LogicErr() << "The rough homography alignment based on datum and camera geometry shows that input images do not overlap at all. Unable to proceed. Examine your images, or consider using the option --skip-rough-homography.\n");
  }

  TransformRef tx(compose(right_tx, HomographyTransform(rough_homography)));
  BBox2i raster_box = tx.forward_bbox(right_tx.reverse_bbox(box2));
  tx = TransformRef(compose(TranslateTransform(-raster_box.min()),
                            right_tx, HomographyTransform(rough_homography)));
  raster_box -= Vector2i(raster_box.min());
  
  // Detect interest points for the left and (transformed) right image.
  // - It is important that we use NearestPixelInterpolation in the
  //   next step. Using anything else will interpolate nodata values
  //   and stop them from being masked out.
  if (!detect_ip_pair(ip1, ip2,
                      image1.impl(),
                      crop(transform(image2.impl(), compose(tx, inverse(right_tx)),
                                     ValueEdgeExtension<typename Image2T::pixel_type>(boost::math::isnan(nodata2) ? 0 : nodata2),
                                     NearestPixelInterpolation()),
                           raster_box),
                      ip_per_tile, left_file_path, "", // Don't record IP from transformed images.
                      nodata1, nodata2)) {
    vw_out() << "Unable to detect interest points." << std::endl;
    return false;
  }

  // Factor the transform out of the right interest points
  ip::InterestPointList::iterator ip_it;
  for (ip_it = ip2.begin(); ip_it != ip2.end(); ++ip_it) {
    Vector2 pt(ip_it->x, ip_it->y);
    pt = tx.reverse(pt);
    ip_it->ix = ip_it->x = pt.x();
    ip_it->iy = ip_it->y = pt.y();
  }

  return true;
} // End function detect_ip_aligned_pair



template <class Image1T, class Image2T>
bool epipolar_ip_matching(bool single_threaded_camera,
			  vw::ip::InterestPointList const& ip1,
			  vw::ip::InterestPointList const& ip2,
			  vw::camera::CameraModel* cam1,
			  vw::camera::CameraModel* cam2,
			  vw::ImageViewBase<Image1T> const& image1,
			  vw::ImageViewBase<Image2T> const& image2,
			  vw::cartography::Datum const& datum,
			  double epipolar_threshold,
			  double uniqueness_threshold,
			  std::vector<vw::ip::InterestPoint>& matched_ip1,
			  std::vector<vw::ip::InterestPoint>& matched_ip2,
			  double nodata1, double nodata2) {
  using namespace vw;
  
  matched_ip1.clear();
  matched_ip2.clear();

  // Match interest points forward/backward .. constraining on epipolar line
  DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);
  std::vector<size_t> forward_match, backward_match;
  vw_out() << "\t--> Matching interest points using the epipolar line." << std::endl;
  vw_out() << "\t    Uniqueness threshold: " << uniqueness_threshold << "\n";
  vw_out() << "\t    Epipolar threshold:   " << epipolar_threshold   << "\n";
  
  EpipolarLinePointMatcher matcher(single_threaded_camera,
				   uniqueness_threshold, epipolar_threshold, datum);
  vw_out() << "\t    Matching forward" << std::endl;
  matcher(ip1, ip2, detect_method, cam1, cam2, forward_match);
  vw_out() << "\t    ---> Obtained " << forward_match.size() << " matches." << std::endl;
  vw_out() << "\t    Matching backward" << std::endl;
  matcher(ip2, ip1, detect_method, cam2, cam1, backward_match);
  vw_out() << "\t    ---> Obtained " << backward_match.size() << " matches." << std::endl;

  // Perform circle consistency check
  size_t valid_count = 0;
  const size_t NULL_INDEX = (size_t)(-1);
  for (size_t i = 0; i < forward_match.size(); i++) {
    if (forward_match[i] != NULL_INDEX) {
      if (backward_match[forward_match[i]] != i) {
        forward_match[i] = NULL_INDEX;
      } else {
        valid_count++;
      }
    }
  }
  vw_out() << "\t    Matched " << valid_count << " points." << std::endl;

  if (valid_count == 0)
    return false;

  // Produce listing of valid indices that agree with forward and backward matching
  matched_ip1.reserve(valid_count); // Get our allocations out of the way.
  matched_ip2.reserve(valid_count);
  {
    ip::InterestPointList::const_iterator ip1_it = ip1.begin(), ip2_it = ip2.begin();
    for (size_t i = 0; i < forward_match.size(); i++) {
      if (forward_match[i] != NULL_INDEX) {
        matched_ip1.push_back(*ip1_it);
        ip2_it = ip2.begin();
        std::advance(ip2_it, forward_match[i]);
        matched_ip2.push_back(*ip2_it);
      }
      ip1_it++;
    }
  }

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP match debug image prior to geometric filtering.\n";
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  // Apply filtering of IP by a selection of assumptions. Low
  // triangulation error, agreement with klt tracking, and local
  // neighbors are the same neighbors in both images.
  std::list<size_t> good_indices;
  for (size_t i = 0; i < matched_ip1.size(); i++) {
    good_indices.push_back(i);
  }
  if (!stereo_settings().disable_tri_filtering) {
    if (!tri_ip_filtering(matched_ip1, matched_ip2,
			  cam1, cam2, good_indices)){
      vw_out() << "No interest points left after triangulation filtering." << std::endl;
      return false;
    }
  }
  if (!stddev_ip_filtering(matched_ip1, matched_ip2, good_indices)) {
    vw_out() << "No interest points left after stddev filtering." << std::endl;
    return false;
  }

  // Record new list that contains only the inliers.
  vw_out() << "\t    Reduced matches to " << good_indices.size() << "\n";
  std::vector<ip::InterestPoint> buffer(good_indices.size());

  // Subselect, Transform, Copy, Matched Ip1
  size_t w_index = 0;
  BOOST_FOREACH(size_t index, good_indices) {
    Vector2 l(matched_ip1[index].x, matched_ip1[index].y);
    matched_ip1[index].ix = matched_ip1[index].x = l.x();
    matched_ip1[index].iy = matched_ip1[index].y = l.y();
    buffer[w_index] = matched_ip1[index];
    w_index++;
  }
  matched_ip1 = buffer;

  // Subselect, Transform, Copy, Matched ip2
  w_index = 0;
  BOOST_FOREACH(size_t index, good_indices) {
    Vector2 r(matched_ip2[index].x, matched_ip2[index].y);
    matched_ip2[index].ix = matched_ip2[index].x = r.x();
    matched_ip2[index].iy = matched_ip2[index].y = r.y();
    buffer[w_index] = matched_ip2[index];
    w_index++;
  }
  matched_ip2 = buffer;

  // If options are set, filter by elevation.
  if (stereo_settings().elevation_limit[0] < stereo_settings().elevation_limit[1] ||
      !stereo_settings().lon_lat_limit.empty()) {

    std::vector<ip::InterestPoint> matched_ip1_out, matched_ip2_out;
    double ip_scale = 1.0; // left_tx and right_tx already have any scale info
    filter_ip_by_lonlat_and_elevation(cam1, cam2,
                                      datum,  matched_ip1, matched_ip2, ip_scale,  
                                      stereo_settings().elevation_limit,  
                                      stereo_settings().lon_lat_limit,  
                                      matched_ip1_out, matched_ip2_out);
    matched_ip1 = matched_ip1_out;
    matched_ip2 = matched_ip2_out;
  } // End elevation filtering

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP match debug image after geometric filtering.\n";
    write_match_image("InterestPointMatching__ip_matching_debug2.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  return true;
} // End function epipolar_ip_matching

template <class Image1T, class Image2T>
void detect_match_ip(std::vector<vw::ip::InterestPoint>& matched_ip1,
		     std::vector<vw::ip::InterestPoint>& matched_ip2,
		     vw::ImageViewBase<Image1T> const& image1,
		     vw::ImageViewBase<Image2T> const& image2,
		     int    ip_per_tile,
		     std::string const left_file_path,
		     std::string const right_file_path,
		     double nodata1, double nodata2, std::string const& match_file) {
  using namespace vw;

  // Detect Interest Points
  vw::ip::InterestPointList ip1, ip2;
  detect_ip_pair(ip1, ip2, image1.impl(), image2.impl(), ip_per_tile,
                 left_file_path, right_file_path, nodata1, nodata2);
  
  // Match the interset points using the default matcher
  vw_out() << "\t--> Matching interest points using homography.\n";

  // Replace the IP lists with IP vectors
  std::vector<vw::ip::InterestPoint> ip1_copy, ip2_copy;
  ip1_copy.reserve(ip1.size());
  ip2_copy.reserve(ip2.size());
  std::copy(ip1.begin(), ip1.end(), std::back_inserter(ip1_copy));
  std::copy(ip2.begin(), ip2.end(), std::back_inserter(ip2_copy));

  DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

  // Best point must be closer than the next best point
  double adj_uniqueness_thresh = (0.8/0.7)*stereo_settings().ip_uniqueness_thresh;
  adj_uniqueness_thresh = std::min(0.99, adj_uniqueness_thresh);
  vw_out() << "\t--> Adjusted uniqueness threshold: " << adj_uniqueness_thresh << "\n";
  // TODO: Should probably unify the ip::InterestPointMatcher class
  // with the EpipolarLinePointMatcher class!
  if (detect_method != DETECT_IP_METHOD_ORB) {
    // For all L2Norm distance metrics
    ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(adj_uniqueness_thresh);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "));
  }
  else {
    // For Hamming distance metrics
    ip::InterestPointMatcher<ip::HammingMetric,ip::NullConstraint> matcher(adj_uniqueness_thresh);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "));
  }

  ip::remove_duplicates(matched_ip1, matched_ip2);

  vw_out() << "\n\t    Matched points: " << matched_ip1.size() << std::endl;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP initial match debug image.\n";
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  // Save ip
  if (match_file != "") {
    // Create the output directory
    vw::create_out_dir(match_file);
    vw_out() << "Writing: " << match_file << std::endl;
    ip::write_binary_match_file(match_file, matched_ip1, matched_ip2);
  }
  
} // End function detect_match_ip


// Homography IP matching
//
// This applies only the homography constraint. Not the best...
template <class Image1T, class Image2T>
bool homography_ip_matching(vw::ImageViewBase<Image1T> const& image1,
			    vw::ImageViewBase<Image2T> const& image2,
			    int    ip_per_tile,
			    int    inlier_threshold,
			    std::string const& output_name,
			    std::string const  left_file_path,
			    std::string const  right_file_path,
			    double nodata1, double nodata2) {

  using namespace vw;

  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  detect_match_ip(matched_ip1, matched_ip2,
		  image1.impl(), image2.impl(),
		  ip_per_tile,
		  left_file_path, right_file_path,
		  nodata1, nodata2);
  if (matched_ip1.size() == 0 || matched_ip2.size() == 0)
    return false;
    
  std::vector<ip::InterestPoint> final_ip1, final_ip2;
  size_t num_left = filter_ip_homog(matched_ip1, matched_ip2, final_ip1, final_ip2,
                                    inlier_threshold);
  if (num_left == 0)
    return false;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing post-homography IP match debug image.\n";
    write_match_image("InterestPointMatching__ip_matching_debug2.tif",
                      image1, image2, final_ip1, final_ip2);
  }

  vw_out() << "\t    * Writing match file: " << output_name << "\n";
  ip::write_binary_match_file(output_name, final_ip1, final_ip2);
  return true;
}



template <class Image1T, class Image2T>
bool ip_matching_no_align(bool single_threaded_camera,
			  vw::camera::CameraModel* cam1,
			  vw::camera::CameraModel* cam2,
			  vw::ImageViewBase<Image1T> const& image1,
			  vw::ImageViewBase<Image2T> const& image2,
			  int ip_per_tile,
			  vw::cartography::Datum const& datum,
			  double epipolar_threshold,
			  double uniqueness_threshold,
			  std::string const& output_name,
			  std::string const  left_file_path,
			  std::string const  right_file_path,
			  double nodata1,
			  double nodata2) {
  using namespace vw;
  
  // Find IP
  vw::ip::InterestPointList ip1, ip2;
  detect_ip_pair(ip1, ip2, image1.impl(), image2.impl(),
                 ip_per_tile, left_file_path, right_file_path,
                 nodata1, nodata2);

  // Match them
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  if (!epipolar_ip_matching(single_threaded_camera,
			    ip1, ip2, cam1, cam2, image1, image2,
			    datum, epipolar_threshold, uniqueness_threshold,
			    matched_ip1, matched_ip2, nodata1, nodata2))
    return false;

  // Write to disk
  vw_out() << "\t    * Writing match file: " << output_name << "\n";
  ip::write_binary_match_file(output_name, matched_ip1, matched_ip2);

  return true;
}


template <class Image1T, class Image2T>
bool ip_matching_w_alignment(bool single_threaded_camera,
			     vw::camera::CameraModel* cam1,
			     vw::camera::CameraModel* cam2,
			     vw::ImageViewBase<Image1T> const& image1,
			     vw::ImageViewBase<Image2T> const& image2,
			     int ip_per_tile,
			     vw::cartography::Datum const& datum,
			     std::string const& output_name,
			     double epipolar_threshold,
			     double uniqueness_threshold,
			     std::string const left_file_path,
			     double nodata1,
			     double nodata2) {

  using namespace vw;

  vw_out() << "Performing IP matching with alignment." << std::endl;

  // This call aligns the right image to the left image then detects IPs in the two images.
  vw::ip::InterestPointList ip1, ip2;
  Matrix<double> rough_homography;
  detect_ip_aligned_pair(cam1, cam2, image1.impl(), image2.impl(),
                         ip_per_tile, datum, ip1, ip2, rough_homography, 
                         left_file_path, nodata1, nodata2);


  // Match the detected IPs which are in the original image coordinates.
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  bool inlier =
    epipolar_ip_matching(single_threaded_camera,
			 ip1, ip2,
			 cam1, cam2,
			 image1.impl(), image2.impl(),
			 datum, epipolar_threshold, uniqueness_threshold,
			 matched_ip1, matched_ip2,
			 nodata1, nodata2);
  if (!inlier)
    return false;

  // Write the matches to disk
  vw_out() << "\t    * Writing match file: " << output_name << "\n";
  ip::write_binary_match_file(output_name, matched_ip1, matched_ip2);

  // Use the interest points that we found to compute an aligning
  // homography transform for the two images.
  // - This is just a sanity check.
  bool adjust_left_image_size = true;
  Matrix<double> matrix1, matrix2;
  homography_rectification(adjust_left_image_size,
			   image1.get_size(), image2.get_size(),
			   matched_ip1, matched_ip2, matrix1, matrix2);
  if (sum(abs(submatrix(rough_homography,0,0,2,2) - submatrix(matrix2,0,0,2,2))) > 4) {
    vw_out() << "Post homography has largely different scale and skew from rough fit. Post solution is " 
	     << matrix2 << ". Examine your images, or consider using the option --skip-rough-homography.\n";
    //return false;
  }

  return inlier;
}
