#ifndef _KEYPOINT_ALIGN_H_
#define _KEYPOINT_ALIGN_H_

#include "asp_config.h"

#include <vw/Image/ImageView.h>
#include <vw/Math/Matrix.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Math/Vector.h>

#if defined(ASP_HAVE_PKG_VW_INTEREST_POINT) && ASP_HAVE_PKG_VW_INTEREST_POINT==1
#include <vw/InterestPoint.h>
#include <vw/InterestPoint/RANSAC.h>
using namespace vw::ip;
#endif

#include "SIFT.h"

template <class ImageElemT, class MatrixElemT>
void disparity_linear_transform(vw::ImageView<vw::PixelDisparity<ImageElemT> > &disparity_map, 
                                vw::Matrix<MatrixElemT> const& transform_matrix) {
  
  //  std::cout << "The transform: " << transform_matrix << "\n";

  // Apply the inverse transform to the disparity map 
  vw::Vector3 left_point, right_point;
  for (int i = 0; i < disparity_map.cols(); i++) {
    for (int j = 0; j < disparity_map.rows(); j++) {
      if ( !disparity_map(i,j).missing() ) {
        right_point(0) = i + disparity_map(i,j).h();
        right_point(1) = j + disparity_map(i,j).v();
        right_point(2) = 1;
        right_point = transform_matrix * right_point;     // apply the transform
        right_point = right_point / right_point(2);       // and re-normalize

//         if (i==500 && j==500) 
//          	std::cout << "Before: " << i+disparity_map(i,j).h() << "   " << j+disparity_map(i,j).v() << "\n";
        disparity_map(i,j).h() = right_point(0) - i;
        disparity_map(i,j).v() = right_point(1) - j;
//         if (i==500 && j==500) {
//           std::cout << "After: " << i+disparity_map(i,j).h() << "   " << j+disparity_map(i,j).v() << "\n\n";
//           std::cout << "Disp: " << disparity_map(i,j).h() << "   " << disparity_map(i,j).v() << "\n\n";
//         }
      }
    }
  }
}


template <class ViewT>
vw::Matrix<double> keypoint_align(vw::ImageViewBase<ViewT> const& primary, 
                                  vw::ImageViewBase<ViewT> const& secondary) {
  
  std::cout << "\nInterest Point Detection\n";
  static const int MAX_KEYPOINT_IMAGE_DIMENSION = 2048;
  std::vector<InterestPoint> ip1 = interest_points(primary.impl(), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
  std::vector<InterestPoint> ip2 = interest_points(secondary.impl(), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
  
  std::cout << "\nInterest Point Matching\n";
  LoweMatcher m;
  std::vector<InterestPoint> matched_ip1, matched_ip2;
  m(ip1, ip2, matched_ip1, matched_ip2);
  std::cout << "\tFound " << matched_ip1.size() << " putative matches.\n";
  
  std::cout << "\nLiam's Interest Point Matching\n";
  InterestPointMatcher<L2NormMetric,NullConstraint> matcher;
  std::vector<InterestPoint> other_matched_ip1, other_matched_ip2;
  matcher.match(ip1, ip2, other_matched_ip1, other_matched_ip2,true);
  std::cout << "\tFound " << other_matched_ip1.size() << " putative matches.\n";
  
  return ransac(matched_ip1, matched_ip2, 
                vw::math::SimilarityFittingFunctor(),
                KeypointErrorMetric());
}

template <class ViewT>
vw::Matrix<double> keypoint_align(vw::ImageViewBase<ViewT> const& primary, 
                                  vw::ImageViewBase<ViewT> const& secondary,
                                  std::vector<InterestPoint> &matched_ip1, 
                                  std::vector<InterestPoint> &matched_ip2) {
  
  std::cout << "\nInterest Point Detection\n";
  static const int MAX_KEYPOINT_IMAGE_DIMENSION = 2048;
  std::vector<InterestPoint> ip1 = interest_points(primary.impl(), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
  std::vector<InterestPoint> ip2 = interest_points(secondary.impl(), LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
  
  std::cout << "\nInterest Point Matching\n";
  LoweMatcher m;
  m(ip1, ip2, matched_ip1, matched_ip2);
  std::cout << "\tFound " << matched_ip1.size() << " putative matches.\n";
  
  std::cout << "\nLiam's Interest Point Matching\n";
  InterestPointMatcher<L2NormMetric,NullConstraint> matcher;
  std::vector<InterestPoint> other_matched_ip1, other_matched_ip2;
  matcher.match(ip1, ip2, other_matched_ip1, other_matched_ip2,true);
  std::cout << "\tFound " << other_matched_ip1.size() << " putative matches.\n";
  
  return ransac(matched_ip1, matched_ip2, 
                vw::math::SimilarityFittingFunctor(),
                KeypointErrorMetric());
}

#endif // _KEYPOINT_ALIGN_H_
