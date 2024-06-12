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

#include <asp/Core/OutlierProcessing.h>

#include <vw/Image/Statistics.h>
#include <vw/Math/Statistics.h>
#include <vw/Core/Stopwatch.h>

using namespace vw;

namespace asp {

// Estimate a bounding box without outliers. Note that individual percentage factors
// are used in x, y, and z. These are supposed to around 0.75 or so. The outlier factor is 3.0
// or so.
void estimate_inliers_bbox(double pct_factor_x, double pct_factor_y, double pct_factor_z,
                           double outlier_factor,
                           std::vector<double> const& x_vals,
                           std::vector<double> const& y_vals,
                           std::vector<double> const& z_vals,
                           vw::BBox3 & inliers_bbox) {
  
  // Initialize the output
  inliers_bbox = BBox3();

  double bx, ex, by, ey, bz, ez;
  if (!vw::math::find_outlier_brackets(x_vals, pct_factor_x, outlier_factor, bx, ex))
    return;
  if (!vw::math::find_outlier_brackets(y_vals, pct_factor_y, outlier_factor, by, ey))
    return;
  if (!vw::math::find_outlier_brackets(z_vals, pct_factor_z, outlier_factor, bz, ez))
    return;

  // NaN values will result in an error further down
  if (ex != ex || ey != ey || ez != ez)
    return;  

  // Need to compute the next double because the VW bounding box is
  // exclusive at the top.
  ex = boost::math::nextafter(ex, std::numeric_limits<double>::max());
  ey = boost::math::nextafter(ey, std::numeric_limits<double>::max());
  ez = boost::math::nextafter(ez, std::numeric_limits<double>::max());

  inliers_bbox.grow(Vector3(bx, by, bz));
  inliers_bbox.grow(Vector3(ex, ey, ez));
}

// Get a generous estimate of the bounding box of the current set
// while excluding outliers
void estimate_points_bdbox(vw::ImageViewRef<vw::Vector3> const& proj_points,
                           vw::ImageViewRef<double> const& error_image,
                           vw::Vector2 const& remove_outliers_params,
                           double estim_max_error,
                           vw::BBox3 & inliers_bbox) {

  // TODO(oalexan1): Here it may help to do several passes. First throw out the worst
  // outliers, then estimate the box from the remaining points, etc.
  
  std::vector<double> x_vals, y_vals, z_vals;
  for (int col = 0; col < proj_points.cols(); col++){
    for (int row = 0; row < proj_points.rows(); row++){

      // Avoid points marked as not valid
      Vector3 P = proj_points(col, row);
      if (P != P)
        continue;

      // Make use of the estimated error, if available
      if (estim_max_error > 0 && error_image(col, row) > estim_max_error) 
        continue;

      x_vals.push_back(P.x());
      y_vals.push_back(P.y());
      z_vals.push_back(P.z());
    }
  }

  double pct_factor     = remove_outliers_params[0]/100.0; // e.g., 0.75
  double outlier_factor = remove_outliers_params[1];       // e.g., 3.0.

  // Make these more generous, as we want to throw out only the worst
  // outliers. Note that we are even more generous in z, to avoid cutting
  // of isolated mountain peaks. This is a bugfix.
  double pct_factor_x = (1.0 + pct_factor)/2.0; // e.g., 0.875 
  double pct_factor_y = (1.0 + pct_factor)/2.0; // e.g., 0.875 
  double pct_factor_z = (3.0 + pct_factor)/4.0; // e.g., 0.9375

  // Double this factor, now it will equal 6.  With a small factor, if
  // the domain of the DEM is a rectangle rotated by 45 degrees, it
  // may cut off corners.
  outlier_factor *= 2.0;

  // Call auxiliary function to do the estimation
  estimate_inliers_bbox(pct_factor_x, pct_factor_y, pct_factor_z, outlier_factor, 
                        x_vals, y_vals, z_vals, inliers_bbox);
  
  return;
}

// A class to pick some samples to estimate the range of values
// of a given dataset
class ErrorRangeEstimAccum: public ReturnFixedType<void> {
  typedef double accum_type;
  std::vector<accum_type> m_vals;
public:
  typedef accum_type value_type;
  
  ErrorRangeEstimAccum() { m_vals.clear(); }
  
  void operator()( accum_type const& value ) {
    // Don't add zero errors, those most likely came from invalid points
    if (value > 0)
      m_vals.push_back(value);
  }
  
  int size(){
    return m_vals.size();
  }
  
  value_type value(Vector2 const& remove_outliers_params){
    VW_ASSERT(!m_vals.empty(), ArgumentErr() << "ErrorRangeEstimAccum: no valid samples");
    
    // How to pick a representative value for maximum error?  The
    // maximum error itself may be no good, as it could be very
    // huge, and then sampling the range of errors will be distorted
    // by that.  The solution adopted here: Find a percentile of the
    // range of errors, mulitply it by the outlier factor, and
    // multiply by another factor to ensure we don't underestimate
    // the maximum. This value may end up being larger than the
    // largest error, but at least it is is not grossly huge
    // if just a few of the errors are very large.
    std::sort(m_vals.begin(), m_vals.end());
    int    len    = m_vals.size();
    double pct    = remove_outliers_params[0]/100.0; // e.g., 0.75
    double factor = remove_outliers_params[1];
    int    k      = std::min(len - 1, (int)(pct*len));
    
    // Care here with empty sets
      if (k >= 0) 
        return m_vals[k]*factor*4.0;
      
      return 0;
  }
  
};
  
// Sample the image and get generous estimates (but without outliers)
// of the maximum triangulation error and of the 3D box containing the
// projected points. These will be tightened later.
double estim_max_tri_error_and_proj_box(vw::ImageViewRef<vw::Vector3> const& proj_points,
                                        vw::ImageViewRef<double> const& error_image,
                                        vw::Vector2 const& remove_outliers_params,
                                        vw::BBox3 & estim_proj_box) {

  // Initialize the outputs
  double estim_max_error = 0.0;
  estim_proj_box = BBox3();
  
  if (error_image.rows() > 0 && error_image.cols() > 0 && 
      (error_image.cols() != proj_points.cols() || 
      error_image.rows() != proj_points.rows())) 
      vw_throw(ArgumentErr() 
                << "The error image and point image must have the same size.");

  // Start with a 256 (2^8) by 256 sampling of the cloud
  bool success = true;
  for (int attempt = 8; attempt <= 18; attempt++) {
    
    double sample = (1 << attempt);
    int32 subsample_amt = int32(norm_2(Vector2(proj_points.cols(), 
                                               proj_points.rows()))/sample);
    
    if (subsample_amt < 1)
      subsample_amt = 1;
    
    PixelAccumulator<asp::ErrorRangeEstimAccum> error_accum;
    if (error_image.cols() > 0 && error_image.rows() > 0) {
      //Stopwatch sw2;
      //sw2.start();
      for_each_pixel(subsample(error_image, subsample_amt),
                    error_accum,
                    TerminalProgressCallback
                    ("asp","Bounding box and triangulation error range estimation: ") );
      if (error_accum.size() > 0) 
        estim_max_error = error_accum.value(remove_outliers_params);
      else
        success = false;
        
      //sw2.stop();
      //vw_out(DebugMessage,"asp") << "Elapsed time: " << sw2.elapsed_seconds() << std::endl;
    }

    asp::estimate_points_bdbox(subsample(proj_points, subsample_amt),
                               subsample(error_image, subsample_amt),
                               remove_outliers_params,  estim_max_error,
                               estim_proj_box);

    if (estim_proj_box.empty()) 
      success = false;
    
    if (success || subsample_amt == 1) 
      break;
      
    vw_out() << "DEM extent estimation failed. Check if your cloud is valid. "
             << "Trying again with finer sampling.\n";
  }
  
  return estim_max_error;
}
  
} // end namespace asp
