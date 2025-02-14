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

// C++ implementation of the Nuth and Kaab alignment method from:
// https://github.com/dshean/demcoreg/tree/master

// TODO(oalexan1): Implement tilt correction.
// Implement --initial-transform and computing final transform in ECEF.
// TODO(oalexan1): Integrate in pc_align.
// TODO(oalexan1): It is assumed both input DEMs are equally dense. The work
// will happen in the source DEM domain. Later we can deal with the case when
// we want it done in the reference DEM domain. This is temporary.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/DemUtils.h>

#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math/Functors.h>

#include <boost/program_options.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <ceres/ceres.h>

inline double DegToRad(double deg) {
  return deg * M_PI / 180.0;
}

// Function for fitting Nuth and Kaab (2011)
// Can use phasor addition, but need to change conversion to offset dx and dy
// https://stackoverflow.com/questions/12397412/i-know-scipy-curve-fit-can-do-better?rq=1
double nuth_func(double x, double a, double b, double c) {
  double y = a * cos(DegToRad(b-x)) + c;
  return y;
}

// An error function minimizing the error of projecting an xyz point
// into a given CSM linescan camera pixel. The variables of optimization are a
// portion of the position and quaternion variables affected by this, and the 
// triangulation point.
const int NUM_RES = 2;
const int NUM_PARAMS = 3;
struct LsPixelReprojErr {
  LsPixelReprojErr(vw::Vector2 const& observation, double weight,
                   int begQuatIndex, int endQuatIndex, 
                   int begPosIndex, int endPosIndex):
    m_observation(observation), m_weight(weight),
    m_begQuatIndex(begQuatIndex), m_endQuatIndex(endQuatIndex),
    m_begPosIndex(begPosIndex),   m_endPosIndex(endPosIndex) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const {
    
    // Can use parameters[i][j]
    
    residuals[0] = 0;
    residuals[1] = 0;
    
    return true;
  } 

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(vw::Vector2 const& observation, double weight,
                                     int begQuatIndex, int endQuatIndex,
                                     int begPosIndex, int endPosIndex) {

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<LsPixelReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<LsPixelReprojErr>
      (new LsPixelReprojErr(observation, weight, 
                                  begQuatIndex, endQuatIndex,
                                  begPosIndex, endPosIndex));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RES);

    // Add a parameter block for each quaternion and each position
    for (int it = begQuatIndex; it < endQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_PARAMS);

    return cost_function;
  }

private:
  vw::Vector2 m_observation; // The pixel observation for this camera/point pair
  double m_weight;
  int m_begQuatIndex, m_endQuatIndex;
  int m_begPosIndex, m_endPosIndex;
}; // End class LsPixelReprojErr


namespace po = boost::program_options;

struct Options: vw::GdalWriteOptions {
  std::string ref, src, out_prefix, res;
  int poly_order, max_iter;
  double tol, max_offset, max_dz;
  bool tiltcorr;
  vw::Vector2 slope_lim;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General options");
  general_options.add_options()
    ("ref,r", po::value(&opt.ref)->default_value(""),
     "Reference DEM.")
    ("src,s", po::value(&opt.src)->default_value(""),
     "Source DEM to align to the reference.")
    ("output-prefix,o", po::value(&opt.out_prefix)->default_value(""), 
     "Output prefix for writing all produced files.")
    ("poly-order", po::value(&opt.poly_order)->default_value(1), 
     "Specify the order of the polynomial fit.")
    ("tol", po::value(&opt.tol)->default_value(0.1),
      "Stop when iterative translation magnitude is below this tolerance (meters).")
    ("max-offset", po::value(&opt.max_offset)->default_value(100.0),
     "Maximum expected horizontal translation magnitude (meters).")
    ("max-dz", po::value(&opt.max_dz)->default_value(100.0),
     "Maximum expected vertical offset in meters, used to filter outliers.")
    ("tiltcorr", po::bool_switch(&opt.tiltcorr)->default_value(false),
     "After a preliminary translation, fit a polynomial to residual elevation offsets "
     "and remove.")
    ("res", po::value(&opt.res)->default_value("mean"),
     "Regrid the input DEMs to this resolution given the resolutions of input datasets. "
     "Options: min, max, mean, common_scale_factor.")
    ("slope-lim", 
     po::value(&opt.slope_lim)->default_value(vw::Vector2(0.1, 40.0), "0.1, 40.0"),
     "Minimum and maximum surface slope limits to consider (degrees).")
    ("max-iter", po::value(&opt.max_iter)->default_value(30),
     "Maximum number of iterations, if tolerance is not reached.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("-r ref_dem.tif -s src_dem.tif -o output_prefix [options]\n");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // On failure print the usage as well, for when the command is called with no
  // arguments.
  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "The output prefix was not set.\n"
             << usage << general_options);

  // The ref and src DEMs must be provided
  if (opt.ref == "" || opt.src == "")
    vw::vw_throw(vw::ArgumentErr() << "The reference and source DEMs must be provided.\n");

  // The poly order must be positive
  if (opt.poly_order < 1)
    vw::vw_throw(vw::ArgumentErr() << "The polynomial order must be at least 1.\n");
  
  // The tol must be positive
  if (opt.tol <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The tolerance must be positive.\n");
  
  // Max offset must be positive. Same with max_dz.
  if (opt.max_offset <= 0.0 || opt.max_dz <= 0.0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "The maximum horizontal and vertical offsets must be positive.\n");

  // Check that res is one of the allowed values
  if (opt.res != "min" && opt.res != "max" && opt.res != "mean" &&
      opt.res != "common_scale_factor")
    vw::vw_throw(vw::ArgumentErr() << "Unknown value for --res: " << opt.res << ".\n");
  
  // Check that slope limits are positive and in the right order
  if (opt.slope_lim[0] <= 0.0 || opt.slope_lim[1] <= 0.0 || 
      opt.slope_lim[0] >= opt.slope_lim[1])
    vw::vw_throw(vw::ArgumentErr() 
                 << "The slope limits must be positive and in increasing order.\n");
    
  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);
}

// Find the valid pixels. Use long long, to avoid integer overflow.
// TODO(oalexan1): Move this to NuthUtils.cc
long long validCount(vw::ImageView<vw::PixelMask<double>> const& img) {
  
  long long count = 0;
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row)))
        count++;
    }
  }
  
  return count;
}

// Compute the median of the valid pixels
// TODO(oalexan1): Move this to NuthUtils.cc
double maskedMedian(vw::ImageView<vw::PixelMask<double>> const& img) {

  // Allocate enough space first  
  std::vector<double> vals(img.cols()*img.rows());
  vals.clear();
  
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row)))
        vals.push_back(img(col, row).child());
    }
  }
  
  if (vals.empty())
    vw::vw_throw(vw::ArgumentErr() << "No valid pixels found in median calculation.\n");
  
  return vw::math::destructive_median(vals);
}

// Comput normalized median absolute deviation
// TODO(oalexan1): Move this to NuthUtils.cc
double normalizedMad(vw::ImageView<vw::PixelMask<double>> const& img, 
                     double median) {
  
  // Compute the median absolute deviation
  std::vector<double> vals(img.cols()*img.rows());
  vals.clear();
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row)))
        vals.push_back(std::abs(img(col, row).child() - median));
    }
  }
  
  if (vals.empty())
    vw::vw_throw(vw::ArgumentErr() << "No valid pixels found in MAD calculation.\n");
  
  double mad = vw::math::destructive_median(vals);

  // The normalization factor is to make this equivalent to the standard deviation  
  return  1.4826 * mad;
}

// Find the mean of valid pixels.
// TODO(oalexan1): Move this to NuthUtils.cc
double maskedMean(vw::ImageView<vw::PixelMask<double>> const& img) {
  
  double sum = 0.0;
  long long count = 0;
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row))) {
        sum += img(col, row).child();
        count++;
      }
    }
  }
  
  if (count == 0)
    return 0.0;
  
  return sum / count;
}

// Find the std dev of valid pixels.
// TODO(oalexan1): Move this to NuthUtils.cc
double maskedStdDev(vw::ImageView<vw::PixelMask<double>> const& img, double mean) {
  
  double sum = 0.0;
  long long count = 0;
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row))) {
        sum += (img(col, row).child() - mean) * (img(col, row).child() - mean);
        count++;
      }
    }
  }
  
  if (count == 0)
    return 0.0;
  
  return sqrt(sum / count);
}
  
// Filter outside this range
// TODO(oalexan1): Move this to NuthUtils.cc
void rangeFilter(vw::ImageView<vw::PixelMask<double>> & diff, 
                 double min_val, double max_val) {

  for (int col = 0; col < diff.cols(); col++) {
    for (int row = 0; row < diff.rows(); row++) {
      if (is_valid(diff(col, row)) && 
          (diff(col, row).child() < min_val || diff(col, row).child() > max_val))
        diff(col, row).invalidate();
    }
  }
}

// Invalidate pixels in first image that are invalid in second image
void intersectValid(vw::ImageView<vw::PixelMask<double>> & img1, 
                    vw::ImageView<vw::PixelMask<double>> const& img2) {
  
  for (int col = 0; col < img1.cols(); col++) {
    for (int row = 0; row < img1.rows(); row++) {
      if (!is_valid(img2(col, row)))
        img1(col, row).invalidate();
    }
  }
}

// Filter by normalized median absolute deviation with given factor
// TODO(oalexan1): Move this to NuthUtils.cc
void madFilter(vw::ImageView<vw::PixelMask<double>> & diff, double outlierFactor) {
  
  double median = maskedMedian(diff);
  double mad = normalizedMad(diff, median);
  double min_val = median - outlierFactor * mad;
  double max_val = median + outlierFactor * mad;
  rangeFilter(diff, min_val, max_val);
}

// Filter outliers by range and then by normalized median absolute deviation
// TODO(oalexan1): Move this to NuthUtils.cc
void rangeMadFilter(vw::ImageView<vw::PixelMask<double>> & diff, 
                    double outlierFactor,
                    double max_dz) {

  double origCount = validCount(diff);

  // Filter out diff whose magnitude is greater than max_dz    
  rangeFilter(diff, -max_dz, max_dz);
  
  // Apply normalized mad filter with given factor
  madFilter(diff, outlierFactor);
  
  double finalCount = validCount(diff);
  
  // Find the percentage, and round it to 2 decimal places
  double removedPct = 100.0 * (origCount - finalCount) / origCount;
  removedPct = std::round(removedPct * 100.0) / 100.0;
  vw::vw_out() << "Percentage of samples removed as outliers: " << removedPct << "%\n";
}

// Calculate the DEM slope and aspect in degrees at a given location.
// The code below is adapted from GDAL.
// https://github.com/OSGeo/gdal/blob/46412f0fb7df3f0f71c6c31ff0ae97b7cef6fa61/apps/gdaldem_lib.cpp#L1284C1-L1301C1
//https://github.com/OSGeo/gdal/blob/46412f0fb7df3f0f71c6c31ff0ae97b7cef6fa61/apps/gdaldem_lib.cpp#L1363
// It looks to be an approximation of the Zevenbergen and Thorne method, not the
// real thing. Using same logic for consistency.
void SlopeAspectZevenbergenThorneAlg(vw::ImageView<vw::PixelMask<double>> const& dem,
                                     vw::cartography::GeoReference const& georef,
                                     int col, int row,
                                     vw::PixelMask<double> & slope,
                                     vw::PixelMask<double> & aspect) {

  // The slope and aspect start as invalid
  slope.invalidate();
  aspect.invalidate();
  
  int num_cols = dem.cols(), num_rows = dem.rows();
  
  // If the pixel is at the border, return invalid
  if (col <= 0 || row <= 0 || col >= num_cols - 1 || row >= num_rows - 1)
    return;
  
  // Form the afWin vector with the pixel values
  std::vector<double> afWin(9);
  int count = 0;
  for (int irow = -1; irow <= 1; irow++) {
    for (int icol = -1; icol <= 1; icol++) {
      
      auto val = dem(col + icol, row + irow);
      
      // If invalid, return invalid right away. Note how we check here only
      // either icol or irow being zero, as we don't use the diagonal values.
      // The original Zevenbergen and Thorne method uses those values,
      // but the GDAL implementation does not.
      if (!is_valid(val) && (icol == 0 || irow == 0))
        return;
        
      afWin[count] = val.child();
      count++;
    }
  }
  
  // To find the local grid size in x, convert pixels to projected coordinates.
  vw::Vector2 left_pt = georef.pixel_to_point(vw::Vector2(col - 1, row));
  vw::Vector2 right_pt = georef.pixel_to_point(vw::Vector2(col + 1, row));
  double grid_x = vw::math::norm_2(right_pt - left_pt) / 2.0;

  // Same for y
  vw::Vector2 top_pt = georef.pixel_to_point(vw::Vector2(col, row - 1));
  vw::Vector2 bottom_pt = georef.pixel_to_point(vw::Vector2(col, row + 1));
  double grid_y = vw::math::norm_2(bottom_pt - top_pt) / 2.0;
  
  // Compute the slope
  double dx = (afWin[3] - afWin[5]) / grid_x;
  double dy = (afWin[7] - afWin[1]) / grid_y;
  double key = dx * dx + dy * dy;
  // The division by 2 is needed because that's how centered differences are found
  slope = atan(sqrt(key)/2.0) * 180.0 / M_PI;
  slope.validate();

  // Compute the aspect   
  // TODO(oalexan1): It is not clear if need to divide by grid_x or grid_y here.
  // The GDAL code does not, but I think it should. These grid sizes are usually 
  // about the same in projected coordinates, so likely it does not matter.
  dx = afWin[5] - afWin[3];
  dy = afWin[7] - afWin[1];  
  if (dx == 0 && dy == 0) {
    aspect.invalidate();
    return;
  }
  double a = atan2(dy, -dx) * 180.0 / M_PI;
  // Using the logic for when psData->bAngleAsAzimuth is true
  if (a > 90.0)
      a = 450.0 - a;
  else
      a = 90.0 - a;  
  if (a >= 360.0)
    a -= 360.0;
    
  aspect = a;
  aspect.validate();  
}

// Calculate the DEM slope and aspect in degrees. Use the same logic as gdaldem. 
void calcSlopeAspect(vw::ImageView<vw::PixelMask<double>> const& dem, 
                     vw::cartography::GeoReference const& georef,
                     // Outputs
                     vw::ImageView<vw::PixelMask<double>> & slope,
                     vw::ImageView<vw::PixelMask<double>> & aspect) {

  // Allocate space for the output slope and aspect
  slope.set_size(dem.cols(), dem.rows());
  aspect.set_size(dem.cols(), dem.rows());
  
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      SlopeAspectZevenbergenThorneAlg(dem, georef, col, row, 
                                      slope(col, row), aspect(col, row)); // outputs
    }
  }

}

// Put the above logic in a function that prepares the data
void prepareData(Options const& opt, 
                  vw::ImageView<vw::PixelMask<double>> & ref_copy,
                  vw::ImageView<vw::PixelMask<double>> & src_copy,
                  double & ref_nodata, double & src_nodata,
                  vw::cartography::GeoReference & crop_georef) {

  vw::vw_out() << "Reference DEM: " << opt.ref << "\n";
  vw::vw_out() << "Source DEM:    " << opt.src << "\n";
  
  // The ref DEM must have a georeference
  vw::cartography::GeoReference ref_georef, src_georef;
  bool has_ref_georef = vw::cartography::read_georeference(ref_georef, opt.ref);
  bool has_src_georef = vw::cartography::read_georeference(src_georef, opt.src);
  if (!has_ref_georef || !has_src_georef)
    vw::vw_throw(vw::ArgumentErr() << "The input DEMs must have georeferences.\n");
  
  // The georeferences must be in projected coordinates
  if (!ref_georef.is_projected() || !src_georef.is_projected())
    vw::vw_throw(vw::ArgumentErr() 
      << "The input DEMs must be in a projected coordinate system, in units of meter. "
      << "Use gdalwarp with the option -r cubic to reproject them. "
      << "Consider using an equidistant or UTM projection.\n");

  // We don't support datum changes
  if (std::abs(ref_georef.datum().semi_major_axis()
               - src_georef.datum().semi_major_axis()) > 0.1 ||
      std::abs(ref_georef.datum().semi_minor_axis()
               - src_georef.datum().semi_minor_axis()) > 0.1 ||
      ref_georef.datum().meridian_offset() != src_georef.datum().meridian_offset())
    vw::vw_throw(vw::NoImplErr() 
                 << "The input DEMs must have the same datum radii and meridian "
                 << "Use gdalwarp -r cubic to reproject to a shared datum.");

  // Reference and source DEM resolutions
  double ref_tr = ref_georef.transform()(0, 0);
  double src_tr = src_georef.transform()(0, 0);
  vw::vw_out() << "Reference DEM grid size: " << ref_tr << " meters.\n";
  vw::vw_out() << "Source DEM grid size: " << src_tr << " meters.\n";

  // Sanity check. This code was not tested with different grid sizes in x and y.
  double ref_tr_y = ref_georef.transform()(1, 1);
  double src_tr_y = src_georef.transform()(1, 1);
  double ref_err  = std::abs(std::abs(ref_tr_y/ref_tr) - 1.0);
  double src_err  = std::abs(std::abs(src_tr_y/src_tr) - 1.0);
  if (ref_err > 1e-2 || src_err > 1e-2)
    vw::vw_throw(vw::NoImplErr() 
                 << "The input DEM grid sizes in x and y do not agree. Please reproject "
                 << "the DEMs with gdalwarp -r cubic.\n");
  
  // Prefer the reference resolution to be smaller
  if (ref_tr > src_tr)
    vw::vw_out(vw::WarningMessage) 
      << "The reference DEM resolution is larger than the source DEM resolution. "
      << "This may lead to suboptimal results. It is strongly suggest to swap the "
      << "reference and source DEMs.\n";
  
  // The (1, 1) value in both transforms must be negative, as otherwise this a
  // non-standard transform that we do not handle.
  if (ref_georef.transform()(1, 1) >= 0 || src_georef.transform()(1, 1) >= 0)
    vw::vw_throw(vw::ArgumentErr() 
      << "The input DEMs must have y direction pointing down, so the second coordinate "
      << " of the pixel size in the geoheader be negative. The provided input DEMs "
      << " are not standard and are not supported.\n");

  // Read the DEMs and their no-data values as double
  vw::DiskImageResourceGDAL ref_rsrc(opt.ref), src_rsrc(opt.src);
  ref_nodata = -std::numeric_limits<double>::max();
  src_nodata = ref_nodata;
  if (ref_rsrc.has_nodata_read())
    ref_nodata = ref_rsrc.nodata_read();
  if (src_rsrc.has_nodata_read())
    src_nodata = src_rsrc.nodata_read(); 
  vw:: DiskImageView<double> ref_dem(ref_rsrc), src_dem(src_rsrc);
  
  // Copying verbatim from dem_align.py: Use original source dataset coordinate
  // system. Potentially issues with distortion and xyz / tilt corr offsets for
  // DEMs with large extent.
  
  // TODO(oalexan1): Here is where dem_align.py starts assuming that 
  // one works in the source DEM domain. 
  
  vw::cartography::GeoReference local_georef = src_georef;
  std::string local_srs = local_georef.get_wkt();

  // Transform the second DEM's bounding box to first DEM's pixels
  vw::BBox2i src_crop_box = bounding_box(src_dem);
  vw::cartography::GeoTransform gt(ref_georef, src_georef);
  vw::BBox2i ref2src_box = gt.forward_bbox(bounding_box(ref_dem));
  src_crop_box.crop(ref2src_box);

  if (src_crop_box.empty()) 
    vw::vw_throw(vw::ArgumentErr() << "The two DEMs do not have a common area.\n");

  // Crop the src DEM to the shared box, then transform and crop the ref DEM.
  // to src DEM domain. Use bicubic interpolation for the transform.
  // TODO(oalexan1): This will need changing when the grids are not the same.
  vw::ImageViewRef<vw::PixelMask<double>> src_crop 
    = vw::crop(create_mask(src_dem, src_nodata), src_crop_box);
  vw::ImageViewRef<vw::PixelMask<double>> ref_trans 
    = asp::warpCrop(ref_dem, ref_nodata, src_georef, ref_georef, src_crop_box, 
                    "bicubic");
  
  // Recall that we work in the src domain. 
  // TODO(oalexan1): This will need to change.
  vw::vw_out() << "Working in cropped source domain. Regrid the reference DEM.\n";
  crop_georef = vw::cartography::crop(local_georef, src_crop_box);

#if 1  
  // Write the cropped and warped ref
  vw::TerminalProgressCallback ref_tpc("asp", ": ");
  bool has_ref_no_data = true;
  std::string ref_crop_file = opt.out_prefix + "-ref-crop.tif"; 
  vw::vw_out() << "Writing: " << ref_crop_file << "\n";
  vw::cartography::block_write_gdal_image(ref_crop_file,
                                          vw::apply_mask(ref_trans, ref_nodata),
                                          has_ref_georef, crop_georef,
                                          has_ref_no_data, ref_nodata,
                                          opt, ref_tpc);
  // Write the cropped src
  vw::TerminalProgressCallback src_tpc("asp", ": ");
  bool has_src_no_data = true;
  std::string src_crop_file = opt.out_prefix + "-src-crop.tif";
  vw::vw_out() << "Writing: " << src_crop_file << "\n";
  vw::cartography::block_write_gdal_image(src_crop_file,
                                          vw::apply_mask(src_crop, src_nodata),
                                          has_src_georef, crop_georef,
                                          has_src_no_data, src_nodata,
                                          opt, src_tpc);
#endif

  // TODO(oalexan1): Keeping the DEMs on disk will require changing a lot 
  // of code. For now we keep them in memory.
  vw::vw_out() << "Reading the input DEMs in memory for faster processing.\n";
  ref_copy = ref_trans;
  src_copy = src_crop;
}

// Compute the Nuth offset. By now the DEMs have been regrided to the same
// resolution and use the same projection. 
void computeNuthOffset(vw::ImageView<vw::PixelMask<double>> const& ref,
                       vw::ImageView<vw::PixelMask<double>> const& src,
                       vw::cartography::GeoReference const& crop_georef,
                       double ref_nodata, double src_nodata,
                       double max_offset, double max_dz, vw::Vector2 const& slope_lim) {

  // TODO(oalexan1): Implement the regrid here as in the dem_align.py code.
  vw::vw_out() << "Must regrid each time at this stage, as the georef will change.\n";
  
  // Ref and src must have the same size
  if (ref.cols() != src.cols() || ref.rows() != src.rows())
    vw::vw_throw(vw::ArgumentErr() 
                 << "The reference and source DEMs must have the same size.\n"); 
    
  // Estimate the ground resolution
  double gridx = 0.0, gridy = 0.0;
  int sample_rate = std::min(ref.cols(), ref.rows()) / 50;
  if (sample_rate < 1) 
    sample_rate = 1;
  asp::calcGsd(vw::apply_mask(ref, ref_nodata), crop_georef, ref_nodata,
               sample_rate, sample_rate, gridx, gridy);
  double res = (gridx + gridy) / 2.0;
  if (res <= 0.0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "Could not estimate the DEM grid size. Check if your input clouds "
                 "are dense enough and with solid overlap.\n");
    
  int max_offset_px = int(max_offset/res) + 1;
  vw::Vector2i pad(max_offset_px, max_offset_px);
  
  // Find the diff
  vw::ImageView<vw::PixelMask<double>> diff(src.cols(), src.rows());
  for (int col = 0; col < src.cols(); col++) {
    for (int row = 0; row < src.rows(); row++) {
       diff(col, row) = src(col, row) - ref(col, row);
    }
  }
  
  double outlierFactor = 3.0;
  rangeMadFilter(diff, outlierFactor, max_dz);
  
  // Calculate the slope and aspect of the src_copy DEM
  vw::ImageView<vw::PixelMask<double>> slope, aspect;
  calcSlopeAspect(src, crop_georef, slope, aspect);
  
  std::cout << "--slope lim: " << slope_lim << std::endl;
  std::cout << "number of valid slopes: " << validCount(slope) << "\n";
  
  // Filter slopes by range
  rangeFilter(slope, slope_lim[0], slope_lim[1]);
  
  std::cout << "--number of valid slopes after range filter: " << validCount(slope) << "\n";
  std::cout << "number of diff before filter by slope and aspect: " << validCount(diff) << "\n";
  
  // Invalidate diffs with invalid slope or aspect
  intersectValid(diff, slope);
  intersectValid(diff, aspect);
  
  std::cout << "number of diff after filter by slope: " << validCount(diff) << "\n";
  std::cout << "number of diff after filter by aspect: " << validCount(diff) << "\n";
  
  // Important sanity check
  int minCount = 100;
  if (validCount(diff) < minCount)
    vw::vw_throw(vw::ArgumentErr() 
                 << "Too little valid data left after filtering.\n");
      
  double median_diff = maskedMedian(diff);
  double median_slope = maskedMedian(slope);
  double c_seed = (median_diff/tan(DegToRad(median_slope)));

  std::cout << "median_diff: " << median_diff << std::endl;
  std::cout << "median_slope: " << median_slope << std::endl;
  std::cout << "c_seed: " << c_seed << std::endl;
  
  // For invalid diff make the slope and aspect invalid
  intersectValid(slope, diff);
  intersectValid(aspect, diff);

  // Form xdata and ydata. We don't bother removing the flagged invalid pixels.
  // xdata = aspect[common_mask].data
  // ydata = (diff[common_mask]/np.tan(np.deg2rad(slope[common_mask]))).data
  vw::ImageView<vw::PixelMask<double>> xdata = copy(aspect);
  vw::ImageView<vw::PixelMask<double>> ydata(src.cols(), src.rows());
  for (int col = 0; col < src.cols(); col++) {
    for (int row = 0; row < src.rows(); row++) {
      
      ydata(col, row).invalidate();
      if (is_valid(diff(col, row)) && is_valid(slope(col, row)) && 
          slope(col, row).child() != 0) {
        ydata(col, row) 
          = diff(col, row).child() / tan(DegToRad(slope(col, row).child()));
        ydata(col, row).validate();
      }
    }
  }
  
  // Filter outliers in y by mean and std dev
  outlierFactor = 3.0;
  double mean_y = maskedMean(ydata);
  double std_dev_y = maskedStdDev(ydata, mean_y);
  double rmin = mean_y - outlierFactor * std_dev_y;
  double rmax = mean_y + outlierFactor * std_dev_y;
  
  std::cout << "mean_y: " << mean_y << std::endl;
  std::cout << "std_dev_y: " << std_dev_y << std::endl;
  std::cout << "range for y: " << rmin << ' ' << rmax << std::endl;
  std::cout << "before filtring num of y is " << validCount(ydata) << std::endl;

  rangeFilter(ydata, rmin, rmax);

  std::cout << "after filtring num of y is " << validCount(ydata) << std::endl;
  
}

void run_nuth(Options const& opt) {

  // Load and prepare the data
  vw::ImageView<vw::PixelMask<double>> ref_copy, src_copy;
  double ref_nodata = -std::numeric_limits<double>::max();
  double src_nodata = ref_nodata;
  vw::cartography::GeoReference crop_georef;
  prepareData(opt, ref_copy, src_copy, ref_nodata, src_nodata, crop_georef);
  
  // Compute the Nuth offset
  computeNuthOffset(ref_copy, src_copy, crop_georef, ref_nodata, src_nodata,
                    opt.max_offset, opt.max_dz, opt.slope_lim);  
  
}

#if 0  
  // Write the slope
  {
  vw::TerminalProgressCallback src_tpc("asp", ": ");
  bool has_src_no_data = true, has_src_georef = true;
  std::string src_slope_file = opt.out_prefix + "-src-slope.tif";
  vw::vw_out() << "Writing: " << src_slope_file << "\n";
  vw::cartography::block_write_gdal_image(src_slope_file,
                                          vw::apply_mask(slope, src_nodata),
                                          has_src_georef, crop_georef,
                                          has_src_no_data, src_nodata,
                                          opt, src_tpc);
  }
  // Write the aspect
  {
  vw::TerminalProgressCallback src_tpc("asp", ": ");
  bool has_src_no_data = true, has_src_georef = true;
  std::string src_aspect_file = opt.out_prefix + "-src-aspect.tif";
  vw::vw_out() << "Writing: " << src_aspect_file << "\n";
  vw::cartography::block_write_gdal_image(src_aspect_file,
                                          vw::apply_mask(aspect, src_nodata),
                                          has_src_georef, crop_georef,
                                          has_src_no_data, src_nodata,
                                          opt, src_tpc);
  }
#endif  

int main(int argc, char *argv[]) {

  Options opt;
  try {

    // TODO(oalexan1): Need to convert from boost conventions to dem_align.py
    // conventions. Example: instead of --poly-order use -polyorder, and instead
    // of --max-offset use -max_offset, etc. 
    handle_arguments(argc, argv, opt);
    run_nuth(opt);

  } ASP_STANDARD_CATCHES;

  return 0;
}
