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
// Paper: https://tc.copernicus.org/articles/5/271/2011/

// TODO(oalexan1): Is it worth reading ref and source as double?
// TODO(oalexan1): Merely including Common.h results in 10 seconds extra compile time.
// TODO(oalexan1): Implement tilt correction.
// TODO(oalexan1): What to do about opt.res, opt.polyorder.

#include <asp/Core/Common.h>
#include <asp/PcAlign/NuthFit.h>
#include <asp/PcAlign/NuthAlignment.h>

#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Statistics.h>
#include <asp/PcAlign/MaskedImageAlgs.h>
#include <asp/PcAlign/SlopeAspect.h>
#include <vw/Core/Settings.h>
#include <vw/Math/RandomSet.h>
#include <vw/Math/Geometry.h>

#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <omp.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <thread>

namespace po = boost::program_options;

namespace asp {
  
struct Options: vw::GdalWriteOptions {
  std::string ref, src, out_prefix, res;
  int poly_order, max_iter, inner_iter;
  double tol, max_horiz_offset, max_vert_offset, max_displacement;
  bool tiltcorr, compute_translation_only;
  vw::Vector2 slope_lim;
  Options() {}
};

// Parse the arguments. Some were set by now directly into opt.
void handle_arguments(int argc, char *argv[], Options& opt) {

  double nan = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("General options");
  general_options.add_options()
    ("slope-lim", 
     po::value(&opt.slope_lim)->default_value(vw::Vector2(0.1, 40.0), "0.1, 40.0"),
     "Minimum and maximum surface slope limits to consider (degrees).")
    ("tol", po::value(&opt.tol)->default_value(0.01),
     "Stop when the addition to the computed translation at given iteration has magnitude "
     "below this tolerance (meters).")
    ("max-horizontal-offset", po::value(&opt.max_horiz_offset)->default_value(nan),
     "Maximum expected horizontal translation magnitude (meters). Used to filter outliers.")
    ("max-vertical-offset", po::value(&opt.max_vert_offset)->default_value(nan),
     "Maximum expected vertical translation (meters). Used to filter outliers.")
    ("num-inner-iter", po::value(&opt.inner_iter)->default_value(10),
      "Maximum number of iterations for the inner loop, when finding the best "
      "fit parameters for the current translation.")
    // The options below are not yet supported
    ("tiltcorr", po::bool_switch(&opt.tiltcorr)->default_value(false),
     "After a preliminary translation, fit a polynomial to residual elevation offsets "
     "and remove.")
    ("res", po::value(&opt.res)->default_value("mean"),
     "Regrid the input DEMs to this resolution given the resolutions of input datasets. "
     "Options: min, max, mean, common_scale_factor.")
    ("poly-order", po::value(&opt.poly_order)->default_value(1), 
     "Specify the order of the polynomial fit.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;
  std::string usage; // not used
   
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "The output prefix was not set.\n");

  // The ref and src DEMs must be provided
  if (opt.ref == "" || opt.src == "")
    vw::vw_throw(vw::ArgumentErr() << "The reference and source DEMs must be provided.\n");

  // The poly order must be positive
  if (opt.poly_order < 1)
    vw::vw_throw(vw::ArgumentErr() << "The polynomial order must be at least 1.\n");
  
  // The tol must be positive
  if (opt.tol <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "The tolerance must be positive.\n");

  // If horizontal and vertical offset are NaN (so, not set), use max_displacement
  if (std::isnan(opt.max_horiz_offset))
     opt.max_horiz_offset = opt.max_displacement;
  if (std::isnan(opt.max_vert_offset))
      opt.max_vert_offset = opt.max_displacement;
         
  // All these offsets must be positive
  if (opt.max_displacement <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "The maximum displacement must be positive.\n");
  if (opt.max_horiz_offset <= 0.0 || opt.max_vert_offset <= 0.0)
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

  if (opt.tiltcorr)
    vw::vw_throw(vw::NoImplErr() << "Tilt correction is not implemented yet.\n");
     
  // TODO(oalexan1): Sort out the number of threads. Now it comes from .vwrc if
  // not set.

  // Set the number of threads for OpenMP.  
  int processor_count = std::thread::hardware_concurrency();
  omp_set_dynamic(0);
  omp_set_num_threads(processor_count);
  vw::vw_out() << "Using " << processor_count << " threads with OpenMP.\n";
}

inline double DegToRad(double deg) {
  return deg * M_PI / 180.0;
}

// Put the above logic in a function that prepares the data.
void prepareData(Options const& opt, 
                  vw::ImageView<vw::PixelMask<float>> & ref,
                  vw::ImageView<vw::PixelMask<float>> & src,
                  double & ref_nodata, double & src_nodata,
                  vw::cartography::GeoReference & ref_georef,
                  vw::cartography::GeoReference & src_georef) {

  // The DEMs must have a georeference
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
      << "The reference DEM grid size is larger than of the source DEM. "
      << "This may lead to suboptimal results. It is strongly suggest to "
      << "swap or regrid the reference and source DEMs.\n";
  
  // The (1, 1) value in both transforms must be negative, as otherwise this a
  // non-standard transform that we do not handle.
  if (ref_georef.transform()(1, 1) >= 0 || src_georef.transform()(1, 1) >= 0)
    vw::vw_throw(vw::ArgumentErr() 
      << "The input DEMs must have y direction pointing down, so the second coordinate "
      << " of the pixel size in the geoheader be negative. The provided input DEMs "
      << " are not standard and are not supported.\n");

  // Read the no-data values
  vw::DiskImageResourceGDAL ref_rsrc(opt.ref), src_rsrc(opt.src);
  ref_nodata = -std::numeric_limits<double>::max();
  src_nodata = ref_nodata;
  if (ref_rsrc.has_nodata_read())
    ref_nodata = ref_rsrc.nodata_read();
  if (src_rsrc.has_nodata_read())
    src_nodata = src_rsrc.nodata_read(); 
  
  // Read the ref and source images fully in memory, for speed
  ref = copy(create_mask(vw::DiskImageView<float>(opt.ref), ref_nodata));

  // Get a handle to the source image on disk. We will warp it and read it in memory
  // as needed, when it is being shifted.
  src = copy(create_mask(vw::DiskImageView<float>(opt.src), src_nodata));
  
  // TODO(oalexan1): Crop the reference given the extent of the source
  // and estimated max movement.
#if 0  
  // Estimate the ground resolution
  double gridx = 0.0, gridy = 0.0;
  int sample_rate = std::min(ref.cols(), ref.rows()) / 50;
  if (sample_rate < 1) 
    sample_rate = 1;
  asp::calcGsd(vw::apply_mask(ref, ref_nodata), ref_georef, ref_nodata,
               sample_rate, sample_rate, gridx, gridy);
  double res = (gridx + gridy) / 2.0;
  if (res <= 0.0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "Could not estimate the DEM grid size. Check if your input clouds "
                 "are dense enough and with solid overlap.\n");
    
  int max_horiz_offset_px = int(max_horiz_offset/res) + 1;
#endif  
}

// Warp the source DEM to the reference DEM's georef, while applying a
// translation defined in the projected space of the reference DEM.
void shiftWarp(int ref_cols, int ref_rows, 
               vw::ImageView<vw::PixelMask<float>> const& src,
               vw::cartography::GeoReference const& ref_georef,
               vw::cartography::GeoReference const& src_georef,
               double dx_total, double dy_total, double dz_total,
               // Outputs
               vw::ImageView<vw::PixelMask<float>> & src_warp) {
  
  // Set up the interpolation
  vw::PixelMask<float> nodata_val;
  nodata_val.invalidate();
  auto nodata_ext = vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_val);
  vw::ImageViewRef<vw::PixelMask<float>> src_interp 
    = vw::interpolate(src, vw::BicubicInterpolation(), nodata_ext);
  src_warp.set_size(ref_cols, ref_rows);
  vw::cartography::GeoTransform gt(ref_georef, src_georef);
  
  // Interpolate  
  #pragma omp parallel for
  for (int col = 0; col < ref_cols; col++) {
    for (int row = 0; row < ref_rows; row++) {
      vw::Vector2 ref_pix(col, row);
      
      // Convert to projected coordinates
      vw::Vector2 proj_pt = ref_georef.pixel_to_point(ref_pix);
      
      // We want to shift src by this much in projected coordinates
      // So shift this proj pt in reverse and find the corresponding pixel in src
      proj_pt -= vw::Vector2(dx_total, dy_total);
      
      // Convert back to pixel coordinates, in the ref domain, as that
      // is where all operations take place.
      ref_pix = ref_georef.point_to_pixel(proj_pt);
      
      // go to src domain
      vw::Vector2 src_pix = gt.forward(ref_pix);
      src_warp(col, row) = src_interp(src_pix[0], src_pix[1]);
      
      // Add the vertical offset
      src_warp(col, row) += dz_total;
    }
  }

  return;
}

// Convert a translation in projected coordinates that aligns the source to the
// reference to a rotation + translation transform in ECEF. Make use of the
// filtered differences we employed to find the translation.
void calcEcefTransform(vw::ImageView<vw::PixelMask<float>> const& ref,
                       vw::ImageView<vw::PixelMask<float>> const& diff,
                       vw::cartography::GeoReference const& ref_georef,
                       double dx_total, double dy_total, double dz_total,
                       bool compute_translation_only,
                       // Outputs
                       Eigen::MatrixXd & ecef_transform) {
  
  std::vector<vw::Vector3> ref_pts, src_pts;
  
  // The images can be huge. A sample is enough here.
  int col_rate = std::max(ref.cols() / 1000, 1);
  int row_rate = std::max(ref.rows() / 1000, 1);
  #pragma omp parallel for
  for (int col = 0; col < ref.cols(); col += col_rate) {
    for (int row = 0; row < ref.rows(); row += row_rate) {
      
      vw::Vector2 ref_pix(col, row);
      vw::PixelMask<float> ht = ref(col, row);
      if (!is_valid(ht)) 
        continue;
      if (!is_valid(diff(col, row))) 
        continue;
      
      // Convert to projected coordinates, then to ECEF
      vw::Vector2 ref_pt = ref_georef.pixel_to_point(ref_pix);
      vw::Vector2 ref_lon_lat = ref_georef.point_to_lonlat(ref_pt);
      vw::Vector3 ref_ecef = ref_georef.datum().geodetic_to_cartesian
        (vw::Vector3(ref_lon_lat[0], ref_lon_lat[1], ht.child()));

      // Same for the source. Must go back to the source domain, in x, y, z.
      vw::Vector2 src_pt = ref_pt - vw::Vector2(dx_total, dy_total);
      vw::Vector2 src_lon_lat = ref_georef.point_to_lonlat(src_pt);
      vw::Vector3 src_ecef = ref_georef.datum().geodetic_to_cartesian
        (vw::Vector3(src_lon_lat[0], src_lon_lat[1], ht.child() - dz_total));
  
      #pragma omp critical
      {
        ref_pts.push_back(ref_ecef);
        src_pts.push_back(src_ecef);
      }
      
    }
  }
  
  // This will not be robust unless we have a lot of samples
  int minSamples = 10;
  if (ref_pts.size() < minSamples)
    vw::vw_throw(vw::ArgumentErr() 
                 << "Too few valid samples to compute the ECEF transform.\n");
  
  // A subset should be enough
  std::vector<int> sampleIndices;
  int maxSamples = 100000;  
  vw::math::pick_random_indices_in_range(ref_pts.size(), maxSamples, sampleIndices);
  
  // Keep only the samples in the vectors. Copy in-place.
  int numSamples = sampleIndices.size();
  for (int i = 0; i < numSamples; i++) {
    ref_pts[i] = ref_pts[sampleIndices[i]];
    src_pts[i] = src_pts[sampleIndices[i]];
  }
  ref_pts.resize(numSamples);
  src_pts.resize(numSamples);
  
  // It is safe to assume by now that there are no outliers in the data, given 
  // how much filtering we did.
  vw::Matrix<double, 3, 3> rotation;
  vw::Vector<double, 3> translation;
  double scale = 1.0;
  std::string transform_type = "rigid";
  if (compute_translation_only)
    transform_type = "translation";
  vw::math::find_3D_transform_no_outliers(src_pts, ref_pts,
                                          rotation, translation, scale, // outputs
                                          transform_type);
  
  // We do a rigid transform, as that's all Nuth supports.
  if (scale != 1.0)
    vw::vw_throw(vw::NoImplErr() << "Found an unexpected scale factor.\n");
    
  // Copy the rotation and translation
  ecef_transform.setIdentity(4, 4);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ecef_transform(i, j) = rotation(i, j);
    }
  }
  for (int i = 0; i < 3; i++)
    ecef_transform(i, 3) = translation[i];
  
  return;
}

// Compute the Nuth offset. By now the DEMs have been regrided to the same
// resolution and use the same projection. All data is kept as float,
// but the calculations are done in double precision, for accuracy.
void computeNuthOffset(Options const& opt,
                       vw::ImageView<vw::PixelMask<float>> const& ref,
                       vw::ImageView<vw::PixelMask<float>> const& src,
                       vw::cartography::GeoReference const& ref_georef,
                       vw::cartography::GeoReference const& src_georef,
                       double ref_nodata, double src_nodata,
                       double max_horiz_offset, double max_vert_offset, 
                       vw::Vector2 const& slope_lim,
                       double dx_total, double dy_total, double dz_total,
                       // Outputs
                       vw::ImageView<vw::PixelMask<float>> & diff,
                       vw::Vector3 & fit_params,
                       double & median_diff) {

  // Find shifted src, and interpolate it onto same grid as ref.
  vw::ImageView<vw::PixelMask<float>> src_warp;
  shiftWarp(ref.cols(), ref.rows(), src, ref_georef, src_georef,
            dx_total, dy_total, dz_total, 
            src_warp); // output
  
  // Ref and src must have the same size
  if (ref.cols() != src_warp.cols() || ref.rows() != src_warp.rows())
    vw::vw_throw(vw::ArgumentErr() 
                 << "The reference and source DEMs must have the same size.\n"); 
    
  // Find the diff
  diff.set_size(src_warp.cols(), src_warp.rows());
  for (int col = 0; col < src_warp.cols(); col++) {
    for (int row = 0; row < src_warp.rows(); row++) {
       diff(col, row) = src_warp(col, row) - ref(col, row);
    }
  }
   
  // Filter out diff whose magnitude is greater than max_vert_offset    
  // Apply normalized mad filter with given factor
  double outlierFactor = 3.0;
  vw::rangeFilter(diff, -max_vert_offset, max_vert_offset);
  
  vw::madFilter(diff, outlierFactor);
  
  // Calculate the slope and aspect of the src_warp_copy DEM
  vw::ImageView<vw::PixelMask<float>> slope, aspect;
  vw::cartography::calcSlopeAspect(src_warp, ref_georef, slope, aspect);
  
  // Filter slopes by range
  vw::rangeFilter(slope, slope_lim[0], slope_lim[1]);
  
  // Invalidate diffs with invalid slope or aspect
  vw::intersectValid(diff, slope);
  vw::intersectValid(diff, aspect);
  
  // Important sanity check
  int minCount = 100;
  if (validCount(diff) < minCount)
    vw::vw_throw(vw::ArgumentErr() 
                 << "Too little valid data left after filtering.\n");
  
  // Find the initial guess to fit_params
  median_diff = vw::maskedMedian(diff); // will return
  double median_slope = vw::maskedMedian(slope);
  double c_seed = (median_diff/tan(DegToRad(median_slope)));
  fit_params = vw::Vector3(0, 0, c_seed); // will update later and return

  // For invalid diff make the slope and aspect invalid
  vw::intersectValid(slope, diff);
  vw::intersectValid(aspect, diff);

  // Form xdata and ydata. We don't bother removing the flagged invalid pixels.
  // xdata = aspect[common_mask].data
  // ydata = (diff[common_mask]/np.tan(np.deg2rad(slope[common_mask]))).data
  // Make xdata an alias, to save on memory
  vw::ImageView<vw::PixelMask<float>> & xdata = aspect;
  vw::ImageView<vw::PixelMask<float>> ydata(src_warp.cols(), src_warp.rows());
  for (int col = 0; col < src_warp.cols(); col++) {
    for (int row = 0; row < src_warp.rows(); row++) {
      
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
  double mean_y = vw::maskedMean(ydata);
  double std_dev_y = vw::maskedStdDev(ydata, mean_y);
  double rmin = mean_y - outlierFactor * std_dev_y;
  double rmax = mean_y + outlierFactor * std_dev_y;
  
  // Filter y by range, and then apply to x
  vw::rangeFilter(ydata, rmin, rmax);
  vw::intersectValid(xdata, ydata);
  
  // Prepare for binning
  int numBins = 360;
  vw::Vector2 binRange(0.0, numBins);

  // Bin counts. Keep these are double as they are small.
  std::vector<double> bin_count, bin_edges, bin_centers;
  vw::binnedStatistics(xdata, ydata, "count", numBins, binRange, 
                       bin_count, bin_edges, bin_centers); // outputs  
  
  // Bin medians
  std::vector<double> bin_median;
  vw::binnedStatistics(xdata, ydata, "median", numBins, binRange, 
                       bin_median, bin_edges, bin_centers); // outputs
  
  // Find the best fit. This refines fit_params.
  asp::nuthFit(bin_count, bin_centers, bin_median, opt.inner_iter, opt.num_threads,
               fit_params);
} // End function computeNuthOffset

// Given a command-line string, form argc and argv. In addition to pointers,
// will also store the strings in argv_str, to ensure permanence.
void formArgcArgv(std::string const& cmd,
                  int & argc,
                  std::vector<char*> & argv,
                  std::vector<std::string> & argv_str) {

  // Break by space and read into argv
  argv_str.clear();
  argv_str.push_back("nuthAlignment"); // program name
  std::istringstream iss(cmd);
  std::string word;
  while (iss >> word) 
    argv_str.push_back(word);
  
  argc = argv_str.size();  
  argv.resize(argc);
  for (int i = 0; i < argc; i++)
    argv[i] = &argv_str[i][0];
  
  return;
} // End function formArgcArgv

// Compute the Nuth alignment transform in projected coordinates and then
// convert it to an ECEF transform.
Eigen::MatrixXd nuthAlignment(std::string const& ref_file, 
                              std::string const& src_file, 
                              std::string const& out_prefix, 
                              double max_displacement,
                              int num_iterations,
                              int num_threads,
                              bool compute_translation_only,
                              std::string const& nuth_options) { 

  // Pass some of these directly to opt
  Options opt;
  opt.ref = ref_file;
  opt.src = src_file;
  opt.out_prefix = out_prefix;
  opt.max_displacement = max_displacement;
  opt.max_iter = num_iterations;
  opt.num_threads = num_threads;
  opt.compute_translation_only = compute_translation_only;

  // Parse the others from nuth_options, or defaults are used.
  int argc = 0;
  std::vector<char*> argv;
  std::vector<std::string> argv_str;
  formArgcArgv(nuth_options, argc, argv, argv_str);
  handle_arguments(argc, &argv[0], opt);
  
  // If no iterations, return the identity
  Eigen::MatrixXd ecef_transform = Eigen::MatrixXd::Identity(4, 4);
  if (opt.max_iter == 0)
    return ecef_transform;
  
  // Load and prepare the data
  vw::ImageView<vw::PixelMask<float>> ref, src;
  double ref_nodata = -std::numeric_limits<double>::max();
  double src_nodata = ref_nodata;
  vw::cartography::GeoReference ref_georef, src_georef;
  // start stopwatch
  prepareData(opt, ref, src, ref_nodata, src_nodata, ref_georef, src_georef);
  
  // Initialize
  double dx_total = 0, dy_total = 0, dz_total = 0;
  vw::Vector3 fit_params;
  double median_diff = 0.0; // will be returned
  int iter = 1;
  double change_len = -1.0;
  vw::ImageView<vw::PixelMask<float>> diff;
  
  vw::vw_out() << "Iteration and change in transform (meters)\n";
  while (1) {
    
    // Compute the Nuth offset
    computeNuthOffset(opt, ref, src, ref_georef, src_georef, ref_nodata, src_nodata,
                      opt.max_horiz_offset, opt.max_vert_offset, opt.slope_lim,
                      dx_total, dy_total, dz_total, 
                      diff, fit_params, median_diff); // outputs
    
    // Note: minus signs here since we are computing dz = src-ref, but adjusting src
    double dx = -fit_params[0] * sin(DegToRad(fit_params[1]));
    double dy = -fit_params[0] * cos(DegToRad(fit_params[1]));
    double dz = -median_diff;
    
    dx_total += dx;
    dy_total += dy;
    dz_total += dz;
    
    change_len = vw::math::norm_2(vw::Vector3(dx, dy, dz));

    vw::vw_out() << iter << '\t' << change_len << "\n";
    
    iter++;
    if (iter > opt.max_iter) {
      vw::vw_out() << "Reached the maximum number of iterations, before convergence.\n";
      break;
    }
    
    if (change_len < opt.tol) {
      vw::vw_out() << "Reached the prescribed tolerance.\n";
      break;
    }
  }
  
  // Warning. Not sure if it should be an error.
  double horiz_total = vw::math::norm_2(vw::Vector2(dx_total, dy_total));
  if (horiz_total > opt.max_horiz_offset) 
    vw::vw_out(vw::WarningMessage)
      << "Total horizontal offset is: " << horiz_total << " meters. It exceeds the "
      << "specified max horizontal offset: " << opt.max_horiz_offset << " meters. Consider increasing the --max-horizontal-offset value.\n";
    
  // Compute the ECEF transform
  calcEcefTransform(ref, diff, ref_georef, dx_total, dy_total, dz_total, 
                    opt.compute_translation_only,
                    ecef_transform); // output 
     
   return ecef_transform;
}

#if 0  
  // Write the cropped and warped ref
  {
  vw::TerminalProgressCallback ref_tpc("asp", ": ");
  bool has_ref_no_data = true;
  std::string ref_crop_file = opt.out_prefix + "-ref-crop.tif"; 
  vw::vw_out() << "Writing: " << ref_crop_file << "\n";
  vw::cartography::block_write_gdal_image(ref_crop_file,
                                          vw::apply_mask(ref_dem, ref_nodata),
                                          has_ref_georef, ref_georef,
                                          has_ref_no_data, ref_nodata,
                                          opt, ref_tpc);
  }
    
  // Write the cropped src
  { 
  vw::TerminalProgressCallback src_tpc("asp", ": ");
  bool has_src_no_data = true;
  std::string src_crop_file = opt.out_prefix + "-src-crop.tif";
  vw::vw_out() << "Writing: " << src_crop_file << "\n";
  vw::cartography::block_write_gdal_image(src_crop_file,
                                          vw::apply_mask(src_trans, src_nodata),
                                          has_src_georef, ref_georef,
                                          has_src_no_data, src_nodata,
                                          opt, src_tpc);
  }

  // Write the slope
  {
  vw::TerminalProgressCallback src_tpc("asp", ": ");
  bool has_src_no_data = true, has_src_georef = true;
  std::string src_slope_file = opt.out_prefix + "-src-slope.tif";
  vw::vw_out() << "Writing: " << src_slope_file << "\n";
  vw::cartography::block_write_gdal_image(src_slope_file,
                                          vw::apply_mask(slope, src_nodata),
                                          has_src_georef, ref_georef,
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
                                          has_src_georef, ref_georef,
                                          has_src_no_data, src_nodata,
                                          opt, src_tpc);
  }
#endif  

} // end namespace asp
