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

/// \file ccd_solve.cc
///

// A solver to find an x and y CCD correction value for each column in
// an image by minimizing the disparity discontinuity. Hence it is
// assumed that two images in a stereo pair are given, with both
// having the same artifacts (so the same satellite, TDI, scan
// direction, and ideally also the same cross-track angle and
// direction of travel, which is usually North to South).

// The computation done by the solver is analogous to finding the
// per-column averaged disparity (the disp_avg tool), but the solver
// approach is more robust to noise as it uses Google CERES to
// attenuate outliers.

// Note that the disparity discontinuity is due to CCD artifacts in
// both the left and the right images. This tool works best when it
// uses as input multiple such disparities, when the contribution from
// the right image can be averaged away. One can also add to the mix
// both the left-to-right and right-to-left disparity for a given
// stereo pair.

// The parameters for this tool have been carefully tested and the
// defaults represent the best values found. The user may want to play
// with the sample rate. Picking about 100 lines from each disparity
// is about right.

// Usage: ccd_solve --disparities 'disp1-RD.tf ... dispN-RD.tif' -o run/run-solve

// The correction obtained from this tool can be applied with
// wv_correct with the --dx and --dy options. So far only corrections
// for WV03 TDI 32 and reverse scan direction have been
// computed. These have been stored for reference in the
// src/asp/WVCorrect directory.

// To work only on some clips, the disparities can be found by
// invoking stereo with the --left-image-crop-win and
// --right-image-crop-win options (same window should be passed to
// both, or at least the same left and right bounds to have correct
// book-keeping). Then, ccd_solve is used to find the corrections, and
// those are applied on corresponding clips cut out with
// gdal_translate using wv_correct. Stereo can be told to use those
// corrected clips by invoking it as above with --left-image-crop-win
// and --right-image-crop-win and the original images, but setting in
// addition the environmental variables LCROP and RCROP to point to
// the clips with the corrections applied to them (this way the
// original images are overridden at the right time in the flow).

// This solver was used successfully for the full image width (while
// using 12,000 rows from each of the left and right images) with
// either 3 or 6 disparities (3 stereo pairs, with left-to-right and
// right-to-left disparities for each). It fits in memory if one picks
// 100 samples for each column for each of the six disparities, which
// is enough to get a good result. It takes about 4 hours to run with
// 5 iterations and uses 20 GB of RAM.

// In addition to using cropped images in stereo, or independently of
// it, the disparities passed to ccd_solve can be further cropped to a
// smaller area by invoking ccd_solve with the --crop-win option. Once
// the corrections are found, with their number being the width of
// this crop window, they are padded with zero to make their size
// equal the width of the input disparities. This is recommended
// only for some sanity check experiments, normally one should
// use the full image width or at least a clip of at least 15,000
// columns, as the corrections for clips with smaller widths may
// not be good enough.

// To find good regions to work on to start with, use the
// find_bounds.cc tool. It will take as input a DEM clip and will
// return the left and right image crop wins corresponding to that
// clip. Those should be grown to have the same dimensions or at least
// the same left and right bounds (the lower and upper bounds, so the
// precise rows from each image, need not be the same).

#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/LinescanDGModel.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Core/Stopwatch.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace vw;

// A Ceres cost function. Measure how discontinuous the disparity is.
struct DisparityError {
  DisparityError(ImageViewRef< PixelMask<Vector2f> > disparity,
                 int col, int row): m_disparity(disparity), m_col(col), m_row(row) {}

  // takes an array of arrays. Each such array is a parameter block of a
  // size that is determined at run-time.
  bool operator()(double const * const * parameters, double * residuals) const {
    PixelMask<Vector2> disp1 = PixelMask<Vector2>(m_disparity(m_col, m_row))
      -  PixelMask<Vector2>(Vector2(parameters[0][0], parameters[1][0]));
    PixelMask<Vector2> disp2 = PixelMask<Vector2>(m_disparity(m_col + 1, m_row))
      -  PixelMask<Vector2>(Vector2(parameters[2][0], parameters[3][0]));

    PixelMask<Vector2> diff = disp1 - disp2;
    
    if (is_valid(diff)) {
      residuals[0] = diff.child()[0];
      residuals[1] = diff.child()[1];
    } else {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
    }
    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(ImageViewRef< PixelMask<Vector2f> > disparity,
                                     int col, int row){
    ceres::DynamicNumericDiffCostFunction<DisparityError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<DisparityError>
      (new DisparityError(disparity, col, row));
    
    // The residual size is always the same.
    cost_function->SetNumResiduals(2);

    // Set the size of each optimization block
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(1);
    return cost_function;
  }
  
private:
  ImageViewRef< PixelMask<Vector2f> > m_disparity;
  int m_col, m_row;
}; // End class DisparityError


// A ceres cost function. Minimize the sum of squares of CCD offsets
// with given weight
struct OffsetError {
  OffsetError(double weight): m_weight(weight) {}

  // Call to work with ceres::DynamicCostFunction. Takes an array of
  // arrays. Each such array is a parameter block of a size that is
  // determined at run-time.
  bool operator()(double const * const * parameters, double * residuals) const {
    residuals[0] = m_weight * parameters[0][0];
    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(double weight){
    ceres::DynamicNumericDiffCostFunction<OffsetError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<OffsetError>(new OffsetError(weight));
    
    // The residual size is always the same.
    cost_function->SetNumResiduals(1);
    cost_function->AddParameterBlock(1);
    return cost_function;
  }

private:
  double m_weight;
}; // End class OffsetError

// A ceres cost function. Minimize the sum of squares of second order
// derivatives of CCD offsets with given weight.
struct SmoothnessError {
  SmoothnessError(double weight): m_weight(weight) {}

  // Call to work with ceres::DynamicCostFunction. Takes an array of
  // arrays. Each such array is a parameter block of a size that is
  // determined at run-time.
  bool operator()(double const * const * parameters, double * residuals) const {
    residuals[0] = m_weight * (parameters[0][0] - 2 * parameters[1][0] + parameters[2][0])/2.0;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(double weight){
    ceres::DynamicNumericDiffCostFunction<SmoothnessError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<SmoothnessError>(new SmoothnessError(weight));
    
    // The residual size is always the same.
    cost_function->SetNumResiduals(1);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(1);
    return cost_function;
  }

private:
  double m_weight;
}; // End class SmoothnessError

struct Options : vw::cartography::GdalWriteOptions {
  std::string disparities, out_prefix, crop_win;
  double disparity_threshold, offset_weight, smoothness_weight, max_offset;
  int num_iterations, sample_rate;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("disparities",  po::value(&opt.disparities),
     "The input disparities.")
    ("disparity-threshold",  po::value(&opt.disparity_threshold)->default_value(0.1),
     "The robust threshold for disparity.")
    ("offset-weight",  po::value(&opt.offset_weight)->default_value(0.05),
     "A larger offset weight will keep the optimized offsets closer to 0.")
     ("smoothness-weight",  po::value(&opt.smoothness_weight)->default_value(10),
     "A larger smoothness weight will make the optimized offsets a smoother curve.")
    ("crop-win",  po::value(&opt.crop_win),
     "Left crop window (minx miny widx widy).")
    ("num-iterations",       po::value(&opt.num_iterations)->default_value(5),
     "Set the maximum number of iterations.") 
    ("sample-rate",       po::value(&opt.sample_rate)->default_value(200),
     "Pick one out of these many lines.")
    ("max-offset",  po::value(&opt.max_offset)->default_value(0.4),
     "Clamp the offsets to this value.")
    ("output-prefix,o",  po::value(&opt.out_prefix),
     "Prefix for output filenames.");

  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));
  
  po::options_description positional("");
  po::positional_options_description positional_desc;
  
  std::string usage("[options] <disparity> -o <output prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                             allow_unregistered, unregistered);
  
  if (opt.disparities == "") 
    vw::vw_throw( vw::ArgumentErr() << "The disparity is required.\n\n"
                  << usage << general_options);

  if (opt.out_prefix == "") 
    vw::vw_throw( vw::ArgumentErr() << "The output prefix is required.\n\n"
                  << usage << general_options);


  if (opt.offset_weight < 0 || opt.smoothness_weight < 0) 
    vw::vw_throw( vw::ArgumentErr() << "Expecting non-negative weights.\n\n"
                  << usage << general_options);

  if (opt.sample_rate <= 0) 
    vw::vw_throw( vw::ArgumentErr() << "Expecting positive sample rate.\n\n"
                  << usage << general_options);

  // Create the directory in which the output image will be written.
  vw::create_out_dir(opt.out_prefix);
}

int main(int argc, char* argv[]) {
  
  Options opt;
  try {

    handle_arguments(argc, argv, opt);

    int minx = 0, miny = 0, widx = 0, widy = 0;
    BBox2i crop_win;
    std::istringstream l_iss(opt.crop_win);
    if (l_iss >> minx >> miny >> widx >> widy) {
      crop_win.min() = Vector2(minx, miny);
      crop_win.max() = Vector2(minx + widx, miny + widy);
    }

    std::istringstream dss(opt.disparities);
    std::string disp_file;

    int num_full_cols = 0;
    
    // TODO(oalexan1): Here we assume that the crop window is
    // fully contained in the image.
    std::vector<double> x_offset(crop_win.width(), 0.0), y_offset(crop_win.width(), 0.0);

    // The ceres problem
    ceres::Problem problem;

    // Keep here handles to the disparities
    std::vector< ImageViewRef< PixelMask<Vector2f> > > disparity_vec;

    // Read the disparities and add them to the problem
    int count = -1;
    while (dss >> disp_file) {

      count++;
      
      vw_out() << "Reading " << disp_file << std::endl;
      DiskImageView< PixelMask<Vector2f> >full_disparity(disp_file);

      if (num_full_cols == 0) {
        num_full_cols = full_disparity.cols();
      } else {
        if (num_full_cols != full_disparity.cols())
          vw::vw_throw( vw::ArgumentErr()
                        << "All disparities must have the same number of columns.\n\n");
      }
      
      crop_win.crop(bounding_box(full_disparity));

      ImageViewRef< PixelMask<Vector2f> > disparity;
      if (!crop_win.empty()) 
        disparity = crop(full_disparity, crop_win);
      else
        disparity = full_disparity;

      if (crop_win.empty()) {
        x_offset.resize(disparity.cols(), 0.0);
        y_offset.resize(disparity.cols(), 0.0);
      }
      
      if (!crop_win.empty() && disparity.cols() != crop_win.width()) 
        vw::vw_throw( vw::ArgumentErr()
                      << "All cropped disparities must have the same number of columns.\n\n");
      
      disparity_vec.push_back(disparity);
      
      int cols = disparity_vec[count].cols(), rows = disparity_vec[count].rows();
    
      // Make the disparity less discontinuous
      for (int col = 0; col < cols - 1; col++) {
        for (int row = 0; row < rows; row += opt.sample_rate) {
          ceres::CostFunction* cost_function = DisparityError::Create(disparity_vec[count],
                                                                      col, row);
          ceres::LossFunction* loss_function = new ceres::CauchyLoss(opt.disparity_threshold);
          problem.AddResidualBlock(cost_function, loss_function,
                                   &x_offset[col], &y_offset[col],
                                   &x_offset[col + 1], &y_offset[col + 1]);
        }
      }
    }
    
    // Constrain the offsets to be small
    int cols = x_offset.size();
    
    for (int col = 0; col < cols; col++) {
      ceres::CostFunction* cost_function_x = OffsetError::Create(opt.offset_weight);
      ceres::LossFunction* loss_function_x = NULL;
      problem.AddResidualBlock(cost_function_x, loss_function_x, &x_offset[col]);
      
      ceres::CostFunction* cost_function_y = OffsetError::Create(opt.offset_weight);
      ceres::LossFunction* loss_function_y = NULL;
      problem.AddResidualBlock(cost_function_y, loss_function_y, &y_offset[col]);
    }
    
    // Constrain the offsets to be smooth
    for (int col = 1; col < cols - 1; col++) {
      ceres::CostFunction* cost_function_x = SmoothnessError::Create(opt.smoothness_weight);
      ceres::LossFunction* loss_function_x = NULL;
      problem.AddResidualBlock(cost_function_x, loss_function_x,
                               &x_offset[col - 1], &x_offset[col], &x_offset[col + 1]);
      
      ceres::CostFunction* cost_function_y = SmoothnessError::Create(opt.smoothness_weight);
      ceres::LossFunction* loss_function_y = NULL;
      problem.AddResidualBlock(cost_function_y, loss_function_y,
                               &y_offset[col - 1], &y_offset[col], &y_offset[col + 1]);
    }
    
    Stopwatch sw;
    sw.start();
    // Solve the problem
    ceres::Solver::Options options;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.max_num_iterations = opt.num_iterations;
    options.max_num_consecutive_invalid_steps = 100; // try hard
    options.minimizer_progress_to_stdout = true;
    options.num_threads = opt.num_threads;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    //options.ordering_type = ceres::SCHUR;
    //options.eta = 1e-3; // FLAGS_eta;
    //options->max_solver_time_in_seconds = FLAGS_max_solver_time;
    //options->use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
    //if (FLAGS_line_search) 
    //  options->minimizer_type = ceres::LINE_SEARCH;
    
    vw_out() << "Solving the problem." << std::endl;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    vw_out() << summary.FullReport() << "\n";
    if (summary.termination_type == ceres::NO_CONVERGENCE){
      // Print a clarifying message, so the user does not think that the algorithm failed.
      vw_out() << "Found a valid solution, but did not reach the actual minimum." << std::endl;
    }
    
    double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> full_x(num_full_cols, nan);
    std::vector<double> full_y(num_full_cols, nan);
    if (crop_win.empty()) {
      full_x = x_offset;
      full_y = y_offset;
    } else {
      int start = crop_win.min().x();
      int width = crop_win.width();
      for (size_t col = 0; col < x_offset.size(); col++) {
        full_x[col + start] = std::min(std::max(x_offset[col], -opt.max_offset), opt.max_offset);
        full_y[col + start] = std::min(std::max(y_offset[col], -opt.max_offset), opt.max_offset);
      }
    }
    
    // Save coeffs to disk
    std::string dx = opt.out_prefix + "-dx.txt";
    vw_out() << "Writing: " << dx << std::endl;
    std::ofstream ox(dx.c_str());
    ox.precision(18);
    for (size_t it = 0; it < full_x.size(); it++) 
    ox << full_x[it] << std::endl;
    ox.close();
    
    std::string dy = opt.out_prefix + "-dy.txt";
    vw_out() << "Writing: " << dy << std::endl;
    std::ofstream oy(dy.c_str());
    oy.precision(18);
    for (size_t it = 0; it < full_y.size(); it++) 
      oy << full_y[it] << std::endl;
    oy.close();
    
    sw.stop();
    vw::vw_out() << "CCD solve elapsed time: " << sw.elapsed_seconds() << std::endl;
    
  } ASP_STANDARD_CATCHES;
}
