// __BEGIN_LICENSE__
//  Copyright (c) 2006-2012, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Image/Statistics.h>

using namespace vw;

namespace po = boost::program_options;

// Average the rows in a given disparity image. Save them to disk as
// two text files (x and y values), with as many entries as there
// were columns in the disparity.

struct Options : vw::GdalWriteOptions {
  std::string disparity, dx, dy;
  int beg_row, end_row;
  Vector2     remove_outliers_params;
  bool        save_no_metadata;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("remove-outliers-params", po::value(&opt.remove_outliers_params)->default_value(Vector2(95.0, 3.0), "pct factor"))
    ("beg-row",  po::value(&opt.beg_row)->default_value(-1), "Start averaging the disparity from this row.")
    ("end-row",  po::value(&opt.end_row)->default_value(-1), "Stop the disparity before this row.")
    ("save-no-metadata", po::bool_switch(&opt.save_no_metadata)->default_value(false),
     "Do not save in the output correction file the start and end image columns (not saving this makes processing easier).");
  
  general_options.add(vw::GdalWriteOptionsDescription(opt));
  
  po::options_description positional("");
  positional.add_options()
    ("disparity", po::value(&opt.disparity))
    ("dx", po::value(&opt.dx))
    ("dy", po::value(&opt.dy));
  
  po::positional_options_description positional_desc;
  positional_desc.add("disparity", 1);
  positional_desc.add("dx", 1);
  positional_desc.add("dy", 1);
  
  std::string usage("[options] <disparity> <dx.txt> <dy.txt>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  
  if ( !vm.count("disparity") || !vm.count("dx") || !vm.count("dy") )
    vw::vw_throw( vw::ArgumentErr() << "Not all inputs were specified.\n\n"
                  << usage << general_options );
  
  // Create the directory in which the output image will be written.
  vw::create_out_dir(opt.dx);
}

int main( int argc, char *argv[] ){

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    DiskImageView < PixelMask<Vector2f> > D(opt.disparity);

    int cols = D.cols(), rows = D.rows();

    // Handle optional row ROI arguments.
    // - No col ROI since we want the entire column range.
    int beg_row = 0;
    int end_row  = rows;
    int col_start = 0;
    int col_stop  = cols;
    if (opt.beg_row >= 0 && opt.end_row > opt.beg_row) {
      beg_row = std::min(opt.beg_row, rows);
      end_row = std::min(opt.end_row, rows);
    }
    TerminalProgressCallback disp_progress("asp", "\tAveraging:   ");
    double disp_progress_mult = 1.0/double(std::max(col_stop - col_start, 1));

    std::vector<double> Dx(cols, 0), Dy(cols, 0);
    
    for (int col = col_start; col < col_stop; col++){
      disp_progress.report_progress((col - col_start) * disp_progress_mult);

      std::vector<double> px, py;
      for (int row = beg_row; row < end_row; row++){
        PixelMask<Vector2f> p = D(col, row);
        if (!is_valid(p)) continue;
        px.push_back(p.child()[0]);
        py.push_back(p.child()[1]);
      }

      std::sort(px.begin(), px.end());
      std::sort(py.begin(), py.end());
      
      // Being too strict with outlier removal can cause trouble
      double pct_factor     = opt.remove_outliers_params[0]/100.0;
      double outlier_factor = opt.remove_outliers_params[1];
      
      double bx = 0, ex = 0, by = 0, ey = 0;
      if (!vw::math::find_outlier_brackets(px, pct_factor, outlier_factor, bx, ex))
        continue;
      if (!vw::math::find_outlier_brackets(py, pct_factor, outlier_factor, by, ey))
        continue;

      int len = px.size();
      int num_valid = 0;
      Dx[col] = 0;
      Dy[col] = 0;
      for (int k = 0; k < len; k++){

        if (px[k] < bx || px[k] > ex || py[k] < by || py[k] > ey) continue;

        num_valid++;
        Dx[col] += px[k];
        Dy[col] += py[k];
      }
      
      if (num_valid > 0){
        Dx[col] /= num_valid;
        Dy[col] /= num_valid;
      }
    }
    
    disp_progress.report_finished();
  
    // Write dx file
    std::ofstream dx(opt.dx.c_str());
    dx.precision(16);
    std::cout << "Writing: " << opt.dx << std::endl;
    if (!opt.save_no_metadata) {
      dx << col_start << std::endl;
      dx << col_stop  << std::endl;
    }
    for (int col = 0; col < cols; col++) 
      dx << Dx[col] << std::endl;
    dx.close();
  
    // Write dy file
    std::ofstream dy(opt.dy.c_str());
    dy.precision(16);
    std::cout << "Writing: " << opt.dy << std::endl;
    if (!opt.save_no_metadata) {
      dy << col_start << std::endl;
      dy << col_stop  << std::endl;
    }
    for (int col = 0; col < cols; col++) 
      dy << Dy[col] << std::endl;
    dy.close();

  } ASP_STANDARD_CATCHES;
  
  return 0;
}
