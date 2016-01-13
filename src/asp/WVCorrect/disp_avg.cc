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

#include <vw/FileIO/DiskImageView.h>
#include <vw/Stereo/DisparityMap.h>

using namespace vw;
using namespace std;

// Average the rows in a given disparity image. Save them to disk as
// two text files (x and y values), with as many entries as there
// were columns in the disparity.

int main( int argc, char *argv[] ){

  // TODO: Use Boost program options.

  // TODO: No need for outdx.txt and outdy.txt, just save with with same prefix.
  if (argc <= 3) {
    vw_out() << "Usage: disp_avg disp.tif outdx.txt outdy.txt [row_start] [num_rows] [col_start] [num_cols]\n";
    return 1;
  }

  std::string in_file = argv[1], outx = argv[2], outy = argv[3];

  std::cout << "Reading: " << in_file << std::endl;
  DiskImageView < PixelMask<Vector2f> > D(in_file);

  int cols = D.cols(), rows = D.rows();
  std::cout << "Number of cols and rows is " << cols << ' ' << rows << std::endl;

  // Handle optional row ROI arguments.
  // - No col ROI since we want the entire column range.
  int row_start = 0;
  int row_stop  = rows;
  int col_start = 0;
  int col_stop  = cols;
  if (argc > 4) row_start = atoi(argv[4]);
  if (argc > 5) row_stop  = row_start + atoi(argv[5]);
  if (argc > 6) col_start = atoi(argv[6]);
  if (argc > 7) col_stop  = col_start + atoi(argv[7]);

  // TODO: Add a progress bar.
  vector<double> Dx(cols, 0), Dy(cols, 0); // Always full sized, even if crop is used.
  for (int col = col_start; col < col_stop; col++){
    if (col%100 == 0){
      std::cout << "column " << col << std::endl;
    }
    vector<double> px, py;
    for (int row = row_start; row < row_stop; row++){
      PixelMask<Vector2f> p = D(col, row);
      if (! is_valid(p)) continue;
      px.push_back(p.child()[0]);
      py.push_back(p.child()[1]);
    }
    std::sort(px.begin(), px.end());
    std::sort(py.begin(), py.end());
    int len = px.size();
    int num_valid = 0;
    Dx[col] = 0;
    Dy[col] = 0;
    for (int k = 0; k < len; k++){
      num_valid++;
      Dx[col] += px[k];
      Dy[col] += py[k];
    }

    if (num_valid > 0){
      Dx[col] /= num_valid;
      Dy[col] /= num_valid;
    }
  }

  // Write dx file
  ofstream dx(outx.c_str());
  dx.precision(16);
  std::cout << "Writing: " << outx << std::endl;
  dx << col_start << std::endl << col_stop << std::endl; // Column crop on header lines
  for (int col = 0; col < cols; col++) 
    dx << Dx[col] << std::endl;

  // Write dy file
  ofstream dy(outy.c_str());
  dy.precision(16);
  std::cout << "Writing: " << outy << std::endl;
  dy << col_start << std::endl << col_stop << std::endl; // Column crop on header lines
  for (int col = 0; col < cols; col++) 
    dy << Dy[col] << std::endl;

  return 0;
}
