// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file AlignmentUtils.cc

#include <asp/Core/AlignmentUtils.h>
#include <vw/FileIO/MatrixIO.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace asp {

std::string leftAlignFile(std::string const& out_prefix) {
  std::cout << "---will return align-L.exr\n";
  return out_prefix + "-align-L.exr";
}

std::string rightAlignFile(std::string const& out_prefix) {
  std::cout << "--will return align-R.exr\n";
  return out_prefix + "-align-R.exr"; 
}

// These are kept for backward compatibility and will be removed
// in the future.

std::string leftAlignExrFile(std::string const& out_prefix) {
  return out_prefix + "-align-L.exr";
}

std::string rightAlignExrFile(std::string const& out_prefix) {
  return out_prefix + "-align-R.exr"; 
}

// Read the left or right alignment matrix from the file  
vw::Matrix<double> alignmentMatrix(std::string const& out_prefix,
                                   std::string const& alignment_method,
                                   std::string const& side) {
 
  std::string align_file;
  if (side == "left")
    align_file = leftAlignFile(out_prefix);
  else if (side == "right")
    align_file = rightAlignFile(out_prefix);
  else
    vw::vw_throw(vw::ArgumentErr() << "Invalid side: " << side << "\n");

  // Initalize with the identity matrix      
  vw::Matrix<double> align_matrix = vw::math::identity_matrix<3>();

  if (alignment_method == "homography" ||
      alignment_method == "affineepipolar" ||
      alignment_method == "local_epipolar") {
    if (!fs::exists(align_file))
      vw::vw_throw(vw::ArgumentErr() << "Could not read: " << align_file);
    vw::read_matrix(align_matrix, align_file);
  }

  return align_matrix;
}

} // namespace asp
