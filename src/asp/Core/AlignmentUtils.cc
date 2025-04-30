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
  return out_prefix + "-align-L.txt";
}

std::string rightAlignFile(std::string const& out_prefix) {
  return out_prefix + "-align-R.txt"; 
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
 
  std::string align_file, align_file_exr;
  if (side == "left") {
    align_file = leftAlignFile(out_prefix);
    align_file_exr = leftAlignExrFile(out_prefix);
  } else if (side == "right") {
    align_file = rightAlignFile(out_prefix);
    align_file_exr = rightAlignExrFile(out_prefix);
  } else {
    vw::vw_throw(vw::ArgumentErr() << "Invalid side: " << side << "\n");
  }
  
  // Initalize with the identity matrix      
  vw::Matrix<double> align_matrix = vw::math::identity_matrix<3>();

  if (alignment_method == "homography" ||
      alignment_method == "affineepipolar" ||
      alignment_method == "local_epipolar") {
    if (fs::exists(align_file)) // read alignment matrix from text file
      vw::read_matrix_as_txt(align_file, align_matrix);
    else if (fs::exists(align_file_exr)) // read exr, for backward compatibility
      vw::read_matrix(align_matrix, align_file_exr);
    else
      vw::vw_throw(vw::IOErr() << "Could not read alignment matrix from: "
                 << align_file << " or " << align_file_exr << "\n");
    
  }

  return align_matrix;
}

} // namespace asp
