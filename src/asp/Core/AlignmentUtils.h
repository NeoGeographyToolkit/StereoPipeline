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

/// \file AlignmentUtils.h

#ifndef __ALIGNMENT_UTILS_H__
#define __ALIGNMENT_UTILS_H__

#include <vw/Math/BBox.h>
#include <vw/Math/Matrix.h>

namespace asp {
  
  std::string leftAlignFile(std::string const& out_prefix);
  std::string rightAlignFile(std::string const& out_prefix);

  // These are kept for backward compatibility and will be removed
  // in the future.
  std::string leftAlignExrFile(std::string const& out_prefix);
  std::string rightAlignExrFile(std::string const& out_prefix);
 
 // Read the left or right alignment matrix from the file  
 vw::Matrix<double> alignmentMatrix(std::string const& out_prefix,
                                    std::string const& alignment_method,
                                    std::string const& side);

} // end namespace asp

#endif // __ALIGNMENT_UTILS_H__