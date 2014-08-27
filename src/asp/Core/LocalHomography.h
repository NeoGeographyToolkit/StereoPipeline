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


/// \file LocalHomography.h
///

#ifndef __LOCAL_DISPARITY_H__
#define __LOCAL_DISPARITY_H__

#include <vw/Image/ImageView.h>
#include <vw/Math/Matrix.h>
#include <vector>

// Forward declaration
namespace asp {
  class Options;
}

namespace asp {

  void split_n_into_k(int n, int k, std::vector<int> & partition);

  void create_local_homographies(Options const& opt);

  void write_local_homographies(std::string const& local_hom_file,
                                vw::ImageView<vw::Matrix3x3> const& local_hom);
  void read_local_homographies(std::string const& local_hom_file,
                               vw::ImageView<vw::Matrix3x3> & local_hom);


} // namespace asp

#endif
