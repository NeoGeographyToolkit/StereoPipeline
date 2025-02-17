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

/// \file NuthAlignment.h
/// Nuth and Kaab alignment.

#ifndef __ASP_PC_ALIGN_NUTH_ALIGNMENT_H__
#define __ASP_PC_ALIGN_NUTH_ALIGNMENT_H__

#include <Eigen/Dense>
#include <vector>

namespace asp {

Eigen::MatrixXd nuthAlignment(std::string const& ref_file, 
                              std::string const& src_file, 
                              std::string const& output_prefix, 
                              double max_displacement,
                              int num_iterations,
                              int num_threads,
                              bool compute_translation_only,
                              std::string const& nuth_options);

} // end namespace asp

#endif // __ASP_PC_ALIGN_NUTH_ALIGNMENT_H__
