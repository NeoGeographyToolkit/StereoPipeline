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

#ifndef ASP_PCALIGN_NUTH_ALIGNMENT_PARSE_H
#define ASP_PCALIGN_NUTH_ALIGNMENT_PARSE_H

#include <string>
#include <vector>
#include <vw/Math/Vector.h>
#include <vw/FileIO/GdalWriteOptions.h>

namespace asp {

// Definition of the options struct
struct NuthOptions: vw::GdalWriteOptions {
  std::string ref, src, out_prefix, res;
  int poly_order, max_iter, inner_iter;
  double tol, max_horiz_offset, max_vert_offset, max_displacement;
  bool tiltcorr, compute_translation_only;
  vw::Vector2 slope_lim;
   NuthOptions() {}
};

// Parse the arguments. Some were set by now directly into opt.
void handleNuthArgs(int argc, char *argv[],  NuthOptions& opt);

// Given a command-line string, form argc and argv. In addition to pointers,
// will also store the strings in argv_str, to ensure permanence.
void formArgcArgv(std::string const& cmd,
                  int & argc,
                  std::vector<char*> & argv,
                  std::vector<std::string> & argv_str);

} // end namespace asp

#endif // ASP_PCALIGN_NUTH_ALIGNMENT_PARSE_H
