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

// Implement least square alignment using the FGR method

#include <asp/PcAlign/pc_align_fgr.h>
#include <asp/PcAlign/pc_align_utils.h>
#include <asp/Core/EigenUtils.h>

// Can't do much about external warnings except hide them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <FastGlobalRegistration/app.h>
#pragma GCC diagnostic pop

namespace asp {

using namespace vw;

// Parse a string like:
// div_factor: 1.4 use_absolute_scale: 0 max_corr_dist: 0.025 iteration_number: 100 tuple_scale: 0.95 tuple_max_cnt: 10000
void parse_fgr_options(std::string const & options,
                       double            & div_factor,
                       bool              & use_absolute_scale,
                       double            & max_corr_dist,
                       int               & iteration_number,
                       float             & tuple_scale,
                       int               & tuple_max_cnt){

  // Initialize the outputs
  div_factor         = -1;
  use_absolute_scale = false;
  max_corr_dist      = -1;
  iteration_number   = -1;
  tuple_scale        = -1;
  tuple_max_cnt      = -1;

  std::istringstream is(options);
  std::string name, val;
  while( is >> name >> val){
    if (name.find("div_factor") != std::string::npos)
      div_factor = atof(val.c_str());
    if (name.find("use_absolute_scale") != std::string::npos)
      use_absolute_scale = atof(val.c_str());
    if (name.find("max_corr_dist") != std::string::npos)
      max_corr_dist = atof(val.c_str());
    if (name.find("iteration_number") != std::string::npos)
      iteration_number = atof(val.c_str());
    if (name.find("tuple_scale") != std::string::npos)
      tuple_scale = atof(val.c_str());
    if (name.find("tuple_max_cnt") != std::string::npos)
      tuple_max_cnt = atof(val.c_str());
  }
  
  // Sanity check
  if (div_factor <= 0 || max_corr_dist < 0 || iteration_number < 0 || tuple_scale <= 0 ||
      tuple_max_cnt <= 0) {
    vw_throw( ArgumentErr() << "Could not parse correctly --fgr-options.");
  }
}
  
// Convert a point clould to the format expected by FGR
void export_to_fgr(DP const & data, fgr::Points& pts, fgr::Feature & feat){

  pts.clear();
  feat.clear();
  for (int c = 0; c < data.features.cols(); c++){

    Eigen::Vector3f pts_v;
    for (int r = 0; r < 3; r++) pts_v[r] = data.features(r, c);

    pts.push_back(pts_v);

    // fgr expects features in addition to points. This works well enough,
    // but need to get to the bottom of whether they are necessary.
    feat.push_back(pts_v); 
  }
  
}
  
/// Compute alignment using FGR
Eigen::MatrixXd fgr_alignment(DP const & source_point_cloud, 
                                          DP const & ref_point_cloud, 
                                          std::string const& fgr_options) {

  // Parse the options and initialize the FGR object
  double  div_factor; 
  bool    use_absolute_scale;
  double  max_corr_dist;
  int     iteration_number;
  float   tuple_scale;
  int     tuple_max_cnt;
  parse_fgr_options(fgr_options, div_factor, use_absolute_scale, max_corr_dist,
                    iteration_number, tuple_scale, tuple_max_cnt);
  fgr::CApp app(div_factor, use_absolute_scale, max_corr_dist, iteration_number,  
                tuple_scale, tuple_max_cnt);

  // Intermediate data
  fgr::Points pts;
  fgr::Feature feat;

  // Pass the reference cloud to FGR
  export_to_fgr(ref_point_cloud, pts, feat);
  app.LoadFeature(pts, feat);

  // Pass the source cloud to FGR
  export_to_fgr(source_point_cloud, pts, feat);
  app.LoadFeature(pts, feat);

  // Perform alignment
  app.NormalizePoints();
  app.AdvancedMatching();
  app.OptimizePairwise(true);
  Eigen::Matrix4f S = app.GetOutputTrans();

  // Export the transform
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(DIM + 1, DIM + 1);
  if (T.cols() != S.cols() || T.rows() != S.rows()) 
    vw_throw( LogicErr() << "Error: size mis-match in FGR.\n");
  for (int row = 0; row < T.rows(); row++) {
    for (int col = 0; col < T.cols(); col++) {
      T(row, col) = S(row, col);
    }
  }

  return T;
}
    
} // end namespace asp
