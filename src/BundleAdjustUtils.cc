// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file BundleAdjustUtils.cc
///

#include "BundleAdjustUtils.h"

#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ip;

void read_adjustments(std::string const& filename, Vector3& position_correction, Quaternion<double>& pose_correction) {
  std::ifstream istr(filename.c_str());
  istr >> position_correction[0] >> position_correction[1] >> position_correction[2];
  istr >> pose_correction.w() >> pose_correction.x() >> pose_correction.y() >> pose_correction.z();
}

void write_adjustments(std::string const& filename, Vector3 const& position_correction, Quaternion<double> const& pose_correction) {
  std::ofstream ostr(filename.c_str());
  ostr << position_correction[0] << " " << position_correction[1] << " " << position_correction[2] << "\n";
  ostr << pose_correction.w() << " " << pose_correction.x() << " " << pose_correction.y() << " " << pose_correction.z() << " " << "\n";
}

void compute_stereo_residuals(std::vector<boost::shared_ptr<CameraModel> > const& camera_models,
                              ControlNetwork const& cnet) {

  // Compute pre-adjustment residuals and convert to bundles
  int n = 0;
  double error_sum = 0; 
  double min_error = ScalarTypeLimits<double>::highest();
  double max_error = ScalarTypeLimits<double>::lowest();
  for (unsigned i = 0; i < cnet.size(); ++i) {
    for (unsigned j = 0; j+1 < cnet[i].size(); ++j) {
      ++n;
      int cam1 = cnet[i][j].image_id();
      int cam2 = cnet[i][j+1].image_id();
      Vector2 pix1 = cnet[i][j].position();
      Vector2 pix2 = cnet[i][j+1].position();

      StereoModel sm(*(camera_models[cam1]), *(camera_models[cam2]));
      double error;
      Vector3 pos = sm(pix1,pix2,error);
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
  }  
  std::cout << "\nStereo Intersection Residuals -- Min: " << min_error << "  Max: " << max_error << "  Average: " << (error_sum/n) << "\n";
}
