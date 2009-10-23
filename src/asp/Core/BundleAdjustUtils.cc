// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file BundleAdjustUtils.cc
///

#include <asp/Core/BundleAdjustUtils.h>

#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ip;

void read_adjustments(std::string const& filename,
                      Vector3& position_correction,
                      Quaternion<double>& pose_correction) {
  std::ifstream istr(filename.c_str());
  istr >> position_correction[0] >> position_correction[1]
       >> position_correction[2];
  istr >> pose_correction.w() >> pose_correction.x()
       >> pose_correction.y() >> pose_correction.z();
}

void write_adjustments(std::string const& filename,
                       Vector3 const& position_correction,
                       Quaternion<double> const& pose_correction) {
  std::ofstream ostr(filename.c_str());
  ostr << position_correction[0] << " " << position_correction[1] << " " << position_correction[2] << "\n";
  ostr << pose_correction.w() << " " << pose_correction.x() << " "
       << pose_correction.y() << " " << pose_correction.z() << " " << "\n";
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

      StereoModel sm( camera_models[cam1].get(),
                      camera_models[cam2].get() );
      double error;
      Vector3 pos = sm(pix1,pix2,error);
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
  }
  vw_out(0) << "\nStereo Intersection Residuals -- Min: " << min_error
            << "  Max: " << max_error << "  Average: " << (error_sum/n) << "\n";
}
