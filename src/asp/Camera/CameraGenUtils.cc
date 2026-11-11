// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file CameraGenUtils.cc

#include <asp/Camera/CameraGenUtils.h>
#include <asp/Camera/CsmModel.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/FileIO/FileTypes.h>
#include <vw/Math/Matrix.h>
#include <vw/Core/Exception.h>

#include <boost/make_shared.hpp>

#include <cmath>

using namespace vw;

namespace asp {

// Build an initial camera (pinhole or optical bar) from the given intrinsics or
// a sample camera file. For a CSM (.json) sample, read its intrinsics back.
void manufacture_cam(std::string const& camera_type,
                     std::string const& sample_file,
                     int wid, int hgt,
                     double & focal_length,
                     double & pixel_pitch,
                     vw::Vector2 & optical_center,
                     std::vector<double> & distortion,
                     std::string & distortion_type,
                     boost::shared_ptr<vw::CameraModel> & out_cam) {
  if (camera_type == "opticalbar") {
    boost::shared_ptr<vw::OpticalBarModel> opticalbar_cam =
      boost::make_shared<vw::OpticalBarModel>(sample_file);
    // Make sure the image size matches the input image file.
    // TODO(oalexan1): This looks fishy if the pitch is not 1.
    opticalbar_cam->set_image_size(Vector2i(wid, hgt));
    opticalbar_cam->set_optical_center(Vector2(wid/2.0, hgt/2.0));
    out_cam = opticalbar_cam;
  } else if (camera_type == "pinhole") { // csm frame comes here too

    std::string sample_ext = vw::get_extension(sample_file);
    if (sample_file != "" && sample_ext == ".json") {
      // Read the intrinsics from the csm file. The distortion will be set later.
      asp::CsmModel csm(sample_file);
      focal_length = csm.focal_length();
      pixel_pitch = csm.frame_pixel_pitch();
      optical_center = csm.optical_center() * csm.frame_pixel_pitch();
      // TODO(oalexan1): Must copy the target name from CSM
      distortion = csm.distortion();
      if (distortion.size() == 5)
        distortion_type = "radtan";
      else if (distortion.size() == 3)
        distortion_type = "radial";
      else if (distortion.size() == 20)
        distortion_type = "transverse";
      else
        vw_throw(ArgumentErr()
                 << "Unsupported distortion model in: " << sample_file << ".\n");
    }

    boost::shared_ptr<PinholeModel> pinhole_cam;
    if (sample_file != "" &&  sample_ext != ".json") {
      // Use the initial guess from file
      pinhole_cam = boost::make_shared<PinholeModel>(sample_file);
    } else {
      // Use the intrinsics from the command line or read from the CSM sample
      // file. Use trivial rotation and translation.
      Vector3 ctr(0, 0, 0);
      Matrix<double, 3, 3> rotation;
      rotation.set_identity();
      // When the user does not set the optical center, use the image center
      Vector2 opt_ctr = optical_center;
      if (std::isnan(opt_ctr[0]) || std::isnan(opt_ctr[1]))
        opt_ctr = Vector2(pixel_pitch * wid/2.0, pixel_pitch * hgt/2.0);

      vw::LensDistortion const* dist = NULL; // no distortion info yet
      pinhole_cam = boost::make_shared<PinholeModel>
                      (ctr, rotation, focal_length, focal_length,
                       opt_ctr[0], opt_ctr[1], dist,
                       pixel_pitch);
    }
    out_cam = pinhole_cam;
  }
}

} // end namespace asp
