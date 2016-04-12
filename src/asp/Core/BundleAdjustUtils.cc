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


/// \file BundleAdjustUtils.cc
///

#include <asp/Core/BundleAdjustUtils.h>

#include <vw/Core/Log.h>
#include <vw/Camera/CameraModel.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Stereo/StereoModel.h>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ba;

std::string g_piecewise_adj_str = "PIECEWISE_ADJUSTMENTS";
std::string g_session_str = "SESSION";

void asp::read_adjustments(std::string const& filename,
                           bool & piecewise_adjustments,
                           vw::Vector2 & adjustment_bounds,
                           std::vector<Vector3> & position_correction,
                           std::vector<Quat> & pose_correction,
                           std::string & session) {

  // Initialize the outputs
  piecewise_adjustments = false;
  adjustment_bounds = Vector2();
  position_correction.clear();
  pose_correction.clear();
  session = "dg"; // default session, for historical reasons
  
  Vector3 pos;
  Vector4 q_buf;
  std::ifstream istr(filename.c_str());

  // Peek to see if the file contains piecewise adjustments
  std::string line;
  if (!std::getline(istr, line))
    vw_throw( ArgumentErr() << "Could not read adjustment file: " << filename << "\n" );
  if (line == g_piecewise_adj_str) {
    piecewise_adjustments = true;

    // Read the session
    std::string a, b;
    if (istr >> a >> b) {
      if (a == g_session_str) {
        session = b;
      }
    }
    
    if (! (istr >> adjustment_bounds[0] >> adjustment_bounds[1]))
      vw_throw( ArgumentErr() << "Could not read adjustment bounds from: " << filename << "\n");
  }else{
    // No piecewise adjustments. Rewind to beginning.
    piecewise_adjustments = false;
    istr.clear();
    istr.seekg(0, std::ios::beg);
  }

  // Read the actual adjustments
  while (1){
    if (! (istr >> pos[0] >> pos[1] >> pos[2]) ) break;
    if (! (istr >> q_buf[0] >> q_buf[1] >> q_buf[2] >> q_buf[3]) ) break;

    position_correction.push_back(pos);
    pose_correction.push_back(Quat(q_buf));
  }
}

// Write piecewise adjustments
void asp::write_adjustments(std::string const& filename,
                            vw::Vector2 const& adjustment_bounds,
                            std::vector<vw::Vector3> const& position_correction,
                            std::vector<vw::Quat> const& pose_correction,
                            std::string const& session) {

  std::ofstream ostr(filename.c_str());
  ostr.precision(18);

  ostr << g_piecewise_adj_str << std::endl;
  ostr << g_session_str << " " << boost::to_lower_copy(session) << std::endl;
  ostr << adjustment_bounds[0] << ' ' << adjustment_bounds[1] << std::endl;

  for (size_t adj = 0; adj < position_correction.size(); adj++) {
    ostr << position_correction[adj][0] << " "
         << position_correction[adj][1] << " "
         << position_correction[adj][2] << "\n";
    ostr << pose_correction[adj].w() << " "
         << pose_correction[adj].x() << " "
         << pose_correction[adj].y() << " "
         << pose_correction[adj].z() << " " << "\n";
  }
  ostr.close();
}


void asp::write_adjustments(std::string const& filename,
                       Vector3 const& position_correction,
                       Quat const& pose_correction) {
  std::ofstream ostr(filename.c_str());
  ostr.precision(18);
  ostr << position_correction[0] << " " << position_correction[1] << " "
       << position_correction[2] << "\n";
  ostr << pose_correction.w() << " " << pose_correction.x() << " "
       << pose_correction.y() << " " << pose_correction.z() << " " << "\n";
  ostr.close();
}

void asp::compute_stereo_residuals(std::vector<boost::shared_ptr<CameraModel> > const& camera_models,
                              ControlNetwork const& cnet) {

  // Compute pre-adjustment residuals and convert to bundles
  int n = 0;
  double error_sum = 0;
  double min_error = ScalarTypeLimits<double>::highest();
  double max_error = ScalarTypeLimits<double>::lowest();
  for (size_t i = 0; i < cnet.size(); ++i) {
    for (size_t j = 0; j+1 < cnet[i].size(); ++j) {
      ++n;
      size_t cam1 = cnet[i][j].image_id();
      size_t cam2 = cnet[i][j+1].image_id();
      Vector2 pix1 = cnet[i][j].position();
      Vector2 pix2 = cnet[i][j+1].position();

      StereoModel sm( camera_models[cam1].get(),
                      camera_models[cam2].get() );
      double error;
      sm(pix1,pix2,error);
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
  }
  vw_out() << "\nStereo Intersection Residuals -- Min: " << min_error
           << "  Max: " << max_error << "  Average: " << (error_sum/n) << "\n";
}
