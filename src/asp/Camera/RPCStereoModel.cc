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

#include <vw/Core/Log.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Math/Vector.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>

#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPCStereoModel.h>

// TODO(oalexan1): See if to wipe all this code as it is no longer in use.

using namespace vw;
namespace asp {

  Vector3 RPCStereoModel::operator()(std::vector<Vector2> const& pixVec,
                                     Vector3& errorVec) const {

    // Note: This is a re-implementation of StereoModel::operator().

    int num_cams = m_cameras.size();
    VW_ASSERT((int)pixVec.size() == num_cams,
              vw::ArgumentErr() << "the number of rays must match "
                                << "the number of cameras.\n");

    errorVec = Vector3();

    try {
      std::vector<Vector3> camDirs(num_cams), 
                      camCtrs(num_cams);
      std::vector<const RPCModel*> rpc_cams(num_cams);
      camDirs.clear(); 
      camCtrs.clear(); 
      rpc_cams.clear();

      // Pick the valid rays
      for (int p = 0; p < num_cams; p++){

        // Get the RPC pointer so we can call RPC specific functions on it
        const RPCModel *rpc_cam = dynamic_cast<const RPCModel*>(vw::camera::unadjusted_model(m_cameras[p]));
        VW_ASSERT(rpc_cam != NULL,
                  vw::ArgumentErr() << "Camera models are not RPC.\n");
        rpc_cams.push_back(rpc_cam);

        Vector2 pix = pixVec[p];
        if (pix != pix || // i.e., NaN
            pix == camera::CameraModel::invalid_pixel() ) continue;

        // The base class function would call point_and_dir twice, but we only need to call it once!
        Vector3 ctr, dir;
        rpc_cam->point_and_dir(pix, ctr, dir);
        camDirs.push_back(dir);
        camCtrs.push_back(ctr);
      }


      // Not enough valid rays
      if (camDirs.size() < 2) 
          return Vector3();

      if (are_nearly_parallel(m_angle_tol, camDirs)) 
          return Vector3();

      // Determine range by triangulation
      Vector3 result = triangulate_point(camDirs, camCtrs, errorVec);

      // Reflect points that fall behind one of the two cameras
      bool reflect = false;
      for (int p = 0; p < (int)camCtrs.size(); p++)
        if (dot_prod(result - camCtrs[p], camDirs[p]) < 0 ) reflect = true;
      if (reflect)
        result = -result + 2*camCtrs[0];

      return result;

    } catch (const camera::PixelToRayErr& /*e*/) {}
    return Vector3();
  }

  Vector3 RPCStereoModel::operator()(vw::Vector2 const& pix1,
                                     vw::Vector2 const& pix2,
                                     double& error ) const {
    return StereoModel::operator()(pix1, pix2, error);
  }

} // namespace asp
