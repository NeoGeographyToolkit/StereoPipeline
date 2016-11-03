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

using namespace vw;
using namespace std;
namespace asp {

  namespace detail {
    class RPCTriangulateLMA : public math::LeastSquaresModelBase<RPCTriangulateLMA> {
      const RPCModel *m_rpc_model1, *m_rpc_model2;
    public:
      typedef Vector<double, 4>    result_type;
      typedef Vector<double, 3>    domain_type;
      typedef Matrix<double, 4, 3> jacobian_type;

      RPCTriangulateLMA( RPCModel const* rpc_model1,
                         RPCModel const* rpc_model2 ) :
        m_rpc_model1(rpc_model1), m_rpc_model2(rpc_model2) {}

      inline result_type operator()( domain_type const& x ) const {
        result_type output;
        subvector(output, 0, 2) = m_rpc_model1->geodetic_to_pixel(x);
        subvector(output, 2, 2) = m_rpc_model2->geodetic_to_pixel(x);
        return output;
      }

      inline jacobian_type jacobian( domain_type const& x ) const {
        jacobian_type J;
        submatrix(J, 0, 0, 2, 3) = m_rpc_model1->geodetic_to_pixel_Jacobian(x);
        submatrix(J, 2, 0, 2, 3) = m_rpc_model2->geodetic_to_pixel_Jacobian(x);
        return J;
      }

    };
  }

  Vector3 RPCStereoModel::operator()(vector<Vector2> const& pixVec,
                                     Vector3& errorVec) const {

    // Note: This is a re-implementation of StereoModel::operator().

    int num_cams = m_cameras.size();
    VW_ASSERT((int)pixVec.size() == num_cams,
              vw::ArgumentErr() << "the number of rays must match "
                                << "the number of cameras.\n");

    errorVec = Vector3();

    try {
      vector<Vector3> camDirs(num_cams), 
                      camCtrs(num_cams);
      vector<const RPCModel*> rpc_cams(num_cams);
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

      if (are_nearly_parallel(m_least_squares, m_angle_tol, camDirs)) 
          return Vector3();

      // Determine range by triangulation
      Vector3 result = triangulate_point(camDirs, camCtrs, errorVec);

      if ( m_least_squares ){

        // Refine triangulation

        if (num_cams != 2)
          vw::vw_throw(vw::NoImplErr() << "Least squares refinement is not "
                       << "implemented for multi-view stereo.");

        detail::RPCTriangulateLMA model(rpc_cams[0], rpc_cams[1]);
        Vector4 objective(pixVec[0][0], pixVec[0][1], pixVec[1][0], pixVec[1][1]);
        int status = 0;

        Vector3 initialGeodetic = rpc_cams[0]->datum().cartesian_to_geodetic(result);

        // To do: Find good values for the numbers controlling the convergence
        Vector3 finalGeodetic = levenberg_marquardt( model, initialGeodetic,
                                                     objective, status, 1e-3, 1e-6, 10 );

        if ( status > 0 )
          result = rpc_cams[0]->datum().geodetic_to_cartesian(finalGeodetic);
      } // End least squares case


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
