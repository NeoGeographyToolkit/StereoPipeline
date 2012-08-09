// __BEGIN_LICENSE__
//  Copyright (c) 2006-2012, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#include <vw/Camera/CameraModel.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <asp/Sessions/RPC/RPCModel.h>
#include <asp/Sessions/RPC/RPCStereoModel.h>

using namespace vw;
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

  Vector3 RPCStereoModel::operator()(Vector2 const& pix1, Vector2 const& pix2, double& error) const {

    const RPCModel *rpc_model1 = dynamic_cast<const RPCModel*>(m_camera1);
    const RPCModel *rpc_model2 = dynamic_cast<const RPCModel*>(m_camera2);

    if (rpc_model1 == NULL || rpc_model2 == NULL){
      VW_OUT(ErrorMessage) << "RPC camera models expected.\n";
      error = 0.0;
      return Vector3();
    }
    
    try {
    
      detail::RPCTriangulateLMA model(rpc_model1, rpc_model2);
      Vector4 objective( pix1[0], pix1[1], pix2[0], pix2[1] );
      int status = 0;
    
      Vector3 initialGeodetic = ( rpc_model1->lonlatheight_offset() +
                                  rpc_model2->lonlatheight_offset() )/2.0;

      // To do: Find good values for the pixels below
      Vector3 finalGeodetic = levenberg_marquardt( model, initialGeodetic,
                                                   objective, status, 1e-6, 1e-8, 10 );

      // Notice that the error is in pixels rather than in meters
      error = norm_2(objective - model(finalGeodetic));
      
      return rpc_model1->datum().geodetic_to_cartesian(finalGeodetic);

    } catch (...) {}
    
    error = 0.0;
    return Vector3();
  }
  
} // namespace asp
