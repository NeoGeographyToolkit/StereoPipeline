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


/// \file StereoSessionDGMapRPC.h
///
/// This a session that support RPC Mapproject DG images. It is built
/// entirely so that left and right TX are objects and not TransformRefs.

#ifndef __STEREO_SESSION_DGMAPRPC_H__
#define __STEREO_SESSION_DGMAPRPC_H__

#include <asp/Sessions/DG/StereoSessionDG.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Image/Transform.h>

namespace asp {
 
  
  /*TODO:
     This class has a lot of code dedicated to handling alignment + map projection, but we don't even allow
     alignment with map projected images! (The map projection should take care of the alignment)
     
     All this extra handling is not needed and we can greatly simplify this class without
     losing any functionality.
  */
  
  class RPCModel;
/*
  // TODO: Delete this class, just use Map2CamTrans!
  /// Specialize CompositionTransform that allows passing of BBox so Map2CamTrans can cache itself.
  template <class Tx1T, class Tx2T>
  class CompositionTransformPassBBox : public vw::TransformBase<CompositionTransformPassBBox<Tx1T,Tx2T> > {
  public:
    CompositionTransformPassBBox(Tx1T const& tx1, Tx2T const& tx2) : tx1(tx1), tx2(tx2) {}

    Tx1T tx1; // Be sure to copy!
    Tx2T tx2; // public so that we can invoke caching manually for Map2CamTrans

    // Forward and reverse chained transforms
    inline vw::Vector2 forward(vw::Vector2 const& p) const { return tx1.forward( tx2.forward( p ) ); }
    inline vw::Vector2 reverse(vw::Vector2 const& p) const { return tx2.reverse( tx1.reverse( p ) ); }

    // Reverse bbox transform chained
    inline vw::BBox2i reverse_bbox( vw::BBox2i const& bbox ) const {
      return this->tx2.reverse_bbox(this->tx1.reverse_bbox(bbox));
    }
  };
*/
  
  
  
  /// Specialization of the StereoSessionDG class to use map-projected inputs with the RPC sensor model.
  class StereoSessionDGMapRPC : public StereoSessionDG {
  public:
    StereoSessionDGMapRPC(){};
    virtual ~StereoSessionDGMapRPC(){};

    
    /// Initializer verifies that the input is map projected
    virtual void initialize(BaseOptions const& options,
                            std::string const& left_image_file,
                            std::string const& right_image_file,
                            std::string const& left_camera_file,
                            std::string const& right_camera_file,
                            std::string const& out_prefix,
                            std::string const& input_dem);

    virtual std::string name() const { return "dgmaprpc"; }

    /// Disable this function 
    /// - IP matching is not needed because alignment is not supported for map projected images!
    virtual bool ip_matching(std::string const& input_file1,
                             std::string const& input_file2,
                             float nodata1, float nodata2,
                             std::string const& match_filename,
                             vw::camera::CameraModel* cam1,
                             vw::camera::CameraModel* cam2);

    /// Transforms from pixel coordinates on disk to original unwarped image coordinates.
    /// - For reversing the arithmetic applied in preprocessing plus the map projection.
    /// - This combines the homography/affineEpipolar transform with the map-to-camera transform
    ///   so that we can recover the original image pixels from a warped map projected image.
    typedef vw::cartography::Map2CamTrans tx_type;
    typedef vw::stereo::StereoModel       stereo_model_type;
    tx_type tx_left () const;
    tx_type tx_right() const;

    static StereoSession* construct() { return new StereoSessionDGMapRPC; }

    // TODO: Why is this public?
    /// RPC camera models used only in the tx_left and tx_right functions.
    /// - Read in initialize()
    boost::shared_ptr<RPCModel> m_left_model, m_right_model;
  };

}

#endif//__STEREO_SESSION_DGMAPRPC_H__
