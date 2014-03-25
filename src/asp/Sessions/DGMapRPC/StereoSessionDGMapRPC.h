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
/// entirely so that left and right TX are objects and not
/// TransformRefs.

#ifndef __STEREO_SESSION_DGMAPRPC_H__
#define __STEREO_SESSION_DGMAPRPC_H__

#include <asp/Sessions/DG/StereoSessionDG.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Image/Transform.h>

namespace asp {
 
  class RPCModel;

  // Specialize CompositionTransform that allows passing of BBox so
  // Map2CamTrans can cache itself.
  template <class Tx1T, class Tx2T>
  class CompositionTransformPassBBox : public vw::TransformBase<CompositionTransformPassBBox<Tx1T,Tx2T> > {
  public:
    CompositionTransformPassBBox( Tx1T const& tx1, Tx2T const& tx2 ) : tx1(tx1), tx2(tx2) {}

    Tx1T tx1; // Be sure to copy!
    Tx2T tx2; // public so that we can invoke caching manually for Map2CamTrans

    inline vw::Vector2 forward( vw::Vector2 const& p ) const { return tx1.forward( tx2.forward( p ) ); }
    inline vw::Vector2 reverse( vw::Vector2 const& p ) const { return tx2.reverse( tx1.reverse( p ) ); }

    inline vw::BBox2i reverse_bbox( vw::BBox2i const& bbox ) const {
      return this->tx2.reverse_bbox(this->tx1.reverse_bbox(bbox));
    }
  };

  class StereoSessionDGMapRPC : public StereoSessionDG {
  public:
    StereoSessionDGMapRPC(){};
    virtual ~StereoSessionDGMapRPC(){};

    // Initializer verifies that the input is map projected
    virtual void initialize(BaseOptions const& options,
                            std::string const& left_image_file,
                            std::string const& right_image_file,
                            std::string const& left_camera_file,
                            std::string const& right_camera_file,
                            std::string const& out_prefix,
                            std::string const& input_dem,
                            std::string const& extra_argument1,
                            std::string const& extra_argument2,
                            std::string const& extra_argument3);

    virtual std::string name() const { return "dgmaprpc"; }

    // Allows specialization of how matches are captured.
    virtual bool ip_matching( std::string const& match_filename,
                              double left_nodata_value,
                              double right_nodata_value );

    // For reversing the arithmetic applied in preprocessing plus the
    // map projection.
    typedef CompositionTransformPassBBox<vw::cartography::Map2CamTrans,vw::HomographyTransform> left_tx_type;
    typedef CompositionTransformPassBBox<vw::cartography::Map2CamTrans,vw::HomographyTransform> right_tx_type;
    typedef vw::stereo::StereoModel stereo_model_type;
    left_tx_type tx_left() const;
    right_tx_type tx_right() const;

    static StereoSession* construct() { return new StereoSessionDGMapRPC; }

    boost::shared_ptr<RPCModel> m_left_model, m_right_model;
  };

}

#endif//__STEREO_SESSION_DGMAPRPC_H__
