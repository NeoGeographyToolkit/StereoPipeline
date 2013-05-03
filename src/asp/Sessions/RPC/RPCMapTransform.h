// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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

#ifndef __STEREO_SESSION_RPC_MAP_TRANSFORM_H__
#define __STEREO_SESSION_RPC_MAP_TRANSFORM_H__

#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Transform.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/GeoReference.h>
#include <asp/Sessions/RPC/RPCModel.h>

namespace asp {

  // RPCMapTransform. Used to test the validity of IP matching on map
  // projected images. However, this could be used for performing an RPC
  // map projection.
  class RPCMapTransform : public vw::TransformHelper<RPCMapTransform,vw::ConvexFunction,vw::ConvexFunction> {
    RPCModel m_rpc;
    vw::cartography::GeoReference m_image_georef, m_dem_georef;
    vw::DiskImageView<float> m_dem;
    vw::ImageViewRef<vw::Vector3> m_point_cloud;
    vw::Vector2i m_image_size;

    // We will always be modifying these
    mutable vw::ImageView<vw::Vector3> m_point_cloud_cache;
    mutable vw::BBox2i m_cache_size;
  public:
    RPCMapTransform( asp::RPCModel const& rpc,
                     vw::cartography::GeoReference const& image_georef,
                     vw::cartography::GeoReference const& dem_georef,
                     boost::shared_ptr<vw::DiskImageResource> dem_rsrc,
                     vw::Vector2i image_size = vw::Vector2(-1, -1)
                     );

    // Convert Map Projected Coordinate to camera coordinate
    vw::Vector2 reverse(const vw::Vector2 &p) const;

    vw::BBox2i reverse_bbox( vw::BBox2i const& bbox ) const;
  };

}

#endif//__STEREO_SESSION_RPC_MAP_TRANSFORM_H__
