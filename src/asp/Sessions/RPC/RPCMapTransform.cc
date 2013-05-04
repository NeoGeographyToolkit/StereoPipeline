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


#include <asp/Sessions/RPC/RPCMapTransform.h>
#include <vw/Image/MaskViews.h>
#include <vw/Cartography.h>

using namespace vw;

namespace asp {

  RPCMapTransform::RPCMapTransform( RPCModel const& rpc,
                                    cartography::GeoReference const& image_georef,
                                    cartography::GeoReference const& dem_georef,
                                    boost::shared_ptr<DiskImageResource> dem_rsrc,
                                    Vector2i image_size ) :
    m_rpc(rpc), m_image_georef(image_georef), m_dem_georef(dem_georef),
    m_dem(dem_rsrc), m_image_size(image_size) {
    using namespace vw;
    using namespace vw::cartography;

    if ( dem_rsrc->has_nodata_read() )
      m_point_cloud =
        geo_transform(
          geodetic_to_cartesian(
            dem_to_geodetic( create_mask( m_dem, dem_rsrc->nodata_read()),
                             m_dem_georef ), m_dem_georef.datum() ),
          m_dem_georef, m_image_georef,
          ValueEdgeExtension<Vector3>( Vector3() ),
          BicubicInterpolation());
    else
      m_point_cloud =
        geo_transform(
          geodetic_to_cartesian(
            dem_to_geodetic( m_dem, m_dem_georef ),
            m_dem_georef.datum() ),
          m_dem_georef, m_image_georef,
          ValueEdgeExtension<Vector3>( Vector3() ),
          BicubicInterpolation());
  }

  vw::Vector2
  RPCMapTransform::reverse(const vw::Vector2 &p) const {

    Vector3 xyz =
      m_cache_size.contains( p ) ?
      m_point_cloud_cache(p.x() - m_cache_size.min().x(),
                          p.y() - m_cache_size.min().y()):
      m_point_cloud(p.x(),p.y());

    int neg = -1;
    if (xyz == Vector3()) return Vector2(neg, neg);

    Vector2 rv = m_rpc.point_to_pixel(xyz);

    // Straying too far from the intended image size is wasteful of
    // memory in rasterization.
    if (m_image_size != Vector2i(-1, -1)){
      for (int i = 0; i < 2; i++){
        // Note: rv is double precision, so the check below
        // must be exactly as it is now.
        if (rv[i] < 0.0 || rv[i] > m_image_size[i]-1.0) rv = Vector2(neg, neg);
      }
    }

    return rv;
  }

  // This function will be called whenever we start to apply the
  // transform in a tile. It computes and caches the point cloud at
  // each pixel in the tile, to be used later when we iterate over
  // pixels.
  vw::BBox2i
  RPCMapTransform::reverse_bbox( vw::BBox2i const& bbox ) const {
    cache_dem( bbox );
    return vw::TransformBase<RPCMapTransform>::reverse_bbox( bbox );
  }

  void
  RPCMapTransform::cache_dem( vw::BBox2i const& bbox ) const {
    m_point_cloud_cache = crop( m_point_cloud, bbox );
    m_cache_size = bbox;
  }
}
