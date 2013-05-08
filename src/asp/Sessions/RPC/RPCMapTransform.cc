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
                                    boost::shared_ptr<DiskImageResource> dem_rsrc ) :
    m_rpc(rpc), m_image_georef(image_georef), m_dem_georef(dem_georef),
    m_dem(dem_rsrc) {
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

    // If possible, we will interpolate into the cached point cloud
    ImageViewRef<Vector3> interp_point_cloud =
      interpolate(m_point_cloud_cache, BicubicInterpolation(), ZeroEdgeExtension());

    // Avoid interpolating close to edges
    BBox2i shrank_bbox = m_cache_size;
    shrank_bbox.contract(BicubicInterpolation::pixel_buffer);

    Vector3 xyz =
      shrank_bbox.contains( p ) ?
      interp_point_cloud(p.x() - m_cache_size.min().x(),
                         p.y() - m_cache_size.min().y()):
      m_point_cloud(p.x(),p.y());

    return m_rpc.point_to_pixel(xyz);
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
    m_cache_size = bbox;

    // For bicubic interpolation later
    m_cache_size.expand(BicubicInterpolation::pixel_buffer);

    m_point_cloud_cache = crop( m_point_cloud, m_cache_size );
  }
}
