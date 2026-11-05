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

#ifndef __ASP_CORE_IP_FILTER_UTILS_H__
#define __ASP_CORE_IP_FILTER_UTILS_H__

#include <vw/InterestPoint/InterestPoint.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Math/Transform.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Matrix.h>

#include <vector>
#include <list>
#include <string>

namespace asp {

void filter_ip_by_ground_loc(vw::InterestPointList & ip1,
                             vw::InterestPointList & ip2,
                             vw::GeoReference const& georef1,
                             vw::GeoReference const& georef2,
                             double radius);

size_t remove_ip_bbox(vw::BBox2i const& roi, vw::InterestPointList & ip_list,
                      bool remove_inside);

void side_ip_filtering(vw::InterestPointList& ip1,
                       vw::InterestPointList& ip2,
                       vw::BBox2i const& bbox1, vw::BBox2i const& bbox2);

bool tri_ip_filtering(std::vector<vw::InterestPoint> const& ip1,
                      std::vector<vw::InterestPoint> const& ip2,
                      vw::CameraModel* cam1,
                      vw::CameraModel* cam2,
                      std::list<size_t>& valid_indices);

bool stddev_ip_filtering(std::vector<vw::InterestPoint> const& ip1,
                         std::vector<vw::InterestPoint> const& ip2,
                         std::list<size_t>& valid_indices);

size_t filter_ip_by_lonlat_and_elevation(vw::TransformPtr tx_left,
                                         vw::TransformPtr tx_right,
                                         vw::CameraModel* left_camera_model,
                                         vw::CameraModel* right_camera_model,
                                         vw::Datum const& datum,
                                         std::vector<vw::InterestPoint> const& ip1_in,
                                         std::vector<vw::InterestPoint> const& ip2_in,
                                         vw::Vector2 const & elevation_limit,
                                         vw::BBox2   const & lon_lat_limit,
                                         std::vector<vw::InterestPoint> & ip1_out,
                                         std::vector<vw::InterestPoint> & ip2_out);

void filter_ip_using_cameras(std::vector<vw::InterestPoint> & ip1,
                             std::vector<vw::InterestPoint> & ip2,
                             vw::CameraModel const* cam1,
                             vw::CameraModel const* cam2,
                             vw::Datum  const& datum,
                             double pct, double factor);

void ip_filter_using_dem(std::string const & ip_filter_using_dem,
                         vw::TransformPtr tx_left,
                         vw::TransformPtr tx_right,
                         boost::shared_ptr<vw::CameraModel> left_camera_model,
                         boost::shared_ptr<vw::CameraModel> right_camera_model,
                         std::vector<vw::InterestPoint> & left_aligned_ip,
                         std::vector<vw::InterestPoint> & right_aligned_ip);

vw::BBox2 search_range_using_spread(double max_disp_spread,
                                    std::vector<vw::InterestPoint> const& left_ip,
                                    std::vector<vw::InterestPoint> const& right_ip);

} // namespace asp

#endif // __ASP_CORE_IP_FILTER_UTILS_H__
