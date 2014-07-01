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


// This will knock off weak points where we already have a lot of
// points. This is hopefully to produce a better distribution.
#ifndef __ASP_CONTROLNETTK_EQUALIZATION_H__
#define __ASP_CONTROLNETTK_EQUALIZATION_H__

#include <vw/InterestPoint/InterestData.h>
#include <vector>

namespace asp {
namespace cnettk {

  // Algorithm for removing interest points
  inline void remove_max( std::vector<std::vector<vw::ip::InterestPoint> > &b_ip1,
                          std::vector<std::vector<vw::ip::InterestPoint> > &b_ip2 ) {
    using namespace vw;

    size_t max_index = 0;
    size_t max_count = 0;
    for ( size_t i = 0; i < b_ip1.size(); i++ )
      if (b_ip1[i].size() > max_count ) {
        max_count = b_ip1[i].size();
        max_index = i;
      }

    size_t point_idx = 0;
    float point_interest = std::numeric_limits<float>::min();
    for ( size_t i = 0; i < b_ip1[max_index].size(); i++ ) {
      if ( b_ip1[max_index][i].interest > point_interest ) {
        point_interest = b_ip1[max_index][i].interest;
        point_idx = i;
      }
    }

    // Okay dokey making my remove
    b_ip1[max_index].erase( b_ip1[max_index].begin() + point_idx );
    b_ip2[max_index].erase( b_ip2[max_index].begin() + point_idx );
  }

  // divide block
  std::vector<vw::BBox2f> divide_block( vw::BBox2f const& orginal,
                                    ssize_t div_x, ssize_t div_y ) {
    using namespace vw;

    std::vector<BBox2f> bboxes;
    Vector2f min = orginal.min();

    Vector2f lmin, lmax;

    for ( ssize_t i = 0; i < div_x; i++ ) {
      lmin.x() = min.x() + i*orginal.width()/float(div_x);
      if ( i == div_x -1 )
        lmax.x() = orginal.max().x();
      else
        lmax.x() = min.x() + (i+1)*orginal.width()/float(div_x);
      for ( ssize_t j = 0; j < div_y; j++ ) {
        lmin.y() = min.y() + j*orginal.height()/float(div_y);
        if ( j == div_y-1 )
          lmax.y() = orginal.max().y();
        else
          lmax.y() = min.y() + (j+1)*orginal.height()/float(div_y);

        bboxes.push_back( BBox2f( lmin, lmax ) );
      }
    }

    return bboxes;
  }

  void equalization( std::vector<vw::ip::InterestPoint>& l_ip,
                     std::vector<vw::ip::InterestPoint>& r_ip,
                     size_t max_points ) {
    using namespace vw;

    // Checking for early exit condition
    if ( l_ip.size() <= max_points ) {
      vw_out() << "\t> Exiting early, found less than " << max_points << " matches." << std::endl;
      return;
    }

    // Reducing to an even distribution
    vw_out() << "Building Bounding Boxes:" << std::endl;
    BBox2f total_bbox;
    for ( size_t i = 0; i < l_ip.size(); ++i )
      total_bbox.grow( Vector2f( l_ip[i].x, l_ip[i].y ) );
    vw_out(DebugMessage,"equalization") << "Total bbox: " << total_bbox << "\n";
    std::vector<BBox2f> bboxes;
    if ( max_points < 10 )
      bboxes = divide_block( total_bbox, 2, 2 );
    else if ( max_points < 30 )
      bboxes = divide_block( total_bbox, 3, 3 );
    else if ( max_points < 110 )
      bboxes = divide_block( total_bbox, 5, 5 );
    else
      bboxes = divide_block( total_bbox, 10, 10 );
    std::vector<std::vector<ip::InterestPoint> > b_ip1;
    std::vector<std::vector<ip::InterestPoint> > b_ip2;
    b_ip1.resize( bboxes.size() );
    b_ip2.resize( bboxes.size() );
    for ( size_t b = 0; b < bboxes.size(); ++b )
      for ( size_t i = 0; i < l_ip.size(); ++i )
        if ( bboxes[b].contains( Vector2f( l_ip[i].x, l_ip[i].y ) ) ) {
          b_ip1[b].push_back( l_ip[i] );
          b_ip2[b].push_back( r_ip[i] );
        }

    // Finding how many points there are
    size_t count = 0;
    for ( size_t b = 0; b < b_ip1.size(); ++b )
      count += b_ip1[b].size();

    // Remove until less that max
    while ( count > max_points ) {
      remove_max( b_ip1, b_ip2 );
      count--;
    }

    // Reorganize back into correct form
    l_ip.clear();
    r_ip.clear();
    for ( size_t b = 0; b < b_ip1.size(); ++b )
      for ( size_t i = 0; i < b_ip1[b].size(); ++i ) {
        l_ip.push_back( b_ip1[b][i] );
        r_ip.push_back( b_ip2[b][i] );
      }
  }

}} // end asp::cnettk

#endif//__ASP_CONTROLNETTK_EQUALIZATION_H__
