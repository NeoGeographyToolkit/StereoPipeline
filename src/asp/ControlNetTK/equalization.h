// This will knock off weak points where we already have a lot of
// points. This is hopefully to produce a better distribution.
#ifndef __EQUALIZATION_H__
#define __EQUALIZATION_H__

#include <vw/InterestPoint/InterestData.h>
#include <vector>

// Algorithm for removing interest points
inline void remove_max( std::vector<std::vector<InterestPoint> > &b_ip1,
                        std::vector<std::vector<InterestPoint> > &b_ip2 ) {
  unsigned max_index = 0;
  unsigned max_count = 0;
  for ( unsigned i = 0; i < b_ip1.size(); i++ )
    if (b_ip1[i].size() > max_count ) {
      max_count = b_ip1[i].size();
      max_index = i;
    }

  unsigned point_idx = 0;
  unsigned point_interest = -200000;
  for ( unsigned i = 0; i < b_ip1[max_index].size(); i++ ) {
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
std::vector<BBox2i> divide_block( BBox2i const& orginal,
                                  int div_x, int div_y ) {
  std::vector<BBox2i> bboxes;
  Vector2 min = orginal.min();

  Vector2 lmin, lmax;

  for ( int i = 0; i < div_x; i++ ) {
    lmin.x() = min.x() + i*orginal.width()/float(div_x);
    if ( i == div_x -1 )
      lmax.x() = orginal.max().x();
    else
      lmax.x() = min.x() + (i+1)*orginal.width()/float(div_x);
    for ( int j = 0; j < div_y; j++ ) {
      lmin.y() = min.y() + j*orginal.height()/float(div_y);
      if ( j == div_y-1 )
        lmax.y() = orginal.max().y();
      else
        lmax.y() = min.y() + (j+1)*orginal.height()/float(div_y);

      bboxes.push_back( BBox2i( lmin, lmax ) );
    }
  }

  return bboxes;
}

void equalization( std::vector<vw::ip::InterestPoint>& l_ip,
                   std::vector<vw::ip::InterestPoint>& r_ip,
                   unsigned max_points ) {

  // Checking for early exit condition
  if ( l_ip.size() <= max_points ) {
    std::cout << "\t> Exiting early, found less than " << max_points << " matches.\n";
    return;
  }

  //TerminalProgressCallback progress;
  //progress.report_progress(0);

  // Reducing to an even distribution
  std::cout << "Building Bounding Boxes:\n";
  BBox2i total_bbox;
  for ( unsigned i = 0; i < l_ip.size(); ++i )
    total_bbox.grow( Vector2( l_ip[i].x, l_ip[i].y ) );
  std::vector<BBox2i> bboxes = divide_block( total_bbox, 5, 5 );
  std::vector<std::vector<InterestPoint> > b_ip1;
  std::vector<std::vector<InterestPoint> > b_ip2;
  b_ip1.resize( bboxes.size() );
  b_ip2.resize( bboxes.size() );
  for ( unsigned b = 0; b < bboxes.size(); ++b )
    for ( unsigned i = 0; i < l_ip.size(); ++i )
      if ( bboxes[b].contains( Vector2( l_ip[i].x, l_ip[i].y ) ) ) {
        b_ip1[b].push_back( l_ip[i] );
        b_ip2[b].push_back( r_ip[i] );
      }

  // Finding how many points there are
  unsigned count = 0;
  for ( unsigned b = 0; b < b_ip1.size(); ++b )
    count += b_ip1[b].size();

  // Remove until less that max
  //int diff = count - max_points;
  while ( count > max_points ) {
    //progress.report_progress(1-float(count-max_points)/float(diff));
    remove_max( b_ip1, b_ip2 );
    count--;
  }
  //progress.report_finished();

  // Reorganize back into correct form
  l_ip.clear();
  r_ip.clear();
  for ( unsigned b = 0; b < b_ip1.size(); ++b )
    for ( unsigned i = 0; i < b_ip1[b].size(); ++i ) {
      l_ip.push_back( b_ip1[b][i] );
      r_ip.push_back( b_ip2[b][i] );
    }
}

#endif//__EQUALIZATION_H__
