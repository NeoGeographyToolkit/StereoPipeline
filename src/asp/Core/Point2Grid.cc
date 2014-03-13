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

// Given a set of xyz points, create an xy grid. For every node in the
// grid, combine all points within given radius of the grid point and
// calculate a single z value at the grid point.

#include <vw/Core/Exception.h>
#include <vw/Core/FundamentalTypes.h>
#include <asp/Core/Point2Grid.h>

#include <iostream>

using namespace std;
using namespace vw;
using namespace stereo;

// ===========================================================================
// Class Member Functions
// ===========================================================================

Point2Grid::Point2Grid(int width, int height,
                       ImageView<double> & buffer, ImageView<double> & weights,
                       double x0, double y0, double spacing,
                       double radius): m_width(width), m_height(height),
                                       m_buffer(buffer), m_weights(weights),
                                       m_x0(x0), m_y0(y0), m_spacing(spacing),
                                       m_radius(radius){
  if (m_spacing <= 0)
    vw_throw( ArgumentErr() << "Point2Grid: Grid size must be > 0.\n" );
  if (m_radius <= 0)
    vw_throw( ArgumentErr() << "Point2Grid: Radius size must be > 0.\n" );
}

// Free up resources that are allocated in the constructor
Point2Grid::~Point2Grid() {

}

void Point2Grid::Clear(const float value) {
  m_buffer.set_size (m_width, m_height);
  m_weights.set_size (m_width, m_height);
  for (int c = 0; c < m_buffer.cols(); c++){
    for (int r = 0; r < m_buffer.rows(); r++){
      m_buffer (c, r) = value;
      m_weights(c, r) = 0.0;
    }
  }
}

void Point2Grid::AddPoint(double x, double y, double z){

  int minx = std::max( (int)ceil( (x - m_radius - m_x0)/m_spacing ), 0 );
  int miny = std::max( (int)ceil( (y - m_radius - m_y0)/m_spacing ), 0 );
  
  int maxx = std::min( (int)floor( (x + m_radius - m_x0)/m_spacing ), m_buffer.cols() - 1 );
  int maxy = std::min( (int)floor( (y + m_radius - m_y0)/m_spacing ), m_buffer.rows() - 1 );

  // Add the contribution of current point to all grid points within radius
  for (int ix = minx; ix <= maxx; ix++){
    for (int iy = miny; iy <= maxy; iy++){
      
      double gx = m_x0 + ix*m_spacing;
      double gy = m_y0 + iy*m_spacing;
      double dist = sqrt( (x-gx)*(x-gx) + (y-gy)*(y-gy) );
      if ( dist > m_radius ) continue;

      if (m_weights(ix, iy) == 0) m_buffer(ix, iy) = 0.0;
      m_buffer(ix, iy)  += z;
      m_weights(ix, iy) += 1;
    }
    
  }
}

void Point2Grid::normalize(){
  for (int c = 0; c < m_buffer.cols(); c++){
    for (int r = 0; r < m_buffer.rows(); r++){
      if (m_weights(c, r) > 0)
        m_buffer (c, r) /= m_weights(c, r);
    }
  }
}
