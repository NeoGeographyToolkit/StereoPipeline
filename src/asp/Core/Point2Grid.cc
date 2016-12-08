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
                       double x0, double y0, double grid_size, double min_spacing,
                       double radius, double sigma_factor):
  m_width(width), m_height(height),
  m_buffer(buffer), m_weights(weights),
  m_x0(x0), m_y0(y0), m_grid_size(grid_size),
  m_radius(radius){
  if (m_grid_size <= 0)
    vw_throw( ArgumentErr() << "Point2Grid: Grid size must be > 0.\n" );
  if (m_radius <= 0)
    vw_throw( ArgumentErr() << "Point2Grid: Search radius must be > 0.\n" );

  // By the time we reached the distance 'spacing' from the origin, we
  // want the Gaussian exp(-sigma*x^2) to decay to given value.  Note
  // that the user may choose to make the grid size very small, but we
  // put a limit to how small 'spacing' gets, that is, how large sigma
  // gets, to ensure that the DEM stays smooth.
  double spacing = std::max(grid_size, min_spacing);
  double val = 0.25;
  double sigma = -log(val)/spacing/spacing;

  // Override this if passed from outside
  if (sigma_factor > 0)
    sigma = sigma_factor/spacing/spacing;
  
  // Sample the gaussian for speed
  int num_samples = 1000;
  m_dx = m_radius/(num_samples - 1.0);
  m_sampled_gauss.resize(num_samples);
  for (int k = 0; k < num_samples; k++){
    double dist = k*m_dx;
    m_sampled_gauss[k] = exp(-sigma*dist*dist);
  }
  
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

  int minx = std::max( (int)ceil( (x - m_radius - m_x0)/m_grid_size ), 0 );
  int miny = std::max( (int)ceil( (y - m_radius - m_y0)/m_grid_size ), 0 );
  
  int maxx = std::min( (int)floor( (x + m_radius - m_x0)/m_grid_size ), m_buffer.cols() - 1 );
  int maxy = std::min( (int)floor( (y + m_radius - m_y0)/m_grid_size ), m_buffer.rows() - 1 );

  // Add the contribution of current point to all grid points within radius
  for (int ix = minx; ix <= maxx; ix++){
    for (int iy = miny; iy <= maxy; iy++){
      
      double gx = m_x0 + ix*m_grid_size;
      double gy = m_y0 + iy*m_grid_size;
      double dist = sqrt( (x-gx)*(x-gx) + (y-gy)*(y-gy) );
      if ( dist > m_radius ) continue;

      if (m_weights(ix, iy) == 0) m_buffer(ix, iy) = 0.0;
      double wt = m_sampled_gauss[(int)round(dist/m_dx)];
      if (wt <= 0) continue;
      m_buffer(ix, iy)  += z*wt;
      m_weights(ix, iy) += wt;
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
