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
#include <vw/Math/Functors.h>

#include <iostream>

using namespace std;
using namespace vw;

namespace asp {
  
// ===========================================================================
// Class Member Functions
// ===========================================================================

Point2Grid::Point2Grid(int width, int height,
                       ImageView<double> & buffer, ImageView<double> & weights,
                       double x0, double y0, double grid_size, double min_spacing,
                       double radius, double sigma_factor,
                       FilterType filter, double percentile):
  m_width(width), m_height(height),
  m_buffer(buffer), m_weights(weights),
  m_x0(x0), m_y0(y0), m_grid_size(grid_size),
  m_radius(radius), m_filter(filter), m_percentile(percentile){
  
  if (m_grid_size <= 0)
    vw_throw( ArgumentErr() << "Point2Grid: Grid size must be > 0.\n" );
  if (m_radius <= 0)
    vw_throw( ArgumentErr() << "Point2Grid: Search radius must be > 0.\n" );

  if (m_filter == f_percentile && (m_percentile < 0 || m_percentile > 100.0) )  
    vw_throw( ArgumentErr() << "Point2Grid: Expecting the percentile in the range 0.0 to 100.0.\n" );

  // Stop here if we don't need to create gaussian weights
  if (m_filter != f_weighted_average) 
    return; 

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
      m_buffer (c, r) = value; // usually this is the no-data value
      m_weights(c, r) = 0.0;
    }
  }

  // For these we need to keep all values (in fact, for stddev we could get away with less,
  // but it is not worth trying so hard).
  if (m_filter == f_median || m_filter == f_stddev ||
      m_filter == f_nmad || m_filter == f_percentile) {
    m_vals.set_size(m_width, m_height);
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

      if (m_filter == f_weighted_average) {
        double wt = m_sampled_gauss[(int)round(dist/m_dx)];
        if (wt <= 0) continue;
        if (m_weights(ix, iy) == 0) m_buffer(ix, iy) = 0.0; // set to 0 before incrementing below
        m_buffer(ix, iy)  += z*wt;
        m_weights(ix, iy) += wt;

      }else if (m_filter == f_mean){
        if (m_weights(ix, iy) == 0) m_buffer(ix, iy) = 0.0; // set to 0 before incrementing below
        m_buffer(ix, iy)  += z;
        m_weights(ix, iy) += 1;
        
      }else if (m_filter == f_min){
        if (m_weights(ix, iy) == 0) {
          m_buffer(ix, iy)  = z; // first time we set the value
          m_weights(ix, iy) = 1; // mark the fact that the buffer was initialized
        }else
          m_buffer(ix, iy) = std::min(m_buffer(ix, iy), z);
      
      }else if (m_filter == f_max){
        if (m_weights(ix, iy) == 0) {
          m_buffer(ix, iy)  = z; // first time we set the value
          m_weights(ix, iy) = 1; // mark the fact that the buffer was initialized
        }else
          m_buffer(ix, iy) = std::max(m_buffer(ix, iy), z);
        
      }else if (m_filter == f_count){
        if (m_weights(ix, iy) == 0) {
          m_weights(ix, iy) = 1; // mark the fact that the buffer was initialized
        }else
          m_weights(ix, iy) += 1;
        
      }else if (m_filter == f_stddev || m_filter == f_median ||
                m_filter == f_nmad   || m_filter == f_percentile){
        m_vals(ix, iy).push_back(z); // not strictly needed for stddev
      }
      
    }
    
  }
}

void Point2Grid::normalize(){
  for (int c = 0; c < m_buffer.cols(); c++){
    for (int r = 0; r < m_buffer.rows(); r++){

      if (m_filter == f_weighted_average || m_filter == f_mean) {
        if (m_weights(c, r) > 0)
          m_buffer (c, r) /= m_weights(c, r);

      }else if (m_filter == f_count)
        m_buffer(c, r) = m_weights(c, r); // hence instead of no-data we will have always 0

      else if (m_filter == f_stddev){
        if (m_vals(c, r).empty())
          continue; // nothing to compute
        vw::math::StdDevAccumulator<double> V;
        for (size_t it = 0; it < m_vals(c, r).size(); it++) 
          V(m_vals(c, r)[it]);
        m_buffer(c, r) = V.value();
      }
      
      else if (m_filter == f_median){
        if (m_vals(c, r).empty())
          continue; // nothing to compute
        vw::math::MedianAccumulator<double> V;
        for (size_t it = 0; it < m_vals(c, r).size(); it++) 
          V(m_vals(c, r)[it]);
        m_buffer(c, r) = V.value();
      }

      else if (m_filter == f_nmad){
        if (m_vals(c, r).empty())
          continue; // nothing to compute
        m_buffer(c, r) = vw::math::destructive_nmad(m_vals(c, r));
      }
      
      else if (m_filter == f_percentile){
        if (m_vals(c, r).empty())
          continue; // nothing to compute
        m_buffer(c, r) = vw::math::destructive_percentile(m_vals(c, r), m_percentile);
      }
      
    }
  }
}
  
} // end namespace asp
