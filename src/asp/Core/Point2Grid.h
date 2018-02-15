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


/// \file Point2Grid.h
///

#ifndef __VW_POINT2GRID_H__
#define __VW_POINT2GRID_H__

#include <vw/Image/ImageView.h>

namespace asp {

  // The type of filter to apply to points within a circular bin.
  enum FilterType {f_weighted_average, f_min, f_max, f_mean, f_median, f_stddev, f_count,
                   f_nmad, f_percentile};
    
  class Point2Grid {
    
  public:
    Point2Grid(int width, int height,
               vw::ImageView<double> & buffer, vw::ImageView<double> & weights,
               double x0, double y0,
               double grid_size, double min_spacing, double radius,
	       double sigma_factor,
               FilterType filter, double percentile);
    ~Point2Grid(){}
    void Clear(const float val);
    void AddPoint(double x, double y, double z);
    void normalize();

  private:
    int m_width, m_height; // DEM dimensions
    vw::ImageView<double> & m_buffer;
    vw::ImageView<double> & m_weights;
    vw::ImageView< std::vector<double> > m_vals; // when need to keep all individual values
    double m_x0, m_y0; // lower-left corner
    double m_grid_size;  // spacing between output DEM pixels
    double m_radius;   // how far to search for cloud points
    double m_dx;       // spacing between samples
    std::vector<double> m_sampled_gauss;
    FilterType m_filter;
    double m_percentile; // The actual value of the percentile to use if in that mode
    
  };
  
}

#endif  // __VW_POINT2GRID_H__
