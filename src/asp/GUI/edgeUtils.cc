// MIT License Terms (http://en.wikipedia.org/wiki/MIT_License)
// 
// Copyright (C) 2011 by Oleg Alexandrov
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <vector>
#include <cassert>
#include <asp/GUI/edgeUtils.h>
#include <asp/GUI/baseUtils.h>
using namespace std;

bool utils::edgeIntersectsBox(// Input: arbitrary edge
                              double bx, double by,
                              double ex, double ey,
                              // Input: Box
                              double xl, double yl,
                              double xh, double yh
                              ){

  assert(xl <= xl && yl <= yh);

  // See if edge is to the left/right/above/below the box
  if (bx > xh && ex > xh) return false;
  if (bx < xl && ex < xl) return false;
  if (by > yh && ey > yh) return false;
  if (by < yl && ey < yl) return false;
  
  // See if any of the two edge endpoints are in the box
  if (xl <= bx && bx <= xh && yl <= by && by <= yh) return true;
  if (xl <= ex && ex <= xh && yl <= ey && ey <= yh) return true;

  // See if the edge intersects the four edges of the box
  if (edgeIntersectsHorizontalEdge(bx, by, ex, ey, xl, xh, yl)) return true;
  if (edgeIntersectsHorizontalEdge(bx, by, ex, ey, xl, xh, yh)) return true;
  if (edgeIntersectsHorizontalEdge(by, bx, ey, ex, yl, yh, xl)) return true;
  if (edgeIntersectsHorizontalEdge(by, bx, ey, ex, yl, yh, xh)) return true;

  return false;
}
  
bool utils::edgeIntersectsHorizontalEdge(// Input: arbitrary edge
                                         double x0, double y0,
                                         double x1, double y1,
                                         // Input: horizontal edge
                                         double begx, double endx,
                                         double yval
                                         ){

  // See if an arbitrary edge intersects a horizontal edge. Care has
  // be taken to avoid as much as possible floating point operations
  // which can be problematic in cases close to degeneracy.

  if (y0 > yval && y1 > yval) return false;
  if (y0 < yval && y1 < yval) return false;

  double diff = y1 - y0;

  if (diff == 0.0){ // The two edges are collinear
    assert(y0 == yval && y1 == yval);
    return (
            max(min(x0, x1), min(begx, endx)) <=
            min(max(x0, x1), max(begx, endx))
            );
  }

  // See if the points (begx, yval) and (endx, yval) are on different
  // sides of the non-degenerate edge (x0, y0), (x1, y1).

  double det_beg = (begx - x0)*(y1 - y0) - (yval - y0)*(x1 - x0);
  double det_end = (endx - x0)*(y1 - y0) - (yval - y0)*(x1 - x0);

  if (det_beg > 0.0 && det_end > 0.0) return false;
  if (det_beg < 0.0 && det_end < 0.0) return false;
    
  return true;
}

bool utils::isPointOnEdge(double x0, double y0, double x1, double y1,
                          double x, double y){
  double dist1 = distance(x0, y0, x1, y1);
  double dist2 = distance(x0, y0, x,  y);
  double dist3 = distance(x,  y,  x1, y1);
  double tol = 1e-12;
  return(dist2 + dist3 <= (1+tol)*dist1);
}

bool utils::collinearEdgesIntersect(// Input: first edge
                                    double ax0, double ay0,
                                    double ax1, double ay1,
                                    // Input: second edge
                                    double bx0, double by0,
                                    double bx1, double by1,
                                    // Output: intersection
                                    // if it exists
                                    double & x, double & y
                                    ){
  for (int t = 0; t < 2; t++){
    if (isPointOnEdge(ax0, ay0, ax1, ay1, bx0, by0)){
      x = bx0; y = by0;
      return true;
    }
    if (isPointOnEdge(ax0, ay0, ax1, ay1, bx1, by1)){
      x = bx1; y = by1;
      return true;
    }
    swap(ax0, bx0); swap(ay0, by0);
    swap(ax1, bx1); swap(ay1, by1);
  }
  
  return false;
}

bool utils::edgesIntersect(// Input: first edge
                           double ax0, double ay0,
                           double ax1, double ay1,
                           // Input: second edge
                           double bx0, double by0,
                           double bx1, double by1,
                           // Output: intersection if it exists
                           double & x, double & y
                           ){

  // See if two arbitrary edges intersect. This code does NOT treat
  // all cases and can give incorrect results. To be fixed.

  for (int count = 0; count < 2; count++){
    // See if the points (bx0, by0) and (bx1, by1) are on different
    // sides of the edge (ax0, ay0), (ax1, ay1).
    double det0 = (bx0 - ax0)*(ay1 - ay0) - (by0 - ay0)*(ax1 - ax0);
    double det1 = (bx1 - ax0)*(ay1 - ay0) - (by1 - ay0)*(ax1 - ax0);
    
    if (det0 > 0.0 && det1 > 0.0) return false;
    if (det0 < 0.0 && det1 < 0.0) return false;

    if (count == 0){
      swap(ax0, bx0); swap(ay0, by0);
      swap(ax1, bx1); swap(ay1, by1);
      continue;
    }

    if (det0 == det1){
      return collinearEdgesIntersect(// Input: first edge
                                     ax0, ay0, ax1, ay1,  
                                     // Input: second edge
                                     bx0, by0, bx1, by1,  
                                     // Output: intersection
                                     // if it exists
                                     x, y
                                     );
    }
    
    double t = -det0/(det1 - det0);
    
    x = (1 - t)*bx0 + t*bx1;
    y = (1 - t)*by0 + t*by1;
    return true;
  }
  
  return true;
}

void utils::cutEdge(double x0, double y0, double x1, double y1,
                    double nx, double ny, double H,
                    double & cutx, double & cuty){

  // Find the point at which the horizontal or vertical line
  // nx*x + ny*y = H intersects the edge (x0, y0) --> (x1, y1).
  // We assume that by now we already know that the edge
  // intersects this line.

  // To do: This needs careful reading.

  double dot0 = nx*x0 + ny*y0;
  double dot1 = nx*x1 + ny*y1;
  
  assert( (dot0 <= H && dot1 >= H) || (dot0 >= H && dot1 <= H ) );
  assert( dot0 != dot1 );

  // Find t such that (1-t)*(x0, y0) + t*(x1, y1) intersect the
  // cutting line
  
  double t = (H - dot0)/(dot1 - dot0);
  t = max(t, 0.0); t = min(t, 1.0); // extra precautions
  cutx = (1-t)*x0 + t*x1;
  cuty = (1-t)*y0 + t*y1;

  // Cut symmetrically in x0 and x1. We count on this symmetry
  // in a few places.
  t = (H - dot1)/(dot0 - dot1);
  t = max(t, 0.0); t = min(t, 1.0); // extra precautions
  double cutx2 = (1-t)*x1 + t*x0;
  double cuty2 = (1-t)*y1 + t*y0;

  cutx = 0.5*cutx + 0.5*cutx2;
  cuty = 0.5*cuty + 0.5*cuty2;
  
  // The above formulas have floating point errors. Use the fact that
  // the cutting line is either vertical or horizontal to recover
  // precisely one of the two coordinates above.
  if (nx == 0){
    cuty = H/ny;
  }else if (ny == 0){
    cutx = H/nx;
  }
  
  return;
}

