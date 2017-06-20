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
#ifndef EDGE_UTILS_H
#define EDGE_UTILS_H

namespace utils {
  bool edgeIntersectsBox(// Input: arbitrary edge
                         double bx, double by,
                         double ex, double ey,
                         // Input: Box
                         double xl, double yl,
                         double xh, double yh
                         );
  bool edgeIntersectsHorizontalEdge(// Input: arbitrary edge
                                    double x0, double y0,
                                    double x1, double y1,
                                    // Input: horizontal edge
                                    double begx, double endx,
                                    double yval
                                    );
  bool isPointOnEdge(double x0, double y0, double x1, double y1,
                     double x, double y);
  bool collinearEdgesIntersect(// Input: first edge
                                    double ax0, double ay0,
                                    double ax1, double ay1,
                                    // Input: second edge
                                    double bx0, double by0,
                                    double bx1, double by1,
                                    // Output: intersection
                                    // if it exists
                                    double & x, double & y
                                    );
  bool edgesIntersect(// Input: first edge
                      double ax0, double ay0,
                      double ax1, double ay1,
                      // Input: second edge
                      double bx0, double by0,
                      double bx1, double by1,
                      // Output: intersection if it exists
                      double & x, double & y
                      );
  void cutEdge(double x0, double y0, double x1, double y1,
               double nx, double ny, double H,
               double & cutx, double & cuty);
  
}

#endif

