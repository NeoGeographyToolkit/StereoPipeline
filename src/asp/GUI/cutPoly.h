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
#ifndef CUT_POLY_H
#define CUT_POLY_H

#include <vector>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <asp/GUI/geomUtils.h>

namespace utils{

  struct valIndex{
    double val;
    int    index;
    bool   isOutward;
    int    nextIndexInward; // Useful only when isOutward is true
  };

  void cutPolyLine(// inputs -- the polygonal line
                   int numVerts,
                   const double * xv, const double * yv,
                   // inputs -- the cutting window
                   double xll, double yll, double xur, double yur,
                   // outputs -- the cut polygons
                   std::vector< double> & cutX,
                   std::vector< double> & cutY,
                   std::vector< int>    & cutNumPolys);
  
  void cutPoly(// inputs -- the polygons
               int numPolys, const int * numVerts,
               const double * xv, const double * yv,
               // inputs -- the cutting window
               double xll, double yll, double xur, double yur,
               // outputs -- the cut polygons
               std::vector< double> & cutX,
               std::vector< double> & cutY,
               std::vector< int>    & cutNumPolys);

  inline bool lessThan (valIndex A, valIndex B){ return A.val < B.val; }
  
  void processPointsOnCutline(std::vector<valIndex> & ptsOnCutline);

  void cutToHalfSpace(// inputs 
                      double nx, double ny, double dotH,
                      int numV, 
                      const double * xv, const double * yv,
                      // outputs -- the cut polygons
                      std::vector<double> & cutX,
                      std::vector<double> & cutY,
                      std::vector<int>    & cutNumPolys);

  
}
#endif
