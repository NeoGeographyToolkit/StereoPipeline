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
#ifndef GEOMUTILS_H
#define GEOMUTILS_H
#include <sstream>
#include <vector>
#include <fstream>
#include <cmath>
#include <set>
#include <cassert>

struct dPoint{
  double x, y;
  dPoint(): x(0), y(0){}
  dPoint(double x_in, double y_in): x(x_in), y(y_in){}
};


inline bool operator< (dPoint P, dPoint Q){
  return ( P.x < Q.x ) || (P.x == Q.x && P.y < Q.y);
}

inline bool greaterThan (dPoint P, dPoint Q){
  return ( P.x > Q.x ) || (P.x == Q.x && P.y > Q.y);
}

struct anno {

  double x;
  double y;
  std::string label;

  void appendTo(std::ofstream & outfile) const{
    outfile << "anno " << x << ' ' << y << ' ' << label << std::endl;
  }

};

std::ostream& operator<<(std::ostream& os, const anno& A);

namespace utils{

  void snapPolyLineTo45DegAngles(bool isClosedPolyLine,
                                 int numVerts, double * xv, double * yv);
  void snapOneEdgeTo45(int numAngles, double* xs, double* ys,
                       bool snap2ndClosest,
                       double & x0, double & y0,
                       double & x1, double & y1);

  void minDistFromPtToSeg(//inputs
                          double xin, double yin,
                          double x0, double y0,
                          double x1, double y1,
                          // outputs
                          double & minX, double & minY,
                          double & minDist
                          );

  void searchForLayer(std::string   lineStr, // input
                      std::string & layer    // output
                      );

  double signedPolyArea(int numV, const double* xv, const double* yv);

  void searchForColor(std::string lineStr, // input, not a reference on purpose
                      std::string & color  // output
                      );

  bool searchForAnnotation(std::string lineStr, anno & annotation);

  void expandBoxToGivenRatio(// inputs
                             double aspectRatio,
                             // inputs/outputs
                             double & xll,  double & yll,
                             double & widx, double & widy);


  struct dRect{
    dRect(double xl_in = 0.0, double yl_in = 0.0,
          double xh_in = 0.0, double yh_in = 0.0):
      xl(xl_in), yl(yl_in), xh(xh_in), yh(yh_in) {}
    double xl, yl, xh, yh;
  };


  struct dRectWithId: public dRect{
    int id;
    dRectWithId(double xl_in = 0.0, double yl_in = 0.0,
                double xh_in = 0.0, double yh_in = 0.0,
                int id_in = 0):
      dRect(xl_in, yl_in, xh_in, yh_in), id(id_in){}
  };

  struct seg{
    double begx, begy, endx, endy;
    seg(double begx_in = 0.0, double begy_in = 0.0,
        double endx_in = 0.0, double endy_in = 0.0):
      begx(begx_in), begy(begy_in), endx(endx_in), endy(endy_in){}
  };


  struct segDist: public seg{
    double dist;
    segDist(double begx_in, double begy_in, double endx_in,
            double endy_in, double dist_in):
      seg(begx_in, begy_in, endx_in, endy_in), dist(dist_in){}
  };

  inline bool segDistGreaterThan(segDist s, segDist t){
    if (s.dist > t.dist) return true;
    if (s.dist < t.dist) return false;
    if (s.begx > t.begx) return true;
    if (s.begx < t.begx) return false;
    if (s.begy > t.begy) return true;
    if (s.begy < t.begy) return false;
    return false;
  }

  inline bool operator==(segDist s, segDist t){
    return ( s.dist == t.dist ) && ( s.begx == t.begx ) && ( s.begy == t.begy );
  }

  inline std::ostream& operator<<(std::ostream & output, const segDist & S) {
    output << S.begx << ' ' << S.begy << ' ' << S.endx << ' ' << S.endy << ' '
           << S.dist;
    return output;  // for multiple << operators
  }

  bool boxesIntersect(double xl1, double yl1, double xh1, double yh1,
                      double xl2, double yl2, double xh2, double yh2
                      );


  bool mergePolys(int an,
                  const double * ax_in, const double * ay_in,
                  int bn,
                  const double * bx_in, const double * by_in,
                  std::vector<double> & mergedX,
                  std::vector<double> & mergedY
                  );
  bool isPointInPolyOrOnEdges(double x, double y,
                              int n, const double* xv, const double*  yv);

  struct linTrans{
    // Linear transform
    double a11, a12, a21, a22, sx, sy;
    linTrans(){
      a11 = a22 = 1.0;
      a21 = a12 = sx = sy = 0.0;
    }
    void reset(){
      *this = linTrans();
    }
    void print(){
      std::cout << "transform " << a11 << ' ' << a12 << ' '
                << a21 << ' ' << a22 << ' ' << sx << ' ' << sy << std::endl;
    }
  };

  utils::linTrans composeTransforms(utils::linTrans P, utils::linTrans Q);

  struct matrix2{
    // A 2x2 matrix
    double a11, a12, a21, a22;
    matrix2(){
      a11 = a22 = 1.0;
      a21 = a12 = 0.0;
    }
    void reset(){
      *this = matrix2();
    }
    void print(){
      std::cout << "matrix " << a11 << ' ' << a12 << ' ' << a21 << ' ' << a22 << std::endl;
    }
  };

  utils::linTrans transAroundPt(const utils::matrix2 & M, dPoint P);

}


#endif
