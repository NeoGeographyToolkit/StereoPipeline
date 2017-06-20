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
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <cstring>
#include <cassert>
#include <asp/GUI/baseUtils.h>
#include <asp/GUI/geomUtils.h>
#include <asp/GUI/edgeUtils.h>

using namespace std;
using namespace utils;

std::ostream& operator<<(std::ostream& os, const anno& A){
  os << A.x << ' ' << A.y << ' ' << A.label << std::endl;
  return os;
}

void utils::snapPolyLineTo45DegAngles(bool isClosedPolyLine,
                                      int numVerts, double * xv, double * yv){

  // Given a polygonal line, transform it such that all vertices are
  // integers and all edges make an angle multiple of 45 degrees
  // with the x axis.

  if (numVerts <= 0) return;

  // The vectors corresponding to angles multiple of 45 degree
  int numAngles = 8;
  double xs[numAngles], ys[numAngles];
  for (int a = 0; a < numAngles; a++){
    double theta = a*45*M_PI/180;
    xs[a] = cos(theta);
    ys[a] = sin(theta);
  }

  // Snap first vertex to int grid
  xv[0] = round(xv[0]);
  yv[0] = round(yv[0]);

  for (int v = 0; v < numVerts - 1; v++){

    bool snap2ndClosest = false; // snap to closest, not second closest
    snapOneEdgeTo45(numAngles, xs, ys, snap2ndClosest,  // inputs
                    xv[v], yv[v], xv[v + 1], yv[v + 1]  // in-out
                    );

  }

  if (!isClosedPolyLine || numVerts < 3) return;

  // The poly line is closed. After vertex n - 1 we have vertex 0.
  // Form a closed polygon satisfying the requirements. To do that,
  // walk backwards from vertex 0 and adjust edges until all edges
  // intersect on the integer grid and make 45 degree angles.
  for (int v = numVerts; v >= 0; v--){
    int vp = (v + 1)            % numVerts;
    int vc = v                  % numVerts;
    int vn = (v + numVerts - 1) % numVerts;
    double xp = xv[vp], xc = xv[vc], xc2 = xv[vc], xn = xv[vn];
    double yp = yv[vp], yc = yv[vc], yc2 = yv[vc], yn = yv[vn];

    bool snap2ndClosest = false;
    snapOneEdgeTo45(numAngles, xs, ys, snap2ndClosest,  // inputs
                   xn, yn, xc2, yc2                     // in-out
                   );

    // Find the intersection of the edges
    // (xp, yp) --> (xc, yc) and (xc2, yc2) --> (xn, yn).
    double det = ( (xc-xp)*(yn-yc2)  - (yc-yp)*(xn-xc2)  );
    double top = ( (xc2-xp)*(yn-yc2) - (yc2-yp)*(xn-xc2) );
    double t = top/det;
    double xi = round( 2*( t*(xc-xp) + xp ) )/2.0;
    double yi = round( 2*( t*(yc-yp) + yp ) )/2.0;
    if (det != 0 &&  xi == round(xi) && yi == round(yi) ){
      // Finally arrived at a point at which all vertices
      // are on grid
      xv[vc] = xi;
      yv[vc] = yi;
      break;
    }
    // Adjust the edge going from vc to vn, and hope that at the next iteration
    // that edge intersects properly with the edge after it.
    xv[vn] += xc - xc2;
    yv[vn] += yc - yc2;
  }

  // Validate
  for (int v = 0; v < numVerts; v++){
    double dx = xv[(v+1)%numVerts] - xv[v];
    double dy = yv[(v+1)%numVerts] - yv[v];
    if ( xv[v] != round(xv[v]) ||
         yv[v] != round(yv[v]) ||
         !( dx == 0 || dy == 0 || abs(dx) == abs(dy) ) ){
      cerr << "Error: Expecting integer vertices with 45 degree angles."  << endl;
      cerr << "Instead, got the vector (" << dx << ", " << dy << ") "
           << "with starting point " << xv[v] << ' ' << yv[v] << endl;
      //assert(false);
    }
  }

  return;
}

void utils::snapOneEdgeTo45(int numAngles, double* xs, double* ys,
                            bool snap2ndClosest,
                            double & x0, double & y0,
                            double & x1, double & y1){


  double dx = x1 - x0, dy = y1 - y0;
  double len = distance(0, 0, dx, dy);
  if (len == 0.0) return;

  dx /= len;
  dy /= len;

  // Find the closest angle multiple of 45 degrees from (dx, dy)
  int minAngle   = 0;
  double minDist = DBL_MAX;
  for (int a = 0; a < numAngles; a++){
    double dist = distance(dx, dy, xs[a], ys[a]);
    if (dist <= minDist){
      minDist  = dist;
      minAngle = a;
    }
  }

  // We prefer to snap to the second closest angle if for some reason
  // we know that snapping to the closest angle does not work.
  if (snap2ndClosest){
    int minAngle2   = 0;
    double minDist2 = DBL_MAX;

    for (int a = 0; a < numAngles; a++){
      double dist = distance(dx, dy, xs[a], ys[a]);
      if (dist <= minDist2 && a != minAngle){
        minDist2  = dist;
        minAngle2 = a;
      }
    }
    minAngle = minAngle2;
  }

  // Snap to integer coordinates in the direction of minAngle
  double factor =  abs(xs[minAngle]) + abs(ys[minAngle]); // 1 or sqrt(2)
  len = factor*round(len/factor);
  x1 = x0 + round( len*xs[minAngle] );
  y1 = y0 + round( len*ys[minAngle] );

  return;
}

void utils::minDistFromPtToSeg(//inputs
                               double xin, double yin,
                               double x0, double y0,
                               double x1, double y1,
                               // outputs
                               double & minX, double & minY,
                               double & minDist
                               ){

  // Given the point (xin, yin) and the segment going from (x0, y0) to
  // (x1, y1), find the point (minX, minY) on this segment (not on its
  // continuation) closest to (xin, yin).

  double a = (x1  - x0)*(x1 - x0) + (y1  - y0)*(y1 - y0);
  double b = (xin - x0)*(x1 - x0) + (yin - y0)*(y1 - y0);

  double t;
  if (a == 0.0) t = 0.0;
  else          t = b/a;
  t = max(t, 0.0);
  t = min(t, 1.0);

  minX = x0 + t*(x1 - x0);
  minY = y0 + t*(y1 - y0);

  minDist = sqrt ( (xin  - minX)*(xin - minX) + (yin  - minY)*(yin - minY) );

  return;
}



void utils::searchForColor(std::string lineStr, // input, not a reference on purpose
                           std::string & color  // output
                           ){

//   const char * xgraph_colors[] =
//     {"black", "white", "red", "blue", "green", "violet",
//      "orange", "yellow", "pink", "cyan", "lightGray",
//      "darkGray", "fuchsia", "aqua", "navy", "gold"};

  const char * xgraph_colors[] =
    {"black", "white", "red", "blue", "green", "violet", // 0,  ..., 5
     "orange", "yellow", "pink", "cyan", "#A2B5CD",      // 6,  ..., 10
     "#6C7B8B", "#FF00FF", "#00CDCD", "navy", "gold"     // 11, ..., 15
    };

  char       * line  = (char*)lineStr.c_str();
  const char * col   = "color";
  char       * start = strstr(line, col);

  if (start == NULL) return;
  if (strlen(start) <= strlen(col)) return;
  start += strlen(col);

  // Strip the equal sign, quotes, etc.
  for (int i = 0; i < (int)strlen(start); i++){

    if (start[i] == '"' || start[i] == '=' || start[i] == '\''){
      start[i] = ' ';
    }

  }

  const char *delimiter = " \t";
  char * pch = strtok (start, delimiter);
  if (pch == NULL) return;

  color = string(pch);

  int numColors = sizeof(xgraph_colors)/sizeof(char*);

  // If the color is given as a number, per xgraph's conventions
  // (e.g., red is color 2), convert that number to the color name.
  if ('0' <= pch[0] && pch[0] <= '9'){
    int colorIndex = atoi(pch)%numColors;
    color = string( xgraph_colors[colorIndex] );
  }

  return;
}

bool utils::searchForAnnotation(std::string lineStr, anno & annotation){

  // Search for annotations, which have the form:
  // anno xval yval label
  // Return true on success.

  istringstream iss (lineStr);
  string an, label;
  double x, y;

  if ( ( !(iss >> an >> x >> y) ) || an != "anno" ){
    return false;
  }

  getline(iss, label); // Everything else goes to the label

  annotation.x     = x;
  annotation.y     = y;
  annotation.label = label;

  return true;
}

void utils::searchForLayer(std::string   lineStr, // input
                           std::string & layer    // output
                           ){

  layer = "";

  // We are searching for ";" followed by zero or more spaces,
  // followed by something like "65:0"
  char * line = (char *) lineStr.c_str();

  char * start1 = strstr(line, ";");
  if (start1 == NULL) return;
  start1++; // Move beyond ";"
  if (*start1 == '\0') return;

  int l1 = atoi(start1);

  char * start2 = strstr(start1, ":");
  if (start2 == NULL) return;
  start2++; // Move beyond ":"
  if (*start2 == '\0') return;

  int l2 = atoi(start2);

  ostringstream strout;
  strout << l1 << ':' << l2;
  layer = strout.str();

  return;
}

double utils::signedPolyArea(int numV, const double* xv, const double* yv){

  // Subtract the first vertex when computing the area to handle more
  // accurately polygons very far from the origin.

  double area = 0.0;

  for (int vIter = 0; vIter < numV; vIter++){

    int vNext = (vIter + 1)%numV;
    area += (xv[vIter] - xv[0])*(yv[vNext] - yv[0]) -
      (xv[vNext] - xv[0])*(yv[vIter] - yv[0]);

  }

  area /= 2.0;

  return area;
}

void utils::expandBoxToGivenRatio(// inputs
                                  double aspectRatio,
                                  // inputs/outputs
                                  double & xll,  double & yll,
                                  double & widx, double & widy){

  // Expand the given box to have the aspect ratio equal to the number aspectRatio.
  assert(widx > 0.0 && widy > 0.0 && aspectRatio > 0.0);
  double nwidx = widx, nwidy = widy;
  if (widy/widx <= aspectRatio) nwidy = widx*aspectRatio;
  else                          nwidx = widy/aspectRatio;

  // Sanity checks
  double tol = 1.0e-3;
  bool check = ( nwidx >= widx*(1 - tol) && nwidy >= widy*(1 - tol)
                 && abs(nwidy/nwidx - aspectRatio) < tol*aspectRatio );
  if (!check){
    cout << "ERROR!" << endl;
    cout << "widx widy are "   << widx  << ' ' << widy  << endl;
    cout << "nwidx nwidy are " << nwidx << ' ' << nwidy << endl;
    cout << "Aspect ratio is " << aspectRatio << endl;
    cout << "|nwidy/nwidx - aspectRatio| = " << abs(nwidy/nwidx - aspectRatio) << endl;
    cout << "Max allowed error is " << tol*aspectRatio << endl;
  }
  assert(check);

  // Make the new bounding box have the same center as the old one
  xll += widx/2.0 - nwidx/2.0;
  yll += widy/2.0 - nwidy/2.0;

  // Overwrite the previous box
  widx = nwidx;
  widy = nwidy;

  return;
}

bool utils::boxesIntersect(double xl1, double yl1, double xh1, double yh1,
                           double xl2, double yl2, double xh2, double yh2
                           ){

  assert(xl1 <= xh1 && yl1 <= yh1);
  assert(xl2 <= xh2 && yl2 <= yh2);

  return
    (
     std::max(xl1, xl2) <= std::min(xh1, xh2)
     &&
     std::max(yl1, yl2) <= std::min(yh1, yh2)
     );

}

utils::linTrans utils::composeTransforms(utils::linTrans P, utils::linTrans Q){

  // Composition of two transforms

  linTrans R;

  R.a11 = P.a11*Q.a11 + P.a12*Q.a21;
  R.a12 = P.a11*Q.a12 + P.a12*Q.a22;
  R.a21 = P.a21*Q.a11 + P.a22*Q.a21;
  R.a22 = P.a21*Q.a12 + P.a22*Q.a22;

  R.sx = P.a11*Q.sx + P.a12*Q.sy + P.sx;
  R.sy = P.a21*Q.sx + P.a22*Q.sy + P.sy;

  return R;
}

utils::linTrans utils::transAroundPt(const utils::matrix2 & M, dPoint P){

  // Find the linear transformation which applies a given matrix
  // transform around the current point (for example, rotation around
  // the current point).

  linTrans T;
  T.a11 = M.a11; T.a12 = M.a12;
  T.a21 = M.a21; T.a22 = M.a22;
  T.sx = P.x - M.a11*P.x - M.a12*P.y;
  T.sy = P.y - M.a21*P.x - M.a22*P.y;
  return T;
}

bool utils::mergePolys(int an,
                       const double * ax_in, const double * ay_in,
                       int bn,
                       const double * bx_in, const double * by_in,
                       std::vector<double> & mergedX,
                       std::vector<double> & mergedY
                       ){

  // Merge two polygons. This function is INCOMPLETE and BUGGY.
  // To be finished.

  mergedX.clear();
  mergedY.clear();

  // The tol value needs careful thinking
  double tol = 1e-12;

  // Copy the pointers to non-constant pointers so what we can swap
  // them.
  double* ax = (double*)ax_in; double* ay = (double*)ay_in;
  double* bx = (double*)bx_in; double* by = (double*)by_in;

  bool mergeWasSuccessful = false;

  int i = 0, in = 0, j = 0;

  // Start at a vertex of A which is not inside of B.
  for (int t = 0; t < an; t++){
    if (isPointInPolyOrOnEdges(ax[t], ay[t], bn, bx, by)) continue;
    i = t; in = t; j = t;
    break;
  }

  double sx = ax[i], sy = ay[i];
  double currX = ax[i], currY = ay[i];

  mergedX.push_back(currX);
  mergedY.push_back(currY);

  while(1){


    bool foundIntersection = false;
    double x = 0.0, y = 0.0;

    // Of all edges of poly B intersecting the current edge (if any)
    // of poly A find the one for which the intersection point
    // is closest to the start of the current edge of poly A.
    // Care is taken of situations when there is more than one
    // such edge.
    in = (i + 1)% an;
    j  = 0;
    // Make initial minDistA big on purpose by some value.
    double minDistA = 2.0*distance(currX, currY, ax[in], ay[in]) + 1.0;
    double minDistB_beg = -1.0, maxDistB_end = -1.0;
    for (int jl = 0; jl < bn; jl++){
      int jnl = (jl + 1) % bn;
      double xl, yl;
      if (edgesIntersect(currX, currY, ax[in], ay[in],
                         bx[jl], by[jl], bx[jnl], by[jnl],
                         xl, yl) &&
          isPointOnEdge(currX, currY, ax[in], ay[in], xl, yl) &&
          (abs(currX - xl) > tol || abs(currY - yl) > tol)
          ){
        foundIntersection  = true;
        mergeWasSuccessful = true;
        double distA     = distance(currX,  currY,  xl, yl);
        double distB_beg = distance(bx[jl], by[jl], xl, yl);
        double distB_end = distance(xl, yl, bx[jnl], by[jnl]);
        if (minDistB_beg < 0.0) minDistB_beg = distB_beg;
        if (maxDistB_end < 0.0) maxDistB_end = distB_end;
        if (distA < minDistA                                   ||
            (distA == minDistA && distB_beg < minDistB_beg)    ||
            (distA == minDistA && distB_beg == minDistB_beg
             && distB_end >= maxDistB_end )
            ){
          minDistA = distA;
          minDistB_beg = distB_beg;
          maxDistB_end = distB_end;
          j = jl; ; x = xl; y = yl;
        }
      }
    }

    if (!foundIntersection){
      i = in;
      if (sx == ax[i] && sy == ay[i]) break; // reached the starting point
      currX = ax[i]; currY = ay[i];
      mergedX.push_back(currX);
      mergedY.push_back(currY);
      continue;
    }

    currX = x;
    currY = y;
    mergedX.push_back(currX);
    mergedY.push_back(currY);
    swap(ax, bx);
    swap(ay, by);
    swap(an, bn);

    i = j;
    //if (sx == ax[i] && sy == ay[i]) break;
  }

  return mergeWasSuccessful;
}

bool utils::isPointInPolyOrOnEdges(double x, double y,
                                   int n, const double* xv, const double*  yv){

  // Is the given point either completely inside or on the edges
  // of the given polygon.

  if (n <= 0) return false;

  bool isInsideOrOnEdges = false;

  for (int i = 0; i < n; i++){
    int j = (i + 1)%n;

    double x0 = xv[i], x1 = xv[j];
    double y0 = yv[i], y1 = yv[j];

    if (x0 > x1){
      swap(x0, x1);
      swap(y0, y1);
    }

    if (x < x0 || x > x1) continue;

    if (x0 == x1){
      if (y >= min(y0, y1) && y <= max(y0, y1)) return true;
      else                                      continue;
    }

    double det = (y - y0)*(x1 - x0) - (x - x0)*(y1 - y0);
    if (det == 0) return true; // is on edge

    if (x < x1 && det < 0) isInsideOrOnEdges = !isInsideOrOnEdges;
  }

  return isInsideOrOnEdges;
}
