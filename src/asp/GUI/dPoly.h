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
#ifndef DPOLY_H
#define DPOLY_H

#include <vector>
#include <map>
#include <asp/GUI/baseUtils.h>
#include <asp/GUI/geomUtils.h>


// A class holding a set of polygons in double precision
class dPoly{

public:

  dPoly(){
    reset();
  }

  void reset();

  bool read_pol_or_cnt_format(std::string filename,
                              std::string type,
                              bool isPointCloud = false
                              );

  bool readPoly(std::string filename,
                bool isPointCloud = false
                );

  void writePoly(std::string filename, std::string defaultColor = "yellow");
  void bdBoxCenter(double & mx, double & my) const;

  void appendPolygon(int numVerts,
                     const double * xv,
                     const double * yv,
                     bool isPolyClosed,
                     const std::string & color,
                     const std::string & layer
                     );

  void appendPolygons(const dPoly & poly);

  void appendRectangle(double xl, double yl, double xh, double yh,
                       bool isPolyClosed,
                       const std::string & color, const std::string & layer
                       );

  void setRectangle(double xl, double yl, double xh, double yh,
                    bool isPolyClosed,
                    const std::string & color, const std::string & layer
                    );

  bool isXYRect();

  void clipPoly(// inputs
                double clip_xll, double clip_yll,
                double clip_xur, double clip_yur,
                dPoly & clippedPoly // output
                );

  void shift(double shift_x, double shift_y);
  void rotate(double angle);
  void scale(double scale);
  void transformMarkedPolys(std::map<int, int> & mark, const utils::linTrans & T);
  void transformMarkedPolysAroundPt(std::map<int, int> & mark, const utils::matrix2 & M, dPoint P);
  void applyTransform(double a11, double a12, double a21, double a22,
                      double sx, double sy,
                      utils::linTrans & T // save the transform here
                      );
  void applyTransformAroundBdBoxCenter(double a11, double a12,
                                       double a21, double a22,
                                       utils::linTrans & T
                                       );

  const int    * get_numVerts         () const { return utils::vecPtr(m_numVerts); }
  const double * get_xv               () const { return utils::vecPtr(m_xv);       }
  const double * get_yv               () const { return utils::vecPtr(m_yv);       }
  int get_numPolys                    () const { return m_numPolys;                }
  int get_totalNumVerts               () const { return m_totalNumVerts;           }
  std::vector<char> get_isPolyClosed  () const { return m_isPolyClosed;            }
  std::vector<std::string> get_colors () const { return m_colors;                  }
  std::vector<std::string> get_layers () const { return m_layers;                  }

  void set_color(std::string color);

  void set_isPolyClosed(bool isPolyClosed);

  void eraseMarkedPolys(// Inputs
                        std::map<int, int> & mark
                        );
  void erasePolysIntersectingBox(double xll, double yll, double xur, double yur);
  void appendAndShiftMarkedPolys(// Inputs
                                 std::map<int, int> & mark,
                                 double shift_x, double shift_y
                                 );
  void set_isPointCloud(bool isPointCloud){ m_isPointCloud = isPointCloud; }
  bool isPointCloud() { return m_isPointCloud;}

  void set_pointCloud(const std::vector<dPoint> & P, std::string color,
                      std::string layer);
  void buildGrid(double xl, double yl, double xh, double yh,
                 double gridSize, std::string gridColor);
  void markPolysIntersectingBox(// Inputs
                                double xll, double yll,
                                double xur, double yur,
                                // Outputs
                                std::map<int, int> & mark
                                ) const;
  void replaceOnePoly(int polyIndex, int numV, const double* x, const double* y);
  // Annotations
  void get_annotations (std::vector<anno> & annotations) const;
  void get_layerAnno(std::vector<anno> & annotations) const;
  void get_vertIndexAnno(std::vector<anno> & annotations) const;
  void set_annotations(const std::vector<anno> & A);
  void set_layerAnno(const std::vector<anno> & annotations);
  void set_vertIndexAnno(const std::vector<anno> & annotations);

  void addAnno(const anno & A){m_annotations.push_back(A); }
  void compVertIndexAnno();
  void compLayerAnno();

  void bdBox(double & xll, double & yll, double & xur, double & yur) const;

  void bdBoxes(std::vector<double> & xll, std::vector<double> & yll,
               std::vector<double> & xur, std::vector<double> & yur) const;

  void setPolygon(int numVerts,
                  const double * xv,
                  const double * yv,
                  bool isPolyClosed,
                  const std::string & color,
                  const std::string & layer
                  );

  void eraseAnno(int annoIndex);

  void findClosestAnnotation(// inputs
                             double x0, double y0,
                             // outputs
                             int & annoIndex,
                             double & min_dist
                             ) const;

  void findClosestPolyVertex(// inputs
                             double x0, double y0,
                             // outputs
                             int & polyIndex,
                             int & vertIndex,
                             double & min_x, double & min_y,
                             double & min_dist
                             ) const;

  void findClosestPolyEdge(//inputs
                           double x0, double y0,
                           // outputs
                           int & polyIndex, int & vertIndex,
                           double & minX, double & minY, double & minDist
                           ) const;

  void eraseOnePoly(int polyIndex);
  void insertVertex(int polyIndex, int vertIndex,
                    double x, double y);
  void eraseVertex(int polyIndex, int vertIndex);
  void changeVertexValue(int polyIndex, int vertIndex, double x, double y);
  void shiftEdge(int polyIndex, int vertIndex, double shift_x, double shift_y);
  void shiftOnePoly(int polyIndex, double shift_x, double shift_y);
  void shiftMarkedPolys(const std::map<int, int> & mark, double shift_x, double shift_y);
  void extractOnePoly(int polyIndex, // input
                      dPoly & poly   // output
                      ) const;
  void extractMarkedPolys(std::map<int, int> & mark, // input
                          dPoly & polys              // output
                          ) const;
  void reverseOnePoly(int polyIndex);
  void sortFromLargestToSmallest();

  void sortBySizeAndMaybeAddBigContainingRect(// inputs
                                              double bigXll, double bigYll,
                                              double bigXur, double bigYur
                                              );

  void enforce45();

private:

  bool getColorInCntFile(const std::string & line, std::string & color);
  void get_annoByType(std::vector<anno> & annotations, int annoType);
  void set_annoByType(const std::vector<anno> & annotations, int annoType);

  // If isPointCloud is true, treat each point as a set of unconnected points
  bool                     m_isPointCloud;

  std::vector<double>      m_xv;
  std::vector<double>      m_yv;
  std::vector<int>         m_numVerts;
  int                      m_numPolys;
  int                      m_totalNumVerts;
  std::vector<char>        m_isPolyClosed;
  std::vector<std::string> m_colors;
  std::vector<std::string> m_layers;
  std::vector<anno>        m_annotations;
  std::vector<anno>        m_vertIndexAnno; // Anno showing vertex index
  std::vector<anno>        m_layerAnno;     // Anno showing layer number

};

#endif
