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


/// \file nff_terrain.cc
///

/************************************************************************/
/*     File: nff_terrain.c                                              */
/*     Date: August 1996                                                */
/*       By: Eric Zbinden                                               */
/* Modified: August 1999, by Larry Edwards                              */
/*      For: NASA Ames Research Center, Intelligent Mechanisms Group    */
/* Function: Main program for the stereo panorama pipeline              */
/*    Links:    stereo.h        stereo.c                                */
/*              filters.h       filters.c                               */
/*              stereo_lib.h    stereolib.c                             */
/*              model_lib.h     modellib.c                              */
/*              stereo.default                                          */
/*                                                                      */
/*    Notes: Stereo.c rebuild a 3D model of the world from a stereo     */
/*           Panorama. St_pan->filtering->disparity_map->range_map:     */
/*              -> dot cloud 3D model                                   */
/*              -> 3D .nff file                                         */
/*                                                                      */
/************************************************************************/

/* The following three includes are for debugging -LJE */
#include <sys/types.h>
#ifdef __APPLE__
#include <malloc/malloc.h>
#include <float.h>                         // for DBL_MAX & FLT_MAX
#else
#include <malloc.h>
#include <values.h>                        // for DBL_MAX & FLT_MAX
#endif
#include <assert.h>
#include <string.h> /* strrchr() */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// STL include
#include <stdexcept>
#include <vector>

// ASP includes:
#include <asp/asp_config.h>
#include <asp/Tools/nff_terrain.h>

// VW
#include <vw/Core/Exception.h>
#include <vw/Math/Vector.h>

struct PixelCoords
{
  PixelCoords() { row = col = 0; }
  PixelCoords(int x, int y) { row = y; col = x; }
  int PixelIndex(int imageWidth);

  int row,col;
};                      /* xy vector or vertex position */

#if defined(ASP_HAVE_PKG_OPENSCENEGRAPH) && ASP_HAVE_PKG_OPENSCENEGRAPH==1
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Geode>
#include <osg/Texture2D>
#include <osg/Image>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#endif

// STL includes:
#include <iostream>                        // debugging
#include <list>
#include <set>
#include <functional>

// Boost includes
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp> // For manipulating paths

using namespace std;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                          -- Constants --                                */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#define THETA_LIMIT 1E-3

enum { LeftSide = 0, RightSide = 1, BottomSide = 2, TopSide = 3 };
enum { RightToLeftDiagonal = 0, LeftToRightDiagonal = 1 };

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                        -- Type declarations --                          */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

typedef double Array3x3[3][3];

class Line
{
 public:
  double EvaluateLineFunction(POS2D point);
  void SetEquation(POS2D *pt1, POS2D *pt2);
  void SetEquation(PixelCoords *pt1, PixelCoords *pt2);
  void GetEquation(double coefficients[3]);

  double a,b,c;
};

class Plane
{
 public:
  double EvaluatePlaneFunction(POS3D *point);
  void SetEquation(POS3D *pt1, POS3D *pt2, POS3D *pt3);
  void GetEquation(double coefficients[4]);
 private:
  double a,b,c,d;
};

class QuadtreeNode
{
 public:
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  QuadtreeNode(PixelCoords lowerLeft, PixelCoords upperRight);
  void SetTriangleGradients(BUFFER *buffer, int width);
  void SetTrianglePlanes(BUFFER *buffer, int width);
  void SetTriangles(int leftToRight, BUFFER *buffer, int width);
  void SetError(double error);
  void AverageDistanceError(BUFFER *buffer, int width, double *error);
  void MaxDistanceError(BUFFER *buffer, int width, double *error);
  void OldGradientError(BUFFER *buffer, int width, POS2D *diff);
  void AverageGradientError(BUFFER *buffer, int width, double *error);
  void MaxGradientError(BUFFER *buffer, int width, double *error);
  double CalculateError(BUFFER *buffer, int width);
  void PickMinErrorTriangles(BUFFER *buffer, int width);
  QuadtreeNode *GetChild(int verticalSide, int horizontalSide);
  void SetChildNeighbors();
  int Subdivide();
  int Descendants(list<QuadtreeNode *> *nodes);
  int Leaves(list<QuadtreeNode *> *leaves);
  bool IsLeaf();
  bool HasGrandChildren();
  bool IsAdjacent(QuadtreeNode *other, int *adjacentSide);
  QuadtreeNode *GetAncestralNeighbor(int side);
  //   bool operator<(QuadtreeNode &x) { return (m_error < x.m_error); }
  //   bool operator>(QuadtreeNode &x) { return (m_error > x.m_error); }
  //   bool operator==(QuadtreeNode &x) { return (m_error == x.m_error); }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  QuadtreeNode *m_parent;
  list<QuadtreeNode *> *m_children;
  QuadtreeNode *m_neighbors[4];
  PixelCoords m_lowerLeft,m_upperRight;
  Line m_leftRightDiagonal,m_rightLeftDiagonal;
  bool m_diagonalIsLeftRight;
  POS2D m_triangleGradients[2];
  Plane m_trianglePlanes[2];
  bool m_goodSide[4];
  double m_error;
 protected:
 private:
  void SetExternalNeighbor(int parentSide, int childQualifier);
  bool OverLappingInterval(int aLeft, int aRight, int bLeft, int bRight);
};

class Quadtree
{
 public:
  Quadtree(BUFFER *buffer, int width, int height);
  void Subdivide(double threshold, int maxNumTriangles);
  void CreateTriangleMesh();
  QuadtreeNode *Root() { return m_root; }
  void Root(QuadtreeNode *root) { m_root = root; }

 protected:
 private:
  struct ContentsGreater
  {
    bool operator()(QuadtreeNode *x, QuadtreeNode *y)
    {
      return (x->m_error > y->m_error);
    }
  };
  QuadtreeNode *m_root;
  BUFFER *m_buffer;
  int m_bufferWidth;
  int m_bufferHeight;
};

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                 -- Forward function declarations --                     */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void AddQuadTriangles(QuadtreeNode *quad, BUFFER *buffer,
                             int width, int *numTriangles, int *numVertices,
                             int *indexMap);

static void MendTriangleMeshSeams(BUFFER *buffer, int width, int *indexMap,
                                  Quadtree *quadtree);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*             -- Global variables private to this file --                 */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                      -- For Debugging --                                */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// #if 0
// static void
// FindGradientMinMax(BUFFER *buffer, int size, double *min, double *max,
//                 char direction)
// {
//   pixel *disparities = buffer->disp;
//   int index;
//   double magGradient;

//   /* find first good pixel */
//   for (index = 0; index < size; index++)
//     if ((disparities[index] != 0) && (disparities[index] != MISSING_PIXEL) &&
//      (disparities[index] != FAR_FIELD_PIXEL))
//       break;

//   switch (direction)
//   {
//   case 'x':
//     *min = *max = buffer->gradients[index++].x;
//     for (; index < size; index++)
//     {
//       if ((disparities[index] != 0) && (disparities[index] != MISSING_PIXEL) &&
//        (disparities[index] != FAR_FIELD_PIXEL)) {
//      if (buffer->gradients[index].x < *min) {
//        *min = buffer->gradients[index].x;
//      }
//      else if (buffer->gradients[index].x > *max) {
//        *max = buffer->gradients[index].x;
//      }
//       }
//     }
//     break;
//   case 'y':
//     *min = *max = buffer->gradients[index++].y;
//     for (; index < size; index++)
//     {
//       if ((disparities[index] != 0) && (disparities[index] != MISSING_PIXEL) &&
//        (disparities[index] != FAR_FIELD_PIXEL)) {
//      if (buffer->gradients[index].y < *min) {
//           *min = buffer->gradients[index].y;
//         }
//         else if (buffer->gradients[index].y > *max) {
//        *max = buffer->gradients[index].y;
//      }
//       }
//     }
//     break;
//   case 'm':
//     *min = *max = sqrt(buffer->gradients[index].x*buffer->gradients[index].x +
//                     buffer->gradients[index].y*buffer->gradients[index].y);
//     index++;
//     for (; index < size; index++)
//     {
//       if ((disparities[index] != 0) && (disparities[index] != MISSING_PIXEL) &&
//        (disparities[index] != FAR_FIELD_PIXEL))
//       {
//      magGradient =
//        sqrt(buffer->gradients[index].x*buffer->gradients[index].x +
//             buffer->gradients[index].y*buffer->gradients[index].y);
//      if (magGradient < *min)
//        *min = magGradient;
//      else if (magGradient > *max)
//        *max = magGradient;
//       }
//     }
//   }
// }

// static void
// FindGradientMax(BUFFER *buffer, int size, double *max, char direction)
// {
//   pixel *disparities = buffer->disp;
//   int index;
//   double magGradient;

//   /* find first good pixel */
//   for (index = 0; index < size; index++)
//     if ((disparities[index] != 0) && (disparities[index] != MISSING_PIXEL) &&
//      (disparities[index] != FAR_FIELD_PIXEL))
//       break;

//   switch (direction)
//   {
//   case 'x':
//     *max = fabs(buffer->gradients[index++].x);
//     for (; index < size; index++)
//     {
//       if (buffer->gradients[index].x > *max)
//      *max = fabs(buffer->gradients[index].x);
//     }
//     break;
//   case 'y':
//     *max = fabs(buffer->gradients[index++].y);
//     for (; index < size; index++)
//     {
//       if ((disparities[index] != 0) && (disparities[index] != MISSING_PIXEL) &&
//        (disparities[index] != FAR_FIELD_PIXEL))
//      if (buffer->gradients[index].y > *max)
//        *max = fabs(buffer->gradients[index].y);
//     }
//     break;
//   case 'm':
//     *max = sqrt(buffer->gradients[index].x*buffer->gradients[index].x +
//              buffer->gradients[index].y*buffer->gradients[index].y);
//     index++;
//     for (; index < size; index++)
//     {
//       magGradient = sqrt(buffer->gradients[index].x*buffer->gradients[index].x +
//                       buffer->gradients[index].y*buffer->gradients[index].y);
//       if (magGradient > *max)
//      *max = magGradient;
//     }
//   }
// }

// static void
// MapGradientToImage(BUFFER *buffer, int size, double min, double max,
//                 int direction, pixel *imageBuffer)
// {
//   double absGrad, scale, offset, range = max - min;
//   pixel *disparities = buffer->disp;
//   double fudge = 0.5;
//   int i;

//   scale = 255.0/range;
//   offset = -min*scale;

//   switch (direction)
//   {
//   case 'x':
//     for (i=0; i < size; i++)
//     {
//       if ((disparities[i] != 0) && (disparities[i] != MISSING_PIXEL) &&
//        (disparities[i] != FAR_FIELD_PIXEL))
//       {
//      absGrad = fabs(buffer->gradients[i].x);
//      imageBuffer[i] = scale*absGrad + offset;
//       }
//       else
//      imageBuffer[i] = min;
//     }
//     break;
//   case 'y':
//     for (i=0; i < size; i++)
//     {
//       if ((disparities[i] != 0) && (disparities[i] != MISSING_PIXEL) &&
//        (disparities[i] != FAR_FIELD_PIXEL))
//       {
//      absGrad = fabs(buffer->gradients[i].y);
//      imageBuffer[i] = scale*absGrad + offset;
//       }
//       else
//      imageBuffer[i] = min;
//     }
//     break;
//   case 'm':
//     for (i=0; i < size; i++)
//     {
//       if ((disparities[i] != 0) && (disparities[i] != MISSING_PIXEL) &&
//        (disparities[i] != FAR_FIELD_PIXEL))
//       {
//      absGrad = sqrt(buffer->gradients[i].x*buffer->gradients[i].x +
//                     buffer->gradients[i].y*buffer->gradients[i].y);
//      if ((fudge*scale*absGrad + offset) <= 255.0)
//        imageBuffer[i] = fudge*scale*absGrad + offset;
//      else
//        imageBuffer[i] = 255;
//       }
//       else
//      imageBuffer[i] = min;
//     }
//     break;
//   }
// }
// #endif

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                        -- PixelCoord functions --                       */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

int
PixelCoords::PixelIndex(int imageWidth)
{
  return (row * imageWidth + col);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                        -- Line functions --                             */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

double
Line::EvaluateLineFunction(POS2D point)
{
  /* If a^2 + b^2 == 1 then the value returned is the signed distance of */
  /* the point to the line. */
  /* If c < 0 then positive values mean the point is on the same side */
  /* as the normal. */
  return (a*point.x + b*point.y + c);
}

void
Line::SetEquation(POS2D *pt1, POS2D *pt2)
{
  double deltaX = pt2->x - pt1->x;
  double deltaY = pt2->y - pt1->y;

  a = deltaY;
  b = -deltaX;
  c = -a*pt1->x - b*pt1->y;
}

void
Line::SetEquation(PixelCoords *pt1, PixelCoords *pt2)
{
  double deltaX = pt2->col - pt1->col;
  double deltaY = pt2->row - pt1->row;

  a = deltaY;
  b = -deltaX;
  c = -a * ((double) pt1->col) - b * ((double) pt1->row);
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                        -- Plane functions --                            */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

double
Plane::EvaluatePlaneFunction(POS3D *point)
{
  /* If a^2 + b^2 + c^2 == 1 then the value returned is the signed distance */
  /* of the point to the plane. */
  /* If d < 0 then positive values mean the point is on the same side */
  /* as the normal. */
  return (a*point->x + b*point->y + c*point->z + d);
}

void
Plane::SetEquation(POS3D *pt1, POS3D *pt2, POS3D *pt3)
{
  POS3D normal,*v1, *v2;
  double normalizer,mag;

  /* This is Newell's method for generating a plane equation */
  v1 = pt1;
  v2 = pt2;
  normal.x = (v1->y - v2->y) * (v1->z + v2->z);
  normal.y = (v1->z - v2->z) * (v1->x + v2->x);
  normal.z = (v1->x - v2->x) * (v1->y + v2->y);
  v1 = pt2;
  v2 = pt3;
  normal.x += (v1->y - v2->y) * (v1->z + v2->z);
  normal.y += (v1->z - v2->z) * (v1->x + v2->x);
  normal.z += (v1->x - v2->x) * (v1->y + v2->y);
  v1 = pt3;
  v2 = pt1;
  normal.x += (v1->y - v2->y) * (v1->z + v2->z);
  normal.y += (v1->z - v2->z) * (v1->x + v2->x);
  normal.z += (v1->x - v2->x) * (v1->y + v2->y);

  if ((mag = sqrt(normal.x*normal.x + normal.y*normal.y +
                  normal.z*normal.z)) == 0)
  {
    /* If this happens points are geometrically coincident, and we
       arbitrarily pick a normal */
    /* printf("Warning in SetEquation: points are "
       "geometrically concident.\n"); */
    a = b = 0.0;
    c = 1.0;
    d = 0.0;
    return;
  }
  normalizer = 1.0/mag;
  a = normal.x * normalizer;
  b = normal.y * normalizer;
  c = normal.z * normalizer;
  d = -(pt1->x*a + pt1->y*b + pt1->z*c);
}

void
Plane::GetEquation(double coefficients[4])
{
  coefficients[0] = a;
  coefficients[1] = b;
  coefficients[2] = c;
  coefficients[3] = d;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                     -- QuadtreeNode Functions --                        */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

QuadtreeNode::QuadtreeNode(PixelCoords lowerLeft, PixelCoords upperRight)
{                               /* QuadtreeNode constructor */
  PixelCoords lowerRight(upperRight.col,lowerLeft.row);
  PixelCoords upperLeft(lowerLeft.col,upperRight.row);

  m_parent = 0;
  m_children = new list<QuadtreeNode *>;
  m_neighbors[0] = m_neighbors[1] = m_neighbors[2] = m_neighbors[3] = 0;

  m_lowerLeft = lowerLeft;
  m_upperRight = upperRight;

  /* diagonal from lower left to upper right */
  m_leftRightDiagonal.SetEquation(&lowerLeft,&upperRight);
  /* diagonal from lower right to upper left */
  m_rightLeftDiagonal.SetEquation(&lowerRight,&upperLeft);

  /* arbitrarily choose one diagonal initially */
  m_diagonalIsLeftRight = 1;
  m_goodSide[0] = m_goodSide[1] = m_goodSide[2] = m_goodSide[3] = false;
  m_error = 0.0;
}

void
QuadtreeNode::SetTriangleGradients(BUFFER *buffer, int width)
{
  POS3D lowerLeft,lowerRight,upperLeft,upperRight;
  int pixelIndex, rowStartIndex = m_lowerLeft.row * width;

  pixelIndex = rowStartIndex + m_lowerLeft.col;
  lowerLeft = buffer->dot[pixelIndex];
  pixelIndex = rowStartIndex + m_upperRight.col;
  lowerRight = buffer->dot[pixelIndex];

  rowStartIndex = m_upperRight.row *width;
  pixelIndex = rowStartIndex + m_lowerLeft.col;
  upperLeft = buffer->dot[pixelIndex];
  pixelIndex = rowStartIndex + m_upperRight.col;
  upperRight = buffer->dot[pixelIndex];

  if (m_diagonalIsLeftRight)
  {
    m_triangleGradients[0].x = lowerRight.z - lowerLeft.z;
    m_triangleGradients[0].y = upperRight.z - lowerRight.z;

    m_triangleGradients[1].x = upperRight.z - upperLeft.z;
    m_triangleGradients[1].y = upperLeft.z - lowerLeft.z;
  }
  else
  {
    m_triangleGradients[0].x = upperRight.z - upperLeft.z;
    m_triangleGradients[0].y = upperRight.z - lowerRight.z;

    m_triangleGradients[1].x = lowerRight.z - lowerLeft.z;
    m_triangleGradients[1].y = upperLeft.z - lowerLeft.z;
  }
}

void
QuadtreeNode::SetTrianglePlanes(BUFFER *buffer, int width)
{
  POS3D lowerLeft,lowerRight,upperLeft,upperRight;
  int pixelIndex, rowStartIndex = m_lowerLeft.row * width;

  pixelIndex = rowStartIndex + m_lowerLeft.col;
  lowerLeft = buffer->dot[pixelIndex];
  pixelIndex = rowStartIndex + m_upperRight.col;
  lowerRight = buffer->dot[pixelIndex];

  rowStartIndex = m_upperRight.row *width;
  pixelIndex = rowStartIndex + m_lowerLeft.col;
  upperLeft = buffer->dot[pixelIndex];
  pixelIndex = rowStartIndex + m_upperRight.col;
  upperRight = buffer->dot[pixelIndex];

  if (m_diagonalIsLeftRight)
  {
    m_trianglePlanes[0].SetEquation(&lowerLeft,&lowerRight,&upperRight);
    m_trianglePlanes[1].SetEquation(&upperLeft,&lowerLeft,&upperRight);
  }
  else
  {
    m_trianglePlanes[0].SetEquation(&upperRight,&upperLeft,&lowerRight);
    m_trianglePlanes[1].SetEquation(&lowerLeft,&lowerRight,&upperLeft);
  }
}

void
QuadtreeNode::SetTriangles(int leftToRight, BUFFER *buffer, int width)
{
  m_diagonalIsLeftRight = leftToRight;
  SetTriangleGradients(buffer,width);
  SetTrianglePlanes(buffer,width);
}

void
QuadtreeNode::SetError(double error)
{
  m_error = error;
}

void
QuadtreeNode::AverageDistanceError(BUFFER *buffer, int width, double *error)
{
  int row,col,rowStartIndex,pixelIndex,numPixels;
  int minRow,minCol,stopRow,stopCol,quadHeight,quadWidth;
  POS2D point;
  Line *diagonal;
  double d;

  quadWidth = m_upperRight.col - m_lowerLeft.col + 1;
  quadHeight = m_upperRight.row - m_lowerLeft.row + 1;
  numPixels = quadWidth*quadHeight;
  minRow = m_lowerLeft.row;
  minCol = m_lowerLeft.col;
  stopRow = minRow+quadHeight;
  stopCol = minCol+quadWidth;
  rowStartIndex = minRow*width;

  diagonal = (m_diagonalIsLeftRight ?
              &(m_leftRightDiagonal) : &(m_rightLeftDiagonal));

  *error = 0.0;
  for (row = minRow; row < stopRow; rowStartIndex += width, row++)
  {
    for (col = minCol; col < stopCol; col++)
    {
      pixelIndex = rowStartIndex+col;
      point.x = col;
      point.y = row;

      if ((d = diagonal->EvaluateLineFunction(point)) > 0.0)
        *error += fabs(m_trianglePlanes[0].
                       EvaluatePlaneFunction(&buffer->dot[pixelIndex]));
      else if (d <= 0.0)
        *error += fabs(m_trianglePlanes[1].
                       EvaluatePlaneFunction(&buffer->dot[pixelIndex]));
    }
  }
  /* we average the distance error... */
  *error = *error / (double) numPixels;
}

void
QuadtreeNode::MaxDistanceError(BUFFER *buffer, int width, double *error)
{
  int row,col,rowStartIndex,pixelIndex;
  int minRow,minCol,stopRow,stopCol,quadHeight,quadWidth;
  POS2D point;
  Line *diagonal;
  double dist;

  quadWidth = m_upperRight.col - m_lowerLeft.col + 1;
  quadHeight = m_upperRight.row - m_lowerLeft.row + 1;
  minRow = m_lowerLeft.row;
  minCol = m_lowerLeft.col;
  stopRow = minRow+quadHeight;
  stopCol = minCol+quadWidth;
  rowStartIndex = minRow*width;

  diagonal = (m_diagonalIsLeftRight ?
              &(m_leftRightDiagonal) : &(m_rightLeftDiagonal));

  *error = 0.0;
  for (row = minRow; row < stopRow; rowStartIndex += width, row++)
  {
    for (col = minCol; col < stopCol; col++)
    {
      pixelIndex = rowStartIndex+col;
      point.x = col;
      point.y = row;

      if ((diagonal->EvaluateLineFunction(point)) > 0.0)
        dist = fabs(m_trianglePlanes[0].
                    EvaluatePlaneFunction(&buffer->dot[pixelIndex]));
      else
        dist = fabs(m_trianglePlanes[1].
                    EvaluatePlaneFunction(&buffer->dot[pixelIndex]));
      if (dist > *error)
        *error = dist;
    }
  }
}

/* Can do this calc. (at least) two different ways: find the max */
/* error, or find the average error. I think the latter method */
/* should give better results */

void
QuadtreeNode::OldGradientError(BUFFER *buffer, int width, POS2D *diff)
{
  int row,col,rowStartIndex,pixelIndex,numPixels;
  int minRow,minCol,stopRow,stopCol,quadHeight,quadWidth;
  POS2D point;
  Line *diagonal;
  double d;

  quadWidth = m_upperRight.col - m_lowerLeft.col + 1;
  quadHeight = m_upperRight.row - m_lowerLeft.row + 1;
  numPixels = quadWidth*quadHeight;
  minRow = m_lowerLeft.row;
  minCol = m_lowerLeft.col;
  stopRow = minRow+quadHeight;
  stopCol = minCol+quadWidth;
  rowStartIndex = minRow*width;

  diagonal = (m_diagonalIsLeftRight ?
              &(m_leftRightDiagonal) : &(m_rightLeftDiagonal));

  diff->x = diff->y = 0.0;
  for (row = minRow; row < stopRow; rowStartIndex += width, row++)
  {
    for (col = minCol; col < stopCol; col++)
    {
      pixelIndex = rowStartIndex+col;
      point.x = col;
      point.y = row;

      if ((d = diagonal->EvaluateLineFunction(point)) > 0.0)
      {
        diff->x += buffer->gradients[pixelIndex].x - m_triangleGradients[0].x;
        diff->y += buffer->gradients[pixelIndex].y - m_triangleGradients[0].y;
      }
      else if (d < 0.0)
      {
        diff->x += buffer->gradients[pixelIndex].x - m_triangleGradients[1].x;
        diff->y += buffer->gradients[pixelIndex].y - m_triangleGradients[1].y;
      }
      else                      /* we average the triangle gradients */
      {
        diff->x += (buffer->gradients[pixelIndex].x -
                    0.5*(m_triangleGradients[0].x + m_triangleGradients[1].x));

        diff->y += (buffer->gradients[pixelIndex].y -
                    0.5*(m_triangleGradients[0].y + m_triangleGradients[1].y));
      }
    }
  }
  /* we average the gradient error... */
  diff->x = diff->x / (double) numPixels;
  diff->y = diff->y / (double) numPixels;
}

void
QuadtreeNode::AverageGradientError(BUFFER *buffer, int width, double *error)
{
  int row,col,rowStartIndex,pixelIndex,numPixels;
  int minRow,minCol,stopRow,stopCol,quadHeight,quadWidth;
  POS2D diff,point;
  Line *diagonal;
  double d;

  quadWidth = m_upperRight.col - m_lowerLeft.col + 1;
  quadHeight = m_upperRight.row - m_lowerLeft.row + 1;
  numPixels = quadWidth*quadHeight;
  minRow = m_lowerLeft.row;
  minCol = m_lowerLeft.col;
  stopRow = minRow+quadHeight;
  stopCol = minCol+quadWidth;
  rowStartIndex = minRow*width;

  diagonal = (m_diagonalIsLeftRight ?
              &(m_leftRightDiagonal) : &(m_rightLeftDiagonal));

  *error = 0.0;
  for (row = minRow; row < stopRow; rowStartIndex += width, row++)
  {
    for (col = minCol; col < stopCol; col++)
    {
      pixelIndex = rowStartIndex+col;
      point.x = col;
      point.y = row;

      if ((d = diagonal->EvaluateLineFunction(point)) > 0.0)
      {
        diff.x = buffer->gradients[pixelIndex].x - m_triangleGradients[0].x;
        diff.y = buffer->gradients[pixelIndex].y - m_triangleGradients[0].y;
      }
      else if (d < 0.0)
        {
          diff.x = buffer->gradients[pixelIndex].x - m_triangleGradients[1].x;
          diff.y = buffer->gradients[pixelIndex].y - m_triangleGradients[1].y;
        }
      else                      /* we average the triangle gradients */
        {
          diff.x = (buffer->gradients[pixelIndex].x -
                    0.5*(m_triangleGradients[0].x + m_triangleGradients[1].x));

          diff.y = (buffer->gradients[pixelIndex].y -
                    0.5*(m_triangleGradients[0].y + m_triangleGradients[1].y));
        }
      *error += sqrt(diff.x*diff.x + diff.y*diff.y);
    }
  }
  /* we average the gradient error... */
  *error = *error / (double) numPixels;
}

void
QuadtreeNode::MaxGradientError(BUFFER *buffer, int width, double *error)
{
  int row,col,rowStartIndex,pixelIndex;
  int minRow,minCol,stopRow,stopCol,quadHeight,quadWidth;
  POS2D diff,point;
  Line *diagonal;
  double d,magSqrDiff;

  quadWidth = m_upperRight.col - m_lowerLeft.col + 1;
  quadHeight = m_upperRight.row - m_lowerLeft.row + 1;
  minRow = m_lowerLeft.row;
  minCol = m_lowerLeft.col;
  stopRow = minRow+quadHeight;
  stopCol = minCol+quadWidth;
  rowStartIndex = minRow*width;

  diagonal = (m_diagonalIsLeftRight ?
              &(m_leftRightDiagonal) : &(m_rightLeftDiagonal));

  *error = 0.0;
  for (row = minRow; row < stopRow; rowStartIndex += width, row++)
    {
      for (col = minCol; col < stopCol; col++)
    {
      pixelIndex = rowStartIndex+col;
      point.x = col;
      point.y = row;

      if ((d = diagonal->EvaluateLineFunction(point)) > 0.0)
      {
        diff.x = buffer->gradients[pixelIndex].x - m_triangleGradients[0].x;
        diff.y = buffer->gradients[pixelIndex].y - m_triangleGradients[0].y;
      }
      else if (d < 0.0)
      {
        diff.x = buffer->gradients[pixelIndex].x - m_triangleGradients[1].x;
        diff.y = buffer->gradients[pixelIndex].y - m_triangleGradients[1].y;
      }
      else                      /* we average the triangle gradients */
      {
        diff.x = (buffer->gradients[pixelIndex].x -
                  0.5*(m_triangleGradients[0].x + m_triangleGradients[1].x));

        diff.y = (buffer->gradients[pixelIndex].y -
                  0.5*(m_triangleGradients[0].y + m_triangleGradients[1].y));
      }
      if ((magSqrDiff = (diff.x*diff.x + diff.y*diff.y)) > *error)
        *error = magSqrDiff;
    }
  }
  *error = sqrt(*error);
}

double
QuadtreeNode::CalculateError(BUFFER *buffer, int width)
{
  double error;

  AverageDistanceError(buffer,width,&error);
#if 0
  MaxDistanceError(buffer,width,&error);
  MaxGradientError(buffer,width,&error);
  AverageGradientError(buffer,width,&error);
#endif
  return (error);
}

void
QuadtreeNode::PickMinErrorTriangles(BUFFER *buffer, int width)
{
  double error,minError;

  SetTriangles(RightToLeftDiagonal,buffer,width);
  minError = CalculateError(buffer,width);

  SetTriangles(LeftToRightDiagonal,buffer,width);
  if ((error = CalculateError(buffer,width)) < minError)
    minError = error;
  else                          /* use the original 2 triangles we tried */
    SetTriangles(RightToLeftDiagonal,buffer,width);

  SetError(minError);
}

QuadtreeNode *
QuadtreeNode::GetChild(int verticalSide, int horizontalSide)
{
  int numChildren,quadWidth,i,iStop;
  list<QuadtreeNode *>::iterator element;

  if (m_children->empty())
    return (0);

  numChildren = m_children->size();
  element = m_children->begin();
  if (numChildren == 4)
  {
    if (horizontalSide == LeftSide)
      iStop = (verticalSide == TopSide) ? 2 : 0;
    else
      iStop = (verticalSide == TopSide) ? 3 : 1;
  }
  else if (numChildren == 2)
  {
    quadWidth = m_upperRight.col - m_lowerLeft.col + 1;
    if (quadWidth > 2)
      iStop = (horizontalSide == LeftSide) ? 0 : 1;
    else
      iStop = (verticalSide == BottomSide) ? 0 : 1;
  }
  else                          /* should never happen */
  {
    printf("ERROR in GetChild: numChildren(=%d) is not 0, 2, or 4. "
           "Aborting.\n",numChildren);
    return (0);
  }

  for (i = 0; i < iStop; i++)
    ++element;

  return (*element);
}

void
QuadtreeNode::SetExternalNeighbor(int parentSide, int childQualifier)
{
  QuadtreeNode *neighborChild;
  int neighborSide;

  if (m_parent == 0)
    return;

  if (m_parent->m_neighbors[parentSide] == 0)
  {
    m_neighbors[parentSide] = 0;
    return;
  }
  if (parentSide > RightSide)
  {
    neighborSide = BottomSide+TopSide - parentSide;
    neighborChild =
      m_parent->m_neighbors[parentSide]->GetChild(neighborSide,childQualifier);
  }
  else
  {
    neighborSide = LeftSide+RightSide - parentSide;
    neighborChild =
      m_parent->m_neighbors[parentSide]->GetChild(childQualifier,neighborSide);
  }

  m_neighbors[parentSide] = neighborChild;
  if (neighborChild != 0)
    neighborChild->m_neighbors[neighborSide] = this;
}

void
QuadtreeNode::SetChildNeighbors()
{
  QuadtreeNode *children[4];
  list<QuadtreeNode *>::iterator element;
  int i,numChildren, quadWidth = m_upperRight.col - m_lowerLeft.col + 1;

  for (i = 0, element = m_children->begin(); !(element == m_children->end());
       element++, i++)
    children[i] = *element;
  numChildren = i;

  /* external neighbors common to both 4 child and 2 child subdivisions */
  children[0]->SetExternalNeighbor(LeftSide,BottomSide);
  children[0]->SetExternalNeighbor(BottomSide,LeftSide);
  if (numChildren == 4)
  {
    /* external neighbors */
    children[1]->SetExternalNeighbor(BottomSide,RightSide);
    children[1]->SetExternalNeighbor(RightSide,BottomSide);
    children[2]->SetExternalNeighbor(LeftSide,TopSide);
    children[2]->SetExternalNeighbor(TopSide,LeftSide);
    children[3]->SetExternalNeighbor(RightSide,TopSide);
    children[3]->SetExternalNeighbor(TopSide,RightSide);
    /* internal neigbors */
    children[0]->m_neighbors[RightSide] = children[1];
    children[1]->m_neighbors[LeftSide] = children[0];
    children[0]->m_neighbors[TopSide] = children[2];
    children[2]->m_neighbors[BottomSide] = children[0];
    children[1]->m_neighbors[TopSide] = children[3];
    children[3]->m_neighbors[BottomSide] = children[1];
    children[2]->m_neighbors[RightSide] = children[3];
    children[3]->m_neighbors[LeftSide] = children[2];
  }
  else if (numChildren == 2)    /* numChildren should always be 2 or 4 */
  {
    if (quadWidth < 3)
    {
      /* external neighbors */
      children[0]->SetExternalNeighbor(RightSide,BottomSide);
      children[1]->SetExternalNeighbor(LeftSide,TopSide);
      children[1]->SetExternalNeighbor(RightSide,TopSide);
      children[1]->SetExternalNeighbor(TopSide,RightSide);
      /* internal neighbors */
      children[1]->m_neighbors[BottomSide] = children[0];
      children[0]->m_neighbors[TopSide] = children[1];
    }
    else
    {
      /* external neighbors */
      children[0]->SetExternalNeighbor(TopSide,LeftSide);
      children[1]->SetExternalNeighbor(RightSide,TopSide);
      children[1]->SetExternalNeighbor(TopSide,RightSide);
      children[1]->SetExternalNeighbor(BottomSide,RightSide);
      children[1]->m_neighbors[RightSide] = m_neighbors[RightSide];
      children[1]->m_neighbors[TopSide] = m_neighbors[TopSide];
      children[1]->m_neighbors[BottomSide] = m_neighbors[BottomSide];

      /* internal neighbors */
      children[0]->m_neighbors[RightSide] = children[1];
      children[1]->m_neighbors[LeftSide] = children[0];
    }
  }
  else
  {
    printf("ERROR in SetChildNeighbors: numChildren is not 2 or 4. "
           "Aborting.\n");
  }
}

int
QuadtreeNode::Subdivide()
{
  PixelCoords leftMiddle,lowerMiddle,rightMiddle,upperMiddle,middle;
  QuadtreeNode *newQuadtreeNodes[4] = {0,0,0,0};
  int quadWidth,quadHeight, numChildren = 0;
//   int i,j;                   /* debugging */

  if (!m_children->empty()) /* only subdivide leaf nodes */
  {
    printf("WARNING in Subdivide(): this QuadtreeNode already has children. "
           "Aborting.\n");
    return (numChildren);
  }

  /* cannot subdivide if quad has <= 4 pixels */
  /* also must have at least 2 pixels in both dimensions */
  quadWidth = m_upperRight.col - m_lowerLeft.col + 1;
  quadHeight = m_upperRight.row - m_lowerLeft.row + 1;
  if ((quadWidth < 2) || (quadHeight < 2) ||
      ((quadWidth < 3) && (quadHeight < 3)))
    return (numChildren);

  /* set indices of new quad corners */
  middle.row = (m_lowerLeft.row + m_upperRight.row)/2;
  middle.col = (m_lowerLeft.col + m_upperRight.col)/2;
  if (quadHeight == 2)
    middle.row += 1;
  if (quadWidth == 2)
    middle.col += 1;
  leftMiddle.row = middle.row;
  leftMiddle.col = m_lowerLeft.col;
  rightMiddle.row = middle.row;
  rightMiddle.col = m_upperRight.col;
  lowerMiddle.row = m_lowerLeft.row;
  lowerMiddle.col = middle.col;
  upperMiddle.row = m_upperRight.row;
  upperMiddle.col = middle.col;

  /* the order of the children will be:
     lower-left, lower-right, upper-left, upper-right */

  /*
   * If the mesh elements are much wider than they are long, or vice versa,
   * we should subdivide the mesh in the direction that makes the elements more
   * square.
   */
  if (quadWidth > (2 * quadHeight))
  {
    newQuadtreeNodes[0] = new QuadtreeNode(m_lowerLeft,upperMiddle);
    newQuadtreeNodes[0]->m_parent = this;
    m_children->push_back(newQuadtreeNodes[0]);
    numChildren++;

    if (quadWidth > 2)
    {
      newQuadtreeNodes[1] = new QuadtreeNode(lowerMiddle,m_upperRight);
      newQuadtreeNodes[1]->m_parent = this;
      m_children->push_back(newQuadtreeNodes[1]);
      numChildren++;
    }
  }
  else if (quadHeight > (2 * quadWidth))
  {
    newQuadtreeNodes[0] = new QuadtreeNode(m_lowerLeft, rightMiddle);
    newQuadtreeNodes[0]->m_parent = this;
    m_children->push_back(newQuadtreeNodes[0]);
    numChildren++;

    if (quadHeight > 2)
    {
      newQuadtreeNodes[1] = new QuadtreeNode(leftMiddle,m_upperRight);
      newQuadtreeNodes[1]->m_parent = this;
      m_children->push_back(newQuadtreeNodes[1]);
      numChildren++;
    }
  }
  else  /* The aspect ratio of the quads is fairly square */
  {
    newQuadtreeNodes[0] = new QuadtreeNode(m_lowerLeft,middle);
    newQuadtreeNodes[0]->m_parent = this;
    m_children->push_back(newQuadtreeNodes[0]);
    numChildren++;

    if (quadWidth > 2)
    {
      newQuadtreeNodes[1] = new QuadtreeNode(lowerMiddle,rightMiddle);
      newQuadtreeNodes[1]->m_parent = this;
      m_children->push_back(newQuadtreeNodes[1]);
      numChildren++;
    }

    if (quadHeight > 2)
    {
      newQuadtreeNodes[numChildren] = new QuadtreeNode(leftMiddle,upperMiddle);
      newQuadtreeNodes[numChildren]->m_parent = this;
      m_children->push_back(newQuadtreeNodes[numChildren]);
      numChildren++;
    }

    if ((quadWidth > 2) && (quadHeight > 2))
    {
      newQuadtreeNodes[3] = new QuadtreeNode(middle,m_upperRight);
      newQuadtreeNodes[3]->m_parent = this;
      m_children->push_back(newQuadtreeNodes[3]);
      numChildren++;
    }
  }

  SetChildNeighbors();

//   for (i = 0; i < numChildren; i++)
//     for (j = 0; j < numChildren; j++)
//       if ((newQuadtreeNodes[i] != 0) &&
//        (newQuadtreeNodes[i]->m_neighbors[j] != 0))
//      assert(IsAdjacent(newQuadtreeNodes[i],
//                        newQuadtreeNodes[i]->m_neighbors[j],&j));

  return (numChildren);
}

/* FIX: Larry should really document what funky recursive code like
   this is supposed to do. */

/* Descendants finds all the nodes below a given node in a Quadtree
   and places them in a list.
   The tree is traversed in preorder fashion.
*/
int
QuadtreeNode::Descendants(list<QuadtreeNode *> *nodes)
{
  static int recurseLevel = 0;
  static int numNodes = 0;
  QuadtreeNode *childNode;
  list<QuadtreeNode *>::iterator element,endElement;
  int returnValue;

  ++recurseLevel;
  ++numNodes;

  nodes->push_back(this);

  element = m_children->begin();
  endElement = m_children->end();
  while (!(element == endElement))
  {
    childNode = *element;
    childNode->Descendants(nodes);
    ++element;
  }
  --recurseLevel;

  returnValue = numNodes;
  if (0 == recurseLevel)
    numNodes = 0;

  return (returnValue);
}

int
QuadtreeNode::Leaves(list<QuadtreeNode *> *leaves)
{
  list<QuadtreeNode *>::iterator element,endElement,nextElement;
  QuadtreeNode *node;
  unsigned int numLeaves;

  numLeaves = Descendants(leaves);
  assert(leaves->size() == numLeaves);

  element = leaves->begin();
  endElement = leaves->end();
  while (!(element == endElement))
  {
    nextElement = element;
    ++nextElement;

    node = *element;

    if (!node->m_children->empty())
    {
      leaves->erase(element);
      --numLeaves;

      if (0 > numLeaves)
      {
        fprintf (stdout,"Leaves: numLeaves < 0 (%d)\n",numLeaves);
        abort();
      }
    }

    element = nextElement;
  }
  return (numLeaves);
}

bool
QuadtreeNode::IsLeaf()
{
  return (m_children->empty());
}

bool
QuadtreeNode::HasGrandChildren()
{
  list<QuadtreeNode *>::iterator element;
  QuadtreeNode *childQuadtreeNode;

  element = m_children->begin();
  while (!(element == m_children->end()))
  {
    childQuadtreeNode = *element;
    if (!childQuadtreeNode->IsLeaf())
      return (true);
    ++element;
  }
  return (false);
}

QuadtreeNode *
QuadtreeNode::GetAncestralNeighbor(int side)
{
  QuadtreeNode *neighbor;

  if (m_parent == 0)
    return (0);

  if ((neighbor = m_parent->m_neighbors[side]) != 0)
    return (neighbor);
  else
    return (m_parent->GetAncestralNeighbor(side));
}

bool
QuadtreeNode::OverLappingInterval(int aLeft, int aRight, int bLeft, int bRight)
{
  return (((aRight <= bRight) && (aLeft >= bLeft)) ||
          ((aRight >= bRight) && (aLeft <= bLeft)));
}

bool
QuadtreeNode::IsAdjacent(QuadtreeNode *other, int *adjacentSide)
{
  /* if side is passed (i.e. != -1) we only look for adjacency on that side */
  if (*adjacentSide == -1)
  {
    if ((m_lowerLeft.row == other->m_upperRight.row) &&
        OverLappingInterval(m_lowerLeft.col,m_upperRight.col,
                            other->m_lowerLeft.col,
                            other->m_upperRight.col))
      *adjacentSide = BottomSide;
    else if ((m_upperRight.row == other->m_lowerLeft.row) &&
             OverLappingInterval(m_lowerLeft.col,m_upperRight.col,
                                 other->m_lowerLeft.col,
                                 other->m_upperRight.col))
      *adjacentSide = TopSide;

    else if ((m_upperRight.col == other->m_lowerLeft.col) &&
             OverLappingInterval(m_lowerLeft.row,m_upperRight.row,
                                 other->m_lowerLeft.row,
                                 other->m_upperRight.row))
      *adjacentSide = RightSide;
    else if ((m_lowerLeft.col == other->m_upperRight.col) &&
             OverLappingInterval(m_lowerLeft.row,m_upperRight.row,
                                 other->m_lowerLeft.row,
                                 other->m_upperRight.row))
      *adjacentSide = LeftSide;
    else
      *adjacentSide = -1;
  }
  else
  {
    switch (*adjacentSide)
    {
    case BottomSide:
      if ((m_lowerLeft.row != other->m_upperRight.row) ||
          !OverLappingInterval(m_lowerLeft.col,m_upperRight.col,
                               other->m_lowerLeft.col,other->m_upperRight.col))
        *adjacentSide = -1;
      break;
    case TopSide:
      if ((m_upperRight.row != other->m_lowerLeft.row) ||
          !OverLappingInterval(m_lowerLeft.col,m_upperRight.col,
                               other->m_lowerLeft.col,other->m_upperRight.col))
        *adjacentSide = -1;
      break;
    case RightSide:
      if ((m_upperRight.col != other->m_lowerLeft.col) ||
          !OverLappingInterval(m_lowerLeft.row,m_upperRight.row,
                               other->m_lowerLeft.row,other->m_upperRight.row))
        *adjacentSide = -1;
      break;
    case LeftSide:
      if ((m_lowerLeft.col != other->m_upperRight.col) ||
          !OverLappingInterval(m_lowerLeft.row,m_upperRight.row,
                               other->m_lowerLeft.row,other->m_upperRight.row))
        *adjacentSide = -1;
      break;
    default:
      *adjacentSide = -1;
      break;
    }
  }
  return (*adjacentSide != -1);
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                       -- Quadtree Functions --                          */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* Quadtree constructor */
Quadtree::Quadtree(BUFFER *buffer, int width, int height)
{
  PixelCoords lowerLeft,upperRight;

  lowerLeft.row = 0;
  lowerLeft.col = 0;
  upperRight.row = height - 1;
  upperRight.col = width - 1;

  m_buffer = buffer;
  m_bufferWidth = width;
  m_bufferHeight = height;

  m_root = new QuadtreeNode(lowerLeft,upperRight);

  m_root->m_parent = 0;
  /* pick the diagonal of the root that minimizes error */
  m_root->PickMinErrorTriangles(buffer,width);
}

#if 0
/* depth-first Recursive version */
void
Quadtree::Subdivide(QuadtreeNode *quad, double threshold)
{
  list<QuadtreeNode *>::iterator element;
  QuadtreeNode *childQuadtreeNode;

  if (m_error < threshold)
    return;
  else
  {
    quad->Subdivide();          // Subdivide this quadtreenode
    element = quad->m_children->begin();
    while (!(element == quad->m_children->end()))
    {
      childQuadtreeNode = *element;
      childQuadtreeNode->PickMinErrorTriangles(m_buffer,m_bufferWidth);
      Subdivide(childQuadtreeNode,threshold);
      ++element;
    }
  }
}
#endif

/* breadth-first iterative version */


/*
 * NOTE :
 *
 * Eventually we plan to switch over from using an STL list to an STL multiset
 * for storing the qElements.  There is some commented out code in here to
 * that effect, however it does not yet work properly.  At some point in the future
 * we'll take another look.  -Larry and Michael
 */
void
Quadtree::Subdivide(double threshold, int maxNumTriangles)
{
  /*   For gradient: */
  /*   static const double threshold = 0.0077; */
  /*   static const int maxTriangles = 500000; */
  /*   static const double threshold = -0.5; */
  /*   static const int maxTriangles = 40; */
  /*   static const int maxTriangles = 500000; */

  list<QuadtreeNode *>::iterator qElement, element, endElement;
//   list<QuadtreeNode *>::iterator element, endElement;
//   multiset<QuadtreeNode *>::iterator qElement;
  QuadtreeNode *childQuadtreeNode,*quadtreeNode;
  list<QuadtreeNode *> *queue = new list<QuadtreeNode *>;
//   multiset<QuadtreeNode *,ContentsGreater> *queue = new multiset<QuadtreeNode *,ContentsGreater>;
  int numTriangles = 0, maxQueueSize = 0, queueSize = 0;    /* debugging */
  int numChildren;

  queue->push_back(m_root);
//   queue->insert(m_root);
  queueSize = maxQueueSize = 1;

  numTriangles += 1*2;
  while (!queue->empty() && (numTriangles < maxNumTriangles))
  {
    qElement = queue->begin();
    quadtreeNode = *qElement;
    queue->pop_front();
//     queue->erase(queue->begin());
    --queueSize;

    if (quadtreeNode->m_error > threshold)
    {
      if ((numChildren = quadtreeNode->Subdivide()) > 0)
      {
        numTriangles -= (1*2);
        numTriangles += (numChildren*2);
      }
      element = quadtreeNode->m_children->begin();
      endElement = quadtreeNode->m_children->end();
      while (!(element == endElement))
      {
        childQuadtreeNode = *element;
        childQuadtreeNode->PickMinErrorTriangles(m_buffer,m_bufferWidth);
        queue->push_back(childQuadtreeNode);
//      queue->insert(childQuadtreeNode);
        ++queueSize;
        element = ++element;
      }
    }
    if (queueSize > maxQueueSize)
      maxQueueSize = queueSize;
  }
  queue->clear();
  delete queue;

  printf("Max queue size = %d\n",maxQueueSize);
  printf("Number of triangles = %d\n",numTriangles);
}

void
Quadtree::CreateTriangleMesh()
{
  list<QuadtreeNode *> *leaves = new list<QuadtreeNode *>;
  list<QuadtreeNode *>::iterator element,endElement;
  int numLeaves = 0, numVertices = 0, numPoints = m_bufferWidth*m_bufferHeight;
  int *indexMap,pixelIndex,i,j,row,col,rowStart, numTriangles = 0;
  float deltaU,deltaV,v;
  QuadtreeNode *leaf;

  numLeaves = m_root->Leaves(leaves);

  if (0 > numLeaves)
  {
    fprintf(stderr,"CreateTriangleMesh: numLeave=%d\n",numLeaves);
    abort();
  }

  /* Allocate memory for index map */
  if ((indexMap = (int *) malloc(numPoints*sizeof(int))) == NULL)
  {
    fprintf(stderr, "triangle mesh index map allocation failed.\n");
    exit(EXIT_FAILURE);
  }
  for (i = 0; i < numPoints; i++) /* init so we can tell unused indices */
    indexMap[i] = -1;

  /* Allocate memory for triangle buffer */
  if ((m_buffer->nff.triangle =
       (NFF_TR *) malloc(2*numLeaves*sizeof(NFF_TR))) == NULL)
  {
    fprintf(stderr, "CreateTriangleMesh: malloc numLeaves=%d\n",numLeaves );
    fprintf(stderr, "nff triangle buffer allocation failed.\n");
    exit(EXIT_FAILURE);
  }
  /* Generate triangle list */
  element = leaves->begin();
  endElement = leaves->end();
  while (!(element == endElement))
  {
    leaf = *element;
    AddQuadTriangles(leaf,m_buffer,m_bufferWidth,
                     &numTriangles,&numVertices,indexMap);
    ++element;
  }

  /* Allocate memory for vertex buffer */
  m_buffer->nff.pt_number = numVertices;
  if ((m_buffer->nff.vtx =
       (POS3D *) malloc(numVertices * sizeof(POS3D))) == NULL)
  {
    fprintf(stderr, "nff vertex buffer allocation failed.\n");
    exit(EXIT_FAILURE);
  }
  /* Set vertex coordinates */
  for (i = 0, j = 0; i < numPoints; i++)
  {
    if (indexMap[i] != -1)
    {
      m_buffer->nff.vtx[indexMap[i]] = m_buffer->dot[i];
      if (++j >= numVertices)
        break;
    }
  }

  MendTriangleMeshSeams(m_buffer,m_bufferWidth,indexMap,this);

  /* Allocate memory for texture coord. buffer */
  if ((m_buffer->nff.tex =
       (UV_TEX *) malloc(numVertices * sizeof(UV_TEX))) == NULL)
  {
    fprintf(stderr, "nff texture buffer allocation failed.\n");
    exit(EXIT_FAILURE);
  }
  /* Generate texture coordinates */
  deltaU = 1.0/(float) m_bufferWidth;
  deltaV = 1.0/(float) m_bufferHeight;
  for (row = 0, rowStart = 0, j = 0; row < m_bufferHeight; row++)
  {
    v = deltaV * (float) (m_bufferHeight - row);
    for (col = 0; col < m_bufferWidth; col++)
    {
      pixelIndex = rowStart + col;
      if (indexMap[pixelIndex] != -1)
      {
        m_buffer->nff.tex[indexMap[pixelIndex]].u = deltaU * (float) col;
        m_buffer->nff.tex[indexMap[pixelIndex]].v = v;
        if (++j >= numVertices)
          break;
      }
    }
    rowStart += m_bufferWidth;
  }
  m_buffer->nff.tr_number = numTriangles;
  free(indexMap);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Triangle mesh routines
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //


// Determine if all vertices of a triangle are associated with good
// data.

static bool
GoodTriangle(BUFFER *buffer, int index1, int index2, int index3)
{
  bool good;

  good = !(buffer->dot[index1].x == 0 && buffer->dot[index1].y == 0 && buffer->dot[index1].z == 0);
  good = good && !(buffer->dot[index2].x == 0 && buffer->dot[index2].y == 0 && buffer->dot[index2].z == 0);
  good = good && !(buffer->dot[index3].x == 0 && buffer->dot[index3].y == 0 && buffer->dot[index3].z == 0);
  return good;
}

// Add a triangle to the mesh

static void
AddTriangle(int index1, int index2, int index3, BUFFER *buffer,
            int *numTriangles, int *indexMap, int *numVertices)
{
  NFF_TR *triangle;

  if (indexMap[index1] == -1)
    indexMap[index1] = (*numVertices)++;

  if (indexMap[index2] == -1)
    indexMap[index2] = (*numVertices)++;

  if (indexMap[index3] == -1)
    indexMap[index3] = (*numVertices)++;

  triangle = &(buffer->nff.triangle[*numTriangles]);

  triangle->vtx1 = indexMap[index1];
  triangle->vtx2 = indexMap[index2];
  triangle->vtx3 = indexMap[index3];
  (*numTriangles)++;
}

// Add the triangles associated with a given quad to the mesh

static void
AddQuadTriangles(QuadtreeNode *quad, BUFFER *buffer, int width,
                 int *numTriangles, int *numVertices, int *indexMap)
{
  int quadVertices[4],goodTriangle;
  PixelCoords lowerRight,upperLeft;

  lowerRight.row = quad->m_lowerLeft.row;
  lowerRight.col = quad->m_upperRight.col;

  upperLeft.col = quad->m_lowerLeft.col;
  upperLeft.row = quad->m_upperRight.row;

  quadVertices[0] = quad->m_lowerLeft.PixelIndex(width);
  quadVertices[1] = lowerRight.PixelIndex(width);
  quadVertices[2] = quad->m_upperRight.PixelIndex(width);
  quadVertices[3] = upperLeft.PixelIndex(width);

  if (quad->m_diagonalIsLeftRight) /* diagonal is lower-left to upper-right */
  {
    /* lower triangle */
    goodTriangle = GoodTriangle(buffer,quadVertices[0],quadVertices[1],
                                quadVertices[2]);
    quad->m_goodSide[BottomSide] = quad->m_goodSide[RightSide] = goodTriangle;
    if (goodTriangle)
      AddTriangle(quadVertices[0],quadVertices[1],quadVertices[2],
                  buffer,numTriangles,indexMap,numVertices);

    /* upper triangle */
    goodTriangle = GoodTriangle(buffer,quadVertices[2],quadVertices[3],
                                quadVertices[0]);
    quad->m_goodSide[LeftSide] = quad->m_goodSide[TopSide] = goodTriangle;
    if (goodTriangle)
      AddTriangle(quadVertices[2],quadVertices[3],quadVertices[0],
                  buffer,numTriangles,indexMap,numVertices);
  }
  else                  /* diagonal is lower-right to upper-left */
  {
    /* lower triangle */
    goodTriangle = GoodTriangle(buffer,quadVertices[0],quadVertices[1],
                                quadVertices[3]);
    quad->m_goodSide[LeftSide] = quad->m_goodSide[BottomSide] = goodTriangle;
    if (goodTriangle)
      AddTriangle(quadVertices[0],quadVertices[1],quadVertices[3],
                  buffer,numTriangles,indexMap,numVertices);

    /* upper triangle */
    goodTriangle = GoodTriangle(buffer,quadVertices[3],quadVertices[1],
                                quadVertices[2]);
    quad->m_goodSide[TopSide] = quad->m_goodSide[RightSide] = goodTriangle;
    if (goodTriangle)
      AddTriangle(quadVertices[3],quadVertices[1],quadVertices[2],
                  buffer,numTriangles,indexMap,numVertices);
  }
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Routines to align vertices with neighboring quad edges
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

// AlignPt aligns point3 with the point closest to it on the line
// defined by point1 & point2

static void
AlignPt(PixelCoords *point1, PixelCoords *point2, PixelCoords *point3,
        int *indexMap, POS3D *vertices, int bufferWidth)
{
  int vert1Index,vert2Index,vert3Index;
  POS3D *pt1,*pt2,*pt3,unitRayVec,rayVec,proj;
  float normalizer,magProj;

  // We check indices in case any of the vertices are from a region of
  // missing or bad data missing data
  if ((vert1Index = indexMap[point1->PixelIndex(bufferWidth)]) < 0)
    return;
  if ((vert2Index = indexMap[point2->PixelIndex(bufferWidth)]) < 0)
    return;
  if ((vert3Index = indexMap[point3->PixelIndex(bufferWidth)]) < 0)
    return;

  if ((vert3Index == vert1Index) || (vert3Index == vert2Index))
  {
    /*     printf("REMARK in AlignPt: vert3index is the same as vert2Index or "
           "vert1Index.\n"); */
    return;
  }
  pt1 = &vertices[vert1Index];
  pt2 = &vertices[vert2Index];
  pt3 = &vertices[vert3Index];

  rayVec.x = pt2->x - pt1->x;
  rayVec.y = pt2->y - pt1->y;
  rayVec.z = pt2->z - pt1->z;

  if ((rayVec.x == 0.0) && (rayVec.y == 0.0) && (rayVec.z == 0.0))
  {
    printf("WARNING in AlignPt: point1 and point2 are geometrically "
           "coincident. (%f %f %f) (%f %f %f)\n",
           pt1->x, pt1->y, pt1->z,
           pt2->x, pt2->y, pt2->z);
    pt3->x = pt1->x;
    pt3->y = pt1->y;
    pt3->z = pt1->z;
    return;
  }

  normalizer = 1.0/sqrt(rayVec.x*rayVec.x +
                        rayVec.y*rayVec.y +
                        rayVec.z*rayVec.z);
  unitRayVec.x = rayVec.x*normalizer;
  unitRayVec.y = rayVec.y*normalizer;
  unitRayVec.z = rayVec.z*normalizer;

  magProj = ((pt3->x - pt1->x)*unitRayVec.x +
             (pt3->y - pt1->y)*unitRayVec.y +
             (pt3->z - pt1->z)*unitRayVec.z);

  proj.x = magProj*unitRayVec.x;
  proj.y = magProj*unitRayVec.y;
  proj.z = magProj*unitRayVec.z;

  pt3->x = pt1->x + proj.x;
  pt3->y = pt1->y + proj.y;
  pt3->z = pt1->z + proj.z;
}


static void
AlignLeafToNeighbor(QuadtreeNode *leaf, QuadtreeNode *neighborLeaf,
                    int side, int *indexMap, POS3D *vertices, int bufferWidth)
{
  PixelCoords upperLeft,lowerRight,nbrUpperLeft,nbrLowerRight;

  if (neighborLeaf == 0)
    return;

  if (leaf == neighborLeaf)
  {
    printf("WARNING in AlignLeafToNeighbor: leaf == neighborLeaf. "
           "Aborting.\n");
    return;
  }

  upperLeft.row = leaf->m_upperRight.row;
  upperLeft.col = leaf->m_lowerLeft.col;
  lowerRight.row = leaf->m_lowerLeft.row;
  lowerRight.col = leaf->m_upperRight.col;

  nbrUpperLeft.row = neighborLeaf->m_upperRight.row;
  nbrUpperLeft.col = neighborLeaf->m_lowerLeft.col;
  nbrLowerRight.row = neighborLeaf->m_lowerLeft.row;
  nbrLowerRight.col = neighborLeaf->m_upperRight.col;

  switch (side)
  {
  case LeftSide:
    AlignPt(&nbrLowerRight,&(neighborLeaf->m_upperRight),
            &(leaf->m_lowerLeft),indexMap,vertices,bufferWidth);
    AlignPt(&nbrLowerRight,&(neighborLeaf->m_upperRight),
            &upperLeft,indexMap,vertices,bufferWidth);
    break;
  case RightSide:
    AlignPt(&(neighborLeaf->m_lowerLeft),&nbrUpperLeft,
            &(leaf->m_upperRight),indexMap,vertices,bufferWidth);
    AlignPt(&(neighborLeaf->m_lowerLeft),&nbrUpperLeft,
            &lowerRight,indexMap,vertices,bufferWidth);
    break;
  case TopSide:
    AlignPt(&(neighborLeaf->m_lowerLeft),&nbrLowerRight,
            &(leaf->m_upperRight),indexMap,vertices,bufferWidth);
    AlignPt(&(neighborLeaf->m_lowerLeft),&nbrLowerRight,
            &upperLeft,indexMap,vertices,bufferWidth);
    break;
  case BottomSide:
    AlignPt(&nbrUpperLeft,&(neighborLeaf->m_upperRight),
            &(leaf->m_lowerLeft),indexMap,vertices,bufferWidth);
    AlignPt(&nbrUpperLeft,&(neighborLeaf->m_upperRight),
            &lowerRight,indexMap,vertices,bufferWidth);
    break;
  }
}

static void
AlignSeamVertices(QuadtreeNode *quadtreeNode, int *indexMap,
                  POS3D *vertices, int width)
{
  list<QuadtreeNode *>::iterator element,endElement;
  QuadtreeNode *leaf;
  QuadtreeNode *neighbor;
  list<QuadtreeNode *> *leaves = new list<QuadtreeNode *>;
//   int numLeaves;                        // debugging
//   int side;                             // debugging

  if (quadtreeNode->IsLeaf())
    return;

  if (!quadtreeNode->HasGrandChildren())
    return;

  quadtreeNode->Leaves(leaves);
//   numLeaves = Leaves(quadtreeNode,leaves); // debugging
//   printf("numLeaves = %d\n",numLeaves);         // debugging

  element = leaves->begin();
  endElement = leaves->end();
  while (!(element == endElement))
  {
    leaf = *element;
    if (leaf->m_neighbors[LeftSide] == 0)
    {
      neighbor = leaf->GetAncestralNeighbor(LeftSide);
      if (neighbor)
      {
//      side = LeftSide; // Debugging...
//      assert(IsAdjacent(leaf,neighbor,&side));
        if (neighbor->m_goodSide[RightSide])
          AlignLeafToNeighbor(leaf,neighbor,LeftSide,indexMap,vertices,width);
      }
    }
    if (leaf->m_neighbors[RightSide] == 0)
    {
      neighbor = leaf->GetAncestralNeighbor(RightSide);
      if (neighbor)
      {
//       side = RightSide; // Debugging...
//       assert(IsAdjacent(leaf,neighbor,&side));
      if (neighbor->m_goodSide[LeftSide])
        AlignLeafToNeighbor(leaf,neighbor,RightSide,indexMap,vertices,width);
      }
    }
    if (leaf->m_neighbors[BottomSide] == 0)
    {
      neighbor = leaf->GetAncestralNeighbor(BottomSide);
      if (neighbor)
      {
//      side = BottomSide; // Debugging...
//      assert(IsAdjacent(leaf,neighbor,&side));
        if (neighbor->m_goodSide[TopSide])
          AlignLeafToNeighbor(leaf,neighbor,BottomSide,indexMap,vertices,width);
      }
    }
    if (leaf->m_neighbors[TopSide] == 0)
    {
      neighbor = leaf->GetAncestralNeighbor(TopSide);
      if (neighbor)
      {
//      side = TopSide; // Debugging...
//      assert(IsAdjacent(leaf,neighbor,&side));
        if (neighbor->m_goodSide[BottomSide])
          AlignLeafToNeighbor(leaf,neighbor,TopSide,indexMap,vertices,width);
      }
    }
    ++element;
  }

  delete leaves;
}

static void
MendTriangleMeshSeams(BUFFER *buffer, int width,
                      int *indexMap, Quadtree *quadtree)
{
  POS3D *vertices = buffer->nff.vtx;

  AlignSeamVertices(quadtree->Root(),indexMap,vertices,width);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Main triangle mesh creation routine
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //


/************************************************************************/
/*                                                                      */
/*                 dot to nff updated version                           */
/*                                                                      */
/************************************************************************/
// #if 0
// static void
// SyntheticTerrain(BUFFER *b, int width, int height)
// {
//   int rowStart,row,col,pixelIndex;
//   pixel *disparities = b->disp;
//   double x,y;
//   double maxZ = -999999.9;
//   /*  double a = 0.2, k = 10.0; works */
//   double a = 1.0, k = 10.0;

//   rowStart = 0;
//   for (row = 0; row < height; row++)
//   {
//     y = (double)row - ((double)height-1.0)/2.0;
//     for (col = 0; col < width; col++)
//     {
//       pixelIndex = rowStart + col;
//       disparities[pixelIndex] = 32;
//       b->dot[pixelIndex].x = x = (double)col - ((double)width-1.0)/2.0;
//       b->dot[pixelIndex].y = y;
//       b->dot[pixelIndex].z = k*(((1.0-exp(-a*x))/(1.0+exp(-a*x))) -
//                              ((1.0-exp(-a*(x-y)))/(1.0+exp(-a*(x-y)))));
//       /*      b->dot[pixelIndex].z = 10.0; */
//       if (b->dot[pixelIndex].z > maxZ)
//      maxZ = b->dot[pixelIndex].z;
//     }
//     rowStart += width;
//   }

//   printf("SyntheticTerrain: maxZ = %e\n",maxZ);
// }
//#endif

static void
Init(BUFFER *b, int width, int height)
{
  b->nff.pt_number = width * height;

  /* Allocate memory for gradient buffer */
  if ((b->gradients =
       (POS2D *) malloc(b->nff.pt_number * sizeof(POS3D))) == NULL)
    {
      fprintf(stderr, "gradient buffer allocation failed.\n");
      exit(EXIT_FAILURE);
    }

  /*  SyntheticTerrain(b,width,height); */
}

static double
Convolve(Array3x3 mask, int row, int col, BUFFER *buffer, int width)
{
  static const int maskRowOrigin = 1;
  static const int maskColOrigin = 1;
  //  pixel *disparities = buffer->disp;
  int i,j,pixelIndex,rowStartIndex;
  double result = 0.0;

  rowStartIndex = (row-maskRowOrigin)*width;
  for (i=0; i < 3; rowStartIndex += width, i++)
    {
      for (j=0; j < 3; j++)
        {
          pixelIndex = rowStartIndex + (col + j-maskColOrigin);
          if (buffer->dot[pixelIndex].x == 0 &&
              buffer->dot[pixelIndex].y == 0 &&
              buffer->dot[pixelIndex].z == 0)
            return (0.0);
//           if ((disparities[pixelIndex] == 0) ||
//               (disparities[pixelIndex] == MISSING_PIXEL) ||
//               (disparities[pixelIndex] == FAR_FIELD_PIXEL))

          result += mask[i][j]*buffer->dot[pixelIndex].z;
        }
    }
  return (result);
}

static void
CalculateGradients(BUFFER *b, int width, int height)
{
  //  pixel *disparities = b->disp;
  int row,col,pixelIndex, numPixels = width*height;
  Array3x3 xMask = {{-1.0/6.0, 0.0, 1.0/6.0},
                    {-1.0/6.0, 0.0, 1.0/6.0},
                    {-1.0/6.0, 0.0, 1.0/6.0}};
  Array3x3 yMask = {{ 1.0/6.0, 1.0/6.0, 1.0/6.0},
                    {   0.0,     0.0,     0.0  },
                    {-1.0/6.0,-1.0/6.0,-1.0/6.0}};

  /* zero out top border row */
  for (pixelIndex = 0; pixelIndex < width; pixelIndex++)
  {
    b->gradients[pixelIndex].x = 0.0;
    b->gradients[pixelIndex].y = 0.0;
  }

  for (row = 1; row < (height-1); row++)
  {
    /* zero out left border column */
    b->gradients[pixelIndex].x = 0.0;
    b->gradients[pixelIndex].y = 0.0;
    pixelIndex++;
    for (col = 1; col < (width - 1); col++)
      {
        if (b->dot[pixelIndex].x == 0 &&
            b->dot[pixelIndex].y == 0 &&
            b->dot[pixelIndex].z == 0)
        {
          b->gradients[pixelIndex].x = Convolve(xMask,row,col,b,width);
          b->gradients[pixelIndex].y = Convolve(yMask,row,col,b,width);
        }
      else
        {
          b->gradients[pixelIndex].x = 0.0;
          b->gradients[pixelIndex].y = 0.0;
        }
      pixelIndex++;
    }
    /* zero out right border column */
    b->gradients[pixelIndex].x = 0.0;
    b->gradients[pixelIndex].y = 0.0;
    pixelIndex++;
  }

  /* zero out bottom border row */
  for (; pixelIndex < numPixels; pixelIndex++)
  {
    b->gradients[pixelIndex].x = 0.0;
    b->gradients[pixelIndex].y = 0.0;
  }
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Publicly visible functions
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
dot_to_adaptative_mesh(BUFFER *b, int width, int height, double mesh_tolerance, int max_triangles)
{
  Quadtree *quadtree;

  Init(b,width,height);

  CalculateGradients(b,width,height);
  /*WriteGradientImageFile(b,dft,hd,width,height,'m');*/
  quadtree = new Quadtree(b,width,height);
  printf("\tNumber of triangles in full mesh (%d,%d) = %d\n",
         width,height,2*(width-1)*(height-1));
  printf("\tMesh parameters -->  tol: %f    max triangles: %d\n", mesh_tolerance, max_triangles);
  quadtree->Subdivide(mesh_tolerance, max_triangles);

  printf("Number of triangles in subdivided mesh = %d\n",b->nff.tr_number);
  quadtree->CreateTriangleMesh();

  printf("Number of triangles in reduced mesh = %d\n",b->nff.tr_number);
}

/* original code that place the vertices on a regular grid */
void
dot_to_mesh(BUFFER *b, int width, int height, int h_step, int v_step)
{
  int   y, x, x_grid, y_grid;
  //  int       max_jump = dft->nff_max_jump;
  int   grid_width, grid_height;
  int   buff_pos;
  int   n = 0;                  /* number of triangles created */
  int   AI, BI, CI, DI;         /* shift from pointer to the 4 corners Image */
  int   AG, BG, CG, DG;         /* shift from pointer to the 4 corners Grid */
  double        A1,A2,A3,B1,B2,B3,C1,C2,C3,D1,D2,D3; /* easier to read */
  double distBC,distAD;
#if 0
  double distAB,distAC,distBD,distCD;
  double max_dist;
  double pixel_disp;
  double sin_theta1, sin_theta2;
#endif
  double u,v;

  /* initialize the constants */
  grid_width = (width-1)/h_step+1;
  grid_height = (height-1)/v_step+1;
  b->nff.pt_number = (grid_width) * (grid_height);
  b->nff.tr_number = (grid_width-1)*(grid_height-1)*2;
  x_grid = 0;
  y_grid = 0;

  /* debug */
  printf("w=%d  h=%d\n", width, height);
  printf("sw=%d sh=%d\n", h_step, v_step);
  printf("gw=%d gh=%d\n", grid_width, grid_height);
  printf("pt=%d tr=%d\n", b->nff.pt_number, b->nff.tr_number);


  /* Allocate memory for vertex buffer */
  if ((b->nff.vtx = (POS3D *)malloc(b->nff.pt_number * sizeof(POS3D))) == NULL)
  {
    fprintf(stderr, "nff vertex buffer allocation failed.\n");
    exit(EXIT_FAILURE);
  }
  /* Allocate memory for triangle buffer */
  if ((b->nff.triangle = (NFF_TR *)malloc(b->nff.tr_number * sizeof(NFF_TR))) ==
      NULL)
  {
    fprintf(stderr, "nff triangle buffer allocation failed.\n");
    exit (EXIT_FAILURE);
  }
  /* Allocate memory for texure buffer */
  if ((b->nff.tex = (UV_TEX *)malloc(b->nff.pt_number * sizeof(UV_TEX))) ==
      NULL)
  {
    fprintf(stderr, "nff texure buffer allocation failed.\n");
    exit(EXIT_FAILURE);
  }

  /* make the regular grid point */
  for( y=0, y_grid=0 ; y_grid < grid_height ; y += v_step, y_grid++){
    buff_pos = y_grid*grid_width;
    v = (double)(height-y)/(double)(height);
    for( x=0, x_grid=0 ; x_grid < grid_width ; x += h_step, x_grid ++){
      u = (double)(x)/(double)(width);
      b->nff.vtx[buff_pos+x_grid].x = b->dot[y*width+x].x;
      b->nff.vtx[buff_pos+x_grid].y = b->dot[y*width+x].y;
      b->nff.vtx[buff_pos+x_grid].z = b->dot[y*width+x].z;
      b->nff.tex[buff_pos+x_grid].u = u;
      b->nff.tex[buff_pos+x_grid].v = v;
    }
  }

  // make the triangle
  for( y=0; y < (grid_height-1) ; y ++){
    buff_pos = y*grid_width;
    for( x=0 ; x < (grid_width-1) ; x ++){

// #if 0
//       /* calcul max_dist */
//       pixel_disp = (double)b->disp[y*v_step*width+(x+1)*h_step];
//       sin_theta1 = sin ((double)hd->h_theta_pixel * pixel_disp / 2.0);
//       sin_theta2 = sin ((double)hd->h_theta_pixel* (pixel_disp - max_jump) /
//                      2.0);
//       if(sin_theta1 < THETA_LIMIT)
//         sin_theta1 = THETA_LIMIT;
//       if(sin_theta2 < THETA_LIMIT)
//         sin_theta2 = THETA_LIMIT;
//       max_dist = (double)(dft->baseline/sin_theta1 - dft->baseline/sin_theta2);
//       if (max_dist < 0)
//         max_dist = max_dist *(-1);
// #endif

      /* calcul the distance between the first triangle sommets */
      /* top left corner X */
      AG = buff_pos+x;
      AI = (y)*v_step*width+(x)*h_step;
      A1 = b->nff.vtx[AG].x;
      A2 = b->nff.vtx[AG].y;
      A3 = b->nff.vtx[AG].z;
      /* top right corner Y */
      BG = buff_pos+x+1;
      BI = (y)*v_step*width+(x+1)*h_step;
      B1 = b->nff.vtx[BG].x;
      B2 = b->nff.vtx[BG].y;
      B3 = b->nff.vtx[BG].z;
      /* botom left corner Z */
      CG = buff_pos+x+grid_width;
      CI = (y+1)*v_step*width+(x)*h_step;
      C1 = b->nff.vtx[CG].x;
      C2 = b->nff.vtx[CG].y;
      C3 = b->nff.vtx[CG].z;
      /* botom right corner Y */
      DG = buff_pos+grid_width+x+1;
      DI = (y+1)*v_step*width+(x+1)*h_step;
      D1 = b->nff.vtx[DG].x;
      D2 = b->nff.vtx[DG].y;
      D3 = b->nff.vtx[DG].z;

      distBC = (B1-C1)*(B1-C1) + (B2-C2)*(B2-C2) + (B3-C3)*(B3-C3);
      distAD = (A1-D1)*(A1-D1) + (A2-D2)*(A2-D2) + (A3-D3)*(A3-D3);
#if 0
      distAB = (A1-B1)*(A1-B1) + (A2-B2)*(A2-B2) + (A3-B3)*(A3-B3);
      distAC = (A1-C1)*(A1-C1) + (A2-C2)*(A2-C2) + (A3-C3)*(A3-C3);
      distBD = (B1-D1)*(B1-D1) + (B2-D2)*(B2-D2) + (B3-D3)*(B3-D3);
      distCD = (C1-D1)*(C1-D1) + (C2-D2)*(C2-D2) + (C3-D3)*(C3-D3);
#endif
      /* build the triangles with the smallest diagonal |\| versus |/| */
      if(distBC < distAD){      /* case  |/| */

        /* draw triangle |/ if the jumps in disp is not bigger than allowed */
//      if (!(abs(int(b->disp[AI] - b->disp[BI])) > max_jump ||
//            abs(int(b->disp[BI] - b->disp[CI])) > max_jump ||
//            abs(int(b->disp[CI] - b->disp[DI])) > max_jump ||
//            b->disp[AI] == MISSING_PIXEL ||
//            b->disp[BI] == MISSING_PIXEL ||
//            b->disp[CI] == MISSING_PIXEL)) {
        // Draw a triangle if the vertices are not missing pixels
        if(!((b->dot[AI].x == 0 && b->dot[AI].y ==0 && b->dot[AI].z == 0)  ||
             (b->dot[BI].x == 0 && b->dot[BI].y ==0 && b->dot[BI].z == 0) ||
             (b->dot[CI].x == 0 && b->dot[CI].y ==0 && b->dot[CI].z == 0))) {

          b->nff.triangle[n].vtx1 = AG;
          b->nff.triangle[n].vtx2 = BG;
          b->nff.triangle[n].vtx3 = CG;
          n ++;                 /* you've got one more triangle */
        }
//      /* draw triangle /| if the jumps in disp is not bigger than allowed */
//      if (!(abs(int(b->disp[BI] - b->disp[DI])) > max_jump ||
//            abs(int(b->disp[DI] - b->disp[CI])) > max_jump ||
//            abs(int(b->disp[CI] - b->disp[BI])) > max_jump ||
//            b->disp[BI] == MISSING_PIXEL ||
//            b->disp[CI] == MISSING_PIXEL ||
//            b->disp[DI] == MISSING_PIXEL)) {
        // Draw a triangle if the vertices are not missing pixels
        if(!((b->dot[BI].x == 0 && b->dot[BI].y ==0 && b->dot[BI].z == 0)  ||
             (b->dot[CI].x == 0 && b->dot[CI].y ==0 && b->dot[CI].z == 0) ||
             (b->dot[DI].x == 0 && b->dot[DI].y ==0 && b->dot[DI].z == 0))) {

          b->nff.triangle[n].vtx1 = BG;
          b->nff.triangle[n].vtx2 = DG;
          b->nff.triangle[n].vtx3 = CG;
          n ++;                 /* and a second one */
        }
      }
      else {    /* distBC <= distAD  case |\| */
//      /* draw triangle |\ if the jumps in disp is not bigger than allowed */
//      if (!(abs(int(b->disp[AI] - b->disp[DI])) > max_jump ||
//            abs(int(b->disp[DI] - b->disp[CI])) > max_jump ||
//            abs(int(b->disp[CI] - b->disp[AI])) > max_jump ||
//            b->disp[AI] == MISSING_PIXEL ||
//            b->disp[CI] == MISSING_PIXEL ||
//            b->disp[DI] == MISSING_PIXEL)) {
    // Draw a triangle if the vertices are not missing pixels
    if(!((b->dot[AI].x == 0 && b->dot[AI].y ==0 && b->dot[AI].z == 0)  ||
         (b->dot[CI].x == 0 && b->dot[CI].y ==0 && b->dot[CI].z == 0) ||
         (b->dot[DI].x == 0 && b->dot[DI].y ==0 && b->dot[DI].z == 0))) {

          b->nff.triangle[n].vtx1 = AG;
          b->nff.triangle[n].vtx2 = DG;
          b->nff.triangle[n].vtx3 = CG;
          n ++;                 /* you've got one more triangle */
        }
//      if (!(abs(int(b->disp[BI] - b->disp[DI])) > max_jump ||
//            abs(int(b->disp[DI] - b->disp[AI])) > max_jump ||
//            abs(int(b->disp[AI] - b->disp[BI])) > max_jump ||
//            b->disp[AI] == MISSING_PIXEL ||
//            b->disp[BI] == MISSING_PIXEL ||
//            b->disp[DI] == MISSING_PIXEL)) {
    // Draw a triangle if the vertices are not missing pixels
    if(!((b->dot[AI].x == 0 && b->dot[AI].y ==0 && b->dot[AI].z == 0)  ||
         (b->dot[BI].x == 0 && b->dot[BI].y ==0 && b->dot[BI].z == 0) ||
         (b->dot[DI].x == 0 && b->dot[DI].y ==0 && b->dot[DI].z == 0))) {
          b->nff.triangle[n].vtx1 = BG;
          b->nff.triangle[n].vtx2 = DG;
          b->nff.triangle[n].vtx3 = AG;
          n ++;                 /* and a second one */
        }
      }
    }
  }
  b->nff.tr_number = n;

  return;
}

/***********************/
/* write inventor_file */
/***********************/

#if defined(ASP_HAVE_PKG_OPENSCENEGRAPH) && ASP_HAVE_PKG_OPENSCENEGRAPH==1
void write_osg_impl(BUFFER *b, std::string const& filename,
                         std::string const& texture_filename) {

  std::cout << "Writing " << filename << "..." << std::flush;

  // A geode is a "geometry node". It is-a 'Node' and contains 'Drawable's.
  osg::ref_ptr<osg::Geode> geode (new osg::Geode());

  // 'Geometry' is-a 'Drawable'. It is a collection of vertices, normals,
  // colors, texture coordinates and so on. It is organized in "primitive
  // sets", that allow to say that, e.g., "from vertex to 0 to 8 render as
  // triangles, from 9 to 13 render as points, please". For those
  // OpenGL-inclined, think of 'Geometry' as a wrapper around vertex (and
  // normals, and texcoord) arrays and 'glDrawElements()'
  osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

  // Create and set the vertex array for the geometry object
  osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());

  for (int i = 0; i < b->nff.pt_number; ++i)
    vertices->push_back (osg::Vec3 (b->nff.vtx[i].x, b->nff.vtx[i].y, b->nff.vtx[i].z) );

  geometry->setVertexArray (vertices.get());


  std::vector<vw::Vector3> vertex_normals(b->nff.pt_number);

  // Now create the "primitive set", which describes which vertices
  // are to be used when rendering the triangles.
  osg::ref_ptr<osg::DrawElementsUInt> faces(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0));
  for (int i = 0; i < b->nff.tr_number; ++i) {
    faces->push_back(b->nff.triangle[i].vtx1);
    faces->push_back(b->nff.triangle[i].vtx2);
    faces->push_back(b->nff.triangle[i].vtx3);
    vw::Vector3 v1(b->nff.vtx[b->nff.triangle[i].vtx1].x,
                   b->nff.vtx[b->nff.triangle[i].vtx1].y,
                   b->nff.vtx[b->nff.triangle[i].vtx1].z);
    vw::Vector3 v2(b->nff.vtx[b->nff.triangle[i].vtx2].x,
                   b->nff.vtx[b->nff.triangle[i].vtx2].y,
                   b->nff.vtx[b->nff.triangle[i].vtx2].z);
    vw::Vector3 v3(b->nff.vtx[b->nff.triangle[i].vtx3].x,
                   b->nff.vtx[b->nff.triangle[i].vtx3].y,
                   b->nff.vtx[b->nff.triangle[i].vtx3].z);
    vw::Vector3 l1 = v3-v1;
    vw::Vector3 l2 = v3-v2;
    vw::Vector3 c = normalize(vw::math::cross_prod(l1,l2));
    vertex_normals[b->nff.triangle[i].vtx1] = c;
    vertex_normals[b->nff.triangle[i].vtx2] = c;
    vertex_normals[b->nff.triangle[i].vtx3] = c;
  }

  // Create an array for the single normal.
  osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
  geometry->setNormalArray( n.get() );
  for (int i = 0; i < b->nff.pt_number; ++i)
    n->push_back( osg::Vec3( vertex_normals[i].x(),
                             vertex_normals[i].y(),
                             vertex_normals[i].z()) );

  geometry->addPrimitiveSet(faces.get());

  // Next, we associate some texture coordinates with the vertices
  osg::Vec2Array* texcoords = new osg::Vec2Array(b->nff.pt_number);
  for (int i = 0; i < b->nff.pt_number; ++i)
    (*texcoords)[i].set(b->nff.tex[i].u, b->nff.tex[i].v);
  geometry->setTexCoordArray(0,texcoords);

  // The geometry is now full specified, so we associate it with the geode.
  geode->addDrawable (geometry.get());

  // We must now associate the texture in a file with the geode
  osg::ref_ptr<osg::Texture2D> texture(new osg::Texture2D);

  // protect from being optimized away as static state
  texture->setDataVariance(osg::Object::DYNAMIC);
  texture->setUseHardwareMipMapGeneration(false);

  osg::Image* texture_image = osgDB::readImageFile(texture_filename.c_str());
  if (!texture_image)
    throw vw::ArgumentErr() << "OpenSceneGraph couldn't read the texture, " << texture_filename << ".";
  texture->setImage(texture_image);

  // Create a new StateSet with default settings:
  osg::ref_ptr<osg::StateSet> texture_state_set(new osg::StateSet());

  // Assign texture unit 0 of our new StateSet to the texture
  // we just created and enable the texture.
  texture_state_set->setTextureAttributeAndModes(0,texture.get(),osg::StateAttribute::ON);
  geode->setStateSet(texture_state_set.get());

  // Set up lighting properties
  osg::StateSet* state = geode->getOrCreateStateSet();
  osg::ref_ptr<osg::Material> mat = new osg::Material;
  mat->setDiffuse( osg::Material::FRONT_AND_BACK,
                   osg::Vec4( .7f, .7f, .7f, 1.f ) );
  mat->setSpecular( osg::Material::FRONT_AND_BACK,
                    osg::Vec4( .1f, .1f, .1f, 1.f ) );
  mat->setShininess( osg::Material::FRONT_AND_BACK, 20.f );
  state->setAttribute( mat.get() );

  // Add the geometry to the geode and save the geode to a file o disk.
  osgDB::writeNodeFile(*(geode.get()), filename);

  std::cout << " done.\n";
}

#else  // HAVE_PKG_OPENSCENEGRAPH

void write_osg_impl(BUFFER *b, std::string const& filename,
                         std::string const& texture_filename) {
  std::cout << "WARNING: could not write " << filename << ".  Compiled without open scene graph support.";
}
#endif  // HAVE_PKG_OPENSCENEGRAPH


void write_inventor_impl(BUFFER *b, std::string const& filename,
                         std::string const& texture_filename,
                         bool flip_triangles) {
  FILE *outflow;
  int i;

  std::vector<std::string> texture_path_components;
  boost::split( texture_path_components, texture_filename, boost::is_any_of("/") );
  std::string relative_texture_path = "./" + texture_path_components[texture_path_components.size()-1];

  // open output file
  if((outflow = fopen (filename.c_str(), "w" )) == 0) {
    fprintf (stderr, "write_inventor: cannot open output file: %s\n", filename.c_str());
    exit(EXIT_FAILURE);
  }
  fprintf (outflow, "#Inventor V2.0 ascii\n\n");
  fprintf (outflow, "# Created by the Intelligent Robotics Group,\n");
  fprintf (outflow, "# NASA Ames Research Center\n");
  fprintf (outflow, "# File generated by the NASA Ames Stereo-Pipeline.\n");
  fprintf (outflow, "# %d vertices, %d triangles\n\n", b->nff.pt_number, b->nff.tr_number);
  fprintf (outflow, "Separator {\n");
  fprintf (outflow, "   Texture2 {\n");

  // Grab the end of the full path
  fprintf (outflow, "  filename \"%s\"\n", relative_texture_path.c_str());

  fprintf (outflow, "   }\n");
  // Material Section
  fprintf (outflow, "Material {\n");
  fprintf (outflow, "  ambientColor 1.00 1.00 1.00\n");
  fprintf (outflow, "  diffuseColor 1.00 1.00 1.00\n");
  fprintf (outflow, "  specularColor 0.00 0.00 0.00\n");
  fprintf (outflow, "  emissiveColor 0.00 0.00 0.00\n");
  fprintf (outflow, "  shininess 0.00\n");
  fprintf (outflow, "  transparency 0.00\n");
  fprintf (outflow, "}\n");

  // Gouraud shading (comment out in the file, but make it easy to
  // turn on if desired.
  if (flip_triangles) {
    fprintf (outflow, "ShapeHints {\n");
    fprintf (outflow, "   vertexOrdering CLOCKWISE\n");
    fprintf (outflow, "#  creaseAngle 3.1\n");
    fprintf (outflow, "}\n");
  } else {
    fprintf (outflow, "#ShapeHints {\n");
    fprintf (outflow, "#  creaseAngle 3.1\n");
    fprintf (outflow, "#}\n");
  }

  // vertices position
  fprintf (outflow, "    Coordinate3 {\n");
  fprintf (outflow, "        point [    ");
  fprintf (outflow, "# %d vertices\n", b->nff.pt_number);
  for(i = 0 ; i < b->nff.pt_number; i++ )
    fprintf (outflow, "%f %f %f,\n",
             b->nff.vtx[i].x, b->nff.vtx[i].y, b->nff.vtx[i].z);
  fprintf (outflow, "                ]\n");
  fprintf (outflow, "    }\n");

  // uv coordinate
  fprintf (outflow, "    TextureCoordinate2 {\n");
  fprintf (outflow, "        point [    ");
  for(i = 0 ; i < b->nff.pt_number; i++ )
    fprintf (outflow, "%f %f,\n",  b->nff.tex[i].u, b->nff.tex[i].v);
  fprintf (outflow, "                ]\n");
  fprintf (outflow, "    }\n");

  // polygons
  fprintf (outflow, "    IndexedFaceSet {\n");
  fprintf (outflow, "        coordIndex [     \n");
  fprintf (outflow, "# %d triangles\n", b->nff.tr_number);

  for(i = 0 ; i < b->nff.tr_number; i++ )
    fprintf (outflow, "%d, %d, %d, -1,\n", b->nff.triangle[i].vtx1,
             b->nff.triangle[i].vtx3, b->nff.triangle[i].vtx2);

  fprintf (outflow, "        ]\n");
  fprintf (outflow, "    }\n");
  fprintf (outflow, "}\n");

#if 0
  fprintf (outflow, "    ShapeHints {\n");
  fprintf (outflow, "        vertexOrdering  COUNTERCLOCKWISE\n");
  fprintf (outflow, "        creaseAngle     1.0\n");
  fprintf (outflow, "    }\n");
  fprintf (outflow, "    Material {\n");
  fprintf (outflow, "        ambientColor    0.2 0.2 0.2\n");
  fprintf (outflow, "        diffuseColor    0.42 0.41 0.35\n");
  fprintf (outflow, "        specularColor   0.2 0.2 0.2\n");
  fprintf (outflow, "        emissiveColor   0 0 0\n");
  fprintf (outflow, "        shininess       0\n");
  fprintf (outflow, "        transparency    0.5\n");
  fprintf (outflow, "    }\n");
  fprintf (outflow, "#   Texture2 {\n");
  fprintf (outflow, "#      filename \"tex2.rgb\"\n");
  fprintf (outflow, "#       wrapS   REPEAT\n");
  fprintf (outflow, "#       wrapT   REPEAT\n");
  fprintf (outflow, "#       model   MODULATE\n");
  fprintf (outflow, "#   }\n");
  fprintf (outflow, "#   Texture2Transform {\n");
  fprintf (outflow, "#       rotation        0.0\n");
  fprintf (outflow, "#       center          0.5 0.5\n");
  fprintf (outflow, "#       translation     0.0 0.0\n");
  fprintf (outflow, "#       scaleFactor     1.0 1.0\n");
  fprintf (outflow, "#   }\n");
  fprintf (outflow, "    Coordinate3 {\n");
  fprintf (outflow, "        point [    ");
  fprintf (outflow, "# %d vertices\n", b->nff.pt_number);

  for(i = 0 ; i < b->nff.pt_number; i++ )
    fprintf (outflow, "%f %f %f,\n", b->nff.vtx[i].x, b->nff.vtx[i].y,
             b->nff.vtx[i].z);

  fprintf (outflow, "                ]\n");
  fprintf (outflow, "    }\n");
  fprintf (outflow, "    IndexedFaceSet {\n");
  fprintf (outflow, "        coordIndex [     \n");
  fprintf (outflow, "# %d triangles\n", b->nff.tr_number);

  for(i = 0 ; i < b->nff.tr_number; i++ )
    fprintf (outflow, "%d, %d, %d, -1,\n", b->nff.triangle[i].vtx1,
             b->nff.triangle[i].vtx2, b->nff.triangle[i].vtx3);

  fprintf (outflow, "        ]\n");
  fprintf (outflow, "    }\n");
  fprintf (outflow, "}\n");

#endif

  printf("%s writen successfully\n", filename.c_str());

  fclose(outflow);
} // write_inventor_file

/*******************/
/* write vrml_file */
/*******************/

void write_vrml_impl(BUFFER *b, std::string const& filename, std::string const& texture_filename) {
  FILE *outflow;
  int i;

  // open output file
  if((outflow = fopen (filename.c_str(), "w" )) == 0) {
    fprintf (stderr, "write_vrml(): cannot open output file: %s\n", filename.c_str());
    exit(EXIT_FAILURE);
  }
  fprintf(outflow,"#VRML V1.0 ascii\n#\n");
  fprintf(outflow,"# Created by the Intelligent Robotics Group,\n");
  fprintf(outflow,"# NASA Ames Research Center\n");
  fprintf(outflow,"# File generated by the NASA Ames Stereo-Pipeline.\n");
  fprintf(outflow,"# %d vertices, %d triangles\n\n", b->nff.pt_number, b->nff.tr_number);

  fprintf (outflow, "Separator {\n");

  //   fprintf (outflow, "RotationXYZ {\n");
  //   fprintf (outflow, "axis X\n");
  //   fprintf (outflow, "angle 3.14159265358979\n");
  //   fprintf (outflow, "}\n");

  // Disabled for now until we have a better mechanism for passing in
  // these settings. -mbroxton
  //
  //   fprintf (outflow, "Material {\n");
  //   fprintf (outflow, "  ambientColor  %4.2f %4.2f %4.2f \n", dft->ambiColorRed,
  //       dft->ambiColorGreen, dft->ambiColorBlue );
  //   fprintf (outflow, "  diffuseColor  %4.2f %4.2f %4.2f \n", dft->diffColorRed,
  //       dft->diffColorGreen, dft->diffColorBlue );
  //   fprintf (outflow, "  specularColor  %4.2f %4.2f %4.2f \n", dft->specColorRed,
  //       dft->specColorGreen, dft->specColorBlue );
  //   fprintf (outflow, "  emissiveColor  %4.2f %4.2f %4.2f \n", dft->emisColorRed,
  //       dft->emisColorGreen, dft->emisColorBlue );
  //   fprintf (outflow, "  shininess %4.2f \n", dft->shininess);
  //   fprintf (outflow, "  transparency %4.2f \n}\n\n", dft->transparency);

  fprintf (outflow, "ShapeHints {\n");
  fprintf (outflow, "  vertexOrdering COUNTERCLOCKWISE\n");
//   if(dft->shapeType_solid)
    fprintf (outflow, "  shapeType SOLID\n");
//   else
//     fprintf (outflow, "  shapeType UNKNOWN_SHAPE_TYPE\n");
  fprintf (outflow, "  creaseAngle 3.1\n}\n\n");

  /* vertices position */
  fprintf (outflow, "Coordinate3 { \n");
  fprintf (outflow, "  point [\n");
  for(i = 0 ; i < b->nff.pt_number; i++ )
    fprintf (outflow, "    %f %f %f,\n", b->nff.vtx[i].x,
             b->nff.vtx[i].y, b->nff.vtx[i].z);
  fprintf (outflow, "    ]\n");
  fprintf (outflow, "}\n\n");

  // Texture file
  fprintf (outflow, "Texture2 {\n");
  fprintf (outflow, "  filename \"%s\"\n", texture_filename.c_str());
  fprintf (outflow, "}\n\n");

  /* uv coordinate */
  fprintf (outflow, "TextureCoordinate2 {\n");
  fprintf (outflow, "  point [\n");
  for(i = 0 ; i < b->nff.pt_number; i++ )
    fprintf (outflow, "    %f %f,\n",  b->nff.tex[i].u, b->nff.tex[i].v);
  fprintf (outflow, "    ]\n");
  fprintf (outflow, "}\n\n");

  /* polygons */
  fprintf (outflow, "IndexedFaceSet {\n");
  fprintf (outflow, "  coordIndex [\n");
  for(i = 0 ; i < b->nff.tr_number; i++ )
    fprintf (outflow, "    %d, %d, %d, -1,\n", b->nff.triangle[i].vtx1,
             b->nff.triangle[i].vtx3, b->nff.triangle[i].vtx2);
  fprintf (outflow, "    ]\n\n");
  fprintf (outflow, "  textureCoordIndex [\n");
  for(i = 0 ; i < b->nff.tr_number; i++ )
    fprintf (outflow, "    %d, %d, %d, -1,\n", b->nff.triangle[i].vtx1,
             b->nff.triangle[i].vtx3, b->nff.triangle[i].vtx2);
  fprintf (outflow, "    ]\n");
  fprintf (outflow, "}\n");

  // Close off Separator
  fprintf (outflow, "}\n");

  printf("%s written successfully\n", filename.c_str());
  fclose(outflow);
} // write_vrml_file

void write_trimesh_impl(BUFFER *b, std::string const& filename, bool flip_triangles) {
  FILE *outflow;
  int i;

  // open output file
  if((outflow = fopen (filename.c_str(), "w" )) == 0) {
    fprintf (stderr, "write_inventor: cannot open output file: %s\n", filename.c_str());
    exit(EXIT_FAILURE);
  }

  // vertices
  fprintf (outflow, "%d\n", b->nff.pt_number);

  for(i = 0 ; i < b->nff.pt_number; i++ )
    fprintf (outflow, "%f %f %f\n", b->nff.vtx[i].x,
             b->nff.vtx[i].y, b->nff.vtx[i].z);

  // triangles
  fprintf (outflow, "%d\n", b->nff.tr_number);

  for(i = 0 ; i < b->nff.tr_number; i++ ) {
                if (flip_triangles) {
                        fprintf (outflow, "%d %d %d\n", b->nff.triangle[i].vtx1,
                                                         b->nff.triangle[i].vtx2, b->nff.triangle[i].vtx3);
                } else {
                        fprintf (outflow, "%d %d %d\n", b->nff.triangle[i].vtx1,
                                                         b->nff.triangle[i].vtx3, b->nff.triangle[i].vtx2);
                }
        }

} // write_trimesh_file
