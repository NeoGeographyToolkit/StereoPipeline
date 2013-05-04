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


/// \file nff_terrain.h
///

/************************************************************************/
/*     File: stereo.c                                                   */
/*     Date: August 1996                                                */
/*       By: Eric Zbinden                                               */
/*      For: NASA Ames Research Center, Intelligent Mechanisms Group    */
/* Function: Main program for the stereo panorama pipeline              */
/*    Links:    stereo.h        stereo.c                                */
/*              filters.h       filters.c                               */
/*              stereo_lib.h    stereolib.c                             */
/*              model_lib.h     modellib.c                              */
/*              file_lib.h      file_lib.c                              */
/*              stereo.default                                          */
/*                                                                      */
/*    Notes: Stereo.c rebuild a 3D model of the world from a stereo     */
/*           Panorama. St_pan->filtering->disparity_map->range_map:     */
/*              -> dot cloud 3D model                                   */
/*              -> 3D .nff file                                         */
/*                                                                      */
/************************************************************************/
#ifndef NFF_H
#define NFF_H
#include <vw/Image/ImageView.h>
#include <vw/Math/Vector.h>

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>

typedef struct
{
  float x;
  float y;
} POS2D;                        /* xy vector or xy vertex position */

typedef struct
{
  float x;
  float y;
  float z;
} POS3D;                        /* xyz vector or xyz vertex position */

typedef struct
{
  float u;
  float v;
} UV_TEX;                       /* texture coordinates */

/* polygon (triangle) structure   */

typedef struct {
  int   vtx1;
  int   vtx2;
  int   vtx3;
  int   gray;
} NFF_TR;                       /*  triangle sommet for nff terrain model */

typedef struct {
  int    pt_number;             /* number of vertices */
  int    tr_number;             /* number of triangle */
  POS3D  *vtx;                  /* array of vertices */
  UV_TEX *tex;                  /* texture coordinate */
  NFF_TR *triangle;             /* array of triangle */
} NFF;                          /* nff terrain model main structure */

typedef struct {
  POS3D *dot;                   /* pixel position in space */
  POS2D *gradients;             /* distance gradients */
  NFF   nff;                    /* nff structure */
} BUFFER;                       /* buffer structure */

// Function prototypes
extern void dot_to_adaptative_mesh(BUFFER *b, int width, int height, double mesh_tolerance, int max_triangles);
void dot_to_mesh(BUFFER *b, int width, int height, int h_step, int v_step);
extern void write_inventor_impl(BUFFER *b, std::string const& filename, std::string const& texture_filename, bool flip_triangles);
extern void write_vrml_impl(BUFFER *b, std::string const& filename, std::string const& texture_filename);
extern void write_osg_impl(BUFFER *b, std::string const& filename, std::string const& texture_filename);
extern void write_trimesh_impl(BUFFER *b, std::string const& filename, bool flip_triangles);


//  Class for building and tracking a 3D mesh
class Mesh {
  BUFFER buffers;  /* structure containing the different image/data buffers */

public:
  Mesh() {
    buffers.dot = NULL;
  }

  ~Mesh() { this->reset(); }

  void reset() {
    if (buffers.dot)
      delete [] buffers.dot;
    buffers.dot = NULL;
  }

  template <class ViewT>
  void build_adaptive_mesh(vw::ImageViewBase<ViewT> const& point_image, double mesh_tolerance, int max_triangles) {
    const ViewT& point_image_impl = point_image.impl();

    this->reset();

    buffers.dot = new POS3D[point_image_impl.cols() * point_image_impl.rows()];

    for (int j = 0; j < point_image_impl.rows(); j++) {
      for (int i = 0; i < point_image_impl.cols(); i++) {
        buffers.dot[i + (j * point_image_impl.cols())].x = point_image_impl.impl()(i,j)[0];
        buffers.dot[i + (j * point_image_impl.cols())].y = point_image_impl.impl()(i,j)[1];
        buffers.dot[i + (j * point_image_impl.cols())].z = point_image_impl.impl()(i,j)[2];
      }
    }

    dot_to_adaptative_mesh(&buffers,
                           point_image_impl.cols(), point_image_impl.rows(),
                           mesh_tolerance, max_triangles);

  }

  template <class ViewT>
  void build_simple_mesh(vw::ImageViewBase<ViewT> const& point_image, int h_step, int v_step) {
    const ViewT& point_image_impl = point_image.impl();

    this->reset();

    buffers.dot = new POS3D[point_image_impl.cols() * point_image_impl.rows()];

    for (int j = 0; j < point_image_impl.rows(); j++) {
      for (int i = 0; i < point_image_impl.cols(); i++) {
        buffers.dot[i + (j * point_image_impl.cols())].x = point_image_impl(i,j)[0];
        buffers.dot[i + (j * point_image_impl.cols())].y = point_image_impl(i,j)[1];
        buffers.dot[i + (j * point_image_impl.cols())].z = point_image_impl(i,j)[2];
      }
    }

    dot_to_mesh(&buffers,
                point_image_impl.cols(), point_image_impl.rows(),
                h_step, v_step );


  }

  void write_inventor(std::string const& filename, std::string const& texture_filename, bool flip_triangles = false) {
    write_inventor_impl(&buffers, filename, texture_filename, flip_triangles);
  }

  void write_vrml(std::string const& filename, std::string const& texture_filename) {
    write_vrml_impl(&buffers, filename, texture_filename);
  }

  void write_osg(std::string const& filename, std::string const& texture_filename) {
    write_osg_impl(&buffers, filename, texture_filename);
  }

  void write_trimesh(std::string const& filename, bool flip_triangles = false) {
    write_trimesh_impl(&buffers, filename, flip_triangles);
  }

};

#endif /* NFF_H */

/*******/
/* END */
/*******/




















