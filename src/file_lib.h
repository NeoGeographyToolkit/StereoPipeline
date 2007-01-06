/************************************************************************/
/*     File: file_lib.h                                                	*/ 
/*     Date: August 1996                                                */
/*       By: Eric Zbinden					  	*/
/*      For: NASA Ames Research Center, Intelligent Mechanisms Group  	*/
/* Function: Read, write & initialize, Files, buffers & struct		*/
/*									*/
/*    Links: 1. ** stereo_panorama to disparity map **	   		*/
/*		stereo.h  	structures + common stuff for *.c files	*/
/*		stereo.c  	main program (no procedure) 	       	*/
/*		filter.h .c     pgm (stereo+disp) filtering functions	*/
/*		disp.h .c	stereo to disparity fcts (correlation)	*/
/*		file_lib.h .c	read, write, convert functions for	*/
/*				.pgm images files  			*/
/*	     2. range.h .c	disparity to range functions		*/
/*			       	include read+write procedure		*/
/*	     3. dotcloud.h .c	range to 3D dot cloud position		*/
/*			       	include read+write procedure		*/
/*	     4. nff_terrain.h,c	range to 3D nff file (VEVI)		*/
/*			       	include read+write procedure		*/
/*	       	stereo.default	default parameters file		      	*/
/*									*/
/*    Notes: Stereo.c rebuild a 3D model of the world from a stereo	*/
/*	     Panorama. St_pan->filtering->disparity_map->range_map:	*/
/*	     	-> dot cloud 3D model					*/
/*		-> 3D .nff file						*/
/*          								*/
/************************************************************************/

#ifndef FILE_LIB_H
#define FILE_LIB_H

# include <stdlib.h>
# include <string.h>
# include <assert.h>
# include <stdio.h>
# include <stddef.h>
# include <math.h>

#include "stereo.h"
#include <vw/Image/ImageView.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/CameraModel.h>
using namespace vw;

// -----------------------------------------------------------------
// MATRIX I/O
//
// These are routines for writing matrix and vector objects to disk.
// Target file formats currently include OpenEXR and text (tab
// seperated).
// ------------------------------------------------------------------


// Write a matrix object to disk as an image file.  This function
// is particularly useful if you write the matrix as an OpenEXR
// image file; this retains the maximal amount of information.
//
template <class T>
void write_matrix(const std::string &filename, Matrix<T> &out_matrix) {
  
  // Convert the matrix to an image so that we can write it using
  // write_image().  There is probably a more efficient way to do
  // this, but this is the simple way to do it for now.
  
  ImageView<T> outImg(out_matrix.cols(), out_matrix.rows(), 1);
  unsigned int i, j;
  for (i = 0; i < out_matrix.cols(); i++) {
    for (j = 0; j < out_matrix.rows(); j++) {
      outImg(i,j) = out_matrix.impl()(j,i);
    }
  }
  
  write_image(filename, outImg);
}

// Read a matrix object from an image file on disk.  This function
// is particularly useful if you write the matrix as an OpenEXR
// image file; this retains the maximal amount of information.
//
template <class T>
void read_matrix(Matrix<T> &in_matrix, const std::string &filename) {
  
  // Convert the matrix to an image so that we can write it using
  // write_hdr_image().  There is probably a more effecient way to do
  // this, but this is the simple way to do it for now.
  ImageView<T> bufferImg;
  read_image(bufferImg, filename);
  
  VW_ASSERT(bufferImg.planes() == 1,
	    IOErr() << "ReadMatrix: Image file must be monochromatic (1 plane/channel) to read into a matrix.");
  
  Matrix<T> result(bufferImg.rows(), bufferImg.cols());
  unsigned int i, j;
  for (i = 0; i < bufferImg.cols(); i++) {
    for (j = 0; j < bufferImg.rows(); j++) {
      result.impl()(j,i) = bufferImg(i,j);
    }
  }
  in_matrix = result;
}

// Create a VRML file that contains the frames of reference for the
// spacecraft when each image was taken.  These reference frames are
// shown in a scene with a textured, Mars-sized ellipse for reference.
void write_orbital_reference_model(std::string filename, 
                                   vw::camera::CameraModel const& cam1, 
                                   vw::camera::CameraModel const& cam2);

/* Function Prototype  */
void	init_dft_struct(DFT_F *dft, TO_DO *todo);
void	init_header_struct(DFT_F *dft, F_HD *, char *, char *, char *);
void	read_default_file(DFT_F *df, TO_DO *execute, const char *filename);

#define HEIGHT  "height"
#define WIDTH   "width"
#define NBR_IMG "nbr_img"
#define EXTENTION "extention"
#define PAN_MIN "pan_min"
#define PAN_MAX "pan_max"
#define TILT    "tilt"
#define H_THETA_PIXEL "h_theta_pixel"
#define V_THETA_PIXEL "h_theta_pixel"
#define HCF "hcf"
#define VCF "vcf"
#define H_FOV "h_fov"
#define V_FOV "v_fov"
#define MAX_GRAY  "max_gray" 
#define FILE_TYPE "file_type"
#define TYPE "type"

#endif /* _FILE_LIB_H */
