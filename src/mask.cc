//
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//

//
// mask.h
// 
// Algorithms for generating image masks.  Adapted from code in the
// orginal stereo pipeline written by Eric Zbinden.
// 
// Created 13 July 2005 by mbroxton.
//  
//

#include "mask.h"
#include <vw/Core/Exception.h>
#include <vw/Image/Algorithms.h>
#include <vw/FileIO.h>


using namespace vw;

// Write a mask to disk, which basically means to turn all of the
// MISSING_PIXEL values white (these are masked pixels), and the other
// values black.
void write_mask(ImageView<bool> buffer, std::string file_name) {
  unsigned int i,j,p;
  unsigned int count = 0;

  ImageView<vw::uint8> outImg(buffer.cols(), buffer.rows(), buffer.planes());
  
  for (i = 0 ; i < buffer.cols(); i++) {
    for (j = 0 ; j < buffer.rows(); j++) {
      for (p = 0 ; p < buffer.planes(); p++) {
        if (buffer(i,j,p)) {
          count++;
          outImg(i,j,p) = 255;
        } else {
          outImg(i,j,p) = 0;
        }
      }
    }
  }
  printf("\tWriting file %s.         ", file_name.c_str());
  printf("[Mask Image: %0.2f %% of pixels are masked]\n",
	 (double) count * 100 / (double)(buffer.cols() * buffer.rows()) );
  write_image(file_name, outImg);
}

ImageView<bool> read_mask(std::string filename) {
  unsigned int i,j,p;
  std::cout << "\tLoading image " << filename << ":\t";

  ImageView<vw::uint8> in_image;
  read_image(in_image, filename);
  ImageView<bool> out_mask(in_image.cols(), in_image.rows(), in_image.planes());

  for (i = 0 ; i < in_image.cols(); i++) {
    for (j = 0 ; j < in_image.rows(); j++) {
      for(p = 0; p < in_image.planes(); p++) {
        if (in_image(i,j,p)) {
          out_mask(i,j,p) = true;
        } else {
          out_mask(i,j,p) = false;
        }
      }
    }
  }
  return out_mask; 
}


