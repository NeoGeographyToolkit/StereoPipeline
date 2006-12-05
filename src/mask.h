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

#ifndef __MASK_H__
#define __MASK_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Core/Exception.h>
/***************************************/
/* apply the mask on the disparity map */
/***************************************/
template <class PixelT>
void mask_image(vw::ImageView<PixelT> &input_image,
                vw::ImageView<bool> const& mask) {
  
  unsigned int i,j;
  
  VW_ASSERT((input_image.cols() == mask.cols()) && (input_image.rows() == mask.rows()), 
            vw::ArgumentErr() << "mask_image: Mask and input image do not have the same dimensions.");     
  
  for (i = 0; i < input_image.cols() ; i++) 
    for (j = 0; j < input_image.rows() ; j++) 
      if ( !mask(i,j) ) 
        input_image(i,j) = PixelT();
}

void write_mask(vw::ImageView<bool > buffer, std::string file_name);
vw::ImageView<bool> read_mask(std::string file_name);

#endif /* __MASK_H__ */
