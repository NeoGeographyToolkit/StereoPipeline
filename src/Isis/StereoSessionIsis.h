// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
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
// __END_LICENSE__

/// \file StereoSessionIsis.h
///

#ifndef __STEREO_SESSION_ISIS_H__
#define __STEREO_SESSION_ISIS_H__

#include <vw/Image.h>

#include "StereoSession.h"
#include "stereo.h"

// Isis Headers
#include <SpecialPixel.h>

namespace vw {

//  IsisSpecialPixelFunc
//
/// Replace ISIS missing data values with a pixel value of your
/// choice.
template <class PixelT>
class IsisSpecialPixelFunc: public vw::UnaryReturnSameType {
  PixelT m_replacement_value;
  
  // Private
  IsisSpecialPixelFunc() : m_replacement_value(0) {}
  
public:
  IsisSpecialPixelFunc(PixelT const& pix) : m_replacement_value(pix) {}
  
  PixelT operator() (PixelT const& pix) const {
    typedef typename CompoundChannelType<PixelT>::type channel_type;
    for (int n = 0; n < CompoundNumChannels<PixelT>::value; ++n) {
      // Check to see if this is an Isis special value.  If it is,
      // return 0 for now.
      if (Isis::IsSpecial(compound_select_channel<const channel_type&>(pix,n))) 
        return m_replacement_value;
    }
    return pix;
  }
};
    
template <class ViewT>
UnaryPerPixelView<ViewT, IsisSpecialPixelFunc<typename ViewT::pixel_type> > 
remove_isis_special_pixels(ImageViewBase<ViewT> &image, typename ViewT::pixel_type replacement_value = typename ViewT::pixel_type() ) {
  return per_pixel_filter(image.impl(), IsisSpecialPixelFunc<typename ViewT::pixel_type>(replacement_value));
}

} // namespace vw

class StereoSessionIsis: public StereoSession {

  bool m_do_interest_point_alignment;
  
public:

  // Constructor
  StereoSessionIsis() { m_do_interest_point_alignment = true; }

  virtual ~StereoSessionIsis() {}

  virtual boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                                  std::string camera_file = "");

  // Stage 1: Preprocessing
  //
  // Pre file is a pair of images.            ( ImageView<PixelT> )
  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string & output_file1, std::string & output_file2);

  // Stage 2: Correlation
  //
  // Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
  // Post file is a disparity map.            ( ImageView<PixelDisparity<float> > )
  virtual void pre_filtering_hook(std::string const& input_file, std::string & output_file);

  // Stage 4: Point cloud generation
  //
  // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
  virtual void pre_pointcloud_hook(std::string const& input_file, std::string & output_file);

  static StereoSession* construct() { return new StereoSessionIsis; }

private:
  vw::math::Matrix<double> determine_image_alignment(std::string const& input_file1, std::string const& input_file2, float lo, float hi);

};

#endif // __STEREO_SESSION_ISIS_H__
