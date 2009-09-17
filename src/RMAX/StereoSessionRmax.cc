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

/// \file StereoSessionRMAX.cc
///

#include <boost/shared_ptr.hpp>

#include "RMAX/StereoSessionRmax.h"
#include "RMAX/RMAX.h"

using namespace vw;
using namespace vw::camera;

boost::shared_ptr<vw::camera::CameraModel> StereoSessionRmax::camera_model(std::string image_file, 
                                                                           std::string camera_file) {
  ImageInfo info; 
  read_image_info( image_file, info );
  CAHVORModel* cahvor = new CAHVORModel;
  *cahvor = rmax_image_camera_model(info);
  return boost::shared_ptr<camera::CameraModel>(cahvor);
}
