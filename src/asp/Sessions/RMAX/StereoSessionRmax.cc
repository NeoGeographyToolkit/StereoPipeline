// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionRMAX.cc
///

#include <boost/shared_ptr.hpp>

#include <asp/Sessions/RMAX/StereoSessionRmax.h>
#include <asp/Sessions/RMAX/RMAX.h>

using namespace vw;
using namespace vw::camera;

boost::shared_ptr<vw::camera::CameraModel> StereoSessionRmax::camera_model(std::string image_file,
                                                                           std::string /*camera_file*/) {
  ImageInfo info;
  read_image_info( image_file, info );
  CAHVORModel* cahvor = new CAHVORModel;
  *cahvor = rmax_image_camera_model(info);
  return boost::shared_ptr<camera::CameraModel>(cahvor);
}
