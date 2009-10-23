// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionRMAX.h
///

#ifndef __RMAX_STEREO_SESSION_H__
#define __RMAX_STEREO_SESSION_H__

#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

class StereoSessionRmax: public StereoSession {

public:

  virtual ~StereoSessionRmax() {}

  // Correct lens distortion and epipolar-rectify the images
  virtual boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file,
                                                                  std::string camera_file = "");

  static StereoSession* construct() { return new StereoSessionRmax; }
};

#endif // __RMAX_STEREO_SESSION_H__
