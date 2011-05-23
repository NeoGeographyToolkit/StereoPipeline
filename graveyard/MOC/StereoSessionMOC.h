// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionMOC.h
///

#ifndef __STEREO_SESSION_MOC_H__
#define __STEREO_SESSION_MOC_H__

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/Keypoint/StereoSessionKeypoint.h>

class StereoSessionMOC: public StereoSessionKeypoint {

public:

  boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file,
                                                          std::string camera_file);

  static StereoSession* construct() { return new StereoSessionMOC; }
};

#endif // __STEREO_SESSION_MOC_H__
