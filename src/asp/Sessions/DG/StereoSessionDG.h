// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file StereoSessionDG.h
///
/// This a session to hopefully support Digital Globe images from
/// Quickbird and World View.

#ifndef __STEREO_SESSION_DG_H__
#define __STEREO_SESSION_DG_H__

#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

namespace asp {

  class StereoSessionDG : public StereoSessionPinhole {

  public:
    StereoSessionDG();
    virtual ~StereoSessionDG();

    // Produces a camera model from the images
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model( std::string const& image_file,
                  std::string const& camera_file = "" );

    static StereoSession* construct() { return new StereoSessionDG; }
  };

}

#endif//__STEREO_SESSION_DG_H__
