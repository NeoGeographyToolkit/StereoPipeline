// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file StereoSessionRPC.h
///

#ifndef __STEREO_SESSION_RPC_H__
#define __STEREO_SESSION_RPC_H__

#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

namespace asp {

  class StereoSessionRPC : public StereoSessionPinhole {

  public:
    virtual ~StereoSessionRPC() {}

    // Produces a camera model from the images
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model( std::string const& image_file,
                  std::string const& camera_file = "" ) {
      vw::vw_throw( vw::NoImplErr() << "StereoSessionRPC: Not implemented" );
    }

    static StereoSession* construct() { return new StereoSessionRPC; }
  };

}

#endif // __STEREO_SESSION_RPC_H__
