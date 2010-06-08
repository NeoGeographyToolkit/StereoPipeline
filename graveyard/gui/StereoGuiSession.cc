// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoGuiSession.cc
///

#include "StereoGuiSession.h"

#include <vw/Core/Thread.h>

// ---------------------------------------------------
// Create a single instance of the StereoSettings
// ---------------------------------------------------
namespace {
  vw::RunOnce stereo_gui_session_once = VW_RUNONCE_INIT;
  boost::shared_ptr<StereoGuiSession> stereo_gui_session_ptr;
  void init_stereo_gui_session() {
    stereo_gui_session_ptr = boost::shared_ptr<StereoGuiSession>(new StereoGuiSession());
  }
}

StereoGuiSession& stereo_gui_session() {
  stereo_gui_session_once.run( init_stereo_gui_session );
  return *stereo_gui_session_ptr;
}

