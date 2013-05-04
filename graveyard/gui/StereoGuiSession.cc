// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
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

