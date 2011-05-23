// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///
#include <asp/asp_config.h>

#include <asp/Sessions/StereoSession.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/Sessions/ISIS/StereoSessionIsis.h>
#endif

#include <asp/Sessions/RMAX/StereoSessionRmax.h>
#include <asp/Sessions/Keypoint/StereoSessionKeypoint.h>
#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>
