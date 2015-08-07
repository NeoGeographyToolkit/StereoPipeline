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


/// \file Sessions.h
///

#include <asp/Sessions/StereoSession.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/Sessions/StereoSessionIsis.h>
#endif

#include <asp/Sessions/StereoSessionDG.h>
#include <asp/Sessions/StereoSessionDGMapRPC.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionRPC.h>

#include <asp/Sessions/StereoSessionFactory.h>
