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


#ifndef __ASP_ISIS_IO_H__
#define __ASP_ISIS_IO_H__

#include <vw/config.h> // must come before asp_config.h, defines VW_BOOST_VERSION
#include <asp/asp_config.h>

#include <asp/IsisIO/BaseEquation.h>
#include <asp/IsisIO/Equation.h>
#include <asp/IsisIO/PolyEquation.h>
#include <asp/IsisIO/RPNEquation.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/IsisCameraModel.h>
#endif

#endif//__ASP_ISIS_IO_H__
