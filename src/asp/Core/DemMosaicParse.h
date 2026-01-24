// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file DemMosaicParse.h
/// Parse and validate dem_mosaic options

#ifndef __ASP_CORE_DEM_MOSAIC_PARSE_H__
#define __ASP_CORE_DEM_MOSAIC_PARSE_H__

#include <asp/Core/DemMosaicOptions.h>

namespace asp {

// Process the dem_mosaic options and sanity checks
void handleDemMosaicArgs(int argc, char *argv[], asp::DemMosaicOptions& opt);

} // end namespace asp

#endif // __ASP_CORE_DEM_MOSAIC_PARSE_H__
