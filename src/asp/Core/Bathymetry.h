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


/// \file Bathymetry.h
///

#ifndef __ASP_CORE_BATHYMETRY_H__
#define __ASP_CORE_BATHYMETRY_H__

#include <vw/Math/Vector.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography/BathyStereoModel.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>

namespace asp {

class StereoSettings;

// Check that all settings are consistent for doing or not doing bathymetry correction
void bathyChecks(std::string const& session_name,
                 asp::StereoSettings const& stereo_settings,
                 int num_images);

// If to apply bathy correction
bool doBathy(asp::StereoSettings const& stereo_settings);

// Read all bathy data
void read_bathy_data(int num_images,
                     std::string const& bathy_mask_list,
                     std::string const& bathy_plane_files,
                     float refraction_index,
                     vw::BathyData & bathy_data);

} // end namespace asp

#endif //__ASP_CORE_BATHYMETRY_H__
