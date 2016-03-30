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


/// \file ResourceLoader.h
///


/**
  This file contains safe file reading and loading functions to use in ASP.
  
  Most of these functions are required because the existing interfaces in 
  Vision Workbench are not capable of handling certain file types.  If you 
  want to remove these functions, the generic loading functionality in 
  Vision Workbench needs to be rewritten and many files will need to be 
  updated to work with the new interface.
  
*/

#ifndef __RESOURCE_LOADER_H__
#define __RESOURCE_LOADER_H__

#include <vw/FileIO/DiskImageResource.h>
#include <vw/Cartography/GeoReference.h>

namespace asp {

/// Overload of the function in asp/core/Common.h which can handle Spot5 data.
vw::Vector2i file_image_size( std::string const& input, std::string const& camera_file);

/// Return true if the image_file/camera file combination represents a SPOT5 camera file.
/// - Returns false if the camera_file input is empty.
bool has_spot5_extension(std::string const& image_file, std::string const& camera_file="");


/// Function to load a DiskImageResource from a supported camera image.
/// - This function is required because Spot5 images are not self-contained
///   and our generic loading function does not handle them.
boost::shared_ptr<vw::DiskImageResource> load_disk_image_resource(std::string const& image_file,
                                                                  std::string const& camera_file="");

/// Function to load a GeoReference from a supported camera image.
/// - This function is required because Spot5 images are not self-contained
///   and our generic loading function does not handle them.
bool read_georeference_asp(vw::cartography::GeoReference &georef, 
                           std::string const& image_file,
                           std::string const& camera_file="");

} // end namespace asp

#include <asp/Sessions/StereoSessionFactory.tcc>

#endif // __STEREO_SESSION_FACTORY_H__
