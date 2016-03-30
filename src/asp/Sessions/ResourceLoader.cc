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


/// \file ResourceLoader.cc
///

#include <vw/Math/Vector.h>
#include <vw/Image/ImageResource.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/FileIO/DiskImageResourceRaw.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Sessions/ResourceLoader.h>

namespace asp{


vw::Vector2i file_image_size(std::string const& input,
                             std::string const& camera_file) {
  boost::shared_ptr<vw::DiskImageResource> rsrc(load_disk_image_resource(input, camera_file));
  vw::Vector2i size( rsrc->cols(), rsrc->rows() );
  return size;
}


bool has_spot5_extension(std::string const& image_file, std::string const& camera_file){
  // First check the image file
  std::string image_ext = vw::get_extension(image_file);
  boost::algorithm::to_lower(image_ext);
  if ((image_ext == ".bip") || (image_ext == ".bil") || (image_ext == ".bsq"))
    return true;
  // If no camera file was provided it cannot be a Spot5 file
  if ((camera_file == "") || (camera_file == image_file))
    return false;
  // The Spot5 file is the last thing we check
  const std::string camera_ext = vw::get_extension(camera_file);
  return ((camera_ext == ".DIM") || (camera_ext == ".dim"));
}

boost::shared_ptr<vw::DiskImageResource> load_disk_image_resource(std::string const& image_file,
                                                                  std::string const& camera_file) {
  std::cout << "Loading disk image resource with: " << image_file << ", " << camera_file << std::endl;
  
  // Get input info
  const bool        spot5_ext = has_spot5_extension(image_file, camera_file);
  const std::string image_ext = vw::get_extension(image_file);
  
  if (spot5_ext && (image_ext != ".tif")) {
   
    // Special handling for RAW SPOT5 image files
    // - Read format info from the header, then construct the correct resource type.
    vw::ImageFormat format = SpotXML::get_image_format(camera_file);
    return boost::shared_ptr<vw::DiskImageResource>(
            vw::DiskImageResourceRaw::construct(image_file, format));
  }
  else { // Anything other than a RAW Spot5 file can be handled by this call.
    return boost::shared_ptr<vw::DiskImageResource>(vw::DiskImageResource::open(image_file));
  }
}

bool read_georeference_asp(vw::cartography::GeoReference &georef, 
                           std::string const& image_file, 
                           std::string const& camera_file) {
  // SPOT5 images never have a georeference.                       
  if (has_spot5_extension(image_file, camera_file))
    return false;
  // If not SPOT5, the normal function can handle it.
  return read_georeference(georef, image_file);
}




} // end namespace asp
