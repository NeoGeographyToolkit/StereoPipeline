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
#include <vw/FileIO/FileUtils.h>

namespace asp{


vw::Vector2i file_image_size(std::string const& input,
                             std::string const& camera_file) {
  boost::shared_ptr<vw::DiskImageResource> rsrc(load_disk_image_resource(input, camera_file));
  vw::Vector2i size( rsrc->cols(), rsrc->rows() );
  return size;
}

boost::shared_ptr<vw::DiskImageResource> load_disk_image_resource(std::string const& image_file,
                                                                  std::string const& camera_file) {
  //std::cout << "Loading disk image resource with: " << image_file << ", " << camera_file << std::endl;
  
  // Get input info
  const bool        spot5_ext = vw::has_spot5_extension(image_file, camera_file);
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

} // end namespace asp
