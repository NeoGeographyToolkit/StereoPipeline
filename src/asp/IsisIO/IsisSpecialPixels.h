// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

#ifndef __ASP_ISISIO_ISISSPECIALPIXELS_H__
#define __ASP_ISISIO_ISISSPECIALPIXELS_H__

#include <vw/Image/ImageViewRef.h>

namespace asp {

// Adjust a masked image to handle ISIS special pixels. Reads valid min/max
// from the ISIS file and removes special pixels from the image.
void adjustIsisImage(std::string const& input_file,
                     float nodata_value,
                     vw::ImageViewRef<vw::PixelMask<float>> & masked_image);

} // end namespace asp

#endif//__ASP_ISISIO_ISISSPECIALPIXELS_H__
