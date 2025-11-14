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

// Replace ISIS special pixel values with given replacement values.
// Specialize this only for float pixels, as that's all that is needed.
vw::ImageViewRef<float>
remove_isis_special_pixels(vw::ImageViewRef<float> const& image,
                           float r_low, float r_high, float r_null);

} // end namespace asp

#endif//__ASP_ISISIO_ISISSPECIALPIXELS_H__
