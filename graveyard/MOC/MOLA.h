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


/// \file MOLA.h
///

#ifndef __MOC_MOLA_H__
#define __MOC_MOLA_H__

#include <asp/Sessions/MOC/Metadata.h>
#include <vw/Image/ImageView.h>
#include <vw/Math/Vector.h>

void do_mola_comparison(vw::ImageView<vw::Vector3> const& moc_point_image,
                        MOCImageMetadata const& moc_metadata,
                        std::string const& output_prefix);

std::vector<vw::Vector2> mola_track(MOCImageMetadata const& moc_metadata,
                        std::string const& output_prefix);

std::vector<vw::Vector2> synthetic_track(vw::ImageView<vw::Vector3> const& moc_point_image,
                                         MOCImageMetadata const& moc_metadata,
                                         std::string const& output_prefix);


#endif
