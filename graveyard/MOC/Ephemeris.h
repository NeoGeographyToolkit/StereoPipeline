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


/// \file Ephemeris.h
///

#ifndef __MOCEPHEMERIS_H__
#define __MOCEPHEMERIS_H__

#include <asp/Sessions/MOC/Metadata.h>

#include <string>
#include <vw/Camera/OrbitingPushbroomModel.h>

// Use this routine to read MOC telemetry directly from a *.sup file
// on disk.
void ReadSupplementalEphemerisFile(std::string const& filename, MOCImageMetadata &image_data);

// MOC-specific SPICE functions can also be used to read telemetry
// from an archive of raw MGS spice information.
void load_moc_kernels();
void MOC_state(double begin_time, double end_time, double interval,
               std::vector<vw::Vector3> &position,
               std::vector<vw::Vector3> &velocity,
               std::vector<vw::Quaternion<double> > &pose);

#endif // __MOCEPHEMERIS_H__
