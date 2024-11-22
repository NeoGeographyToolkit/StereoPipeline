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


/// \file SpiceUtilities.h
///

#ifndef __SPICE_H__
#define __SPICE_H__

#include <list>
#include <vector>
#include <string>
#include <vw/Core/Exception.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>

namespace asp {
namespace spice {

  VW_DEFINE_EXCEPTION(SpiceErr, vw::Exception);

  //  The NAIF SPICE ID value for various spacecraft and instruments
  const int NAIF_ID_MGS( -94    );    // Mars Global Surveyor
  const int NAIF_ID_MOC( -94031 );    // Mars Orbital Camera (MOC)
  const int NAIF_ID_MRO( -74    );    // Mars Reconnaisance Orbiter
  const int NAIF_ID_CLEMENTINE( -40    );    // Clementine

  // Function prototypes
  void load_kernels(std::list<std::string> &kernels);
  void load_kernels(std::string const& kernels_file, std::string const& prefix="");
  double sclk_to_et(std::string sclk, int naif_id);
  std::string et_to_utc(double ephemeris_time);
  double utc_to_et(std::string const& utc);

  template <int ElemN>
  void kernel_param(std::string const& key, vw::Vector<double, ElemN> &value);
  void kernel_param(std::string const& key, double &value);

  void body_state(double begin_time, double end_time, double interval,
                  std::vector<vw::Vector3> &position,
                  std::vector<vw::Vector3> &velocity,
                  std::vector<vw::Quaternion<double> > &pose,
                  std::string const& spacecraft,
                  std::string const& reference_frame,
                  std::string const& planet,
                  std::string const& instrument);

  void body_state(double time,
                  vw::Vector3 &position,
                  vw::Vector3 &velocity,
                  vw::Quaternion<double> &pose,
                  std::string const& spacecraft,
                  std::string const& reference_frame,
                  std::string const& planet,
                  std::string const& instrument);

}} // namespace asp::spice

#endif // __SPICE_H__
