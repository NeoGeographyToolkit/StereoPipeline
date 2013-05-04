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


/// \file Ephemeris.cc
///

#include <asp/SpiceIO/SpiceUtilities.h>
#include <asp/Sessions/MOC/Ephemeris.h>

#include <vw/Math/Matrix.h>
#include <vw/Math/Quaternion.h>
#include <vw/Camera/OrbitingPushbroomModel.h>

using namespace vw;
using namespace vw::camera;
using namespace std;

// ----------------------------------------------------------------
// SPICE related routines
// ----------------------------------------------------------------

void load_moc_kernels() {
  //  Constants
  const std::string moc_database = "/Users/mbroxton/local/data/mgs/kernels/";
  const std::string spice_database = "/Users/mbroxton/local/data/base/kernels/";

  list<string> spice_kernels;

  // Instrument Kernels and sysclk kernels
  spice_kernels.push_back( moc_database + "/iak/mocAddendum003.ti" );
  spice_kernels.push_back( moc_database + "/ik/moc13.ti" );
  spice_kernels.push_back( moc_database + "/ik/moc20.ti" );
  spice_kernels.push_back( moc_database + "/sclk/MGS_SCLKSCET.00061.tsc" );

  // SPK Kernels
  //  spice_kernels.push_back( moc_database + "/lsk/naif0008.tls" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ab1.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ab2.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_spo1.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_spo2.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map1.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map2.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map3.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map4.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map5.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map6.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map7.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_map8.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext1.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext2.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext3.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext4.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext5.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext6.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext7.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext8.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext9.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext10.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext11.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext12.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext13.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext14.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext15.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext16.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext17.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext18.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext19.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext20.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext21.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext22.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext23.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext24.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext25.bsp" );
  spice_kernels.push_back( moc_database + "/spk/mgs_ext26.bsp" );

  // We load the MOC ROTO kernels before the remaining CK kernels,
  // because these merely contain nominal (extrapolated) ROTO values
  // that might be overidden by actual CK telemetry.
  //  spice_kernels.push_back( moc_database + "/ck/mgs_ext_roto_all_v7.bc" );

  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ab1.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ab2.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_spo1.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_spo2.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map1.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map2.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map3.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map4.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map5.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map6.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map7.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_map8.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext1.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext2.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext3.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext4.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext5.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext6.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext7.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext8.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext9.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext10.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext11.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext12.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext13.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext14.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext15.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext16.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext17.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext18.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext19.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext20.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext21.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext22.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext23.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext24.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext25.bc" );
  spice_kernels.push_back( moc_database + "/ck/mgs_sc_ext26.bc" );

  // Standard kernels
  spice_kernels.push_back( spice_database + "/pck/pck00006.tpc" );
  spice_kernels.push_back( spice_database + "/pck/pck00007.tpc" );
  spice_kernels.push_back( spice_database + "/pck/pck00008.tpc" );
  spice_kernels.push_back( spice_database + "/pck/marsIAU1991.tpc" );
  spice_kernels.push_back( spice_database + "/pck/marsIAU1994.tpc" );
  spice_kernels.push_back( spice_database + "/spk/de118.bsp" );
  spice_kernels.push_back( spice_database + "/spk/de245.bsp" );
  spice_kernels.push_back( spice_database + "/spk/de405.bsp" );
  spice_kernels.push_back( spice_database + "/lsk/naif0007.tls" );
  spice_kernels.push_back( spice_database + "/lsk/naif0008.tls" );
  spice::load_kernels(spice_kernels);
}

// Load the state of the MOC camera for a given time range, returning
// observations of the state for the given time interval.
void MOC_state(double begin_time, double end_time, double interval,
               std::vector<vw::Vector3> &position,
               std::vector<vw::Vector3> &velocity,
               std::vector<vw::Quaternion<double> > &pose) {
  spice::body_state(begin_time, end_time, interval, position, velocity, pose,
                    "MGS", "IAU_MARS", "MARS", "MGS_MOC_NA");
}


