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


/// \file SpiceUtilities.cc
///

#include <asp/SpiceIO/SpiceUtilities.h>

#include "SpiceUsr.h"
#include "SpiceZfc.h"

#include <iostream>
#include <fstream>
#include <list>
#include <string>

#include <vw/Core/Exception.h>
#include <vw/Core/Debugging.h>

#include <string.h>

using namespace std;
using namespace vw;

namespace asp {
namespace spice {

  // The maximum SPICE message length (currently) is 23*80 = 1840
  // chars. This is not definied anywhere so we define it here.

  enum { LONG_MSG_LEN = 1840 };

  void CHECK_SPICE_ERROR() {
    char longms[LONG_MSG_LEN + 1];

    longms[LONG_MSG_LEN] = 0;                // ensure the string is terminated
    if (failed_c()) {
      getlms_(longms, LONG_MSG_LEN);
      reset_c();
      // Trim off the white space at the end
      for (int i = LONG_MSG_LEN-1; (i >= 0) && (longms[i] == ' '); --i)
        longms[i] = 0;
      cout
        << "SPICE: An error occured when accessing the SPICE information:\n\n"
        << longms;
      throw spice::SpiceErr()
        << "SPICE: An error occured when accessing the SPICE information:\n\n"
        << longms;
    }
  }

  // Convert from SCLK to ephemeris time for a spacecraft with a given
  // NAIF ID.
  double sclk_to_et(std::string sclk, int naif_id) {
    SpiceDouble query_time;

    scs2e_c( naif_id, sclk.c_str(), &query_time );
    CHECK_SPICE_ERROR();
    return query_time;
  }

  // Convert from Ephemeris time to UTC time.
  string et_to_utc(double ephemeris_time) {
    char utc_time[18];
    timout_c( ephemeris_time, "MM/DD/YYYY HR:MN:SC", 20, utc_time );
    CHECK_SPICE_ERROR();
    return string(utc_time);
  }

  // Convert from Ephemeris time to UTC time.
  double utc_to_et(std::string const& utc) {
    SpiceDouble et;
    std::string tmp = utc;
    // Spice can't cope with the optional trailing 'Z'
    if( tmp[tmp.size()-1] == 'Z' ) tmp.erase(tmp.size()-1,1);
    utc2et_c( tmp.c_str(), &et );
    CHECK_SPICE_ERROR();
    return et;
  }

  void kernel_param(std::string const& key, double &value) {
    SpiceInt nvalues;
    SpiceBoolean found;
    gdpool_c(key.c_str(), 0, 1, &nvalues, &value, &found);
    if (!found) { vw_throw(LogicErr() << "spice::kernel_param() could not find value for key \"" << key << "\""); }
  }

  template <int ElemN>
  void kernel_param(std::string const& key, Vector<double, ElemN> &value) {
    SpiceDouble elements[ElemN];
    SpiceInt nvalues;
    SpiceBoolean found;
    gdpool_c(key.c_str(), 0, ElemN, &nvalues, elements, &found);
    if (!found) { vw_throw(LogicErr() << "spice::kernel_param() could not find value for key \"" << key << "\""); }
    if (ElemN != nvalues) { vw_throw(LogicErr() << "spice::kernel_param() returned fewer elements than requested"); }
    for (int i=0; i < ElemN; ++i)
      value[i] = elements[i];
  }

  // Explicit instantiation
  template void kernel_param<2>(std::string const& key, Vector<double, 2> &value);
  template void kernel_param<3>(std::string const& key, Vector<double, 3> &value);
  template void kernel_param<4>(std::string const& key, Vector<double, 4> &value);

  // Load the state of a camera for a given time.
  void body_state(double time_,
                  Vector3 &position,
                  Vector3 &velocity,
                  Quat &pose,
                  std::string const& spacecraft,
                  std::string const& reference_frame,
                  std::string const& planet,
                  std::string const& instrument) {

    SpiceDouble time = time_;

    // Obtain the state vector of the spacecraft at the given
    // ephemeris time.
    //
    // The position and velocity of the spacecraft will be reported
    // relative to the MARS IAU2000 coordinate frame.  No light time
    // correction is made here since the spacecraft and the planet
    // surface (where the light originated) are very close.  (Normally
    // light time correction is computed between the origins of the
    // two coordinate frames.)
    SpiceDouble state[6];
    SpiceDouble light_time;

    spkezr_c( spacecraft.c_str(),  time,  reference_frame.c_str(), "NONE", planet.c_str(), state, &light_time );

    // Output is in km and km/s, so we must convert the state array to
    // units of m and m/s.
    position(0) = state[0] * 1000.0;
    position(1) = state[1] * 1000.0;
    position(2) = state[2] * 1000.0;
    velocity(0) = state[3] * 1000.0;
    velocity(1) = state[4] * 1000.0;
    velocity(2) = state[5] * 1000.0;

    // Get pose data from the spacecraft CK kernel
    //
    // Here, we explicity get the pose of the camera relative to the
    // Mars frame.  This incorporates the additional, slight rotation
    // from the spacecraft frame to the camera frame.
    SpiceDouble rotation_matrix[3][3];
    pxform_c( reference_frame.c_str(), instrument.c_str(), time, rotation_matrix);

    // Convert that matrix into a quaternion.
    SpiceDouble quaternion[4];
    m2q_c( rotation_matrix, quaternion );
    pose = Quat(quaternion[0],quaternion[1],quaternion[2],quaternion[3]);

    CHECK_SPICE_ERROR();
  }


  // Load the state of the MOC camera for a given time range, returning
  // observations of the state for the given time interval.
  void body_state(double begin_time, double end_time, double interval,
                  std::vector<Vector3> &position,
                  std::vector<Vector3> &velocity,
                  std::vector<Quat > &pose,
                  std::string const& spacecraft,
                  std::string const& reference_frame,
                  std::string const& planet,
                  std::string const& instrument) {

    unsigned int number_of_samples = (unsigned int)ceil((end_time - begin_time) / interval);

    position.resize(number_of_samples);
    velocity.resize(number_of_samples);
    pose.resize(number_of_samples);

    unsigned int interval_count = 0;
    for( SpiceDouble time = begin_time;
         time < end_time;
         time += interval ) {
      // Obtain the state vector of the spacecraft at the given
      // ephemeris time.
      //
      // The position and velocity of the spacecraft will be reported
      // relative to the MARS IAU2000 coordinate frame.  No light time
      // correction is made here since the spacecraft and the planet
      // surface (where the light originated) are very close.  (Normally
      // light time correction is computed between the origins of the
      // two coordinate frames.)
      SpiceDouble state[6];
      SpiceDouble light_time;

      spkezr_c( spacecraft.c_str(),  time,  reference_frame.c_str(), "NONE", planet.c_str(), state, &light_time );

      // Output is in km and km/s, so we must convert the state array to
      // units of m and m/s.
      position[interval_count](0) = state[0] * 1000.0;
      position[interval_count](1) = state[1] * 1000.0;
      position[interval_count](2) = state[2] * 1000.0;
      velocity[interval_count](0) = state[3] * 1000.0;
      velocity[interval_count](1) = state[4] * 1000.0;
      velocity[interval_count](2) = state[5] * 1000.0;

      // Get pose data from the spacecraft CK kernel
      //
      // Here, we explicity get the pose of the camera relative to the
      // Mars frame.  This incorporates the additional, slight rotation
      // from the spacecraft frame to the camera frame.
      SpiceDouble rotation_matrix[3][3];
      pxform_c( reference_frame.c_str(), instrument.c_str(), time, rotation_matrix);

      // Convert that matrix into a quaternion.
      SpiceDouble quaternion[4];
      m2q_c( rotation_matrix, quaternion );
      pose[interval_count] = Quat(quaternion[0],quaternion[1],quaternion[2],quaternion[3]);
      interval_count++;
    }

    CHECK_SPICE_ERROR();
  }

  // Load all relevent SPICE kernels.
  //
  // Someday, rather than hard coding these values, the user might be
  // able to specify which kernel files to load in the stereo.default
  // file or a similar configuration file.
  void load_kernels(std::list<std::string> &kernels) {

    // Alter the spice error handling behavior to allow us to handle
    // errors ourselves.
    int lenout = 0;
    char set[] = "SET", ret[] = "RETURN";
    erract_c (  set, lenout, ret  );

    // Load the kernels
    list<string>::iterator iter;
    for (iter = kernels.begin(); iter != kernels.end(); iter++)
      furnsh_c( (*iter).c_str() );

    CHECK_SPICE_ERROR();
  }

  // This variant reads a file containing a list of kernel files,
  // one per line.  The optional second parameter specifies a
  // path prefix.
  void load_kernels(std::string const& kernels_file, std::string const& prefix) {
    std::ifstream input_file;
    input_file.open(kernels_file.c_str(), std::ios::in);
    if (!input_file.good())
      vw_throw(IOErr() << "An error occured while opening the kernels file for reading.");

    std::list<std::string> kernel_list;
    char line[1024];
    while (!input_file.eof()) {
      input_file.getline(line,  1024);
      if (strlen(line) > 0) {
        kernel_list.push_back(prefix+line);
        vw_out(DebugMessage) << "Adding kernel....... \"" << (prefix+line) << "\"." << std::endl;;
      }
    }

    load_kernels(kernel_list);
  }

}} // namespace asp::spice
