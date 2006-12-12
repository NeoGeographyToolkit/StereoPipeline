#include "Spice.h"

#include "SpiceUsr.h"
#include "SpiceZfc.h"

#include <iostream> 
#include <list>
#include <string>

#include <string.h>

using namespace std;

namespace spice {

// The maximum SPICE message length (currently) is 23*80 = 1840
// chars. This is not definied anywhere so we define it here.

  enum { LONG_MSG_LEN = 1840 };

void CHECK_SPICE_ERROR() {
  char longms[LONG_MSG_LEN + 1];
  int longms_len = LONG_MSG_LEN;

  longms[LONG_MSG_LEN] = 0;		   // ensure the string is terminated
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

// Load the state of the MOC camera for a given time range, returning 
// observations of the state for the given time interval.
void body_state(double begin_time, double end_time, double interval,
                std::vector<vw::Vector3> &position,
                std::vector<vw::Vector3> &velocity, 
                std::vector<vw::Quaternion<double> > &pose,
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
    pose[interval_count].w() = quaternion[0];
    pose[interval_count].x() = quaternion[1];
    pose[interval_count].y() = quaternion[2];
    pose[interval_count].z() = quaternion[3];
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
  erract_c (  "SET", lenout, "RETURN"  );

  // Load the kernels
  list<string>::iterator iter;
  for (iter = kernels.begin(); iter != kernels.end(); iter++)
    furnsh_c( (*iter).c_str() );
  
  CHECK_SPICE_ERROR();
}

} // namespace spice
