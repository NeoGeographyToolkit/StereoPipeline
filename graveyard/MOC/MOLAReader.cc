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


/// \file MOLAReader.cc
///

#include <asp/Sessions/MOC/MOLAReader.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <math.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/fstream.hpp"
namespace fs = boost::filesystem;

using namespace std;
using namespace vw;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*        General Utilities for parsing data from a PEDR         */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// Takes a four byte MOLA PEDR data record, with MSB first, and
// returns a reconstructed unsigned integer.
static inline unsigned int uchar_to_uint(const unsigned char* record) {
  return ((unsigned int)record[0] << 24) + ((unsigned int)record[1] << 16) +
         ((unsigned int)record[2] << 8)  + ((unsigned int)record[3]);
}


// Takes a four byte MOLA PEDR data record, with MSB first, and
// returns a reconstructed SIGNED integer.
static inline int uchar_to_int(const unsigned char* record) {
  return ((int)record[0] << 24) + ((int)record[1] << 16) +
          ((int)record[2] << 8) + ((int)record[3]);
}

inline double uchars_to_ephemeris_time( const unsigned char* secs,
                                        const unsigned char* msecs ) {
  double seconds = (double) uchar_to_int(secs);
  double microsecs = (double) uchar_to_int(msecs);

  //  printf(" -->  %lf  %lf\n", seconds, microsecs);
  return seconds + (microsecs * 1.0e-6);
}

inline float uchar_to_latlon ( const unsigned char* record ) {
  return ((float) uchar_to_int(record)) / 1.0e6;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                    PEDR_Shot Class Methods                    */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#define FRAME_OFFSET 9.5

PEDR_Shot::PEDR_Shot() : PEDR_Frame() { shot_id = 0; }

PEDR_Shot::PEDR_Shot(unsigned char* pedr_record) : PEDR_Frame(pedr_record) {
  throw MOLA_PEDR_Err() << "Cannot create a MOLA shot without specifying the shot number.";
}

PEDR_Shot::PEDR_Shot(unsigned char* pedr_record,
                     unsigned short shot_number) : PEDR_Frame(pedr_record) {
  if (shot_number > 19) {
    throw MOLA_PEDR_Err() << "Invalid MOLA shot number.  There are only 20 shots per MOLA frame.";
  }
  shot_id = shot_number;
}

// Corrected ephemeris time for this MOLA shot
double PEDR_Shot::ephemeris_time() {
  return PEDR_Frame::ephemeris_time() + ((double)shot_id - FRAME_OFFSET) * shot_dt();
}

// Corrected latitude for this MOLA shot
float PEDR_Shot::areo_latitude() {
  float d_lat = uchar_to_latlon( &pedr_datum[DELTA_LATITUDE] );
  return PEDR_Frame::areo_latitude() + ((float) shot_id - FRAME_OFFSET) / 20.0 * d_lat;
}

// Corrected longitude for this MOLA shot
float PEDR_Shot::areo_longitude() {
  float d_lon = uchar_to_latlon( &pedr_datum[DELTA_LONGITUDE] );
  return PEDR_Frame::areo_longitude() + ((float) shot_id - FRAME_OFFSET) / 20.0 * d_lon;
}

// Planetary radius mesaured for this shot
float PEDR_Shot::shot_planetary_radius() {
  unsigned char* frame_radii = &pedr_datum[SHOT_PLANETARY_RADIUS];
  return (float) uchar_to_uint(&frame_radii[4*shot_id]) / 100.0;  // 4-byte record
}

// Corrected oeuoeuf the Goddard Mars Model 3 for this frame
float PEDR_Shot::areoid_radius() {
  return PEDR_Frame::areoid_radius() +
    ((float) shot_id - FRAME_OFFSET) / 20.0 * PEDR_Frame::delta_areoid();
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                    PEDR_Frame Class Methods                    */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
PEDR_Frame::PEDR_Frame() {
  // Fill the PEDR record with NULL data
  for (unsigned int i = 0; i < PEDR_RECORD_SIZE; i++) {
    pedr_datum[i] = 0;
  }
}

PEDR_Frame::PEDR_Frame(unsigned char* record) {
  // Make a local copy of the PEDR record
  for (unsigned int i = 0; i < PEDR_RECORD_SIZE; i++) {
    pedr_datum[i] = record[i];
  }
}

// Use this function to get a list of 20 MOLA shots for this
// PEDR Frame.
std::list<class PEDR_Shot> PEDR_Frame::MOLA_shots() {

  unsigned short i;
  std::list<PEDR_Shot> shot_list;

  for (i = 0; i < PEDR_SHOTS_PER_FRAME; i++) {
    PEDR_Shot s(pedr_datum, i);  /* Create a MOLA_Shot */
    shot_list.push_back(s);
  }

  return shot_list;
}

double PEDR_Frame::ephemeris_time() {
  return uchars_to_ephemeris_time(&pedr_datum[EPHEMERIS_SECS],
                                  &pedr_datum[EPHEMERIS_MSECS]);
}

double PEDR_Frame::start_time() {
  return ephemeris_time() - FRAME_OFFSET * shot_dt();
}

double PEDR_Frame::end_time() {
  return ephemeris_time() + FRAME_OFFSET * shot_dt();
}

double PEDR_Frame::shot_dt() {
  return 0.1;  // NOMINAL MOLA SHOT RATE IS 10HZ
}

unsigned int PEDR_Frame::orbit_reference_nmuber() {
  return uchar_to_uint(&pedr_datum[ORBIT_NUMBER]);
}

float PEDR_Frame::areo_latitude() {
  return uchar_to_latlon(&pedr_datum[AREO_LAT]);
}

float PEDR_Frame::areo_longitude() {
  return uchar_to_latlon(&pedr_datum[AREO_LON]);
}

// Distance from Mars center of mass to MGS
float PEDR_Frame::mgs_radial_distance() {
  return (float) uchar_to_uint(&pedr_datum[RADIAL_DIST]) / 100.0;
}

// Average of the planetary radii measured for these 20 shots
float PEDR_Frame::shot_planetary_radius() {
  return (float) uchar_to_uint(&pedr_datum[MDPT_PLANET_RADIUS]) / 100.0;
}

/// Radius of the Goddard Mars Model 3 for this frame
float PEDR_Frame::areoid_radius() {
  return (float) uchar_to_uint(&pedr_datum[AREOID_RADIUS]) / 100.0;
}

// Average change in the areoid height across the 20 MOLA shots in this frame
float PEDR_Frame::delta_areoid() {
  return (float) uchar_to_int(&pedr_datum[DELTA_AREOID]) / 100.0;
}

// Average change in the latitude across the 20 MOLA shots in this frame
float PEDR_Frame::delta_latitude() {
  return (float) uchar_to_latlon(&pedr_datum[DELTA_LATITUDE]);
}

// Average change in the longitude across the 20 MOLA shots in this frame
float PEDR_Frame::delta_longitude() {
  return (float) uchar_to_latlon(&pedr_datum[DELTA_LONGITUDE]);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                  MOLA_PEDR_Reader Class Methods               */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */


MOLA_PEDR_Reader::MOLA_PEDR_Reader( std::string const& pedr_data_directory,
                                    unsigned int orbit_number) {

  ostringstream filename;
  unsigned int n = 0;

  filename << pedr_data_directory << "/ap"
           <<(orbit_number + 10000) / 1000 << "xxx/ap"
           << orbit_number + 10000 << "l.b";

  // Search for the PEDR file that has the orbit number that is
  // less than, but closest to, the orbit we are looking for.
  pedr_file = NULL;
  //  std::cout << "MOLA attempting to open " << filename.str() << std::endl;
  while (!(  pedr_file = fopen(filename.str().c_str(), "r"))) {
    long end = filename.tellp();
    filename.seekp(end - 16);
    filename << (orbit_number + 10000 - n) / 1000 << "xxx/ap"
             <<  orbit_number + 10000 - n << "l.b";
    n++;

    //    std::cout << "MOLA attempting to open " << filename.str() << "   " << n << "    " << orbit_number << std::endl;

    // If we have tried all of the orbit numbers in the directory,
    // just give up.
    if (n >= orbit_number) {
      throw MOLA_PEDR_Err() <<  "Could not find MOLA database file.";
    }
  }
  // Read ascii header.
  fread (pedr_header, PEDR_HEADER_SIZE, 1, pedr_file);
  cout << "\tSuccesfully opened MOLA PEDR record: " << filename.str() << ".\n";
}

MOLA_PEDR_Reader::~MOLA_PEDR_Reader() {
  if (pedr_file) {
    fclose(pedr_file);
  }
}

/* Reset the file reader to the beginning of the file */
void MOLA_PEDR_Reader::reset_filepointer() {
  fseek(pedr_file, PEDR_HEADER_SIZE, 0);
}


/*
 * Search for a record that is within tol seconds of the
 * target time.  The time reference is J2000 epoch (i.e.
 * "ephemeris time."
 */
PEDR_Shot MOLA_PEDR_Reader::get_pedr_by_time(double target_time,
                                             double tolerance) {

  double closest_et = 0.0;
  list<PEDR_Shot> nearby_shot_list;
  list<PEDR_Shot>::iterator shot_iterator;
  PEDR_Shot closest_shot;
  nearby_shot_list = get_pedr_by_time_range(target_time - tolerance / 2.0,
                                            target_time + tolerance / 2.0);

  // If there were no matches, throw an exception
  if (nearby_shot_list.size() == 0)
    throw MOLA_PEDR_Err() << "Could not find a shot that matched the target time.";

  // Otherwise, look for the closest match
  for(shot_iterator = nearby_shot_list.begin();
      shot_iterator != nearby_shot_list.end();
      shot_iterator++) {

    if (fabs((*shot_iterator).ephemeris_time() - target_time) <
        fabs(closest_et - target_time)) {
      closest_et = (*shot_iterator).ephemeris_time();
      closest_shot = *shot_iterator;
    }
  }

  return closest_shot;
}

std::list<PEDR_Shot>
MOLA_PEDR_Reader::get_pedr_by_time_range(double et_start, double et_end) {

  unsigned char current_pedr[PEDR_RECORD_SIZE];
  list<PEDR_Shot> current_shot_list;
  list<PEDR_Shot> selected_shot_list;
  list<PEDR_Shot>::iterator shot_iterator;

  /* Seek to the beginning of the data records */
  reset_filepointer();

  /* Search through the file and find the best match. */
  while (fread((char*) current_pedr, PEDR_RECORD_SIZE, 1, pedr_file)) {

    /* Extract the timestamp for the current ephemeris entry */
    PEDR_Frame frame(current_pedr);

    /*
     * First off, check to make sure that we haven't gone
     * past a time index in the file that is greater than
     * et_end.  If we have, we can safely terminate the file
     * search.
     */
    if (frame.start_time() > et_end)
      break;

    /*
     * Optimization: don't bother parsing frames that
     * do not contain the time ranges of interest.
     */
    if (frame.end_time() < et_start)
      continue;

    /*
     * We have to parse this PEDR frame into individual
     * shots and check each one individually to see if it falls
     * in the specified time range.  If it does, add it to the
     * list of selected shots.
     */
    current_shot_list = frame.MOLA_shots();
    for(shot_iterator = current_shot_list.begin();
        shot_iterator != current_shot_list.end();
        shot_iterator++) {

      if ((*shot_iterator).ephemeris_time() > et_start &&
          (*shot_iterator).ephemeris_time() < et_end) {
        selected_shot_list.push_back(*shot_iterator);
      }

    }
  }

  /* If there were no matches, throw an exception */
  if (selected_shot_list.size() == 0)
    throw MOLA_PEDR_Err() << "Could not find a shot that matched the target time range.";
  return selected_shot_list;
}


/*
 * Search for PEDR entries that lie within a region bounded by
 * the box given below.
 *
 * IMPORTANT NOTE:  Coordinates must be given in a planetocentric,
 * east positive frame of reference so that they are compatible with
 * the MOLA standard.
 *
 * Unfortunately this search method is not very fast (it takes several
 * seconds per orbit).
 */
std::list<PEDR_Shot>
MOLA_PEDR_Reader::get_pedr_by_areo_latlon(float north, float south,
                                          float east, float west) {


  float current_lat, current_lon;
  unsigned char current_pedr[PEDR_RECORD_SIZE];
  list<PEDR_Shot> selected_shot_list;
  list<PEDR_Shot> current_shot_list;
  list<PEDR_Shot>::iterator shot_iterator;

  /* Seek to the beginning of the data records */
  reset_filepointer();

  /* Search through the file and find the best match. */
  while (fread((char*) current_pedr, PEDR_RECORD_SIZE, 1, pedr_file)) {

    /* Extract the timestamp for the current ephemeris entry */
    PEDR_Frame frame(current_pedr);

    /*
     * We have to parse this PEDR frame into individual
     * shots and check each one individually to see if it falls
     * in the specified lat/lon range.  If it does, add it to the
     * list of selected shots.
     */
    current_shot_list = frame.MOLA_shots();
    for(shot_iterator = current_shot_list.begin();
        shot_iterator != current_shot_list.end();
        shot_iterator++) {

      current_lat = (*shot_iterator).areo_latitude();
      current_lon = (*shot_iterator).areo_longitude();

      if ((current_lat > south) && (current_lat < north) &&
          (current_lon > west) && (current_lon < east)) {
        selected_shot_list.push_back((*shot_iterator));
      }
    }
  }

  return selected_shot_list;
}


