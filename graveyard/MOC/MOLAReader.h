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

/************************************************************************
 *     File: MOLA.h
 *     Date: February 2006
 *       By: Michael Broxton
 *      For: NASA Ames Research Center, Intelligent Robotics Group
 * Function: A C++ class for reading MOLA PEDR data records.
 *
 * Based in part on the splitpedr.c code written by C. David Brown,
 * Washington University.  splitpedr.c and other MOLA software can be
 * found at ftp://ltpftp.gsfc.nasa.gov/projects/tharsis/MOLA/SOFTWARE/
 *
 * Refer to the PEDRSEC1.FMT in the MOLA PEDR archive for a description
 * of each field returned by the MOLA_PEDR class.
 *
 ************************************************************************/
#ifndef __MOLA_PEDR_READER_H__
#define __MOLA_PEDR_READER_H__

#include <string>
#include <list>
#include <stdlib.h>

#include <vw/Core/Exception.h>

#define PEDR_HEADER_SIZE 7760
#define PEDR_RECORD_SIZE 776
#define PEDR_SHOTS_PER_FRAME 20


/* Byte indices for fields in the PEDR */
enum { EPHEMERIS_SECS = 0,  // seconds
       EPHEMERIS_MSECS = 4, // microseconds
       ORBIT_NUMBER = 8,    // uint
       AREO_LAT = 12,       // deg * 1e6
       AREO_LON = 16,       // deg * 1e6
       RADIAL_DIST = 20,    // centimeters
       SHOT_PLANETARY_RADIUS = 48,  // centimeters
       MDPT_PLANET_RADIUS = 128,    // centimeters
       AREOID_RADIUS = 612, // centimeters
       DELTA_AREOID = 640,  // centimeters
       MOLA_CLOCK_RATE = 644,  // hertz
       DELTA_LATITUDE = 768,// degrees * 1e6
       DELTA_LONGITUDE = 772// degrees * 1e6
};

class PEDR_Frame {
 public:
  PEDR_Frame();
  PEDR_Frame(unsigned char* pedr_record);

  /*
   * Use this function to get a list of 20 MOLA shots for this
   * PEDR Frame.
   */
  std::list<class PEDR_Shot> MOLA_shots();

  double ephemeris_time();
  double start_time();
  double end_time();
  double shot_dt();
  unsigned int orbit_reference_nmuber();
  float areo_latitude();
  float areo_longitude();
  float mgs_radial_distance();   /* Distance from Mars center of mass to MGS */
  float shot_planetary_radius(); /* Average of the planetary radii measured for these 20 shots */
  float areoid_radius();         /* Radius of the Goddard Mars Model 3 for this frame */
  float delta_areoid();          /* Average change in the areoid height across the 20 MOLA shots in this frame */
  float delta_latitude();        /* Average change in the latitude across the 20 MOLA shots in this frame */
  float delta_longitude();       /* Average change in the longitude across the 20 MOLA shots in this frame */

 protected:
  unsigned char pedr_datum[PEDR_RECORD_SIZE];
};

class PEDR_Shot : public PEDR_Frame {
 public:
  PEDR_Shot();
  PEDR_Shot(unsigned char* pedr_record);
  PEDR_Shot(unsigned char* pedr_record, unsigned short shot_number);

  double ephemeris_time();       /* Corrected ephemeris time for this MOLA shot */
  float areo_latitude();         /* Corrected latitude for this MOLA shot */
  float areo_longitude();        /* Corrected longitude for this MOLA shot */
  float shot_planetary_radius(); /* Planetary radius mesaured for this shot */
  float areoid_radius();         /* Corrected oeuoeuf the Goddard Mars Model 3 for this frame */

 protected:
  unsigned short shot_id;
};

class MOLA_PEDR_Reader {
 public:
  /* Constructor / Destructor */
  MOLA_PEDR_Reader( std::string const& pedr_data_directory,
                    unsigned int orbit_number);
  ~MOLA_PEDR_Reader();

  /* Accessors */
  PEDR_Shot get_pedr_by_time(double target_time, double tol);
  std::list<PEDR_Shot> get_pedr_by_time_range(double et_start, double et_end);
  std::list<PEDR_Shot> get_pedr_by_areo_latlon(float north, float south,
                                               float east, float west);

 private:
  void reset_filepointer();

  char pedr_header[PEDR_HEADER_SIZE];
  FILE* pedr_file;
};

VW_DEFINE_EXCEPTION(MOLA_PEDR_Err, vw::Exception);

#endif // __MOLA_PEDR_READER_H__
