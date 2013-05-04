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


/// \file TestPEDR.h
///

#include <cxxtest/TestSuite.h>
#include "stereo.h"
#include "MOLA.h"
#include "vw/vw.h"

using namespace std;
using namespace vw;

class TestPEDR : public CxxTest::TestSuite {
 public:

  void testReadPEDRByTimeRange() {

    printf("\n\n");
    string database_dir = "/irg/data/mgs/mgs-m-mola-3-pedr-l1a-v1/mgsl_21xx/data";
    double query_time = -20723097.688171;   // Epoch for M01-00115
    double dt = 7.034798;                   // Scan duration for M01-00115
    unsigned orbit = 717;                  // Orbit number for M01-00115

    try {
      MOLA_PEDR_Reader reader(database_dir, orbit);

      list<PEDR_Shot> mola_shots = reader.get_pedr_by_time_range(query_time, 
								 query_time + dt);
      
      list<PEDR_Shot>::iterator iter;
      for (iter = mola_shots.begin(); iter != mola_shots.end(); iter++) {
	PEDR_Shot shot = *iter;
	printf("Orbit: %d  Time: %10.10f  Coord: [%f, %f, %f]  Areoid: %f  Alt: %f\n",
	       shot.orbit_reference_nmuber(),
	       shot.ephemeris_time(),
	       shot.areo_latitude(),
	       shot.areo_longitude(),
	       shot.shot_planetary_radius(),
	       shot.areoid_radius(),
	       shot.shot_planetary_radius() -
	       shot.areoid_radius());
      }
      
    } catch (MOLA_PEDR_Err &e) {
      printf("Could not find MOLA database file... Exiting.\n");
      exit(1);
    }   
    
  }


  void testReadPEDRByTime() {

    printf("\n\n");
    string database_dir = "/irg/data/mgs/mgs-m-mola-3-pedr-l1a-v1/mgsl_21xx/data";
    double query_time = -20723097.688171;   // Epoch for M01-00115
    double dt = 7.034798;                   // Scan duration for M01-00115
    unsigned orbit = 717;                  // Orbit number for M01-00115

    try {
      MOLA_PEDR_Reader reader(database_dir, orbit);

      PEDR_Shot shot = reader.get_pedr_by_time(query_time, dt);
     
      printf("Orbit: %d  Time: %10.10f  Coord: [%f, %f, %f]  Areoid: %f  Alt: %f\n",
	     shot.orbit_reference_nmuber(),
	     shot.ephemeris_time(),
	     shot.areo_latitude(),
	     shot.areo_longitude(),
	     shot.shot_planetary_radius(),
	     shot.areoid_radius(),
	     shot.shot_planetary_radius() -
	     shot.areoid_radius());

    } catch (MOLA_PEDR_NotFound_Err &e) {
      printf("No matching entry found in the MOLA database... Exiting.\n");
    } catch (MOLA_PEDR_Err &e) {
      printf("\n\nAn error occured:\n");
      e.print();
      exit(1);
    }  
  }

  
  void testReadPEDRByLatLon() {

    printf("\n\n");
    string database_dir = "/irg/data/mgs/mgs-m-mola-3-pedr-l1a-v1/mgsl_21xx/data";
    unsigned orbit = 717;                  // Orbit number for M01-00115

    float north = 34.466537;
    float south = 34.114227;
    float east = 141.575714;
    float west = 141.521225;

    try {
      MOLA_PEDR_Reader reader(database_dir, orbit);

      list<PEDR_Shot> mola_shots = reader.get_pedr_by_areo_latlon(north, 
								  south, 
								  east, 
								  west);
      
      list<PEDR_Shot>::iterator iter;
      for (iter = mola_shots.begin(); iter != mola_shots.end(); iter++) {
	PEDR_Shot shot = *iter;
	printf("Orbit: %d  Time: %10.10f  Coord: [%f, %f, %f]  Areoid: %f  Alt: %f\n",
	       shot.orbit_reference_nmuber(),
	       shot.ephemeris_time(),
	       shot.areo_latitude(),
	       shot.areo_longitude(),
	       shot.shot_planetary_radius(),
	       shot.areoid_radius(),
	       shot.shot_planetary_radius() -
	       shot.areoid_radius());
	

      }

    } catch (MOLA_PEDR_NotFound_Err &e) {
      printf("No matching entry found in the MOLA database... Exiting.\n");
    } catch (MOLA_PEDR_Err &e) {
      printf("\n\nAn error occured:\n");
      e.print();
      exit(1);
    }  
  }

};

