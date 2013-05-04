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


/// \file MOLA.cc
///

#include <asp/Sessions/MOC/MOLA.h>
#include <asp/Sessions/MOC/MOLAReader.h>

using namespace vw;

#define MOLA_PEDR_EQUATORIAL_RADIUS 3396000.0

static const std::string mola_database = "/irg/data/mgs/mgs-m-mola-3-pedr-l1a-v1/mgsl_21xx/data";

void do_mola_comparison(ImageView<Vector3> const& moc_point_image,
                        MOCImageMetadata const& moc_metadata,
                        std::string const& output_prefix) {
  std::vector<Vector2> mola;
  std::vector<Vector2> synthetic;
  try{
    vw_out(0) << "Reading MOLA track:\n";
    mola = mola_track(moc_metadata,output_prefix);

    vw_out(0) << "Generating synthetic track:\n";
    synthetic = synthetic_track(moc_point_image, moc_metadata, output_prefix);
  } catch (MOLA_PEDR_Err &e) {
    vw_out(0) << "\tFailed to read entry from MOLA database file.  Proceeding without MOLA data.\n";
  } catch (LogicErr &e) {
    vw_out(0) << "\tFailed to create synthetic MOLA track. Proceeding without MOLA data.\n";
  }
}

std::vector<Vector2> mola_track(MOCImageMetadata const& moc_metadata,
                                std::string const& output_prefix) {

  std::cout << "\tAttempting to locate MOLA data for orbit number "
            << moc_metadata.orbit_number() << ".\n";

  // Hard code database location for now
  MOLA_PEDR_Reader reader(mola_database, moc_metadata.orbit_number());

  double query_time = moc_metadata.ephemeris_time();
  double scan_duration = moc_metadata.scan_duration();

  printf("\tQuery time: %f   Scan duration: %f\n", query_time, scan_duration);
  std::list<PEDR_Shot> mola_shots = reader.get_pedr_by_time_range(query_time,
                                                                  query_time + scan_duration);

  std::vector<Vector2> result;
  //    ImageView<double> mola_track(mola_shots.size(), 5, 1);

  std::string filename = output_prefix + "-mola-track.txt";
  FILE* output = fopen(filename.c_str(),"w");
  if (!output) throw IOErr() << "mola_track: could not open file for writing.";
  fprintf(output, "Orbit\tTime\tLat\tLon\tPlanetary_Radius\tGMM3_Areoid\tGMM3_Altitude\tSpheroid_Altitude\n");

  std::list<PEDR_Shot>::iterator iter;
  int nn;
  for (iter = mola_shots.begin(), nn = 0;
       iter != mola_shots.end(); ++iter, nn++) {
    result.push_back(Vector2((*iter).ephemeris_time() - query_time,
                             (*iter).shot_planetary_radius() - MOLA_PEDR_EQUATORIAL_RADIUS));

    printf("%d\t%10.10f\t%f\t%f\t%f\t%f\t%f\t%f\n",
           (*iter).orbit_reference_nmuber(),
           (*iter).ephemeris_time() - query_time,
           (*iter).areo_latitude(),
           (*iter).areo_longitude(),
           (*iter).shot_planetary_radius(),
           (*iter).areoid_radius(),
           (*iter).shot_planetary_radius() - (*iter).areoid_radius(),
           (*iter).shot_planetary_radius() - MOLA_PEDR_EQUATORIAL_RADIUS);
    fprintf(output, "%d\t%10.10f\t%f\t%f\t%f\t%f\t%f\t%f\n",
            (*iter).orbit_reference_nmuber(),
            (*iter).ephemeris_time() - query_time,
            (*iter).areo_latitude(),
            (*iter).areo_longitude(),
            (*iter).shot_planetary_radius(),
            (*iter).areoid_radius(),
            (*iter).shot_planetary_radius() - (*iter).areoid_radius(),
            (*iter).shot_planetary_radius() - MOLA_PEDR_EQUATORIAL_RADIUS);
  }

  fclose(output);
  vw_out(0) << "\tFound MOLA profile for image " << moc_metadata.moc_identifier() << ".\n";
  return result;
}


// Extract the corresponding elevation data from the stereo reconstruction.
std::vector<Vector2> synthetic_track(vw::ImageView<vw::Vector3> const& moc_point_image,
                                     MOCImageMetadata const& moc_metadata,
                                     std::string const& output_prefix) {

  int MOLA_MOC_BORESIGHT = int((double)(1574-(1024+moc_metadata.start_sample()))
                               / moc_metadata.crosstrack_summing());
  vw_out(0) << "\tMOLA Boresight pixel: " << MOLA_MOC_BORESIGHT << "\n";
  if (MOLA_MOC_BORESIGHT < 0 || MOLA_MOC_BORESIGHT > moc_point_image.cols())
    throw LogicErr() << "Boresight pixel not in captured image area.";

  double time_per_scanline = moc_metadata.scan_duration() / moc_point_image.rows();
  std::vector<Vector2> result;

  std::string filename = output_prefix + "-synthetic-track.txt";
  FILE* output = fopen(filename.c_str(),"w");
  if (!output) throw IOErr() << "mola_track: could not open file for writing.";

  fprintf(output, "Time\tLat\tLon\tPlanetary_Radius\tSpheroid_Altitude\n");

  for (int32 scanline = 0; scanline < moc_point_image.rows() ; scanline++) {
    //    vw_out(0) << scanline << "   " <<moc_point_image(MOLA_MOC_BORESIGHT, scanline) << "\n";
    if (moc_point_image(MOLA_MOC_BORESIGHT, scanline) != Vector3() ) {
      double radius = moc_point_image(MOLA_MOC_BORESIGHT, scanline).z() - MOLA_PEDR_EQUATORIAL_RADIUS;
      result.push_back(Vector2(time_per_scanline * scanline,radius));

      fprintf(output, "%f\t%f\t%f\t%f\t%f\n",
              time_per_scanline * scanline,
              moc_point_image(MOLA_MOC_BORESIGHT, scanline)[1],
              moc_point_image(MOLA_MOC_BORESIGHT, scanline)[0],
              moc_point_image(MOLA_MOC_BORESIGHT, scanline)[2],
              radius);
    }
  }

  fclose(output);
  vw_out(0) << "\tGenerated synthetic MOLA profile from reconstructed 3D surface.\n";
  return result;
}
