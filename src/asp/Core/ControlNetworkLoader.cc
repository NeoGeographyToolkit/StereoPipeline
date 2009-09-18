// __BEGIN_LICENSE__
//
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
//
// Copyright 2008 Carnegie Mellon University. All rights reserved.
//
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
//
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file ControlNetworkLoader.cc
///
/// Control Network Loader has one function available to the public
/// and that is build_control_network at the bottom. It will load all
/// match files and triangulate all control points for the control
/// network giving by pointer.

/// Control Network Loader aims to be faster than previous method by
/// using an intermediate data struct called the Camera Relations
/// Network. It reduces the search length of inserting a new match
/// file by organizing it's base layer by camera. Compared to the
/// previous method this should reduce the search length 1/(# cameras)
/// * what ever it was before.

#include <asp/Core/ControlNetworkLoader.h>

// Standard
#include <vector>
#include <iostream>
#include <string>

// VW
#include <vw/Camera/ControlNetwork.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>
#include <vw/Core.h>

// Boost (for good measure)
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ip;

/////////////////////////////
// Prototypes and Snippets //
/////////////////////////////

static std::string prefix_from_filename( std::string const& filename ){
  std::string result = filename;
  int index = result.find(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

static std::string remove_path( std::string const& filename ){
  std::string result = filename;
  int index = result.rfind("/");
  if (index != -1)
    result.erase(0,index+1);
  return result;
}

// Secondary Layer of Camera Relations Network. This contains the
// match relations and is doubly linked.
struct InterestPtRelation {
  boost::shared_ptr<InterestPoint> ip;
  typedef boost::shared_ptr<InterestPtRelation> ipr;
  std::list<ipr> matches;
  unsigned camera_id;

  InterestPtRelation ( InterestPoint& mine, unsigned& camera ) : camera_id(camera) { ip = boost::shared_ptr<InterestPoint>(new InterestPoint(mine)); }
  void AttachIpr( ipr& m );
  void RecursiveListing( std::list<ipr>& listing );
};

// Base Layer of Camera Relations Network
struct CameraRelation {
  typedef boost::shared_ptr<InterestPtRelation> ipr;
  int id;
  std::string name;
  std::list<ipr> relations;

  CameraRelation ( int tid, std::string tname ): id(tid), name(tname) {}
  ipr FindMatching( InterestPoint const& key );
  void AttachIpr( boost::shared_ptr<InterestPtRelation>& m );

};

/////////////////////////////
// Comparisons             //
/////////////////////////////

inline bool operator==( InterestPtRelation const& ipr1,
                        InterestPtRelation const& ipr2 ) {
  if ( ipr1.ip->ix == ipr2.ip->ix &&
       ipr1.ip->iy == ipr2.ip->iy )
    return true;
  return false;
}

inline bool operator==( InterestPtRelation const& ipr1,
                        InterestPoint const& ip2 ) {
  if ( ipr1.ip->ix == ip2.ix &&
       ipr1.ip->iy == ip2.iy )
    return true;
  return false;
}

/////////////////////////////
// Implementations         //
/////////////////////////////

// Interest Pt Relations
//---------------------------

void InterestPtRelation::AttachIpr( ipr& m ) {
  for ( std::list<ipr>::iterator iter =
          matches.begin(); iter != matches.end(); iter++ ) {
    if ( **iter == *m )
      return;
  }
  matches.push_back( m );
}

void InterestPtRelation::RecursiveListing( std::list<ipr>& listing ) {

  for ( std::list<ipr>::iterator iter = matches.begin();
        iter != matches.end(); iter++ ) {
    bool contains = false;
    for ( std::list<ipr>::iterator from_list = listing.begin();
          from_list != listing.end(); from_list++ ) {
      if ( *from_list == *iter ) {
        contains = true;
        break;
      }
    }

    if ( !contains ) {
      listing.push_back( *iter );
      (*iter)->RecursiveListing( listing );
    }
  }
}

// Camera Relations
//---------------------------

boost::shared_ptr<InterestPtRelation> CameraRelation::FindMatching( InterestPoint const& key ) {
  for ( std::list<ipr>::iterator iter = relations.begin();
        iter != relations.end(); iter++ ) {
    if ( **iter == key )
      return *iter;
  }
  return ipr();
}

void CameraRelation::AttachIpr( boost::shared_ptr<InterestPtRelation>& m ) {
  /*
    for ( std::list<ipr>::iterator iter = m_relations.begin();
    iter != m_relations.end(); iter++ ) {
    if ( **iter == *m )
    return;
    }
  */
  relations.push_back( m );
}

// Functions for Users
//----------------------------
void build_control_network( boost::shared_ptr<ControlNetwork> cnet,
                            std::vector<boost::shared_ptr<vw::camera::CameraModel> > const& camera_models,
                            std::vector<std::string> image_files,
                            int min_matches ) {

  cnet->clear();

  // 1.) Build CRN and load matches
  std::vector<CameraRelation> crn;
  for ( unsigned i = 0; i < image_files.size(); i++ )
    crn.push_back( CameraRelation( i,
                                   prefix_from_filename( image_files[i] )));
  vw_out(0) << "Loading matches:\n";
  for ( unsigned i = 0; i < image_files.size(); ++i ) {
    for ( unsigned j = i+1; j < image_files.size(); ++j ) {
      std::string match_filename =
        prefix_from_filename( image_files[i] ) + "__" +
        prefix_from_filename( image_files[j] ) + ".match";

      if ( !fs::exists( match_filename ) ) {
        match_filename = remove_path(prefix_from_filename( image_files[i] ) )
          + "__" + remove_path(prefix_from_filename(image_files[j]))+".match";
        if (!fs::exists( match_filename))
          continue;
      }

      std::vector<InterestPoint> ip1, ip2;
      read_binary_match_file( match_filename, ip1, ip2 );
      if ( int( ip1.size() ) < min_matches ) {
        vw_out(0) << "\t" << match_filename << "    " << i << " <-> "
                  << j << " : " << ip1.size() << " matches. [rejected]\n";
      } else {
        vw_out(0) << "\t" << match_filename << "    " << i << " <-> "
                  << j << " : " << ip1.size() << " matches.\n";

        // Loading individual matches now
        for ( unsigned k = 0; k < ip1.size(); k++ ) {
          boost::shared_ptr<InterestPtRelation> ipr1, ipr2;
          ipr1 = crn[i].FindMatching( ip1[k] );
          ipr2 = crn[j].FindMatching( ip2[k] );

          // Match hasn't been previously loaded
          if ( ipr1 == boost::shared_ptr<InterestPtRelation>() ) {
            ipr1 = boost::shared_ptr<InterestPtRelation>( new InterestPtRelation( ip1[k], i ) );
            crn[i].AttachIpr( ipr1 );
          }
          if ( ipr2 == boost::shared_ptr<InterestPtRelation>() ) {
            ipr2 = boost::shared_ptr<InterestPtRelation>( new InterestPtRelation( ip2[k], j ) );
            crn[j].AttachIpr( ipr2 );
          }

          // Doubly linking
          ipr1->AttachIpr( ipr2 );
          ipr2->AttachIpr( ipr1 );
        }
      }
    }
  }

  // 2.) Build Control Network finally
  int spiral_error_count = 0;
  vw_out(0) << "Assembling Control Network:\n";
  for ( unsigned i = 0; i < crn.size() -1; ++i ) {
    typedef boost::shared_ptr<InterestPtRelation> ipr;

    // Iterating over match relations and building control points for
    // them.
    for ( std::list<ipr>::iterator iter = crn[i].relations.begin();
          iter != crn[i].relations.end(); iter++ ) {
      // 2.1) Building a listing of interest points for a control
      // point
      std::list<ipr> interestpts;
      interestpts.push_back(*iter);
      (*iter)->RecursiveListing( interestpts );

      ControlPoint cpoint( ControlPoint::TiePoint );

      // Now removing listed interest points from CRN, and then adding
      // control measure
      // 2.2) Adding this location
      {
        ControlMeasure m( (*iter)->ip->x,
                          (*iter)->ip->y,
                          (*iter)->ip->scale,
                          (*iter)->ip->scale,
                          (*iter)->camera_id );
        cpoint.add_measure( m );
      }
      // 2.3) Adding & Removing all the other locations
      std::list<ipr>::iterator measure = interestpts.begin();
      measure++;
      for ( ; measure != interestpts.end(); measure++ ) {
        crn[(*measure)->camera_id].relations.remove(*measure);
        ControlMeasure m( (*measure)->ip->x,
                          (*measure)->ip->y,
                          (*measure)->ip->scale,
                          (*measure)->ip->scale,
                          (*measure)->camera_id );
        cpoint.add_measure( m );
      }
      // 2.4) Removing this location
      iter = crn[i].relations.erase(iter);
      iter--;
      // 2.5) Checking for a spiral error
      {
        std::list<unsigned> previous_camera;
        bool match = false;
        for ( std::list<ipr>::iterator interest = interestpts.begin();
              interest != interestpts.end(); interest++ ) {
          for ( std::list<unsigned>::iterator previous = previous_camera.begin();
                previous != previous_camera.end(); previous++ ) {
            if ((*previous) == (*interest)->camera_id) {
              match = true;
              break;
            }
          }
          previous_camera.push_back( (*interest)->camera_id );
          if ( match ) {
            continue;
          }
        }
        if ( match ) {
          spiral_error_count++;
          continue;
        }
      }

      // 2.6) If haven't left for spiral error, add new control point
      if ( cpoint.size() > 1 )
        cnet->add_control_point( cpoint );
    }
  }
  if ( spiral_error_count != 0 )
    vw_out(0) << "\t" << spiral_error_count << " control points removed because of spiral errors.\n";

  VW_ASSERT( cnet->size() != 0, vw::Aborted() << "Failed to load any points, Control Network Empty\n");

  // 3.) Triangulating Positions
  for ( unsigned i = 0; i < cnet->size(); i++ ) {

    std::vector< Vector3 > positions;
    double error = 0, error_sum = 0;

    const double min_convergence_angle = 5.0*3.1415/180.0;

    // 3.1) Building a permutation listing of triangulation
    for ( unsigned j = 0; j < (*cnet)[i].size(); j++ ) {
      for ( unsigned k = j+1; k < (*cnet)[i].size(); k++ ) {

        StereoModel sm( camera_models[ (*cnet)[i][j].image_id()].get(),
                        camera_models[ (*cnet)[i][k].image_id()].get() );

        if ( sm.convergence_angle( (*cnet)[i][j].position(),
                                   (*cnet)[i][k].position() ) >
             min_convergence_angle ) {
          positions.push_back( sm( (*cnet)[i][j].position(),
                                   (*cnet)[i][k].position(),
                                   error ) );
          error_sum += error;
        }
      }
    }

    // 3.2) Summing, Averaging and Storing
    error_sum /= positions.size();
    Vector3 position_avg;
    for ( unsigned j = 0; j < positions.size(); j++ )
      position_avg += positions[j]/positions.size();

    (*cnet)[i].set_position( position_avg );

  }
}

// Loads GCPs the traditional route
void add_ground_control_points_past( boost::shared_ptr<vw::camera::ControlNetwork> cnet,
                                     std::vector<std::string> image_files ) {
  vw_out(0) << "\nLoading Ground Control Points:\n";
  for ( unsigned i = 0; i < image_files.size(); ++i ) {
    std::string potential_gcp = prefix_from_filename( image_files[i] ) + ".gcp";

    if ( fs::exists( potential_gcp ) ) {
      int count = 0;

      // Now Loading the Ground Control Points
      std::ifstream istr(potential_gcp.c_str());

      while (!istr.eof()) {
        Vector2 pix;
        Vector3 loc;
        Vector3 sigma;

        istr >> pix[0] >> pix[1] >> loc[0] >> loc[1] >> loc[2] >> sigma[0] >> sigma[1] >> sigma[2];
        if (loc[0] !=0) {  // ignore blank lines
          Vector3 xyz = cartography::lon_lat_radius_to_xyz(loc);
          vw_out(0) << "GCP: " << xyz << "\n";

          ControlMeasure m(pix[0], pix[1], 1.0, 1.0, i);
          ControlPoint cpoint(ControlPoint::GroundControlPoint);
          cpoint.set_position(xyz[0],xyz[1],xyz[2]);
          cpoint.set_sigma(sigma[0],sigma[1],sigma[2]);
          cpoint.add_measure(m);

          // Deciding where to put it
          bool just_append = true;
          for ( unsigned p = 0; p < cnet->size(); ++p) {
            if ((*cnet)[p].type() == ControlPoint::GroundControlPoint ) {
              if (((*cnet)[p].position()[0] - xyz[0]) < .001 &&
                  ((*cnet)[p].position()[1] - xyz[1]) < .001 &&
                  ((*cnet)[p].position()[2] - xyz[2]) < .001) {
                // I'm not checking measures because there is only one GCP
                // file per camera
                just_append = false;
                (*cnet)[p].add_measure(m);
                break;
              }

            }
          }

          if (just_append)
            cnet->add_control_point(cpoint);

          ++count;
        }
      }
      istr.close();

      vw_out(0) << "\t" << potential_gcp << "    " << " : "
                << count << " GCPs.\n";
    }
  }
}

// This sifts out from a vector of strings, a listing of GCPs.  This
// should be useful for those programs who accept their data in a mass
// input vector.
std::vector<std::string>
sort_out_gcps( std::vector<std::string>& image_files ) {
  std::vector<std::string> gcp_files;
  std::vector<std::string>::iterator it = image_files.begin();
  while ( it != image_files.end() ) {
    if ( boost::iends_with(*it, ".gcp") ){
      gcp_files.push_back( *it );
      it = image_files.erase( it );
    } else
      it++;
  }

  return gcp_files;
}

// This will add a single ground control point file into the control
// network. This is different from in the past as this gcp file
// represents a single gcp. The gcp file inside contains a listing of
// image file names that see it and where it was located in each
// image. The very first line of the file defines the gcps location in
// the world and their sigmas.
void
add_ground_control_points( boost::shared_ptr<vw::camera::ControlNetwork> cnet,
                           std::vector<std::string> const& image_files,
                           std::vector<std::string> const& gcp_files ) {
  // Prep work
  // Creating a version of image_files that doesn't contain the path
  std::vector<std::string> pathless_image_files;
  for ( unsigned i = 0; i < image_files.size(); i++ )
    pathless_image_files.push_back(remove_path(image_files[i]));

  vw_out(0) << "\nLoading Ground Control Points:\n";
  for ( std::vector<std::string>::const_iterator gcp_name = gcp_files.begin();
        gcp_name != gcp_files.end(); gcp_name++ ) {

    if ( !fs::exists( *gcp_name ) )
      continue;

    // Data to be loaded
    std::vector<Vector2> measure_locations;
    std::vector<std::string> measure_cameras;
    Vector3 world_location, world_sigma;

    vw_out(0) << "\tLoading \"" << *gcp_name << "\".\n";
    int count = 0;
    std::ifstream ifile( (*gcp_name).c_str() );
    while (!ifile.eof()) {
      if ( count == 0 ) {
        // First line defines position in the world
        ifile >> world_location[0] >> world_location[1]
              >> world_location[2] >> world_sigma[0]
              >> world_sigma[1] >> world_sigma[2];
      } else {
        // Other lines define position in images
        std::string temp_name;
        Vector2 temp_loc;
        ifile >> temp_name >> temp_loc[0] >> temp_loc[1];
        measure_locations.push_back( temp_loc );
        measure_cameras.push_back( temp_name );
      }
      count++;
    }
    ifile.close();

    // Building Control Point
    Vector3 xyz = cartography::lon_lat_radius_to_xyz(world_location);
    vw_out(0) << "\t\tLocation: " << xyz << std::endl;
    ControlPoint cpoint(ControlPoint::GroundControlPoint);
    cpoint.set_position(xyz[0],xyz[1],xyz[2]);
    cpoint.set_sigma(world_sigma[0],world_sigma[1],world_sigma[2]);

    // Adding measures
    std::vector<Vector2>::iterator m_iter_loc = measure_locations.begin();
    std::vector<std::string>::iterator m_iter_name = measure_cameras.begin();
    while ( m_iter_loc != measure_locations.end() ) {
      unsigned camera_index;
      for (camera_index = 0; camera_index < image_files.size(); camera_index++ ) {
        if ( *m_iter_name == image_files[camera_index] )
          break;
        else if ( *m_iter_name == pathless_image_files[camera_index])
          break;
      }
      if ( camera_index == image_files.size() ) {
        vw_out(0) << "\t\tWarning: no image found matching "
                  << *m_iter_name << std::endl;
      } else {
        vw_out(0) << "\t\tAdded Measure: " << *m_iter_name << " #"
                  << camera_index << std::endl;
        ControlMeasure cm( (*m_iter_loc).x(), (*m_iter_loc).y(),
                           1.0, 1.0, camera_index );
        cpoint.add_measure( cm );
      }
      m_iter_loc++;
      m_iter_name++;
    }

    // Appended GCP
    cnet->add_control_point(cpoint);
  }
}
