// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSession.cc
///

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

#include <vw/Core/Exception.h>

#include <map>

// This creates an anonymous namespace where the lookup table for
// stereo sessions lives.
namespace {
  typedef std::map<std::string,StereoSession::construct_func> ConstructMapType;
  ConstructMapType *stereo_session_construct_map = 0;
}

void StereoSession::remove_duplicates(std::vector<vw::ip::InterestPoint> &ip1,
                                      std::vector<vw::ip::InterestPoint> &ip2) {
  using namespace vw;
  std::vector<ip::InterestPoint> new_ip1, new_ip2;

  for (unsigned i = 0; i < ip1.size(); ++i) {
    bool bad_entry = false;
    for (unsigned j = 0; j < ip1.size(); ++j) {
      if (i != j &&
          ((ip1[i].x == ip1[j].x && ip1[i].y == ip1[j].y) ||
           (ip2[i].x == ip2[j].x && ip2[i].y == ip2[j].y)) ) {
        bad_entry = true;
      }
    }
    if (!bad_entry) {
      new_ip1.push_back(ip1[i]);
      new_ip2.push_back(ip2[i]);
    }
  }

  ip1 = new_ip1;
  ip2 = new_ip2;
}

void StereoSession::register_session_type( std::string const& id,
                                           StereoSession::construct_func func) {
  if( ! stereo_session_construct_map )
    stereo_session_construct_map = new ConstructMapType();
  stereo_session_construct_map->insert( std::make_pair( id, func ) );
}

static void register_default_session_types() {
  static bool already = false;
  if( already ) return;
  already = true;
  StereoSession::register_session_type( "pinhole",
                                        &StereoSessionPinhole::construct);
}

StereoSession* StereoSession::create( std::string const& session_type ) {
  register_default_session_types();
  if( stereo_session_construct_map ) {
    ConstructMapType::const_iterator i =
      stereo_session_construct_map->find( session_type );
    if( i != stereo_session_construct_map->end() )
      return i->second();
  }
  vw_throw( vw::NoImplErr() << "Unsuppported stereo session type: "
            << session_type );
  return 0; // never reached
}
