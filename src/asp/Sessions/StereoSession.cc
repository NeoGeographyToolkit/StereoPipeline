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

void StereoSession::remove_duplicates(std::vector<vw::ip::InterestPoint> &ip1v,
                                      std::vector<vw::ip::InterestPoint> &ip2v) {
  using namespace vw;
  typedef std::vector<ip::IntersetPoint> IPVector;
  IPVector new_ip1v, new_ip2v;
  new_ip1v.reserve( ip1v.size() );
  new_ip2v.reserve( ip2v.size() );

  for ( IPVector::iterator ip1 = ip1v.begin();
        ip1 < ip1v.end() - 1; ip1++ ) {
    bool bad_entry = false;
    for ( IPVector::iterator ip2 = ip1 + 1;
          ip2 < ip2v.end(); ip2++ ) {
      if ( ip1->x == ip2->x &&
           ip1->y == ip2->y &&
           ip1->scale == ip2->scale ) {
        bad_entry = true;
        break;
      }
    }
    if ( !bad_entry ) {
      new_ip1v.push_back( *ip1 );
      new_ip2v.push_back( *ip2 );
    }
  }
  ip1v = new_ip1v;
  ip2v = new_ip2v;
}

void StereoSession::register_session_type( std::string const& id,
                                           StereoSession::construct_func func) {
  if( ! stereo_session_construct_map )
    stereo_session_construct_map = new ConstructMapType();
  stereo_session_construct_map->insert( std::make_pair( id, func ) );
}

static void register_default_session_types() {
  static bool already = false;
  if ( already ) return;
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
