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
  typedef std::map<std::string,asp::StereoSession::construct_func> ConstructMapType;
  ConstructMapType *stereo_session_construct_map = 0;
}

void asp::StereoSession::remove_duplicates(std::vector<vw::ip::InterestPoint> &ip1v,
                                           std::vector<vw::ip::InterestPoint> &ip2v) {
  using namespace vw;
  typedef std::vector<ip::InterestPoint> IPVector;
  IPVector new_ip1v, new_ip2v;
  new_ip1v.reserve( ip1v.size() );
  new_ip2v.reserve( ip2v.size() );

  for ( IPVector::iterator ip1 = ip1v.begin(), ip2 = ip2v.begin();
        ip1 < ip1v.end() - 1; ip1++, ip2++ ) {
    bool bad_entry = false;
    for ( IPVector::iterator ip1a = ip1 + 1, ip2a = ip2 + 1;
          ip1a < ip1v.end(); ip1a++, ip2a++ ) {
      if ( ip1->x == ip1a->x && ip1->y == ip1a->y &&
           ip2->x == ip2a->x && ip2->y == ip2a->y ) {
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

void asp::StereoSession::register_session_type( std::string const& id,
                                                asp::StereoSession::construct_func func) {
  if( ! stereo_session_construct_map )
    stereo_session_construct_map = new ConstructMapType();
  stereo_session_construct_map->insert( std::make_pair( id, func ) );
}

static void register_default_session_types() {
  static bool already = false;
  if ( already ) return;
  already = true;
  asp::StereoSession::register_session_type( "pinhole",
                                             &asp::StereoSessionPinhole::construct);
}

asp::StereoSession* asp::StereoSession::create( std::string const& session_type ) {
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
