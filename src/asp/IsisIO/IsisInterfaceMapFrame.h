// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file IsisInterfaceMapFrame.h
///
/// Map Projected Frame Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_FRAME_H__
#define __ASP_ISIS_INTERFACE_MAP_FRAME_H__

namespace asp {
namespace isis {

  class IsisInterfaceMapFrame : public IsisInterface {

  public:
    IsisInterfaceMapFrame( std::string const& file ) {}

    virtual ~IsisInterfaceMapFrame() {}

    virtual std::string type()  { return "MapFrame"; }

  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_FRAME_H__
