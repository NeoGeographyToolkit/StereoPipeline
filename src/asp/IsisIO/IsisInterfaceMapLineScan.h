// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file IsisInterfaceMapLineScane.h
///
/// Map Projected Line Scan Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
#define __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__

namespace asp {
namespace isis {

  class IsisInterfaceMapLineScan {

  public:
    virtual std::string type()  { return "MapLineScan"; }

  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
