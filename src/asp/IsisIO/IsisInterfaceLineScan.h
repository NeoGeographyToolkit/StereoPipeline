// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file IsisInterfaceLineScane.h
///
/// Line Scan Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_LINESCAN_H__
#define __ASP_ISIS_INTERFACE_LINESCAN_H__

#include <asp/IsisIO/IsisInterface.h>

namespace asp {
namespace isis {

  class IsisInterfaceLineScan : public IsisInterface {

  public:
    IsisInterfaceLineScan( std::string const& file ) {}

    virtual ~IsisInterfaceLineScan() {}

    virtual std::string type()  { return "LineScan"; }

  };

}}

#endif//__ASP_ISIS_INTERFACE_LINESCAN_H__
