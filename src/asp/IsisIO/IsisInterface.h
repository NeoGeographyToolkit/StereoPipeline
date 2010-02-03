// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file IsisInterface.h
///
/// Generic Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_H__
#define __ASP_ISIS_INTERFACE_H__

namespace asp {
namespace isis {

  // The IsisInterface abstract base class
  // -------------------------------------------------------

  class IsisInterface {
  public:
    virtual std::string type() = 0;

    static IsisInterface* open( std::string const& filename );
  };

}}

#endif//__ASP_ISIS_INTERFACE_H__
