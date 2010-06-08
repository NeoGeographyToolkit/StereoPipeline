// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __ASP_EQUATION_H__
#define __ASP_EQUATION_H__

#include <asp/IsisIO/BaseEquation.h>
#include <asp/IsisIO/PolyEquation.h>
#include <asp/IsisIO/RPNEquation.h>

namespace asp {

  // Useful tools
  //-------------------------------------------------------

  // Debugging output of coefficients (constants)
  std::ostream& operator<<( std::ostream&, boost::shared_ptr<BaseEquation> eq);

  // Determines equation type and write appropriate
  void write_equation( std::ofstream& f, boost::shared_ptr<BaseEquation> eq);
  // Determines equation type from file and return appropriate
  boost::shared_ptr<BaseEquation> read_equation( std::ifstream& f);

}

#endif//__ASP_EQUATION_H__
