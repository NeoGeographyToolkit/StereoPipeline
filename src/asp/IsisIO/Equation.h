// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __VW_CAMERA_EQUATION_H__
#define __VW_CAMERA_EQUATION_H__

#include <asp/IsisIO/BaseEquation.h>
#include <asp/IsisIO/PolyEquation.h>
#include <asp/IsisIO/RPNEquation.h>

namespace vw {
namespace camera {

  // Useful tools
  //-------------------------------------------------------

  // Debugging output of coefficients (constants)
  std::ostream& operator<<( std::ostream&, boost::shared_ptr<BaseEquation> eq);

  // Determines equation type and write appropriate
  void write_equation( std::ofstream& f, boost::shared_ptr<BaseEquation> eq);
  // Determines equation type from file and return appropriate
  boost::shared_ptr<BaseEquation> read_equation( std::ifstream& f);

}}

#endif//__VW_CAMERA_EQUATION_H__
