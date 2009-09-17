// __BEGIN_LICENSE__
// 
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2006 Carnegie Mellon University. All rights reserved.
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

#ifndef __VW_CAMERA_EQUATION_H__
#define __VW_CAMERA_EQUATION_H__

#include <Isis/BaseEquation.h>
#include <Isis/PolyEquation.h>
#include <Isis/RPNEquation.h>

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
