// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#ifndef __ASP_EQUATION_H__
#define __ASP_EQUATION_H__

#include <iosfwd>

#include <boost/smart_ptr/shared_ptr.hpp>

namespace asp {

  class BaseEquation;

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
