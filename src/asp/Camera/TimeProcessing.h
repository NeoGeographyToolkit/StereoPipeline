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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

namespace asp {

  /// A functor that returns the difference in seconds from a reference time.
  /// Uses boost::posix_time.
  class SecondsFromRef {
    boost::posix_time::ptime m_reference;
  public:
    inline SecondsFromRef() {}
    inline SecondsFromRef(boost::posix_time::ptime const& ref_time) : m_reference(ref_time) {}
    
    inline void set_base_time(boost::posix_time::ptime const& ref_time) {m_reference = ref_time;}
    
    inline double operator()( boost::posix_time::ptime const& time ) const {
      return double( (time - m_reference).total_microseconds() ) / 1e6;
    }
  };

  // Boost does not like a time string such as "2017-12-07 15:36:40.90795Z"
  // because it expects precisely 6 digits after the dot (hence for the millisecond).
  // Fix that. PeruSat has this problem.
  std::string fix_millisecond(std::string const& in_str);

  boost::posix_time::ptime parse_time(std::string const& s);  
}
