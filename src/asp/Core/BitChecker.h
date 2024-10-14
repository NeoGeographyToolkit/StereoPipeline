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

/// \file BitChecker.h
///

#ifndef __ASP_CORE_BIT_CHECKER_H__
#define __ASP_CORE_BIT_CHECKER_H__

#include <vw/Core/FundamentalTypes.h>

#include <bitset>

namespace asp {

  // TODO: Replace with something else!
  /// Convenience class for setting flags and later on
  ///  making sure that we set all of them.
  /// - Designed to be inherited from.
  class BitChecker {
    std::bitset<32> m_checksum; ///< Store current bits
    std::bitset<32> m_good;     ///< Store target bits

  protected:
    /// Used to check off that one of the arguments has been read.
    void check_argument(vw::uint8 arg);

  public:
    /// Pass in the number of expected arguments, max 32
    BitChecker(vw::uint8 num_arguments);

    bool is_good() const; ///< Return true if all arguments have been checked.
  }; // End class BitChecker

} // end namespace asp

#endif//__ASP_CORE_BIT_CHECKER_H__
