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

#include <asp/Core/BitChecker.h>
#include <vw/Core/Exception.h>

void asp::BitChecker::check_argument(vw::uint8 arg) {
  // Turn on the arg'th bit in m_checksum
  m_checksum.set(arg);
}

asp::BitChecker::BitChecker(vw::uint8 num_arguments): m_checksum(0) {
  VW_ASSERT(num_arguments != 0,
             vw::ArgumentErr() << "There must be at least one thing you read.\n");
  VW_ASSERT(num_arguments <= 32,
             vw::ArgumentErr() << "You can only have up to 32 checks.\n");

  // Turn on the first num_arguments bits in m_good
  m_good.reset();
  m_checksum.reset();
  for (vw::uint8 i = 0; i < num_arguments; i++)
    m_good.set(i);
}

bool asp::BitChecker::is_good() const {
  // Make sure all expected bits in m_checksum have been turned on.
  return (m_good == m_checksum);
}
