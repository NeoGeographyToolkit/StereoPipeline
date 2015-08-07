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


#include <asp/Camera/XMLBase.h>

using namespace vw;

void asp::XMLBase::check_argument( vw::uint8 arg ) {
  m_checksum |= 0x1 << arg;
}

asp::XMLBase::XMLBase( vw::uint8 num_arguments )  : m_num_arguments(num_arguments), m_checksum(0) {
  VW_ASSERT( num_arguments != 0,
             ArgumentErr() << "There must be at least one thing you read.\n");
  VW_ASSERT( num_arguments <= 32,
             ArgumentErr() << "You can only have up to 32 checks.\n" );
  int32 x = 0x1;
  for ( uint8 i = num_arguments - 1; i != 0; --i ) {
    x <<= 1;
    x |= 0x1;
  }
  m_good = x;
}

bool asp::XMLBase::is_good() const {
  return (m_good ^ m_checksum) == 0;
}
