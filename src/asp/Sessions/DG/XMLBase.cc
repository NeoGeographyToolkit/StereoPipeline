// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <asp/Sessions/DG/XMLBase.h>

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
