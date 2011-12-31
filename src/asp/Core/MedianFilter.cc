// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <asp/Core/MedianFilter.h>

using namespace vw;

uint8 find_median_in_histogram(Vector<int, CALC_PIXEL_NUM_VALS> histogram,
                               int kernSize) {
  int acc = 0;
  int acc_limit = kernSize * kernSize / 2;

  uint8 i = 0;

  for (;;) {
    acc += histogram(i);

    if (acc >= acc_limit)
      break;

    i++;
  }

  return i;
}
