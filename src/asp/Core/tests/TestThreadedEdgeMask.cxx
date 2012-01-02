// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <test/Helpers.h>

#include <vw/Image/Algorithms.h>
#include <asp/Core/ThreadedEdgeMask.h>

using namespace vw;
using namespace asp;

TEST( ThreadedEdgeMask, active_area ) {
  ImageView<uint8> input(10,10);
  fill(input,0);
  fill(crop(input,2,2,3,3),255);

  EXPECT_EQ( BBox2i(2,2,3,3),
             threaded_edge_mask(input,0).active_area() );
  EXPECT_EQ( BBox2i(3,3,1,1),
             threaded_edge_mask(input,0,1).active_area() );

  ImageView<uint8> output = threaded_edge_mask(input,0);
  EXPECT_EQ( input, output );

  input.set_size(5,5);
  fill(input,0);
  fill(crop(input,1,1,3,3),255);

  EXPECT_EQ( BBox2i(1,1,3,3),
             threaded_edge_mask(input,0).active_area() );
  EXPECT_EQ( BBox2i(2,2,1,1),
             threaded_edge_mask(input,0,1).active_area() );

  output = threaded_edge_mask(input,0);
  EXPECT_EQ( input, output );
}
