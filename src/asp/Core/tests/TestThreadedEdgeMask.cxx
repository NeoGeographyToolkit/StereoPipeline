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


#include <test/Helpers.h>
#include <vw/Math/BBox.h>
#include <vw/Image/AlgorithmFunctions.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/Manipulation.h>
#include <asp/Core/ThreadedEdgeMask.h>

#include <algorithm>
#include <vector>

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
