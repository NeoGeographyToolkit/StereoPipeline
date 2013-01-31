// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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

#include <asp/Tools/stereo.h>
#include <vw/InterestPoint.h>

using namespace vw;
using namespace asp;

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      CorrelationDescription() );

    // Integer correlator requires 1024 px tiles
    //---------------------------------------------------------
    opt.raster_tile_size = Vector2i(1024,1024);

    // Internal Processes
    //---------------------------------------------------------
    pre_correlation( opt );

  } ASP_STANDARD_CATCHES;

  return 0;
}
