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

#include <asp/Camera/CsmModel.h>
#include <boost/scoped_ptr.hpp>
#include <test/Helpers.h>
#include <iomanip>

using namespace vw;
using namespace asp;


TEST(CSM_camera, basic_check) {

  // TODO: Get a better sample file.
  // Load up the sample ISD file.
  //std::string sample_isd = "/home/smcmich1/repo/StereoPipeline/src/asp/Camera/tests/sample_csm_isd.json";
  std::string sample_isd = "/home/smcmich1/repo/StereoPipeline/src/asp/Camera/tests/simpleFramerISD.json";

  CsmModel csm;
  csm.load_model(sample_isd);

  // Do some simple model tests
  const double eps = 0.00001;
  for (int i=0; i<10; ++i) {
    Vector2 pix(i*1,i*1);
    Vector3 center = csm.camera_center(pix);
    Vector3 vec    = csm.pixel_to_vector(pix);
    Vector3 pt     = center + vec * 10;
    EXPECT_VECTOR_NEAR(pix, csm.point_to_pixel(pt), eps);
  }
  
  EXPECT_TRUE(false);
}



