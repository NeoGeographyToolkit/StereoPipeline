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
#include <boost/foreach.hpp>
#include <boost/random/normal_distribution.hpp>
#include <asp/Core/IntegralAutoGainDetector.h>
#include <vw/FileIO.h>

using namespace vw;
using namespace asp;

TEST( IntegralAutoGainDetector, VerifyMaxima ) {

  ImageView<float> test_image(101,101);

  const float SIGMA = 3;

  // Drawing a sinc function
  Vector2i location, center(50,50);
  for ( ; location.y() < test_image.rows(); location.y()++ ) {
    for ( location.x() = 0; location.x() < test_image.cols(); location.x()++ ) {
      float distance = norm_2( Vector2f(location - center) );
      test_image( location.x(), location.y() ) =
        40 * (
              1/(2*M_PI*SIGMA*SIGMA)*exp(-pow(distance,2)/(2*SIGMA*SIGMA)) -
              1/(2*M_PI*4*SIGMA*SIGMA)*exp(-pow(distance,2)/(2*4*SIGMA*SIGMA))
              ) + 0.5;
    }
  }

  IntegralAutoGainDetector detector;
  ip::InterestPointList list = detector.process_image( test_image );
  const ip::InterestPoint* best_ip = &list.front();
  float best_ip_value = list.begin()->interest;
  BOOST_FOREACH( ip::InterestPoint const& ip, list ) {
    if ( ip.interest > best_ip_value ) {
      best_ip_value = ip.interest;
      best_ip = &ip;
    }
  }
  EXPECT_EQ( best_ip->x, 50 );
  EXPECT_EQ( best_ip->y, 50 );
}
