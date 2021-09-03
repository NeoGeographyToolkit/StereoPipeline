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

#ifndef ASP_GOTCHA_CTIEPT_H
#define ASP_GOTCHA_CTIEPT_H

#include <opencv2/opencv.hpp>

namespace gotcha {
  
class CTiePt {

public:
    CTiePt():m_fSimVal(NOT_DEF){m_pfAffine[0] = 0; m_pfAffine [1] = 0;
                                m_pfAffine[2] = 0; m_pfAffine [3] = 0;}

  cv::Point2f m_ptL;      // left xy position
  cv::Point2f m_ptR;      // right xy position
  float m_fSimVal;        // matching similarity (score) from 0 to 1.0, othewise it is not defined
  
  float m_pfAffine [4];
  cv::Point2f m_ptOffset;
  
  enum {NOT_DEF = -1};
  
  bool operator==(const CTiePt& x){
    if ((this->m_ptL == x.m_ptL) && (this->m_ptR == x.m_ptR))
      return true;
    else return false;
  }
};

} // end namespace gotcha

#endif // ASP_GOTCHA_CTIEPT_H
