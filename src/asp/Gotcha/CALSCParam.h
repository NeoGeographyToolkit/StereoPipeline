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

#ifndef ASP_GOTCHA_CALSCPARAM_H
#define ASP_GOTCHA_CALSCPARAM_H

namespace gotcha {
  
class CALSCParam {

public:
    CALSCParam():m_nMaxIter(10),m_nPatch(30),m_fEigThr(150.f),m_fAffThr(1.5f),m_fDriftThr(0.8f),
                 m_bWeighting(false),m_bIntOffset(true){}

    // matching param
    int m_nMaxIter; // the max num of iterations
    int m_nPatch;   // the size of a matching patch
    float m_fEigThr;
    float m_fAffThr;
    float m_fDriftThr;
    bool m_bWeighting;
    bool m_bIntOffset;
};

} // end namespace gotcha

#endif // ASP_GOTCHA_CALSCPARAM_H

