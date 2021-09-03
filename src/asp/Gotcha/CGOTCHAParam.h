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

#ifndef ASP_GOTCHA_CGOTCHAPARAM_H
#define ASP_GOTCHA_CGOTCHAPARAM_H

#include <string>
#include <asp/Gotcha/CALSCParam.h>

namespace gotcha {

class CGOTCHAParam {

public:
    CGOTCHAParam():m_nNeiType(NEI_4),m_fDiffCoef(0.05),m_fDiffThr(0.1),m_nDiffIter(5), m_bNeedInitALSC(true){ m_nMinTile = 1000000000;}

    std::string getNeiType(){if (m_nNeiType == NEI_X) return "NEI_X";
                        else if (m_nNeiType == NEI_Y) return "NEI_Y";
                        else if (m_nNeiType == NEI_4) return "NEI_4";
                        else if (m_nNeiType == NEI_8) return "NEI_8";
                        else if (m_nNeiType == NEI_DIFF) return "NEI_Difussion";
                        else return "UNKNOWN";}

    int m_nNeiType;
    int m_nMinTile;
    float m_fDiffCoef;
    float m_fDiffThr;
    int m_nDiffIter;

    //std::string m_strMask;

    CALSCParam m_paramALSC;
    bool m_bNeedInitALSC; // set true if initial alsc on seed points are required

    enum {NEI_X, NEI_Y, NEI_4, NEI_8, NEI_DIFF};
};

} // end namespace gotcha

#endif // ASP_GOTCHA_CGOTCHAPARAM_H
