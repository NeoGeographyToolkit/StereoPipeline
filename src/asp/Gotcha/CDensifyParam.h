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

#ifndef ASP_GOTCHA_CDENSIFYPARAM_H
#define ASP_GOTCHA_CDENSIFYPARAM_H

#include <asp/Gotcha/CGOTCHAParam.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace gotcha {

class CDensifyParam {

public:
  CDensifyParam(): m_nProcType(GOTCHA){}
  int m_nProcType;     // growing method
#if 0
  std::string m_strImgL;
  std::string m_strImgR;
  std::string m_strOutPath;
  std::string m_strTPFile;
  std::string m_strDispX;
  std::string m_strDispY;
  std::string m_strUpdatedDispX;
  std::string m_strUpdatedDispY;
  std::string m_strUpdatedDispSim;
#endif
  
  std::string getProcessingType(){
    if (m_nProcType == GOTCHA) return "GOTCHA";
    else if (m_nProcType == P_GOTCHA) return "P_GOTCHA";
    return "UNKNOWN";
  }
  
  CGOTCHAParam m_paramGotcha;
  
  enum {GOTCHA, P_GOTCHA};
  enum {NO_ERR, FILE_IO_ERR, GOTCHA_ERR, LINE_GOTCHA_ERR, P_GOTCHA_ERR};
};

} // end namespace gotcha

#endif // ASP_GOTCHA_CDENSIFYPARAM_H
