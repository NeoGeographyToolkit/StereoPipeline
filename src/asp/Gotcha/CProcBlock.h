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

#ifndef ASP_GOTCHA_CPROCBLOCK_H
#define ASP_GOTCHA_CPROCBLOCK_H

#include <asp/Gotcha/CTiePt.h>
#include <asp/Gotcha/CALSCParam.h>
#include <asp/Gotcha/CGOTCHAParam.h>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <string>
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace gotcha {

class CProcBlock{
public:
  CProcBlock();
  
  // assume matrix has float data type
  //static bool loadMatrix(cv::Mat& matData, const std::string strFile); 
  
protected:
  // file I/O
  //bool loadMatrix(cv::Mat &matData, const std::string strFile, bool bDoublePrecision);
  //bool loadMatrix(std::string strFile);
  //void setImages(std::string strImgL, std::string strImgR, bool bGrey = true);
  //bool loadTP(const std::string strFile);
  bool saveTP(const std::vector<CTiePt>& vecTPs, const std::string strFile);
  bool saveMatrix(const cv::Mat& matData, const std::string strFile);  // assume matrix is in float or double data type
  bool saveALSCParam(const CALSCParam& paramALSC, const std::string strOut);
  bool saveGOTCHAParam(CGOTCHAParam& paramGOTCHA, const std::string strOut);

protected:
  //cv::Mat m_dispX;
};

} // end namespace gotcha
  
#endif // ASP_GOTCHA_CPROCBLOCK_H
