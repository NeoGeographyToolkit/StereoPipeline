#ifndef CPROCBLOCK_H
#define CPROCBLOCK_H

#include <fstream>
#include <string>
#include <time.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "CTiePt.h"
#include "CALSCParam.h"
#include "CGOTCHAParam.h"

class CProcBlock{
public:
  CProcBlock();
  
  // assume matrix has float data type
  //static bool loadMatrix(Mat& matData, const std::string strFile); 
  
protected:
  // file I/O
  //bool loadMatrix(Mat &matData, const std::string strFile, bool bDoublePrecision);
  //bool loadMatrix(std::string strFile);
  //void setImages(std::string strImgL, std::string strImgR, bool bGrey = true);
  //bool loadTP(const std::string strFile);
  bool saveTP(const std::vector<CTiePt>& vecTPs, const std::string strFile);
  bool saveMatrix(const Mat& matData, const std::string strFile);  // assume matrix is in float or double data type
  bool saveALSCParam(const CALSCParam& paramALSC, const std::string strOut);
  bool saveGOTCHAParam(CGOTCHAParam& paramGOTCHA, const std::string strOut);

protected:
  //cv::Mat m_dispX;
};

#endif // CPROCBLOCK_H
