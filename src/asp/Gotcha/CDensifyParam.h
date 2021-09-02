#ifndef CDENSIFYPARAM_H
#define CDENSIFYPARAM_H

#include <string>
#include <opencv2/opencv.hpp>

#include "CGOTCHAParam.h"

class CDensifyParam {

public:
  CDensifyParam(): m_nProcType(GOTCHA){}
  int m_nProcType;     // growing method
  //std::string m_strImgL;
  //std::string m_strImgR;
  std::string m_strOutPath;
  //std::string m_strTPFile;
  //std::string m_strDispX;
  //std::string m_strDispY;
  cv::Mat m_Mask;
  std::string m_strUpdatedDispX;
  std::string m_strUpdatedDispY;
  std::string m_strUpdatedDispSim;
  
  std::string getProcessingType(){
    if (m_nProcType == GOTCHA) return "GOTCHA";
    else if (m_nProcType == P_GOTCHA) return "P_GOTCHA";
    return "UNKNOWN";
  }
  
  CGOTCHAParam m_paramGotcha;
  
  enum{GOTCHA, P_GOTCHA};
  enum{NO_ERR, FILE_IO_ERR, GOTCHA_ERR, LINE_GOTCHA_ERR, P_GOTCHA_ERR};
};
#endif // CDENSIFYPARAM_H
