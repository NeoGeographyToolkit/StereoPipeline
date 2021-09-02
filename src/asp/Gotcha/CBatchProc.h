#ifndef CBATCHPROC_H
#define CBATCHPROC_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vw/Image/ImageView.h>

#include <CTiePt.h>

class CBatchProc {
public:
//    CBatchProc();
  CBatchProc(std::string strMetaFile,
             std::string strLeftImagePath, std::string strRightImagePath,
             vw::ImageView<float> input_dispX, vw::ImageView<float> input_dispY, 
             std::string strOutputPrefix);
    ~CBatchProc();


    void doBatchProcessing();


private:
    void setProjParameter();
    bool validateProjParam();
    bool validateProjInputs();
    void generateMask();
    void generateTPFile(std::vector<CTiePt> & vecTPs);

    Point3f rotate(Point3f ptIn, cv::Mat matQ, bool bInverse);
    void quaternionMultiplication(const float* p, const float* q, float* pfOut);

protected:
    // processing
    void refinement(std::vector<CTiePt> const& vecTPs);

protected:
    std::string m_strMetaFile;   // file path to the Metadata file
    std::string m_strImgL;
    std::string m_strImgR;
    //std::string m_strDispX;
    //std::string m_strDispY;
    //string m_strMask;
    //std::string m_strTPFile;
    std::string m_strOutPath;    // a user-supplied file path for the output directory

  cv::Mat m_Mask;
  cv::Mat m_input_dispX, m_input_dispY;
};

#endif // CBATCHPROC_H
