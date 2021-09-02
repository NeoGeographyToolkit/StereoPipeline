#ifndef CBATCHPROC_H
#define CBATCHPROC_H

#include <iostream>
#include <fstream>
//#include "opencv/cv.h"
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

class CBatchProc
{
public:
//    CBatchProc();
    CBatchProc(string strMetaFile, string strLeftImagePath, string strRightImagePath, string strDisparityX, string strDisparityY, string strOutputPrefix);
    ~CBatchProc();


    void doBatchProcessing();


private:
    void setProjParameter();
    bool validateProjParam();
    bool validateProjInputs();
    void generateMask();
    void generateTPFile();

    Point3f rotate(Point3f ptIn, Mat matQ, bool bInverse);
    void quaternionMultiplication(const float* p, const float* q, float* pfOut);

protected:
    // processing
    void refinement();

protected:
    string m_strMetaFile;   // file path to the Metadata file
    string m_strImgL;
    string m_strImgR;
    string m_strDispX;
    string m_strDispY;

    //string m_strMask;
    string m_strTPFile;
    string m_strOutPath;    // a user-supplied file path for the output directory

  cv::Mat m_Mask;
};

#endif // CBATCHPROC_H
