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

using namespace std;
using namespace cv;

class CProcBlock
{
public:
    CProcBlock();
    int getNumTps() const {int nRes = m_vecTPs.size();
                           if ( nRes > 0 ) return nRes;
                           else return m_vecTPs.size();}

    static bool loadMatrix(Mat& matData, const string strFile);        // assume matrix has float data type

protected:
    // file I/O
    bool loadMatrix(Mat &matData, const string strFile, bool bDoublePrecision);
    void setImages(string strImgL, string strImgR, bool bGrey = true);
    bool loadMatrix(string strFile);
    bool loadTP(const string strFile);
    bool loadTP(const string strFile, const int* pnIndx, const int nSzIdx);
    bool saveTP(const vector<CTiePt>& vecTPs, const string strFile);
    bool saveMatrix(const Mat& matData, const string strFile);  // assume matrix is in float or double data type
    bool saveALSCParam(const CALSCParam& paramALSC, const string strOut);
    bool saveGOTCHAParam(CGOTCHAParam& paramGOTCHA, const string strOut);


protected:
    vector<CTiePt> m_vecTPs;
    Mat m_imgL;
    Mat m_imgR;
    Mat m_dispX;

};

#endif // CPROCBLOCK_H
