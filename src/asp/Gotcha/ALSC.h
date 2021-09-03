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

#ifndef ASP_GOTCHA_ALSC_H
#define ASP_GOTCHA_ALSC_H

#include <asp/Gotcha/CALSCParam.h>
#include <asp/Gotcha/CTiePt.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>

namespace gotcha {

class ALSC {
public:
    ALSC();
    ALSC(cv::Mat imgL, cv::Mat imgR, CALSCParam paramALSC);

    void performALSC(const std::vector<CTiePt>* pvecTpts , const float* pfAffStart = NULL);   // Nb. it is normally used for TC refinement (i.e., verification)
                                                                                         // pfAffStart might be needed if ALSC resumes from the previous TP results
    void getRefinedTps(std::vector<CTiePt>& vecRefTP) const {vecRefTP = m_pvecRefTP;}
    const std::vector<CTiePt>* getRefinedTps() {return &m_pvecRefTP;} // the result which passes the ALSC test
    std::vector<int> getPassList() {return m_vecPassList;}

    enum{NO_ERR, OB_ERR};

private:
    bool isIntersecting(cv::Rect rectA, cv::Rect rectB);
    void getGradientX(Eigen::Ref<Eigen::MatrixXf> matSrc);
    void getGradientY(Eigen::Ref<Eigen::MatrixXf> matSrc);
    void distortPatch(const cv::Mat& matImg, const cv::Point2f ptCentre, const float* pfAff, Eigen::Ref<Eigen::MatrixXf> matImgPatch, cv::Point2f* pptUpdated = NULL);
    bool doMatching(cv::Point2f ptStartL, cv::Point2f ptStartR, CTiePt& tp, const float* pfAffInt = NULL);
    void affineTransform(double x, double y, const cv::Point2f ptCentre, const float *pfAff, double *dNewX, double *dNewY);
    float interpolate(double dNewX, double dNewY, const cv::Mat &matImg);

private:
    // inputs
    cv::Mat m_imgL;
    cv::Mat m_imgR;
    CALSCParam m_paramALSC;
    cv::Mat m_imgR_resize;

    int nMaxIter;
    int nPatchRadius;
    float fDisplacementThr;
    float fAffineThr;
    float fEigenThr;
    bool bNeedW;

    int nRowPatch;
    int nColPatch;
    int nSystemMatrixRows;

    std::vector<std::vector<float> > pfGx;
    std::vector<std::vector<float> > pfGy;

    Eigen::MatrixXf matPatchL;
    Eigen::MatrixXf matPatchR;

    Eigen::Matrix2f matC;

    Eigen::MatrixXf emA;
    Eigen::VectorXf emB;

    int nParam;

    // outputs:
    std::vector<CTiePt> m_pvecRefTP;
    std::vector<int> m_vecPassList; // index list which passes ALSC test
};

class ALSCU16 : public ALSC
{
    float interpolate(double dNewX, double dNewY, const cv::Mat &matImg);
};

class ALSCF32 : public ALSC
{
    float interpolate(double dNewX, double dNewY, const cv::Mat &matImg);
};

} // end namespace gotcha

#endif // ASP_GOTCHA_ALSC_H
