#ifndef ALSC_H
#define ALSC_H

#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "CALSCParam.h"
#include "CTiePt.h"

using namespace cv;
using namespace std;

class ALSC
{
public:
    ALSC();
    ALSC(Mat imgL, Mat imgR, CALSCParam paramALSC);

    void performALSC(const vector<CTiePt>* pvecTpts , const float* pfAffStart = NULL);   // Nb. it is normally used for TC refinement (i.e., verification)
                                                                                         // pfAffStart might be needed if ALSC resumes from the previous TP results
    void getRefinedTps(vector<CTiePt>& vecRefTP) const {vecRefTP = m_pvecRefTP;}
    const vector<CTiePt>* getRefinedTps() {return &m_pvecRefTP;} // the result which passes the ALSC test
    vector<int> getPassList() {return m_vecPassList;}

    enum{NO_ERR, OB_ERR};

private:
    bool isIntersecting(Rect rectA, Rect rectB);
    void getGradientX(Eigen::Ref<Eigen::MatrixXf> matSrc);
    void getGradientY(Eigen::Ref<Eigen::MatrixXf> matSrc);
    void distortPatch(const Mat& matImg, const Point2f ptCentre, const float* pfAff, Eigen::Ref<Eigen::MatrixXf> matImgPatch, Point2f* pptUpdated = NULL);
    bool doMatching(Point2f ptStartL, Point2f ptStartR, CTiePt& tp, const float* pfAffInt = NULL);
    void affineTransform(double x, double y, const Point2f ptCentre, const float *pfAff, double *dNewX, double *dNewY);
    float interpolate(double dNewX, double dNewY, const Mat &matImg);

private:
    // inputs
    Mat m_imgL;
    Mat m_imgR;
    CALSCParam m_paramALSC;
    Mat m_imgR_resize;

    int nMaxIter;
    int nPatchRadius;
    float fDisplacementThr;
    float fAffineThr;
    float fEigenThr;
    bool bNeedW;

    int nRowPatch;
    int nColPatch;
    int nSystemMatrixRows;

    std::vector< std::vector<float> > pfGx;
    std::vector< std::vector<float> > pfGy;

    Eigen::MatrixXf matPatchL;
    Eigen::MatrixXf matPatchR;

    Eigen::Matrix2f matC;

    Eigen::MatrixXf emA;
    Eigen::VectorXf emB;

    int nParam;

    // outputs:
    vector<CTiePt> m_pvecRefTP;
    vector<int> m_vecPassList; // index list which passes ALSC test
};

class ALSCU16 : public ALSC
{
    float interpolate(double dNewX, double dNewY, const Mat &matImg);
};

class ALSCF32 : public ALSC
{
    float interpolate(double dNewX, double dNewY, const Mat &matImg);
};
#endif // ALSC_H
