#include "ALSC.h"

using namespace std;

ALSC::ALSC()
{
}

ALSC::ALSC(Mat imgL, Mat imgR, CALSCParam paramALSC){
    m_imgL = imgL;  // soft data copy
    m_imgR = imgR;

    m_paramALSC = paramALSC; // hard copy

    nMaxIter = m_paramALSC.m_nMaxIter;
    nPatchRadius = m_paramALSC.m_nPatch;
    fDisplacementThr = m_paramALSC.m_fDriftThr;
    fAffineThr = m_paramALSC.m_fAffThr;
    fEigenThr = m_paramALSC.m_fEigThr;
    bNeedW = m_paramALSC.m_bWeighting;

    nParam = 0;
    if (m_paramALSC.m_bIntOffset) nParam = 7;
    else nParam = 6;

    nRowPatch = 2 * nPatchRadius + 1;
    nColPatch = 2 * nPatchRadius + 1;

    matPatchL = Eigen::MatrixXf(nRowPatch,nColPatch);
    matPatchR = Eigen::MatrixXf(nRowPatch,nColPatch);
    matC = Eigen::Matrix2f();

    // prepare buffers for a normal equation, Ax = B
    nSystemMatrixRows = nRowPatch*nColPatch; //4 * nRadius * nRadius + 1 + 4 * nRadius;
    emA = Eigen::MatrixXf(nSystemMatrixRows,nParam);
    emB = Eigen::VectorXf(nSystemMatrixRows);

    pfGx.resize(nRowPatch);
    for (int i = 0; i < nRowPatch; i++) {
        pfGx[i].resize(nColPatch);
    }

    pfGy.resize(nRowPatch);
    for (int i = 0; i < nRowPatch; i++) {
        pfGy[i].resize(nColPatch);
    }

}

bool ALSC::isIntersecting(Rect rectA, Rect rectB){
    bool bRes = false;

    Rect rectInter;
    rectInter = rectA & rectB;
    if (rectInter.area() > 0) bRes = true;
    else bRes = false;

    return bRes;
}

bool ALSC::doMatching(Point2f ptStartL, Point2f ptStartR, CTiePt& tp, const float* pfAffStart){
    /////////////////////////////////
    // set ALSC processing parameters
    /////////////////////////////////

    //QElapsedTimer timer;
    //timer.restart();

    // define the size of the input images
    Rect_<float> rectL = Rect(0, 0, m_imgL.cols, m_imgL.rows);
    Rect_<float> rectR = Rect(0, 0, m_imgR.cols, m_imgR.rows); // But I presume they (i.e., imgage L and R) are the same size

    // define the size of an initial patch, i.e., (2*nRadius+1) by (2*nRadius+1)
    Rect_<float> rectPatchL(ptStartL.x - nPatchRadius, ptStartL.y - nPatchRadius, nColPatch, nRowPatch);
    Rect_<float> rectPatchR(ptStartR.x - nPatchRadius, ptStartR.y - nPatchRadius, nColPatch, nRowPatch);

    // checking the validity of the size of an initial patch
    if (!rectL.contains(ptStartL) || !rectR.contains(ptStartR) ||
        !isIntersecting(rectL, rectPatchL) || !isIntersecting(rectR, rectPatchR))
        return false; // don't stop processing as thre might be more tiepoints in the queue

    // set an initial affine transform
    float pfAffine[4] = {0, 0, 0, 0}; // these are actually parameters for dA not A

    distortPatch(m_imgL, ptStartL, (float*) pfAffine, matPatchL); // simple image cropping in this case

    if (pfAffStart != NULL){
        pfAffine[0] = pfAffStart[0];
        pfAffine[1] = pfAffStart[1];
        pfAffine[2] = pfAffStart[2];
        pfAffine[3] = pfAffStart[3];
        ptStartR.x += pfAffStart[4];
        ptStartR.y += pfAffStart[5];
    }

    distortPatch(m_imgR, ptStartR, (float*) pfAffine, matPatchR);

    Point2f ptOffset(0.f, 0.f);

    ///////////////////////////////////////////////////////////////////
    // initilise temporary variables for ALSC
    ///////////////////////////////////////////////////////////////////
    // initilise updated right boundary
    Point2f pptUpdatedBoundary[4];
    pptUpdatedBoundary[0] = Point2f(rectPatchR.x, rectPatchR.y);
    pptUpdatedBoundary[1] = Point2f(rectPatchR.x, rectPatchR.y+rectPatchR.height);
    pptUpdatedBoundary[2] = Point2f(rectPatchR.x+rectPatchR.width, rectPatchR.y+rectPatchR.height);
    pptUpdatedBoundary[3] = Point2f(rectPatchR.x+rectPatchR.width, rectPatchR.y);
    Point2f ptUpdatedR(ptStartR);

    bool bNeed2Stop = false;
    double dEigenVal = 1E20; //Double.MAX_VALUE;// 0.d;

    float fIntOffOld = 0.f;  //intensity offset only used when m_paramALSC.m_bIntOffset
    float fIntOffNew = 0.f;

    Point2f ptBestR(0,0);
    double dBestEig = 1E20;
    float pfAffBest[4]={0,0,0,0};

    ///////////////////////////////////////////////////////////////////
    // starting ALSC operation for a single tiepoint
    ///////////////////////////////////////////////////////////////////

    for (int j = 0; j < nMaxIter; j++) {
        getGradientX(matPatchR);
        getGradientY(matPatchR);

        // make a system matrix A for LMS
        int count = 0;
        for (int y = 0; y < nRowPatch; y++) {

            int yOffset = y - nPatchRadius;

            for (int x = 0; x < nColPatch; x++) {

                int xOffset = x - nPatchRadius;

                float fVal = pfGx[y][x];
                emA(count,0) = fVal;
                emA(count,1) = fVal * xOffset;
                emA(count,2) = fVal * yOffset;

                fVal = pfGy[y][x];
                emA(count,3) = fVal;
                emA(count,4) = fVal * xOffset;
                emA(count,5) = fVal * yOffset;

                if (nParam == 7) {
                    emA(count,6)  = 1;
                }

                emB(count) = matPatchL(y, x) - matPatchR(y, x);
                count++;
            }
        }

        // get LMS solution
        Eigen::VectorXf emS(nParam);
        Eigen::MatrixXf emAS(nParam,nParam);

        /* Don't explicitly calculate the inverse!  Use Cholesky decomposition instead. */
        Eigen::MatrixXf emAT = emA.transpose();
        emAS = (emAT*emA);
        emS = emAS.llt().solve(emAT*emB);

        if (m_paramALSC.m_bIntOffset)
            fIntOffNew = emS(6);

        // error computation
        Eigen::VectorXf emErrors(nSystemMatrixRows);
        emErrors = (emA * emS) - emB;

        // Compute the standard deviation of residual errors
        double dTotElelement = nSystemMatrixRows; //nRowPatch *nColPatch; //2 * nRadius + 1; //dTotElelement *= dTotElelement;

        double dErrorSum = emErrors.squaredNorm();
        double dSTDResidual = dErrorSum / (dTotElelement - nParam);

        // rms of residual error
        double dResidual = 0.f;
        dResidual = sqrt(dErrorSum / (double) nSystemMatrixRows);

        // maximum eigenvalue of shift covariance matrix
        float a = emAS(0, 0);
        float b = emAS(3, 0);
        float d = emAS(3, 3);

        /*
        float pdC[2][2] = {{a, b}, {b, d}};
        Mat matD(2, 2, CV_32FC1, pdC);
        SVD svdb(matD.inv());
        */

        matC(0,0) = d;
        matC(0,1) = -b;
        matC(1,0) = -b;
        matC(1,1) = a;

        matC *= (1.0/(a*d-b*b));

        Eigen::JacobiSVD<Eigen::Matrix2f> svd(matC);
        //cout << svd.singularValues()(0) << " " << svdb.w.at<float>(0) << endl;
        //assert(svd.singularValues()(0) == svdb.w.at<float>(0));

        dEigenVal = svd.singularValues()(0);
        //dEigenVal = svdb.w.at<float>(0,0); // maximum eigenvalues
        dEigenVal *= 10000.0f * dSTDResidual; // scaling

        //////////////////////////////////////////////////////////////
        // check the validity of solution
        //////////////////////////////////////////////////////////////
        float x = emS(0);
        float y = emS(3);
        float dist = sqrt(x * x + y * y);

        if (dist > fDisplacementThr)
            bNeed2Stop = true;
        if (abs(emS(1)) > fAffineThr || std::isnan(emS(1)))
            bNeed2Stop = true;
        if (abs(emS(2)) > fAffineThr || std::isnan(emS(2)))
            bNeed2Stop = true;
        if (abs(emS(4)) > fAffineThr || std::isnan(emS(4)))
            bNeed2Stop = true;
        if (abs(emS(5)) > fAffineThr || std::isnan(emS(5)))
            bNeed2Stop = true;
        if (bNeed2Stop){
            break;
        }

        //////////////////////////////////////////////////////////////
        // update parameters
        //////////////////////////////////////////////////////////////
        pfAffine[0] = emS(1);
        pfAffine[1] = emS(2);
        pfAffine[2] = emS(4);
        pfAffine[3] = emS(5);

        ptOffset.x = x;
        ptOffset.y = y;

        ptUpdatedR.x += ptOffset.x;
        ptUpdatedR.y += ptOffset.y;

        if (dEigenVal < fEigenThr){
            if (dBestEig > dEigenVal){
                dBestEig = dEigenVal;
                ptBestR = ptUpdatedR;
                for (int m = 0; m < 4; m++)
                    pfAffBest[m] = pfAffine[m];
            }
        }

        // get distorted ROI and patch
        distortPatch(m_imgR, ptUpdatedR, (float*)pfAffine, matPatchR, (Point2f*)pptUpdatedBoundary);

        fIntOffOld = fIntOffNew;

    } // the end of single iteration of a ALSC process

    ///////////////////////////////////////////////////////////////////
    // the end of single ALSC operation on a tiepoint
    ///////////////////////////////////////////////////////////////////
    if (dBestEig > fEigenThr && !bNeed2Stop) bNeed2Stop = true;
    if (std::isnan(dEigenVal)) bNeed2Stop = true;

    // collect results
    if (!bNeed2Stop){
        // CRefinedTP tp;
        tp.m_ptL = ptStartL;
        tp.m_ptR = ptBestR; //ptUpdatedR;
//        tp.m_fSimVal = dEigenVal/m_paramALSC.m_fEigThr; // normlise score, the smaller is the better
        tp.m_fSimVal = dBestEig; //dEigenVal;
        for (int i = 0; i < 4; i++)
            tp.m_pfAffine[i] = pfAffBest[i]; //pfAffine[i];
        tp.m_ptOffset = ptBestR - ptStartR;  //ptUpdatedR - ptStartR;
        if (pfAffStart != NULL){
            tp.m_ptOffset.x += pfAffStart[4];
            tp.m_ptOffset.y += pfAffStart[5];
        }
        //m_pvecRefTP.push_back(tp);
    }

    return (!bNeed2Stop);
}

void ALSC::getGradientX(Eigen::Ref<Eigen::MatrixXf> matSrc) {

    int nW = matSrc.cols();
    int nH = matSrc.rows();

    for (int y = 1; y < nH - 1; y++) {
        for (int x = 1; x < nW - 1; x++) {
            pfGx.at(y).at(x) = matSrc(y, x+1) - matSrc(y, x);
        }
    }

    return;
}

void ALSC::getGradientY(Eigen::Ref<Eigen::MatrixXf> matSrc) {

    int nW = matSrc.cols();
    int nH = matSrc.rows();

    for (int y = 1; y < nH - 1; y++) {
        for (int x = 1; x < nW - 1; x++) {
            pfGy.at(y).at(x) = matSrc(y+1, x) - matSrc(y, x);
        }
    }

    return;
}

void ALSC::affineTransform(double x, double y, const Point2f ptCentre, const float *pfAff, double *dNewX, double *dNewY){
    double dOffsetX, dOffsetY;

    dOffsetX = x * pfAff[0] + y * pfAff[1];
    dOffsetY = x * pfAff[2] + y * pfAff[3];

    *dNewX = ptCentre.x + x + dOffsetX;
    *dNewY = ptCentre.y + y + dOffsetY;

    return;
}

void ALSC::distortPatch(const Mat& matImg, const Point2f ptCentre, const float* pfAff, Eigen::Ref<Eigen::MatrixXf> matImgPatch, Point2f* pptUpdated) {

    double dNewX = 0, dNewY = 0;

    int nW = matImgPatch.cols();
    int nH = matImgPatch.rows();
    int i = 0, j = 0;

    int initX = -nW / 2;
    int initY = -nH / 2;

    if(pptUpdated != NULL){
        pptUpdated[0] = Point2f(0,0);
        pptUpdated[1] = Point2f(0,0);
        pptUpdated[2] = Point2f(0,0);
        pptUpdated[3] = Point2f(0,0);
    }

    /* Case when we're just cropping an image, don't waste time doing interpolation */
    if(pfAff[0] == 0 && pfAff[1] == 0 && pfAff[2] == 0 && pfAff[3] == 0){
        for (j = 0; j < nH; j++) {
                for (i = 0; i < nW; i++) {
                    dNewX = ptCentre.x + initX + i;
                    dNewY = ptCentre.y + initY + j;

                    if(dNewX < 0 || dNewY < 0 || dNewX > matImg.cols || dNewY > matImg.rows)
                        matImgPatch(j,i) = 0.0;
                    else{
                        matImgPatch(j,i) = matImg.at<unsigned char>(dNewY, dNewX);
                    }
                }
        }

        if(pptUpdated != NULL){
            pptUpdated[0] = Point2f(ptCentre.x + initX, ptCentre.y + initY);
            pptUpdated[1] = Point2f(ptCentre.x + initX, ptCentre.y + initY + nH-1);
            pptUpdated[2] = Point2f(ptCentre.x + initX + nW-1, 0);
            pptUpdated[3] = Point2f(ptCentre.x + initX + nW-1, ptCentre.y + initY + nH-1);
        }
    }else{
        /* Otherwise interpolate */
        for (j = 0; j < nH; j++) {
                for (i = 0; i < nW; i++) {
                    /* Perform the affine transform on the points */
                    affineTransform(initX+i, initY+j, ptCentre, pfAff, &dNewX, &dNewY);

                    /* Interpolate from the image */
                    matImgPatch(j,i) = interpolate(dNewX, dNewY, matImg);

                    /* Check if we're at one of the patch corners and store into the boundary array if needed */
                    if(i == 0){
                        if(j == 0){
                            if (pptUpdated != NULL) pptUpdated[0] = Point2f(dNewX, dNewY);
                        }else if(j == nH-1){
                            if (pptUpdated != NULL) pptUpdated[1] = Point2f(dNewX, dNewY);
                        }
                    }else if(i == nW-1 && j == 0){
                        if (pptUpdated != NULL) pptUpdated[3] = Point2f(dNewX, dNewY);
                    }
                }
            }
            if (pptUpdated != NULL) pptUpdated[2] = Point2f(dNewX, dNewY);
    }

    return;
}

float ALSC::interpolate(double dNewX, double dNewY, const Mat &matImg){
    int x1, x2, y2, y1;
    float val1, val2, val3, val4;
    float fPixelVal = 0.0;
    double x,y;

    val1 = val2 = val3 = val4 = 0.0;

    x1 = (int) floor(dNewX);
    x2 = (int) ceil(dNewX);
    y2 = (int) ceil(dNewY);
    y1 = (int) floor(dNewY);

    if(x1 < 0 || y1 < 0 || x2 >= matImg.cols || y2 >= matImg.rows)
        return 0.0;

    if (x1 == x2 && y1 == y2){ // when dNewX and dNewY are both integer -> happy case no interpolation is required
        fPixelVal = matImg.at<unsigned char>(y1, x1);
    }
    else if (x1 == x2 && y1 != y2){ // dNewX is integer but dNewY is not. 1D interpolation is required (not bidirectional interpolation)
        val1 = matImg.at<unsigned char>(y1, x1);
        val2 = matImg.at<unsigned char>(y2, x1);

        fPixelVal = (val2 - val1) * (dNewY - y1) + val1;
    }
    else if (x1 != x2 && y1 == y2){
        val1 = matImg.at<unsigned char>(y1, x1);
        val2 = matImg.at<unsigned char>(y1, x2);

        fPixelVal = (val2 - val1) * (dNewX - x1) + val1;
    }
    else{ // bidirectional interpolation
            x = dNewX;
            y = dNewY;

            val1 = matImg.at<unsigned char>(y1, x1);
            val2 = matImg.at<unsigned char>(y1, x2);
            val3 = matImg.at<unsigned char>(y2, x1);
            val4 = matImg.at<unsigned char>(y2, x2);

            fPixelVal = (val1 * (x2 - x) * (y2 - y)
                         + val2 * (x - x1) * (y2 - y)
                         + val3 * (x2 - x) * (y - y1)
                         + val4 * (x - x1) * (y - y1));
    }

    return fPixelVal;
}

float ALSCU16::interpolate(double dNewX, double dNewY, const Mat &matImg){
    int x1, x2, y2, y1;
    float val1, val2, val3, val4;
    float fPixelVal = 0.0;
    double x,y;

    val1 = val2 = val3 = val4 = 0.0;

    x1 = (int) floor(dNewX);
    x2 = (int) ceil(dNewX);
    y2 = (int) ceil(dNewY);
    y1 = (int) floor(dNewY);

    if(x1 < 0 || y1 < 0 || x2 >= matImg.cols || y2 >= matImg.rows)
        return 0.0;

    if (x1 == x2 && y1 == y2){ // when dNewX and dNewY are both integer -> happy case no interpolation is required
        fPixelVal = matImg.at<unsigned short>(y1, x1);
    }
    else if (x1 == x2 && y1 != y2){ // dNewX is integer but dNewY is not. 1D interpolation is required (not bidirectional interpolation)
        val1 = matImg.at<unsigned short>(y1, x1);
        val2 = matImg.at<unsigned short>(y2, x1);

        fPixelVal = (val2 - val1) * (dNewY - y1) / (y2 - y1) + val1;
    }
    else if (x1 != x2 && y1 == y2){
        val1 = matImg.at<unsigned short>(y1, x1);
        val2 = matImg.at<unsigned short>(y1, x2);

        fPixelVal = (val2 - val1) * (dNewX - x1) / (x2 - x1) + val1;
    }
    else{ // bidirectional interpolation
        x = dNewX;
        y = dNewY;

        val1 = matImg.at<unsigned short>(y1, x1);
        val2 = matImg.at<unsigned short>(y1, x2);
        val3 = matImg.at<unsigned short>(y2, x1);
        val4 = matImg.at<unsigned short>(y2, x2);

        fPixelVal = (val1 * (x2 - x) * (y2 - y) / (x2 - x1) / (y2 - y1)
                     + val2 * (x - x1) * (y2 - y) / (x2 - x1) / (y2 - y1)
                     + val3 * (x2 - x) * (y - y1) / (x2 - x1) / (y2 - y1)
                     + val4 * (x - x1) * (y - y1) / (x2 - x1) / (y2 - y1));
    }

    return fPixelVal;
}

float ALSCF32::interpolate(double dNewX, double dNewY, const Mat &matImg){
    int x1, x2, y2, y1;
    float val1, val2, val3, val4;
    float fPixelVal = 0.0;
    double x,y;

    val1 = val2 = val3 = val4 = 0.0;

    x1 = (int) floor(dNewX);
    x2 = (int) ceil(dNewX);
    y2 = (int) ceil(dNewY);
    y1 = (int) floor(dNewY);

    if(x1 < 0 || y1 < 0 || x2 >= matImg.cols || y2 >= matImg.rows)
        return 0.0;

    if (x1 == x2 && y1 == y2){ // when dNewX and dNewY are both integer -> happy case no interpolation is required
        fPixelVal = matImg.at<float>(y1, x1);
    }
    else if (x1 == x2 && y1 != y2){ // dNewX is integer but dNewY is not. 1D interpolation is required (not bidirectional interpolation)
        val1 = matImg.at<float>(y1, x1);
        val2 = matImg.at<float>(y2, x1);

        fPixelVal = (val2 - val1) * (dNewY - y1) / (y2 - y1) + val1;
    }
    else if (x1 != x2 && y1 == y2){
        val1 = matImg.at<float>(y1, x1);
        val2 = matImg.at<float>(y1, x2);

        fPixelVal = (val2 - val1) * (dNewX - x1) / (x2 - x1) + val1;
    }
    else{ // bidirectional interpolation
        x = dNewX;
        y = dNewY;

        val1 = matImg.at<float>(y1, x1);
        val2 = matImg.at<float>(y1, x2);
        val3 = matImg.at<float>(y2, x1);
        val4 = matImg.at<float>(y2, x2);

        fPixelVal = (val1 * (x2 - x) * (y2 - y) / (x2 - x1) / (y2 - y1)
                     + val2 * (x - x1) * (y2 - y) / (x2 - x1) / (y2 - y1)
                     + val3 * (x2 - x) * (y - y1) / (x2 - x1) / (y2 - y1)
                     + val4 * (x - x1) * (y - y1) / (x2 - x1) / (y2 - y1));
    }

    return fPixelVal;
}

// this function for the feature refinement
void ALSC::performALSC(const vector<CTiePt> *pvecTpts, const float* pfAffStart){

    //////////////////////////////////////////////////
    // Start processing for each seed point
    //////////////////////////////////////////////////
    int nLength = pvecTpts->size();
    m_pvecRefTP.clear();   // clear result buffer
    m_vecPassList.clear(); // clear pass index
    for (int i = 0; i < nLength; i++) {
        // get a seed point
        Point2f ptStartL, ptStartR;
        ptStartL = pvecTpts->at(i).m_ptL;
        ptStartR = pvecTpts->at(i).m_ptR;

        CTiePt tp;

        // If the point is matched, update the result list
        if(doMatching (ptStartL, ptStartR, tp, pfAffStart)){
            m_pvecRefTP.push_back(tp); // collect results
            m_vecPassList.push_back(i);
        }

    } // end of operation for all seed points
}

void ALSC::saveMat(Mat& matIn, string strFile){
    ofstream sfTC;
    sfTC.open(strFile.c_str());

    if (sfTC.is_open()){
        for (int i = 0; i < matIn.rows; i++){
            for (int j = 0 ; j < matIn.cols; j++){
                sfTC << matIn.at<double>(i,j) << " ";
            }
            sfTC << endl;
        }
        sfTC.close();
    }
}
