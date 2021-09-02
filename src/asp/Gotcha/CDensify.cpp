#include <fstream>
#include <iostream>
#include <math.h>
#include "CDensify.h"
#include "ALSC.h"

using namespace std;

CDensify::CDensify()
{
}

CDensify::CDensify(CDensifyParam paramDense, std::vector<CTiePt> const& vecTPs,
                   cv::Mat input_dispX, cv::Mat input_dispY){
  setParameters(paramDense, vecTPs, input_dispX, input_dispY);
}

void CDensify::setParameters(CDensifyParam paramDense, std::vector<CTiePt> const& vecTPs,
                             cv::Mat input_dispX, cv::Mat input_dispY){
  m_paramDense = paramDense;
  m_vecTPs = vecTPs;
  m_input_dispX = input_dispX;
  m_input_dispY = input_dispY;
  
  loadImages();
//    m_nCount = 0;
//    cout << m_paramDense.m_paramGotcha.m_paramALSC.m_fEigThr<< endl;
}


void CDensify::loadImages(){
    string strImgL, strImgR;
    // get input images
    strImgL = m_paramDense.m_strImgL;
    strImgR = m_paramDense.m_strImgR;
    setImages(strImgL, strImgR);

}


bool CDensify::loadTPForDensification(string strTPFile){    
    bool bRes = true;
    bRes = loadTP(strTPFile);
    return bRes;
}

int CDensify::performDensitification(){

    ///////////////////////////////////////////
    // validate the input images
    ///////////////////////////////////////////
    if ( m_imgL.data == NULL || m_imgR.data == NULL)
        return CDensifyParam::FILE_IO_ERR;

#if 0
    // TP are now kept in memory the whole time, so no need to load them
    if (!loadTPForDensification(m_paramDense.m_strTPFile))
        return CDensifyParam::FILE_IO_ERR;
#endif
    
    if(m_paramDense.m_nProcType == CDensifyParam::GOTCHA){
        double start = getTickCount();
        // Generate integer-float seed points (nb. feature detection only gives float-float seed points)
        // Also, this is necessary to keep the original seed data, as initial ALSC will remove some seeds if selected.

        //Function below run initial ALSC on disparity based TPs. If use, need to change:
        //1. doGotcha(m_imgL, m_imgR, vecSeedTemp, m_paramDense.m_paramGotcha, m_vectpAdded)
        //2. m_vectpAdded.insert(m_vectpAdded.end(), vecSeedTemp.begin(), vecSeedTemp.end());
        //3. need some change in doPGotcha
        //IMARS still need initial ALSC
        vector<CTiePt> vecSeedTemp = getIntToFloatSeed(m_vecTPs);
        m_vecTPs.clear();
        for (int j = 0; j < (int)vecSeedTemp.size(); j++){
            CTiePt tpTemp = vecSeedTemp.at(j);
            m_vecTPs.push_back(tpTemp);
        }
        //IMARS still need initial ALSC



        cout << "CASP-GO INFO: starting Gotcha" << endl;
        if (!doGotcha(m_imgL, m_imgR, m_vecTPs, m_paramDense.m_paramGotcha, m_vectpAdded))
            return CDensifyParam::GOTCHA_ERR;
        // remove TP with large y disparity
        // nb. sometimes large y disparity can be produced even from a rectified pair

        double dYLimit = 100;
        vector<CTiePt>::iterator iter;
        for (iter = m_vectpAdded.begin(); iter < m_vectpAdded.end(); ){
            double dy = iter->m_ptL.y - iter->m_ptR.y;
            dy = sqrt(dy*dy);
            if (dy > dYLimit)
                m_vectpAdded.erase(iter);
            else
                iter++;
        }

        m_vectpAdded.insert(m_vectpAdded.end(), m_vecTPs.begin(), m_vecTPs.end());

        double end = getTickCount();
        procTime = (end - start)/getTickFrequency();

        cout << "CASP-GO INFO: making data products" << endl;

        makeDataProducts();

        if (!saveResult())
            return CDensifyParam::FILE_IO_ERR;

        // Do not save the log, this is hard to manage with multiple instances
        // running in parallel.
        //if (!saveLog())
        //    return CDensifyParam::FILE_IO_ERR;

    }
    else if (m_paramDense.m_nProcType == CDensifyParam::P_GOTCHA){
        double start = getTickCount();
        if (!doPGotcha(m_paramDense.m_paramGotcha.m_nNeiType))
            return CDensifyParam::P_GOTCHA_ERR;
        // remove TP with large y disparity

        double dYLimit = 100; //IMARS
        vector<CTiePt>::iterator iter;
        for (iter = m_vectpAdded.begin(); iter < m_vectpAdded.end(); ){
            double dy = iter->m_ptL.y - iter->m_ptR.y;
            dy = sqrt(dy*dy);
            if (dy > dYLimit)
                m_vectpAdded.erase(iter);
            else
                iter++;
        }

        m_vectpAdded.insert(m_vectpAdded.end(), m_vecTPs.begin(), m_vecTPs.end());

        double end = getTickCount();
        procTime = (end - start)/getTickFrequency();
        makeDataProducts();

        if (!saveResult())
            return CDensifyParam::FILE_IO_ERR;

        // Do not save the log, this is hard to manage with multiple instances
        // running in parallel.
        //if (!saveLog())
        //    return CDensifyParam::FILE_IO_ERR;
    }

    return CDensifyParam::NO_ERR;
}

vector<CTiePt> CDensify::getIntToFloatSeed(vector<CTiePt>& vecTPSrc) {
    vector<CTiePt> vecRes;

    int nLen =  vecTPSrc.size();
    for (int i = 0; i < nLen; i++){
        //get four neighbours
        CTiePt tp = vecTPSrc.at(i);
        Point2f ptL = tp.m_ptL;
        Point2f ptR = tp.m_ptR;
        float dX = 0, dY = 0; // xy offset to make integer seed

        dX = floor(ptL.x) - ptL.x;
        dY = floor(ptL.y) - ptL.y;
        if (dX == 0 && dY == 0){
            vecRes.push_back(tp);
            continue;
        }

        Point2f ptDelta(dX,dY);
        Point2f ptIntL = ptL + ptDelta; // ptIntL may be in float if ptL is came from the Rectified coordinate
        Point2f ptIntR = ptR + ptDelta;

        /////////////////////////////////////////////////////
        // collect 4 integer seed and validate
        /////////////////////////////////////////////////////
        CTiePt tpTemp = tp;
        vector<CTiePt> vectpSeeds;
        // pt1 (top-left)
        tpTemp.m_ptL = ptIntL;
        tpTemp.m_ptR = ptIntR;
        vectpSeeds.push_back(tpTemp);
        // pt2 (top-right)
        tpTemp.m_ptL = ptIntL + Point2f(1, 0);
        tpTemp.m_ptR = ptIntR + Point2f(1, 0);
        vectpSeeds.push_back(tpTemp);

        // pt3 (bottom-left)
        tpTemp.m_ptL = ptIntL + Point2f(0, 1);
        tpTemp.m_ptR = ptIntR + Point2f(0, 1);
        vectpSeeds.push_back(tpTemp);

        // pt 4 (bottom-right)    
        tpTemp.m_ptL = ptIntL + Point2f(1, 1);
        tpTemp.m_ptR = ptIntR + Point2f(1, 1);
        vectpSeeds.push_back(tpTemp);

        //apply ALSC to collect as new seed
        ALSC alsc(m_imgL, m_imgR,  m_paramDense.m_paramGotcha.m_paramALSC);
        alsc.performALSC(&vectpSeeds);
        vectpSeeds.clear();
        alsc.getRefinedTps(vectpSeeds); // hard-copy

        for (int j = 0; j < (int)vectpSeeds.size(); j++){
            CTiePt tpRef = vectpSeeds.at(j);
            if (tpRef.m_fSimVal != CTiePt::NOT_DEF){
                vecRes.push_back(tpRef);
            }
        }
    }

    return vecRes;
}

bool CDensify::saveProjLog (string strFile){
  
  std::cout << "--writing log " << strFile << std::endl;
  bool bRes = false;
    ofstream sfLog;
    sfLog.open(strFile.c_str(), ios::app | ios::out);
    string strProcType = getProcType();

    std::cout << "---temporary xxx" << std::endl;
    std::cout << "<Project I/O>" << endl;
    std::cout << "Input left image path: " << m_paramDense.m_strImgL << endl;
    std::cout << "Input right image path: " << m_paramDense.m_strImgR << endl;
    std::cout << "Processing type: " << strProcType << endl;

    //std::cout << "Input x disparity map path: " << m_paramDense.m_strDispX << endl;
    //std::cout << "Input y disparity map path: " << m_paramDense.m_strDispY << endl;
    //std::cout << "Input/output mask file path: " << m_paramDense.m_paramGotcha.m_strMask << endl;
    std::cout << "Output x disparity map path: " << m_paramDense.m_strUpdatedDispX << endl;
    std::cout << "Output y disparity map path: " << m_paramDense.m_strUpdatedDispY << endl;
    std::cout << endl;


    if (sfLog.is_open()){
        sfLog << "<Project I/O>" << endl;
        sfLog << "Input left image path: " << m_paramDense.m_strImgL << endl;
        sfLog << "Input right image path: " << m_paramDense.m_strImgR << endl;
        sfLog << "Processing type: " << strProcType << endl;

        //sfLog << "Input x disparity map path: " << m_paramDense.m_strDispX << endl;
        //sfLog << "Input y disparity map path: " << m_paramDense.m_strDispY << endl;
        //sfLog << "Output tie-point file (from disparity) path: " << m_paramDense.m_strTPFile << endl;
        //sfLog << "Input/output mask file path: " << m_paramDense.m_paramGotcha.m_strMask << endl;
        sfLog << "Output x disparity map path: " << m_paramDense.m_strUpdatedDispX << endl;
        sfLog << "Output y disparity map path: " << m_paramDense.m_strUpdatedDispY << endl;
        sfLog << endl;
        sfLog.close();
        bRes = true;
    }
    else
        return bRes = false;

    return bRes;
}

bool CDensify::saveLog(){

  string FILE_LOG = "-GLog.txt";

  string strFile = m_paramDense.m_strOutPath + FILE_LOG;
  bool bRes = false;
  
  // save parameter log
  bRes = saveProjLog(strFile);
  bRes = saveGOTCHAParam(m_paramDense.m_paramGotcha, strFile);
  bRes = bRes && saveALSCParam(m_paramDense.m_paramGotcha.m_paramALSC, strFile);
  // save result log
  bRes = bRes && saveResLog(strFile);
  
  return bRes;
}

bool CDensify::saveResLog(string strFile){

    bool bRes = false;
    int nNumFinalTPs = nNumSeedTPs+m_vectpAdded.size();
    ofstream sfLog;
    sfLog.open(strFile.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){
        sfLog << "<Processing results>" << endl;
        sfLog << "Processing method: " << m_paramDense.getProcessingType() << endl;
        sfLog << "Total number of seed TPs: " << nNumSeedTPs << endl;
        sfLog << "Total number of final TPs: " << nNumFinalTPs << endl;
        sfLog << "Total processing time(sec): " << procTime << endl;
        sfLog << endl;
        sfLog << endl;
        sfLog.close();
        bRes = true;
    }
    else
        return bRes = false;

    return bRes;

}

void CDensify::makeDataProducts(){

    // prepare output data product from the list of densified tiepoints    
    // make output products (disparity map x,y and sim)
    int nW = m_imgL.cols;
    int nH = m_imgL.rows;

    // clear&initialise output buffers
    m_matDisMapX = Mat::ones(nH, nW, CV_32FC1)*0.0;
    m_matDisMapY = Mat::ones(nH, nW, CV_32FC1)*0.0;
    m_matDisMapSim = Mat::ones(nH, nW, CV_32FC1)*-1;

    // fill the disparity map
    int nLen = m_vectpAdded.size();
    for (int i = 0 ; i < nLen; i++){
        CTiePt tp = m_vectpAdded.at(i);
        Point2f ptL = tp.m_ptL;
        Point2f ptR = tp.m_ptR;
        float fSim = tp.m_fSimVal;

        int x = (int) ptL.x;
        int y = (int) ptL.y;
        Rect_<int> rect(0,0,nW,nH);
        if (rect.contains(Point2i(x,y))) {
            m_matDisMapX.at<float>(y,x) = ptR.x - x;
            m_matDisMapY.at<float>(y,x) = ptR.y - y;
            m_matDisMapSim.at<float>(y,x) = fSim;
        }
    }        
}


bool CDensify::saveResult(){

    bool bRes = true;

    //string strFile = m_paramDense.m_strTPFile;
    //bRes = saveTP(m_vectpAdded, strFile);
    
    cout << "Writing results..." << endl;

    Mat dispX = Mat::ones(m_matDisMapX.size(), CV_32FC1)*0.0;
    Mat dispY = Mat::ones(m_matDisMapY.size(), CV_32FC1)*0.0;

    for (int i =0; i<dispX.rows; i++){
        for (int j=0; j<dispX.cols; j++){
            if (m_matDisMapX.at<float>(i,j)!= 0.0){ //-3.40282346639e+038
                float disp = m_matDisMapX.at<float>(i,j);
                dispX.at<float>(i,j)=disp;
            }
        }
    }

    for (int i =0; i<dispY.rows; i++){
        for (int j=0; j<dispY.cols; j++){
            if (m_matDisMapY.at<float>(i,j)!= 0.0){
                float disp = m_matDisMapY.at<float>(i,j);
                dispY.at<float>(i,j)=disp;
            }
        }
    }

    //Mat m_input_dispX = imread(m_paramDense.m_strDispX, CV_LOAD_IMAGE_ANYDEPTH);
    //Mat m_input_dispY = imread(m_paramDense.m_strDispY, CV_LOAD_IMAGE_ANYDEPTH);
    //std::cout << "Reading mask: " << m_paramDense.m_strMask << std::endl;
    //Mat Mask = imread(m_paramDense.m_strMask, CV_LOAD_IMAGE_ANYDEPTH);
    //Mask.convertTo(Mask, CV_8UC1);
    nNumSeedTPs = 0;

    if (m_input_dispX.depth()==2 && m_input_dispY.depth()==2){
        for (int i=0; i<m_input_dispX.rows; i++){
            for (int j=0; j<m_input_dispX.cols; j++){
                if (m_paramDense.m_Mask.at<uchar>(i,j)==1){
                    m_input_dispX.at<ushort>(i,j)=65535;
                    m_input_dispY.at<ushort>(i,j)=65535;
                }
            }
        }
    }
    else if (m_input_dispX.depth()==5 && m_input_dispY.depth()==5){
        for (int i=0; i<m_input_dispX.rows; i++){
            for (int j=0; j<m_input_dispX.cols; j++){
                if (m_paramDense.m_Mask.at<uchar>(i,j)==1){
                    m_input_dispX.at<float>(i,j)= 0.0;
                    m_input_dispY.at<float>(i,j)= 0.0;
                }
            }
        }
    }

    if (m_input_dispX.depth()==2 && m_input_dispY.depth()==2){
        for (int i=0; i<m_input_dispX.rows; i++){
            for (int j=0; j<m_input_dispX.cols; j++){
                if (m_input_dispX.at<ushort>(i,j)!=65535 && m_input_dispY.at<ushort>(i,j)!=65535)
                    nNumSeedTPs+=1;
            }
        }
    }
    else if (m_input_dispX.depth()==5 && m_input_dispY.depth()==5){
        for (int i=0; i<m_input_dispX.rows; i++){
            for (int j=0; j<m_input_dispX.cols; j++){
                if (m_input_dispX.at<float>(i,j)!= 0.0 && m_input_dispY.at<float>(i,j)!= 0.0)
                    nNumSeedTPs+=1;
            }
        }
    }


    //Recover values of original disp map
    if (m_input_dispX.depth()==2 && m_input_dispY.depth()==2){
        for (int i =0; i<dispX.rows; i++){
            for (int j=0; j<dispX.cols; j++){
                if (m_input_dispX.at<ushort>(i,j)!=65535)
                    dispX.at<ushort>(i,j)=m_input_dispX.at<ushort>(i,j);
                if (m_input_dispY.at<ushort>(i,j)!=65535)
                    dispY.at<ushort>(i,j)=m_input_dispY.at<ushort>(i,j);
            }
        }
    }
    else if (m_input_dispX.depth()==5 && m_input_dispY.depth()==5){
        for (int i =0; i<dispX.rows; i++){
            for (int j=0; j<dispX.cols; j++){
                if (m_input_dispX.at<float>(i,j)!= 0.0){  //IMARS
                    float disp = m_input_dispX.at<float>(i,j); //IMARS
                    dispX.at<float>(i,j)=disp; //IMARS
                }
                if (m_input_dispY.at<float>(i,j)!= 0.0){  //IMARS
                    float disp = m_input_dispY.at<float>(i,j);  //IMARS
                    dispY.at<float>(i,j)=disp; //IMARS
                }
            }
        }
    }
    //Cancelled in IMARS, since using sGotcha, we have whole map.


    string strFileC1 = m_paramDense.m_strUpdatedDispX;
    std::cout << "--saving matrix " << strFileC1 << std::endl;
    bRes = bRes && saveMatrix(dispX, strFileC1);
    //imwrite(strFile, dispX);
    string strFileC2 = m_paramDense.m_strUpdatedDispY;
    std::cout << "--saving matrix " << strFileC2 << std::endl;
    bRes = bRes && saveMatrix(dispY, strFileC2);
    //imwrite(strFile, dispY);

#if 0
    // Do not write this file as it is not used. If needed, such a
    // file must be written for every tile ASP processes.
    string strFileC3 = m_paramDense.m_strUpdatedDispSim;
    std::cout << "Saving matrix: " << strFileC3 << std::endl;
    bRes = bRes && saveMatrix(m_matDisMapSim, strFileC3);
#endif

    string strC1Tiff = "-c1_refined.tif";
    string strC2Tiff = "-c2_refined.tif";
#if 0
    string strC3Tiff = "-uncertainty.tif";
#endif
    
    string strFileC1Tiff = m_paramDense.m_strOutPath + strC1Tiff;
    string strFileC2Tiff = m_paramDense.m_strOutPath + strC2Tiff;
#if 0
    string strFileC3Tiff = m_paramDense.m_strOutPath + strC3Tiff;
#endif
    
    ostringstream strCmdGdalConversionC1;
    strCmdGdalConversionC1 << "gdal_translate " << strFileC1 << " -of GTiff " << strFileC1Tiff;
    std::cout << strCmdGdalConversionC1.str() << std::endl;
    system(strCmdGdalConversionC1.str().c_str());

    ostringstream strCmdGdalConversionC2;
    strCmdGdalConversionC2 << "gdal_translate " << strFileC2 << " -of GTiff " << strFileC2Tiff;
    std::cout << strCmdGdalConversionC2.str() << std::endl;
    system(strCmdGdalConversionC2.str().c_str());

#if 0
    // Do not write this file as it is not used. If needed, such a
    // file must be written for every tile ASP processes.
    ostringstream strCmdGdalConversionC3;
    strCmdGdalConversionC3 << "gdal_translate " << strFileC3 << " -of GTiff " << strFileC3Tiff;
    std::cout << strCmdGdalConversionC3.str() << std::endl;
    system(strCmdGdalConversionC3.str().c_str());
#endif
    
    return bRes;
}

void CDensify::getNeighbour(const CTiePt tp, vector<CTiePt>& vecNeiTp, const int nNeiType, const Mat& matSim){
    //
    Point2f ptLeft = tp.m_ptL;
    Point2f ptRight = tp.m_ptR;

    if (nNeiType == CGOTCHAParam::NEI_DIFF){
        getDisffusedNei(vecNeiTp, tp, matSim);
    }
    /*if(nNeiType == CGOTCHAParam::NEI_X || nNeiType == CGOTCHAParam::NEI_Y || nNeiType == CGOTCHAParam::NEI_4 || nNeiType == CGOTCHAParam::NEI_8)*/
    else {
        if (nNeiType == CGOTCHAParam::NEI_4  || nNeiType == CGOTCHAParam::NEI_Y || nNeiType == CGOTCHAParam::NEI_8){
            CTiePt tp2;
            tp2.m_ptL = ptLeft + Point2f(0,1);
            tp2.m_ptR = ptRight + Point2f(0,1);
            vecNeiTp.push_back(tp2);
            tp2.m_ptL = ptLeft + Point2f(0,-1);
            tp2.m_ptR = ptRight + Point2f(0,-1);
            vecNeiTp.push_back(tp2);
        }

        if (nNeiType == CGOTCHAParam::NEI_X || nNeiType == CGOTCHAParam::NEI_4 || nNeiType == CGOTCHAParam::NEI_8){

            CTiePt tp;
            tp.m_ptL = ptLeft + Point2f(1,0);
            tp.m_ptR = ptRight + Point2f(1,0);
            vecNeiTp.push_back(tp);
            tp.m_ptL = ptLeft + Point2f(-1,0);
            tp.m_ptR = ptRight + Point2f(-1,0);
            vecNeiTp.push_back(tp);
        }

        if (nNeiType == CGOTCHAParam::NEI_8){
            CTiePt tp8;
            tp8.m_ptL = ptLeft + Point2f(-1,-1);
            tp8.m_ptR = ptRight + Point2f(-1,-1);
            vecNeiTp.push_back(tp8);
            tp8.m_ptL = ptLeft + Point2f(1,1);
            tp8.m_ptR = ptRight + Point2f(1,1);
            vecNeiTp.push_back(tp8);
            tp8.m_ptL = ptLeft + Point2f(-1,1);
            tp8.m_ptR = ptRight + Point2f(-1,1);
            vecNeiTp.push_back(tp8);
            tp8.m_ptL = ptLeft + Point2f(1,-1);
            tp8.m_ptR = ptRight + Point2f(1,-1);
            vecNeiTp.push_back(tp8);
        }
    }
}

void CDensify::getDisffusedNei(vector<CTiePt>& vecNeiTp, const CTiePt tp, const Mat& matSim){
    // estimate the growth
    // make a diffusion map
    int nSzDiff = m_paramDense.m_paramGotcha.m_paramALSC.m_nPatch;//12
    double pdDiffMap[nSzDiff*2+1][nSzDiff*2+1];
    int nW = m_imgL.cols;//m_imgL.getMaxX()+1;//m_imgL.getWidth();
    int nH = m_imgL.rows;//m_imgL.getMaxY()+1;//m_imgL.getHeight();
    Rect rectRegion(0,0,nW, nH);

    Point2f ptLeft = tp.m_ptL;
    Point2f ptRight = tp.m_ptR;
    for (int j = -nSzDiff ; j < nSzDiff + 1; j++){
        for (int i = -nSzDiff ; i < nSzDiff + 1; i++){
            double val = 0.f;
            int nX = (int)floor(ptLeft.x+i);
            int nY = (int)floor(ptLeft.y+j);
            if (rectRegion.contains(Point(nX, nY)) &&
                matSim.at<float>(nY, nX) > 0){
                // assume sim value has been already normalised
                val = 1.f - matSim.at<float>(nY, nX);// /m_paramDense.m_paramGotcha.m_paramALSC.m_fEigThr;
            }
            pdDiffMap[j+nSzDiff][i+nSzDiff] = val;
        }
    }

    // heat diffusion equation
    double dAlpha = m_paramDense.m_paramGotcha.m_fDiffCoef;// 0.05; // diffusion coefficient
    double dThr = m_paramDense.m_paramGotcha.m_fDiffThr;// 0.1;
    int nIter = m_paramDense.m_paramGotcha.m_nDiffIter;// 5;

    int nRow = nSzDiff*2+1;
    int nCol = nSzDiff*2+1;
    double pdDiffMapNext[nRow][nCol];
    double pdDiffMapCurrnet[nRow][nCol];

    for(int j = 0 ; j < nRow; j++){
        for(int i = 0; i < nCol; i++){
            pdDiffMapCurrnet[j][i] = pdDiffMap[j][i];
            pdDiffMapNext[j][i] = 0;
        }
    }

    // define neighbours
    for (int k = 0 ; k < nIter; k++){
        // get diffused
        double pdTemp[nRow][nCol];
        for (int j = 1; j < nRow-1; j++){
            for (int i = 1; i < nCol-1; i++){
                pdDiffMapNext[j][i] = pdDiffMapCurrnet[j][i] + dAlpha *
                                      (pdDiffMapCurrnet[j+1][i] + pdDiffMapCurrnet[j][i+1] +
                                       pdDiffMapCurrnet[j-1][i] + pdDiffMapCurrnet[j][i-1] - 4*pdDiffMapCurrnet[j][i]);
                pdTemp[j][i] = pdDiffMapNext[j][i];
            }
        }
        for(int j = 1 ; j < nRow-1; j++){
            for(int i = 1; i < nCol-1; i++){
                pdDiffMapCurrnet[j][i] = pdTemp[j][i];
            }
        }
    }

    int count = 0;

    for (int j = 1; j < nRow-1; j++){
        for (int i = 1; i < nCol-1; i++){
            if (pdDiffMapNext[j][i] > dThr) { //&& (i != nSzDiff && j != nSzDiff)){ // add as a neighbour
                if (i == nSzDiff && j == nSzDiff) continue;
                CTiePt tpNei;
                tpNei.m_ptL.x = ptLeft.x + i - nSzDiff;
                tpNei.m_ptL.y = ptLeft.y + j - nSzDiff;
                tpNei.m_ptR.x = ptRight.x + i - nSzDiff;
                tpNei.m_ptR.y = ptRight.y + j - nSzDiff;

                vecNeiTp.push_back(tpNei);

                count++;
            }
        }
    }
            // add minimum num of neighbours
            CTiePt tp2;
            tp2.m_ptL = ptLeft + Point2f(0,1);
            tp2.m_ptR = ptRight + Point2f(0,1);
            if (!isHavingTP(vecNeiTp, tp2))
                vecNeiTp.push_back(tp2);

            tp2.m_ptL = ptLeft + Point2f(0,-1);
            tp2.m_ptR = ptRight + Point2f(0,-1);
            if (!isHavingTP(vecNeiTp, tp2))
                vecNeiTp.push_back(tp2);

            //CTiePt tp3;
            tp2.m_ptL = ptLeft + Point2f(1,0);
            tp2.m_ptR = ptRight + Point2f(1,0);
            if (!isHavingTP(vecNeiTp, tp2))
                vecNeiTp.push_back(tp2);

            tp2.m_ptL = ptLeft + Point2f(-1,0);
            tp2.m_ptR = ptRight + Point2f(-1,0);
            if (!isHavingTP(vecNeiTp, tp2))
                vecNeiTp.push_back(tp2);
}

bool CDensify::isHavingTP(vector<CTiePt>& vecNeiTp, CTiePt tp){
    bool bRes = false;
    int nLen = vecNeiTp.size();
    for (int i = 0; i < nLen; i++){
        if (vecNeiTp.at(i) == tp){
            bRes = true;
            break;
        }
    }
    return bRes;
}

void CDensify::removeOutsideImage(vector<CTiePt>& vecNeiTp, const Rect_<float> rectTileL, const Rect_<float> rectImgR){
    vector<CTiePt>::iterator iter;

    for (iter = vecNeiTp.begin(); iter < vecNeiTp.end(); ){

        if (!rectTileL.contains(iter->m_ptL) || !rectImgR.contains(iter->m_ptR))
            vecNeiTp.erase(iter);
       else
            iter++;
    }

}

void CDensify::removePtInLUT(vector<CTiePt>& vecNeiTp, const vector<bool>& pLUT, const int nWidth){
    vector<CTiePt>::iterator iter;

    for (iter = vecNeiTp.begin(); iter < vecNeiTp.end(); ){        
        int nX = (int)floor(iter->m_ptL.x);
        int nY = (int)floor(iter->m_ptL.y);
        //int nIdx = nY * m_imgL.cols + nX;
        int nIdx = nY * nWidth + nX;

        if (pLUT[nIdx])
            vecNeiTp.erase(iter);
        else
            iter++;
    }
}

void CDensify::makeTiles(vector< Rect_<float> >& vecRectTiles, int nMin){

    vector< Rect_<float> >::iterator iter;
    for (iter = vecRectTiles.begin(); iter < vecRectTiles.end(); ){
        Rect_<float> rectParent = *iter;
        if (rectParent.width/2 > nMin && rectParent.height/2 > nMin){
            vecRectTiles.erase(iter);
            vector< Rect_<float> > vecChild;
            breakIntoSubRect(rectParent, vecChild);
            vecRectTiles.insert(vecRectTiles.end(), vecChild.begin(),  vecChild.end());
            iter = vecRectTiles.begin();
        }
        else
            iter++;
    }
}

void CDensify::breakIntoSubRect(Rect_<float> rectParent, vector< Rect_<float> >& vecRes){

    float fHalfW = rectParent.width/2.f;
    float fHalfH = rectParent.height/2.f;

    Point2f ptTopLeft[4];
    ptTopLeft[0] = Point2f(rectParent.x, rectParent.y);
    ptTopLeft[1] = Point2f(rectParent.x+fHalfW, rectParent.y);
    ptTopLeft[2] = Point2f(rectParent.x, rectParent.y+fHalfH);
    ptTopLeft[3] = Point2f(rectParent.x+fHalfW, rectParent.y+fHalfH);

    Size sz(fHalfW, fHalfH);
    Rect_<float> rect(ptTopLeft[0], sz);
    vecRes.push_back(rect);

    sz = Size(rectParent.x+rectParent.width - ptTopLeft[1].x, fHalfH);
    rect = Rect(ptTopLeft[1], sz);
    vecRes.push_back(rect);

    sz = Size(fHalfW, rectParent.y+rectParent.height - ptTopLeft[2].y);
    rect = Rect(ptTopLeft[2], sz);
    vecRes.push_back(rect);

    sz = Size(rectParent.x+rectParent.width - ptTopLeft[3].x, rectParent.y+rectParent.height - ptTopLeft[3].y);
    rect = Rect(ptTopLeft[3], sz);
    vecRes.push_back(rect);

}


bool CDensify::doGotcha(const Mat& matImgL, const Mat& matImgR, vector<CTiePt>& vectpSeeds,
                        const CGOTCHAParam& paramGotcha, vector<CTiePt>& vectpAdded){


    bool bRes = true;
    ////////////////////////////////////////////
    // parameter preparation
    cout << "CASP-GO INFO: initialisaing similarity map" << endl;
    Mat matSimMap = Mat::ones(matImgL.rows, matImgL.cols, CV_32FC1); // 2D similarity map for diffusion
    matSimMap = matSimMap*-1;

    cout << "DEBUG: matImgL has statistics: rows " << matImgL.rows << "; cols: " << matImgL.cols
         << "; channels: " << matImgL.channels() << "; bitdepth code: " << matImgL.depth() << endl;

    Size szImgL(matImgL.cols, matImgL.rows);
    cout << "CASP-GO INFO: initialising pixel LUT" << endl;

    // IMARS bool pLUT[szImgL.area()]; // if true it indicates the pixel has already processed
    vector<bool> pLUT; //IMARS

    // initialise
    for (int i = 0; i< szImgL.area(); i++){
        bool bpLUT = false; //IMARS
        pLUT.push_back(bpLUT); //IMARS
    } //IMARS

    vector< Rect_<float> > vecRectTiles;
    vecRectTiles.push_back(Rect(0., 0., matImgL.cols, matImgL.rows));

    cout << "CASP-GO INFO: Making tiles" << endl;
    makeTiles(vecRectTiles, paramGotcha.m_nMinTile);

    if (paramGotcha.m_bNeedInitALSC){
        cout << "CASP-GO INFO: Running initial ALSC refinement" << endl;
        ALSC alsc(matImgL, matImgR, paramGotcha.m_paramALSC);
        alsc.performALSC(&vectpSeeds);
        vectpSeeds.clear();
        alsc.getRefinedTps(vectpSeeds); // hard-copy
    }

    cout << "CASP-GO INFO: initialise similarity map with seed points" << endl;
    // initialise sim map with seedpoitns
    for (int i = 0; i < (int)vectpSeeds.size(); i++){
        int nX =  (int)floor(vectpSeeds.at(i).m_ptL.x);
        int nY =  (int)floor(vectpSeeds.at(i).m_ptL.y);
        matSimMap.at<float>(nY, nX) = vectpSeeds.at(i).m_fSimVal;

        int nIdx = nY*szImgL.width + nX;
        pLUT[nIdx] = true;

        //cout << "DEBUG: pLUT change successfully to " << pLUT[nIdx] << endl;
    }

    cout << "CASP-GO INFO: apply mask to remove area that no densification is required." << endl;
    // apply mask, remove area where no densification is required
    //std::cout << "Reading mask: " << paramGotcha.m_strMask << std::endl;
    //Mat Mask = imread(paramGotcha.m_strMask, CV_LOAD_IMAGE_ANYDEPTH);
    for (int i=0; i<m_paramDense.m_Mask.rows; i++){
        for (int j=0; j<m_paramDense.m_Mask.cols; j++){
            if (m_paramDense.m_Mask.at<uchar>(i,j)==0){
                int nIdx = i*m_paramDense.m_Mask.cols + j;
                pLUT[nIdx] = true;
                //cout << "DEBUG: pLUT change successfully to " << pLUT[nIdx] << endl;
            }
        }
    }

    /////////////////////////
    vectpAdded.clear();
    //cout << "debug1" << endl;
    cout << "CASP-GO INFO: Desifying disparity... ..." << endl;

    std::cout << "--Number of tiles " << vecRectTiles.size() << std::endl;
    
    for (int i = 0 ; i < (int)vecRectTiles.size(); i++ ){
          vector<CTiePt> vecRes;
          bRes = bRes && doTileGotcha(matImgL, matImgR, vectpSeeds, paramGotcha, vecRes, vecRectTiles.at(i), matSimMap, pLUT);
          // collect result
          vectpAdded.insert(vectpAdded.end(), vecRes.begin(), vecRes.end());

          // debug
          //cout << "Tile " << i << " has been processed: " << vecRes.size() << " points are added" << endl;
    }

    return bRes;
}


bool CDensify::doTileGotcha(const Mat& matImgL, const Mat& matImgR, const
                            vector<CTiePt>& vectpSeeds,
                            const CGOTCHAParam& paramGotcha, vector<CTiePt>& vectpAdded,
                            const Rect_<float> rectTileL, Mat& matSimMap, vector<bool>& pLUT){

    vector<CTiePt> vectpSeedTPs; //= vectpSeeds;                // need this hard copy for sorting

    Size szImgL(matImgL.cols, matImgL.rows);
    Rect_<float> rectImgR (0, 0, matImgR.cols, matImgR.rows);
    vectpAdded.clear(); //clear output tp list

    // set indicator buffer
    for (int i = 0 ; i < (int)vectpSeeds.size(); i++){
        CTiePt tp = vectpSeeds.at(i);

        if (rectTileL.contains(tp.m_ptL)){
            vectpSeedTPs.push_back(tp);
        }

    }
    //std::cout << "Reading mask: " << paramGotcha.m_strMask << std::endl;
    //Mat Mask = imread(paramGotcha.m_strMask, CV_LOAD_IMAGE_ANYDEPTH);
    Mat imgL = matImgL;
    Mat imgR = matImgR;
    imgL.convertTo(imgL, CV_8UC1);
    imgR.convertTo(imgR, CV_8UC1);

    unsigned int nGapSize = 0;
    for (int i=0; i<m_paramDense.m_Mask.rows; i++){
        for (int j=0; j< m_paramDense.m_Mask.cols; j++){
          if (m_paramDense.m_Mask.at<uchar>(i,j)==1 &&
              imgL.at<uchar>(i,j)!=0 &&imgR.at<uchar>(i,j)!=0)
                nGapSize+=1;
        }
    }

    //sort(vectpSeedTPs.begin(), vectpSeedTPs.end(), compareTP); // sorted in ascending order
    /////////////////////////////////////////////////////////////////////
    // stereo region growing
    //int nCount = 0; // for debug
#if 0
      // Hide the verbose messages
      cout << "[--------------------]  0% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
#endif
      
    while (vectpSeedTPs.size() > 0) {
        // get a point from seed
        CTiePt tp = vectpSeedTPs.at(0);

//        mvectpAdded.push_back(tp);
        vectpSeedTPs.erase(vectpSeedTPs.begin());
        vector<CTiePt> vecNeiTp;        
        getNeighbour(tp, vecNeiTp, paramGotcha.m_nNeiType, matSimMap);
        removeOutsideImage(vecNeiTp, rectTileL, rectImgR);
        removePtInLUT(vecNeiTp, pLUT, matImgL.cols);

        //ALSC
        if ((int)vecNeiTp.size() > 0){            
            // get affine data from a seed
            float pfData[6] = {0, 0, 0, 0, 0, 0};
            for (int k = 0; k < 4; k++)
                pfData[k] = tp.m_pfAffine[k];
            pfData[4] = tp.m_ptOffset.x;
            pfData[5] = tp.m_ptOffset.y;

            ALSC alsc(matImgL, matImgR, paramGotcha.m_paramALSC);

            alsc.performALSC(&vecNeiTp, (float*) pfData);
            const vector<CTiePt>* pvecRefTPtemp = alsc.getRefinedTps();

            int nLen = pvecRefTPtemp->size();
            if( nLen > 0){
                // append survived neighbours to the seed point list and the seed LUT
                // vector<CRefinedTP>::iterator iterNei;
                for (int i = 0 ; i < nLen; i++){
                    CTiePt tpNei = pvecRefTPtemp->at(i);

                    int nXnei = (int)floor(tpNei.m_ptL.x);
                    int nYnei = (int)floor(tpNei.m_ptL.y);
                    int nIdxNei = nYnei*szImgL.width + nXnei;

                    matSimMap.at<float>(nYnei,nXnei) = tpNei.m_fSimVal;                    
                    pLUT[nIdxNei] = true;

                    vectpSeedTPs.push_back(tpNei);
                    vectpAdded.push_back(tpNei);
                }

                //sort(vectpSeedTPs.begin(), vectpSeedTPs.end(), compareTP);
            } 
        }
#if 0
        // Hide the verbose messages
        if (vectpAdded.size()>0){
        if (vectpAdded.size() >= (nGapSize)/20 && vectpAdded.size() < 2*(nGapSize)/20)
            cout << "[*-------------------]  5% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 2*(nGapSize)/20 && vectpAdded.size() < 3*(nGapSize)/20)
            cout << "[**------------------] 10% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 3*(nGapSize)/20 && vectpAdded.size() < 4*(nGapSize)/20)
            cout << "[***-----------------] 15% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 4*(nGapSize)/20 && vectpAdded.size() < 5*(nGapSize)/20)
            cout << "[****----------------] 20% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 5*(nGapSize)/20 && vectpAdded.size() < 6*(nGapSize)/20)
            cout << "[*****---------------] 25% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 6*(nGapSize)/20 && vectpAdded.size() < 7*(nGapSize)/20)
            cout << "[******--------------] 30% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 7*(nGapSize)/20 && vectpAdded.size() < 8*(nGapSize)/20)
            cout << "[*******-------------] 35% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 8*(nGapSize)/20 && vectpAdded.size() < 9*(nGapSize)/20)
            cout << "[********------------] 40% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 9*(nGapSize)/20 && vectpAdded.size() < 10*(nGapSize)/20)
            cout << "[*********-----------] 45% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 10*(nGapSize)/20 && vectpAdded.size() < 11*(nGapSize)/20)
            cout << "[**********----------] 50% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 11*(nGapSize)/20 && vectpAdded.size() < 12*(nGapSize)/20)
            cout << "[***********---------] 55% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 12*(nGapSize)/20 && vectpAdded.size() < 13*(nGapSize)/20)
            cout << "[************--------] 60% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 13*(nGapSize)/20 && vectpAdded.size() < 14*(nGapSize)/20)
            cout << "[*************-------] 65% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 14*(nGapSize)/20 && vectpAdded.size() < 15*(nGapSize)/20)
            cout << "[**************------] 70% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 15*(nGapSize)/20 && vectpAdded.size() < 16*(nGapSize)/20)
            cout << "[***************-----] 75% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 16*(nGapSize)/20 && vectpAdded.size() < 17*(nGapSize)/20)
            cout << "[****************----] 80% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 17*(nGapSize)/20 && vectpAdded.size() < 18*(nGapSize)/20)
            cout << "[*****************---] 85% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 18*(nGapSize)/20 && vectpAdded.size() < 19*(nGapSize)/20)
            cout << "[******************--] 90% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() >= 19*(nGapSize)/20 && vectpAdded.size() < 20*(nGapSize)/20)
            cout << "[*******************-] 95% OF THE GAP AREA HAS BEEN DENSIFIED\r" << std::flush;
        else if (vectpAdded.size() == nGapSize)
          cout << "[********************]100% OF THE GAP AREA HAS BEEN DENSIFIED";
        }
#endif
    }
#if 0
    cout << endl;
#endif
    
    return true;
}

bool CDensify::doPGotcha(int nNeiType){

    // pyramid construction
    int nTotLev = getTotPyramidLev(m_paramDense.m_paramGotcha.m_paramALSC.m_nPatch);
    vector<Mat> vecImgPyL;
    vector<Mat> vecImgPyR;
    buildPyramid(m_imgL, vecImgPyL, nTotLev);
    buildPyramid(m_imgR, vecImgPyR, nTotLev);

    vector<CTiePt> vecOrgSeedClone = getIntToFloatSeed(m_vecTPs);
    if (m_paramDense.m_paramGotcha.m_bNeedInitALSC){
        ALSC alsc(m_imgL, m_imgR, m_paramDense.m_paramGotcha.m_paramALSC);
        alsc.performALSC(&vecOrgSeedClone);
        vecOrgSeedClone.clear();
        alsc.getRefinedTps(vecOrgSeedClone); // hard-copy
    }

    CGOTCHAParam paramG = m_paramDense.m_paramGotcha;
    paramG.m_nNeiType = nNeiType;
    paramG.m_bNeedInitALSC = true;

    float fTempLim = m_paramDense.m_paramGotcha.m_paramALSC.m_fEigThr;
    int nTempSz = m_paramDense.m_paramGotcha.m_paramALSC.m_nPatch;
    float fMinRate = 0.3f;
    float fMinRatePatch = 0.5f;

    vector<CTiePt> vecAddedTemp; // addition from previous level
    bool bRes = true;
    for (int i = nTotLev-1; i >= 0; i--){

        // prepare new processing param at level i
        paramG.m_paramALSC.m_nPatch = (int)(nTempSz * (1.f - fMinRatePatch * i/(nTotLev-1)));
        paramG.m_paramALSC.m_fEigThr = fTempLim * (1.f + fMinRate * i/(nTotLev-1));

        // prepare seed for current level
        vector<CTiePt> vecSeedTemp;
        float fDenom = pow(2.f, i);
        int nLen = vecOrgSeedClone.size();
        for (int j = 0; j < nLen; j++){
            CTiePt tp;
            tp = vecOrgSeedClone.at(j);
            tp.m_ptL.x = tp.m_ptL.x / fDenom;
            tp.m_ptL.y = tp.m_ptL.y / fDenom;
            tp.m_ptR.x = tp.m_ptR.x / fDenom;
            tp.m_ptR.y = tp.m_ptR.y / fDenom;
            for (int k = 0; k < 4 ; k++)
                tp.m_pfAffine[k] = 0.;
            tp.m_ptOffset.x = tp.m_ptOffset.x / fDenom;
            tp.m_ptOffset.y = tp.m_ptOffset.y / fDenom;
            vecSeedTemp.push_back(tp);
        }

        // added point propagated points
        nLen = vecAddedTemp.size();
        for (int j = 0; j < nLen; j++){
            vecAddedTemp.at(j).m_ptL.x *= 2.f;
            vecAddedTemp.at(j).m_ptL.y *= 2.f;
            vecAddedTemp.at(j).m_ptR.x *= 2.f;
            vecAddedTemp.at(j).m_ptR.y *= 2.f;
            for (int k = 0; k < 4 ; k++)
                vecAddedTemp.at(j).m_pfAffine[k] = 0.;
            vecAddedTemp.at(j).m_ptOffset.x *= 2.f;
            vecAddedTemp.at(j).m_ptOffset.y *= 2.f;
        }

        if (nLen > 0){
            ALSC alsc(vecImgPyL.at(i), vecImgPyR.at(i), paramG.m_paramALSC);
            alsc.performALSC(&vecAddedTemp);
            vecAddedTemp.clear();
            alsc.getRefinedTps(vecAddedTemp); // hard-copy
        }

        vector<CTiePt> seedIn;
        seedIn.insert(seedIn.end(), vecSeedTemp.begin(), vecSeedTemp.end() );
        seedIn.insert(seedIn.end(), vecAddedTemp.begin(), vecAddedTemp.end());
        vecSeedTemp.clear();

        // debug
        cout << "# seed pts at the " << i << " level" << ": " << seedIn.size() << endl;

        vector<CTiePt> vecResTemp;
        bRes = bRes && doGotcha(vecImgPyL.at(i), vecImgPyR.at(i), seedIn, paramG, vecResTemp);
        vecAddedTemp.insert(vecAddedTemp.end(), vecResTemp.begin(), vecResTemp.end());

        // debug
        cout << "# added pts at the " << i << " level" << ": " << vecResTemp.size() << endl;
    }

    m_vectpAdded.clear();
    m_vectpAdded.insert(m_vectpAdded.end(), vecOrgSeedClone.begin(), vecOrgSeedClone.end());
//    m_vectpAdded.insert(m_vectpAdded.end(), m_vecTPs.begin(), m_vecTPs.end());
    m_vectpAdded.insert(m_vectpAdded.end(), vecAddedTemp.begin(), vecAddedTemp.end());

    return bRes;
}

int CDensify::getTotPyramidLev(int nszPatch) {
    int nRes = 0;

    Size szImgL(m_imgL.cols, m_imgL.rows);
    Size szImgR(m_imgR.cols, m_imgR.rows);
    int nW = 0, nH = 0;

    // get smallest size of width
    nW = min(szImgL.width, szImgR.width);
    nH = min(szImgL.height, szImgR.height);

    int nMinSz = 4*(nszPatch * 2 + 1);
    Size szMin(nMinSz,nMinSz);

    // estimate the minimum of the largest levels
    while(nW > szMin.width || nH > szMin.height){
        nW /= 2;
        nH /= 2;
        nRes++;
    }

    return nRes;
}





