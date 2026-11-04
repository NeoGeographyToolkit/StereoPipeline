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

#include <asp/Gotcha/CProcBlock.h>

using namespace cv;

namespace gotcha {

CProcBlock::CProcBlock(){}

// Images are passed in memory now
#if 0
void CProcBlock::setImages(std::string strImgL, std::string strImgR, bool bGrey){
  
    // read image as a grey -> vital for alsc and gotcha
    if (bGrey){
        m_imgL = imread(strImgL, CV_LOAD_IMAGE_ANYDEPTH); //IMARS
        m_imgR = imread(strImgR, CV_LOAD_IMAGE_ANYDEPTH); //IMARS
    }
    else{
        m_imgL = imread(strImgL, 1);
        m_imgR = imread(strImgR, 1);
    }
}
#endif

bool CProcBlock::saveTP(const std::vector<CTiePt>& vecTPs, const std::string strFile){

    std::ofstream sfTP;
    sfTP.open(strFile.c_str());

    int nLen = vecTPs.size();
    int nEle = 11;

    if (sfTP.is_open()){
        // header
        sfTP << nLen << " " << nEle << std::endl;
        // data
        for (int i = 0 ; i < nLen ;i++){
            CTiePt tp = vecTPs.at(i);
            sfTP << tp.m_ptL.x << " "<< tp.m_ptL.y << " "
                 << tp.m_ptR.x << " "<< tp.m_ptR.y << " "
                 << tp.m_fSimVal << " "<< tp.m_pfAffine[0] << " "
                 << tp.m_pfAffine[1] << " "<< tp.m_pfAffine[2] << " "
                 << tp.m_pfAffine[3] << " " << tp.m_ptOffset.x << " "
                 << tp.m_ptOffset.y << std::endl;

        }
        sfTP.close();
    }
    else
        return false;

    return true;
}

#if 0
bool CProcBlock::loadTP(const std::string strFile){
  std::cout << "Reading " << strFile << std::endl;
    ifstream sfTPFile;
    sfTPFile.open(strFile.c_str());

    m_vecTPs.clear();
    //m_vecRefTPs.clear();
    int nTotLen = 0;
    if (sfTPFile.is_open()){
          // total num of TPs (i.e., lines)
        int nElement; // total num of elements in a TP
        sfTPFile >> nTotLen >> nElement;

        for (int i = 0 ; i < nTotLen; i++){
            CTiePt tp;
            sfTPFile >> tp.m_ptL.x >> tp.m_ptL.y >> tp.m_ptR.x >> tp.m_ptR.y >> tp.m_fSimVal;

            if (nElement > 5){
                float fDummy;
                for (int k  = 0 ; k < 6; k++) sfTPFile >> fDummy;
            }

            if (nElement > 11){                
                float fDummy;
                double dDummy;
                for (int k  = 0 ; k < 6; k++) sfTPFile >> fDummy;
                for (int k  = 0 ; k < 8; k++) sfTPFile >> dDummy;
            }

            m_vecTPs.push_back(tp);
        }

        sfTPFile.close();
    }
    else
        return false;

    if (nTotLen < 8) {
        //cerr << "TP should be more than 8 pts" << std::endl;
        return false;
    }

    return true;
}
#endif

bool CProcBlock::saveMatrix(const Mat& matData, const std::string strFile){

    std::ofstream sfOut;
    sfOut.open(strFile.c_str());

    if (sfOut.is_open()){

        sfOut << "ncols " << matData.cols << std::endl;
        sfOut << "nrows " << matData.rows << std::endl;
        sfOut << "xllcorner 0" << std::endl;
        sfOut << "yllcorner 0" << std::endl;
        sfOut << "cellsize 1" << std::endl;

        for (int i = 0; i < matData.rows; i++){
            for (int j = 0; j < matData.cols; j++){
                if (matData.depth() == CV_32F)
                    sfOut << matData.at<float>(i,j) << " ";
                else if (matData.depth() == CV_64F)
                    sfOut << matData.at<double>(i,j) << " ";
                else if (matData.depth() == CV_8U)
                    sfOut << matData.at<uchar>(i,j) << " ";
            }
            sfOut << std::endl;
        }
        sfOut.close();
    }
    else
        return false;

    return true;
}

#if 0
bool CProcBlock::loadMatrix(Mat &matData, const std::string strFile, bool bDoublePrecision){
    if (bDoublePrecision){
        ifstream sfIn;
        sfIn.open(strFile.c_str());

        if (sfIn.is_open()){
            int nRow;
            int nCol;
            sfIn >> nRow >> nCol;

            matData = Mat::zeros(nRow, nCol, CV_64F);

            for (int i = 0; i < matData.rows; i++){
                for (int j = 0; j < matData.cols; j++){
                    sfIn >> matData.at<double>(i,j);
                }
            }
            sfIn.close();
        }
        else
            return false;

        return true;
    }
    else{
        return loadMatrix(matData, strFile);
    }
}

bool CProcBlock::loadMatrix(Mat &matData, const std::string strFile){

    ifstream sfIn;
    sfIn.open(strFile.c_str());

    if (sfIn.is_open()){
        int nRow;
        int nCol;
        sfIn >> nRow >> nCol;

        matData = Mat::zeros(nRow, nCol, CV_32F);

        for (int i = 0; i < matData.rows; i++){
            for (int j = 0; j < matData.cols; j++){
                sfIn >> matData.at<float>(i,j);
            }
        }
        sfIn.close();
    }
    else
        return false;

    return true;
}

bool CProcBlock::loadMatrix(std::string strFile){
    ifstream sfIn;
    sfIn.open(strFile.c_str());

    if (sfIn.is_open()){
        int nRow;
        int nCol;
        sfIn >> nRow >> nCol;

        m_dispX = Mat::zeros(nRow, nCol, CV_32F);

        for (int i = 0; i < m_dispX.rows; i++){
            for (int j = 0; j < m_dispX.cols; j++){
                sfIn >> m_dispX.at<float>(i,j);
            }
        }
        sfIn.close();
    }
    else
        return false;

    return true;

}
#endif

bool CProcBlock::saveALSCParam(const CALSCParam& paramALSC, const std::string strOut){
    std::ofstream sfLog;
    sfLog.open(strOut.c_str(), std::ios::app | std::ios::out);

    if (sfLog.is_open()){

        sfLog << "<ALSC parameters>" << std::endl;
        sfLog << "The size of a matching patch: " << paramALSC.m_nPatch << std::endl;
        sfLog << "Maximum eigenval: " << paramALSC.m_fEigThr << std::endl;
        sfLog << "Maximum iteration: " << paramALSC.m_nMaxIter << std::endl;
        sfLog << "Affine distortion limit: " << paramALSC.m_fAffThr << std::endl;
        sfLog << "Translation limit: " << paramALSC.m_fDriftThr << std::endl;
        sfLog << "Use intensity offset parameter: " << paramALSC.m_bIntOffset << std::endl;
        sfLog << "Use weighting coefficients: " << paramALSC.m_bWeighting << std::endl;
        sfLog << std::endl;

        sfLog.close();
    }
    else
        return false;

    return true;
}

bool CProcBlock::saveGOTCHAParam(CGOTCHAParam& paramGOTCHA, const std::string strOut){
    std::ofstream sfLog;
    sfLog.open(strOut.c_str(), std::ios::app | std::ios::out);

    if (sfLog.is_open()){

        sfLog << "<GOTCHA parameters>" << std::endl;
        sfLog << "Neighbour type: " << paramGOTCHA.getNeiType()<< std::endl;
        sfLog << "Diffusion iteration: " << paramGOTCHA.m_nDiffIter << std::endl;
        sfLog << "Diffusion threshold: " << paramGOTCHA.m_fDiffThr << std::endl;
        sfLog << "Diffusion coefficient: " << paramGOTCHA.m_fDiffCoef << std::endl;
        //sfLog << "Minimum image tile size: " <<  paramGOTCHA.m_nMinTile << std::endl;
        sfLog << "Need initial ALSC on seed TPs: " << paramGOTCHA.m_bNeedInitALSC << std::endl;
        sfLog << std::endl;

        sfLog.close();
    }
    else
        return false;

    return true;
}

} // end namespace gotcha
  
