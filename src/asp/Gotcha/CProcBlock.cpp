#include "CProcBlock.h"

CProcBlock::CProcBlock()
{
}

void CProcBlock::setImages(string strImgL, string strImgR, bool bGrey){
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


bool CProcBlock::saveTP(const vector<CTiePt>& vecTPs, const string strFile){

    ofstream sfTP;
    sfTP.open(strFile.c_str());

    int nLen = vecTPs.size();
    int nEle = 11;

    if (sfTP.is_open()){
        // header
        sfTP << nLen << " " << nEle << endl;
        // data
        for (int i = 0 ; i < nLen ;i++){
            CTiePt tp = vecTPs.at(i);
            sfTP << tp.m_ptL.x << " "<< tp.m_ptL.y << " "
                 << tp.m_ptR.x << " "<< tp.m_ptR.y << " "
                 << tp.m_fSimVal << " "<< tp.m_pfAffine[0] << " "
                 << tp.m_pfAffine[1] << " "<< tp.m_pfAffine[2] << " "
                 << tp.m_pfAffine[3] << " " << tp.m_ptOffset.x << " "
                 << tp.m_ptOffset.y << endl;

        }
        sfTP.close();
    }
    else
        return false;

    return true;
}

bool CProcBlock::loadTP(const string strFile, const int* pnIndx, const int nSzIdx){
    // Really terrible code!!! I feel like a git.
    // Please correct this function to make it more efficient!

    loadTP(strFile);

    if ((int)m_vecTPs.size() < nSzIdx || (int)m_vecTPs.size() <= 0) return false;

    vector<CTiePt> vecTP;
    for (int i = 0; i < nSzIdx; i++){
        int nID = pnIndx[i];
        vecTP.push_back(m_vecTPs.at(nID));
    }
//    m_vecTPs.clear();
    m_vecTPs = vecTP;
    return true;
}

bool CProcBlock::loadTP(const string strFile){
  std::cout << "--reading " << strFile << std::endl;
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
        //cerr << "TP should be more than 8 pts" << endl;
        return false;
    }

    return true;
}

bool CProcBlock::saveMatrix(const Mat& matData, const string strFile){

    ofstream sfOut;
    sfOut.open(strFile.c_str());

    if (sfOut.is_open()){

        sfOut << "ncols " << matData.cols << endl;
        sfOut << "nrows " << matData.rows << endl;
        sfOut << "xllcorner 0" << endl;
        sfOut << "yllcorner 0" << endl;
        sfOut << "cellsize 1" << endl;

        for (int i = 0; i < matData.rows; i++){
            for (int j = 0; j < matData.cols; j++){
                if (matData.depth() == CV_32F)
                    sfOut << matData.at<float>(i,j) << " ";
                else if (matData.depth() == CV_64F)
                    sfOut << matData.at<double>(i,j) << " ";
                else if (matData.depth() == CV_8U)
                    sfOut << matData.at<uchar>(i,j) << " ";
            }
            sfOut << endl;
        }
        sfOut.close();
    }
    else
        return false;

    return true;
}

bool CProcBlock::loadMatrix(Mat &matData, const string strFile, bool bDoublePrecision){
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

bool CProcBlock::loadMatrix(Mat &matData, const string strFile){

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

bool CProcBlock::loadMatrix(string strFile){
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


bool CProcBlock::saveALSCParam(const CALSCParam& paramALSC, const string strOut){
    ofstream sfLog;
    sfLog.open(strOut.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){

        sfLog << "<ALSC parameters>" << endl;
        sfLog << "The size of a matching patch: " << paramALSC.m_nPatch << endl;
        sfLog << "Maximum eigenval: " << paramALSC.m_fEigThr << endl;
        sfLog << "Maximum iteration: " << paramALSC.m_nMaxIter << endl;
        sfLog << "Affine distortion limit: " << paramALSC.m_fAffThr << endl;
        sfLog << "Translation limit: " << paramALSC.m_fDriftThr << endl;
        sfLog << "Use intensity offset parameter: " << paramALSC.m_bIntOffset << endl;
        sfLog << "Use weighting coefficients: " << paramALSC.m_bWeighting << endl;
        sfLog << endl;

        sfLog.close();
    }
    else
        return false;

    return true;
}

bool CProcBlock::saveGOTCHAParam(CGOTCHAParam& paramGOTCHA, const string strOut){
    ofstream sfLog;
    sfLog.open(strOut.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){

        sfLog << "<GOTCHA parameters>" << endl;
        sfLog << "Neighbour type: " << paramGOTCHA.getNeiType()<< endl;
        sfLog << "Diffusion iteration: " << paramGOTCHA.m_nDiffIter << endl;
        sfLog << "Diffusion threshold: " << paramGOTCHA.m_fDiffThr << endl;
        sfLog << "Diffusion coefficient: " << paramGOTCHA.m_fDiffCoef << endl;
        //sfLog << "Minimum image tile size: " <<  paramGOTCHA.m_nMinTile << endl;
        sfLog << "Need initial ALSC on seed TPs: " << paramGOTCHA.m_bNeedInitALSC << endl;
        sfLog << endl;

        sfLog.close();
    }
    else
        return false;

    return true;
}
