#include <boost/filesystem.hpp>

#include "CBatchProc.h"
#include "CDensifyParam.h"
#include "CDensify.h"

namespace fs = boost::filesystem;

CBatchProc::CBatchProc(string strMetaFile, string strLeftImagePath,
                       string strRightImagePath, string strDisparityX,
                       string strDisparityY, string strOutputPrefix) {
  // initialize
  m_strMetaFile = strMetaFile;
  m_strImgL = strLeftImagePath;
  m_strImgR = strRightImagePath;
  m_strDispX = strDisparityX;
  m_strDispY = strDisparityY;
  m_strOutPath = strOutputPrefix;

  if (!validateProjParam()){
    cerr << "ERROR: The project input files cannot be validated" << endl;
    exit(1);
  }
  if (!validateProjInputs()){
    cerr << "ERROR: The project input files not in correct format" << endl;
    exit(1);
  }
}

CBatchProc::~CBatchProc() {
}

bool CBatchProc::validateProjParam() {
  bool bRes = true;

  // image file existence
  if (!fs::exists(m_strImgL)){
    cerr << "ERROR: Left image file does not exist" << endl;
    bRes = false;
  }

  if (!fs::exists(m_strImgR)){
    cerr << "ERROR: Right image file does not exist" << endl;
    bRes = false;
  }


  if (!fs::exists(m_strDispX)){
    cerr << "Gotcha on given disparity map. ERROR: X disparity file does not exist" << endl;
    bRes = false;
  }

  if (!fs::exists(m_strDispY)){
    cerr << "Gotcha on given disparity map. ERROR: Y disparity file does not exist" << endl;
    bRes = false;
  }

  cout << "Generating bad pixel/gap mask file." << endl;
  generateMask();

  return bRes;
}

bool CBatchProc::validateProjInputs() {
  bool bRes = true;

  Mat dispX = imread(m_strDispX, CV_LOAD_IMAGE_ANYDEPTH);
  Mat dispY = imread(m_strDispY, CV_LOAD_IMAGE_ANYDEPTH);
  if ((dispX.depth()!=2 && dispX.depth()!=5)|| (dispY.depth()!=2 && dispY.depth()!=5)){
    bRes = false;
    cerr << "Gotcha on given disparity map. ERROR: Input x/y disparity map not in 16bit unsigned integer or 32bit floating point" << endl;
  }
  if (dispX.channels()!=1 || dispY.channels()!=1){
    bRes = false;
    cerr << "Gotcha on given disparity map. ERROR: Please take single channel image as input x/y disparity map" << endl;
  }

  Mat Mask = imread(m_strMask, CV_LOAD_IMAGE_ANYDEPTH);
  if (Mask.depth()!=0 || Mask.channels()!=1){
    bRes = false;
    cerr << "ERROR: Input mask file not in 8 bit single channel unsigned int format" << endl;
  }
  return bRes;
}

void CBatchProc::doBatchProcessing() {

  cout << "Starting processing now..." << endl
       << "================================" << endl << endl;

  generateTPFile();
  refinement();

  cout << "Process completed" << endl;
  cout << endl;
  cout << "================================" << endl << endl;
}

void CBatchProc::generateMask() {
  string strMask = "-GM.tif";
  m_strMask = m_strOutPath + strMask;


  Mat dispX = imread(m_strDispX, CV_LOAD_IMAGE_ANYDEPTH);
  Mat dispY = imread(m_strDispY, CV_LOAD_IMAGE_ANYDEPTH);
  Mat Mask = Mat::zeros(dispX.size(), CV_8UC1);

  if (dispX.depth()==2 && dispY.depth()==2){
    for (int i=0; i<Mask.rows; i++){
      for (int j=0; j<Mask.cols; j++){
        if (dispX.at<ushort>(i,j)==65535 || dispY.at<ushort>(i,j)==65535)
          Mask.at<uchar>(i,j)=1;
      }
    }
    imwrite(m_strMask, Mask);
  }

  else if (dispX.depth()==5 && dispY.depth()==5){
    int numBadpixel = 0;
    for (int i=0; i<Mask.rows; i++){
      for (int j=0; j<Mask.cols; j++){
        if (dispX.at<float>(i,j)==0.0) { //Nodata value of ASP disparity is 0.
          Mask.at<uchar>(i,j)=1;
          numBadpixel+=1;
        }
      }
    }
    cout << "GAP Pixels masked: " << numBadpixel << endl;
    imwrite(m_strMask, Mask);
  }

}

void CBatchProc::generateTPFile() {
  Mat dispX = imread(m_strDispX, CV_LOAD_IMAGE_ANYDEPTH);
  Mat dispY = imread(m_strDispY, CV_LOAD_IMAGE_ANYDEPTH);
  Mat Mask = imread(m_strMask, CV_LOAD_IMAGE_ANYDEPTH);
  Mask.convertTo(Mask, CV_8UC1);
  string strTPFile = "-TP.txt";
  m_strTPFile = m_strOutPath + strTPFile;

  if (dispX.depth()==2 && dispY.depth()==2){
    dispX.convertTo(dispX, CV_16UC1);
    dispY.convertTo(dispY, CV_16UC1);
    for (int i=0; i<dispX.rows; i++){
      for (int j=0; j<dispX.cols; j++){
        if (Mask.at<uchar>(i,j)==1){
          dispX.at<ushort>(i,j)=65535;
          dispY.at<ushort>(i,j)=65535;
        }
      }
    }
  }

  else if (dispX.depth()==5 && dispY.depth()==5){
    dispX.convertTo(dispX, CV_32FC1);
    dispY.convertTo(dispY, CV_32FC1);
    for (int i=0; i<dispX.rows; i++){
      for (int j=0; j<dispX.cols; j++){
        if (Mask.at<uchar>(i,j)==1){
          dispX.at<float>(i,j)= 0.0;
          dispY.at<float>(i,j)= 0.0;
        }
      }
    }
  }


  if (dispX.depth()==2 && dispY.depth()==2){
    Mat dispX_tmp(dispX.size(),CV_16UC1);
    Mat dispY_tmp(dispY.size(),CV_16UC1);
    dispX.copyTo(dispX_tmp);
    dispY.copyTo(dispY_tmp);
    for (int i=1; i<dispX.rows-1; i++){
      for (int j=1; j<dispX.cols-1; j++){
        if (dispX.at<ushort>(i-1,j-1)!=65535 && dispX.at<ushort>(i-1,j)!=65535 && dispX.at<ushort>(i-1,j+1)!=65535 && dispX.at<ushort>(i,j-1)!=65535 &&
            dispX.at<ushort>(i,j+1)!=65535 && dispX.at<ushort>(i+1,j-1)!=65535 && dispX.at<ushort>(i+1,j)!=65535 && dispX.at<ushort>(i+1,j+1)!=65535 &&
            dispY.at<ushort>(i-1,j-1)!=65535 && dispY.at<ushort>(i-1,j)!=65535 && dispY.at<ushort>(i-1,j+1)!=65535 && dispY.at<ushort>(i,j-1)!=65535 &&
            dispY.at<ushort>(i,j+1)!=65535 && dispY.at<ushort>(i+1,j-1)!=65535 && dispY.at<ushort>(i+1,j)!=65535 && dispY.at<ushort>(i+1,j+1)!=65535){
          dispX_tmp.at<ushort>(i,j)=65535;
          dispY_tmp.at<ushort>(i,j)=65535;
        }
      }
    }
    dispX_tmp.copyTo(dispX);
    dispY_tmp.copyTo(dispY);
  }

  else if (dispX.depth()==5 && dispY.depth()==5){
    Mat dispX_tmp(dispX.size(),CV_32FC1);
    Mat dispY_tmp(dispY.size(),CV_32FC1);
    Mat dispX_tmp2(dispX.size(),CV_32FC1);
    Mat dispY_tmp2(dispY.size(),CV_32FC1);

    // Only leave the gap border pixels to speed-up the process.
    // Should be cancelled if use sGotcha

    dispX.copyTo(dispX_tmp);
    dispY.copyTo(dispY_tmp);
    for (int i=1; i<dispX.rows-1; i++){
      for (int j=1; j<dispX.cols-1; j++){
        if (dispX.at<float>(i-1,j-1)!=0.0 && dispX.at<float>(i-1,j)!=0.0 && dispX.at<float>(i-1,j+1)!=0.0 && dispX.at<float>(i,j-1)!=0.0 &&
            dispX.at<float>(i,j+1)!=0.0 && dispX.at<float>(i+1,j-1)!=0.0 && dispX.at<float>(i+1,j)!=0.0 && dispX.at<float>(i+1,j+1)!=0.0 &&
            dispY.at<float>(i-1,j-1)!=0.0 && dispY.at<float>(i-1,j)!=0.0 && dispY.at<float>(i-1,j+1)!=0.0 && dispY.at<float>(i,j-1)!=0.0 &&
            dispY.at<float>(i,j+1)!=0.0 && dispY.at<float>(i+1,j-1)!=0.0 && dispY.at<float>(i+1,j)!=0.0 && dispY.at<float>(i+1,j+1)!=0.0){
          dispX_tmp.at<float>(i,j)= 0.0;
          dispY_tmp.at<float>(i,j)= 0.0;
        }
      }
    }

    dispX_tmp.copyTo(dispX_tmp2);
    dispY_tmp.copyTo(dispY_tmp2);

    for (int i=1; i<dispX.rows-1; i++){
      for (int j=1; j<dispX.cols-1; j++){
        if (dispX.at<float>(i,j)!=0.0 && dispX_tmp.at<float>(i,j) ==0.0 &&
            (dispX_tmp.at<float>(i-1,j-1)!=0.0 || dispX_tmp.at<float>(i-1,j)!=0.0 ||
             dispX_tmp.at<float>(i-1,j+1)!=0.0 || dispX_tmp.at<float>(i,j-1)!=0.0 ||
             dispX_tmp.at<float>(i,j+1)!=0.0 || dispX_tmp.at<float>(i+1,j-1)!=0.0 ||
             dispX_tmp.at<float>(i+1,j)!=0.0 || dispX_tmp.at<float>(i+1,j+1)!=0.0 ) ){

          dispX_tmp2.at<float>(i,j) = dispX.at<float>(i,j);
          dispY_tmp2.at<float>(i,j) = dispY.at<float>(i,j);
        }

      }
    }

    //double the TPs
    dispX_tmp2.copyTo(dispX);
    dispY_tmp2.copyTo(dispY);
  }


  if (dispX.depth()==2 && dispY.depth()==2){
    dispX.convertTo(dispX, CV_16UC1);
    dispY.convertTo(dispY, CV_16UC1);
    vector<ushort> Lx, Ly;
    vector<float> Rx, Ry;
    for (int i=0; i<dispX.rows; i++){
      for (int j=0; j<dispX.cols; j++){
        if(dispX.at<ushort>(i,j)!=65535 && dispY.at<ushort>(i,j)!=65535){
          unsigned short lx, ly;
          float rx, ry;
          lx = j;
          ly = i;
          rx = j + (dispX.at<ushort>(i,j)-16384)/16.;
          ry = i + (dispY.at<ushort>(i,j)-16384)/16.;
          Lx.push_back(lx);
          Ly.push_back(ly);
          Rx.push_back(rx);
          Ry.push_back(ry);
        }
      }
    }

    ofstream sfTP;
    sfTP.open(m_strTPFile.c_str());
    sfTP.precision(10);
    int nLen = Lx.size();
    int nEle = 5;
    if (sfTP.is_open()){
      sfTP << nLen << " " << nEle << endl;
      for (int i = 0 ; i < nLen ;i++){
        unsigned short lx, ly;
        float rx, ry;
        lx = Lx.at(i);
        ly = Ly.at(i);
        rx = Rx.at(i);
        ry = Ry.at(i);
        sfTP << lx << " " << ly << " "
             << rx << " " << ry << " "
             << 0.5 << " " << endl;
      }
      sfTP.close();
    }
    else {
      cout << "ERROR: Can not convert X/Y disparity map to TP file. "
           << "Please check the directory for storing Gotcha results is writable."
           << endl;
    }
  } else if (dispX.depth()==5 && dispY.depth()==5){
    dispX.convertTo(dispX, CV_32FC1);
    dispY.convertTo(dispY, CV_32FC1);
    vector<ushort> Lx, Ly;
    vector<float> Rx, Ry;
    for (int i=0; i<dispX.rows; i++){
      for (int j=0; j<dispX.cols; j++){
        if(dispX.at<float>(i,j)!= 0.0 && dispY.at<float>(i,j)!=0.0){

          //nodata should be -3.40282346639e+038, but ASP disparity map uses 0 as nodata
          unsigned short lx, ly;
          float rx, ry;
          lx = j;
          ly = i;
          rx = j + dispX.at<float>(i,j);
          ry = i + dispY.at<float>(i,j);
          Lx.push_back(lx);
          Ly.push_back(ly);
          Rx.push_back(rx);
          Ry.push_back(ry);
        }
      }
    }

    ofstream sfTP;
    sfTP.open(m_strTPFile.c_str());
    sfTP.precision(10);
    int nLen = Lx.size();
    int nEle = 5;
    if (sfTP.is_open()){
      sfTP << nLen << " " << nEle << endl;
      for (int i = 0 ; i < nLen ;i++){
        unsigned short lx, ly;
        float rx, ry;
        lx = Lx.at(i);
        ly = Ly.at(i);
        rx = Rx.at(i);
        ry = Ry.at(i);
        sfTP << lx << " " << ly << " "
             << rx << " " << ry << " "
             << 0.5 << " " << endl;
      }
      sfTP.close();
    }
    else
      cout << "ERROR: Can not convert X/Y disparity map to TP file. Please check the directory for storing Gotcha results is writable." << endl;
  }

}

void CBatchProc::refinement() {
  cout << "Gotcha densification based on existing disparity map:" << endl;
  FileStorage fs(m_strMetaFile, FileStorage::READ);
  FileNode tl = fs["sGotchaParam"];

  // read GOTCHA param
  CDensifyParam paramDense;
  paramDense.m_nProcType = 0;
  paramDense.m_paramGotcha.m_fDiffCoef = 0.05;
  paramDense.m_paramGotcha.m_fDiffThr = 0.1;
  paramDense.m_paramGotcha.m_nDiffIter = 5;
  //paramDense.m_paramGotcha.m_nMinTile = (int)tl["nMinTile"];
  Mat matDummy = imread(m_strImgL, CV_LOAD_IMAGE_ANYDEPTH);
  paramDense.m_paramGotcha.m_nMinTile = matDummy.cols+matDummy.rows;
  paramDense.m_paramGotcha.m_nNeiType = (int)tl["nNeiType"];

  paramDense.m_paramGotcha.m_paramALSC.m_bIntOffset = (int)tl["bIntOffset"];
  paramDense.m_paramGotcha.m_paramALSC.m_bWeighting = (int)tl["bWeight"];
  paramDense.m_paramGotcha.m_paramALSC.m_fAffThr = (float)tl["fAff"];
  paramDense.m_paramGotcha.m_paramALSC.m_fDriftThr = (float)tl["fDrift"];
  paramDense.m_paramGotcha.m_paramALSC.m_fEigThr = (float)tl["fMaxEigenValue"];
  paramDense.m_paramGotcha.m_paramALSC.m_nMaxIter = (int)tl["nALSCIteration"];
  paramDense.m_paramGotcha.m_paramALSC.m_nPatch = (int)tl["nALSCKernel"];

  string strBase = m_strOutPath;
  string strTPFile = "-TP.txt";
  paramDense.m_strTPFile = strBase + strTPFile;
  paramDense.m_paramGotcha.m_strMask = m_strMask;
  string strUpdatedDispX = "-c1_refined.txt";
  paramDense.m_strUpdatedDispX = strBase + strUpdatedDispX;
  string strUpdatedDispY = "-c2_refined.txt";
  paramDense.m_strUpdatedDispY = strBase + strUpdatedDispY;
  string strUpdatedDispSim = "-uncertainty.txt";
  paramDense.m_strUpdatedDispSim = strBase + strUpdatedDispSim;

  paramDense.m_strImgL = m_strImgL;
  paramDense.m_strImgR = m_strImgR;
  paramDense.m_strOutPath = m_strOutPath;
  paramDense.m_strDispX = m_strDispX;
  paramDense.m_strDispY = m_strDispY;
  paramDense.m_strMask = m_strMask;

  CDensify densify(paramDense);
  cout << "CASP-GO INFO: performing Gotcha densification" << endl;
  int nErrCode = densify.performDensitification();
  if (nErrCode != CDensifyParam::NO_ERR){
    cerr << "Warning: Processing error on densifying operation (ERROR CODE: " << nErrCode << " )" << endl;
  }
}


Point3f CBatchProc::rotate(Point3f ptIn, Mat matQ, bool bInverse){
  // this believe matQ is a unit vector
  Point3f ptRes;

  float p[4] = {0, ptIn.x, ptIn.y, ptIn.z};
  float q[4];
  float q_inv[4];
  // get quaternion vector
  for(int i = 0; i < 4; i++) {
    q[i] = matQ.at<float>(i,0);
    q_inv[i] = -1 * matQ.at<float>(i,0);
  }
  q_inv[0] = -1 * q_inv[0]; // q = [w x y z] and q^-1 = [w -x -y -z]

  // compute rotation
  // [0 v'] = q[0 v]q^-1
  float pfTemp[4] = {0.f,};
  float pfTemp2[4] = {0.f,};
  if (bInverse){
    quaternionMultiplication(q_inv, p, pfTemp);
    quaternionMultiplication(pfTemp, q, pfTemp2);
  }
  else{
    quaternionMultiplication(q, p, pfTemp);
    quaternionMultiplication(pfTemp, q_inv, pfTemp2);
  }

  ptRes.x = pfTemp2[1];
  ptRes.y = pfTemp2[2];
  ptRes.z = pfTemp2[3];

  return ptRes;
}

void CBatchProc::quaternionMultiplication(const float* p, const float* q, float* pfOut){
  // compute p*q = pfOut
  //    x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
  //    y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
  //    z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
  //    w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
  // float a = p[0], b = p[1], c = p[2], d =p[3];
  pfOut[0] = p[0]*q[0] - (p[1]*q[1]+p[2]*q[2]+p[3]*q[3]); // w1*w2-v1.v2;
  pfOut[1] = p[1]*q[0] + p[2]*q[3] - p[3]*q[2] + p[0]*q[1];
  pfOut[2] = -1*p[1]*q[3] + p[2]*q[0] + p[3]*q[1] + p[0]*q[2];
  pfOut[3] = p[1]*q[2] - p[2]*q[1] + p[3]*q[0] + p[0]*q[3];
}
