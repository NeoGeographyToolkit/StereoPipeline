#ifndef CDENSIFY_H
#define CDENSIFY_H

#include <iostream>
#include <fstream>
#include <ostream>
#include <iostream>
#include <ostream>
#include <termios.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "CProcBlock.h"
#include "CDensifyParam.h"

#define GOTCHA_SAVE_AUX_FILES 1

using namespace cv;
using namespace std;

class CDensify: public CProcBlock
{
public:
    CDensify();
    CDensify(CDensifyParam paramDense, std::vector<CTiePt> const& vecTPs);

    void setParameters(CDensifyParam paramDense, std::vector<CTiePt> const& vecTPs);    
    int performDensitification();
    int getNumTotTps() const {return m_vectpAdded.size();}
    static bool compareTP(CTiePt tpX, CTiePt tpY){
        return (tpX.m_fSimVal < tpY.m_fSimVal);
    }

    string getProcType(){
        return "GOTCHA_FROM_DISP";
    }
private:
    void loadImages();
    vector<CTiePt> getIntToFloatSeed(vector<CTiePt>& vecTPSrc); // get integer Seed point pairs from a float seed point pair
    bool doGotcha(const Mat& matImgL, const Mat& matImgR, vector<CTiePt>& vectpSeeds,
                  const CGOTCHAParam& paramGotcha, vector<CTiePt>& vectpAdded);
    bool doTileGotcha(const Mat& matImgL, const Mat& matImgR, const vector<CTiePt>& vectpSeeds,
                      const CGOTCHAParam& paramGotcha, vector<CTiePt>& mvectpAdded,
                      const Rect_<float> rectTileL, Mat& matSimMap, vector<bool>& pLUT); //IMARS
    void removePtInLUT(vector<CTiePt>& vecNeiTp, const vector<bool>& pLUT, const int nWidth); //IMARS
    void removeOutsideImage(vector<CTiePt>& vecNeiTp, const Rect_<float> rectTileL, const Rect_<float> rectImgR);
    void getNeighbour(const CTiePt tp, vector<CTiePt>& vecNeiTp, const int nNeiType, const Mat& matSim);
    void getDisffusedNei(vector<CTiePt>& vecNeiTp, const CTiePt tp, const Mat& matSim);
    void breakIntoSubRect(Rect_<float> rectParent, vector< Rect_<float> >& vecRes);
    void makeTiles(vector< Rect_<float> >& vecRectTiles, int nMin);
    bool isHavingTP(vector<CTiePt>& vecNeiTp, CTiePt tp); // used in diffused neighbour
    bool doPGotcha(int nNeiType);
    int getTotPyramidLev(int nszPatch);
    void makeDataProducts();
    bool saveResult();
    bool saveProjLog (string strFile);
    bool saveLog();
    bool saveResLog(string strFile);
    bool loadTPForDensification(string strTPFile);

private:
    // inputs
    CDensifyParam m_paramDense;
    // outputs
    vector<CTiePt> m_vectpAdded; // final tp result (i.e. seed tps + newly added tps)
    // Disparity maps:
    // the disparity of a point p_i at x,y in the left image is stored at
    // (x, y) in the disparity maps, e.g., (m_matDisMapX and m_matDisMapY).
    // For example, the position of the corresponding point p'_i in the right image can be defined as
    // (x+x_dist, y+y_dist), where 'xdist' and 'ydist' are value at (x,y) in m_matDisMapX and m_matDisMapY, respectively.
    // When disparity values are not known, the vaule of the disparity map is set to zero.
    Mat m_matDisMapX;
    Mat m_matDisMapY;
    Mat m_matDisMapSim; // matching score map: smaller score is better score and -1 indicate unknown

    Mat m_matHL;        // used when the rectified images are used
    Mat m_matHR;
    int nNumSeedTPs;
    double procTime;
};

#endif // CDENSIFY_H
