#ifndef CTIEPT_H
#define CTIEPT_H

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class CTiePt {

public:
    CTiePt():m_fSimVal(NOT_DEF){m_pfAffine[0] = 0; m_pfAffine [1] = 0;
                                m_pfAffine[2] = 0; m_pfAffine [3] = 0;}

    Point m_ptIDPair;       // point id pair (leftID, rightID)
    Point2f m_ptL;          // left xy position
    Point2f m_ptR;          // right xy position
    float m_fSimVal;        // matching similarity (score) from 0 to 1.0, othewise it is not defined

    float m_pfAffine [4];
    Point2f m_ptOffset;

    enum {NOT_DEF = -1};

    bool operator==(const CTiePt& x){
        if ((this->m_ptL == x.m_ptL) && (this->m_ptR == x.m_ptR))
            return true;
        else return false;
     }
};

#endif // CTIEPT_H
