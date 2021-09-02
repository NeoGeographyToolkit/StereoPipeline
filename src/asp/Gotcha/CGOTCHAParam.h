#ifndef CGOTCHAPARAM_H
#define CGOTCHAPARAM_H

#include <string>

#include "CALSCParam.h"

using namespace std;

class CGOTCHAParam {

public:
    CGOTCHAParam():m_nNeiType(NEI_4),m_fDiffCoef(0.05),m_fDiffThr(0.1),m_nDiffIter(5), m_bNeedInitALSC(true){ m_nMinTile = 1000000000;}

    string getNeiType(){if (m_nNeiType == NEI_X) return "NEI_X";
                        else if (m_nNeiType == NEI_Y) return "NEI_Y";
                        else if (m_nNeiType == NEI_4) return "NEI_4";
                        else if (m_nNeiType == NEI_8) return "NEI_8";
                        else if (m_nNeiType == NEI_DIFF) return "NEI_Difussion";
                        else return "UNKNOWN";}

    int m_nNeiType;
    int m_nMinTile;
    float m_fDiffCoef;
    float m_fDiffThr;
    int m_nDiffIter;

    string m_strMask;

    CALSCParam m_paramALSC;
    bool m_bNeedInitALSC; // set true if initial alsc on seed points are required

    enum{NEI_X, NEI_Y, NEI_4, NEI_8, NEI_DIFF};
};


#endif // CGOTCHAPARAM_H
