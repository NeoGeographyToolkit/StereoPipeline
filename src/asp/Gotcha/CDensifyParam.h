#ifndef CDENSIFYPARAM_H
#define CDENSIFYPARAM_H

#include <string>

#include "CGOTCHAParam.h"

using namespace std;

class CDensifyParam {

public:
    CDensifyParam(): m_nProcType(GOTCHA){}
    int m_nProcType;     // growing method
    string m_strImgL;
    string m_strImgR;
    string m_strOutPath;
    string m_strTPFile;
    string m_strDispX;
    string m_strDispY;
    string m_strMask;
    string m_strUpdatedDispX;
    string m_strUpdatedDispY;
    string m_strUpdatedDispSim;

    string getProcessingType(){ if (m_nProcType == GOTCHA) return "GOTCHA";
                                else if (m_nProcType == P_GOTCHA) return "P_GOTCHA";
                                else return "UNKNOWN";}

    CGOTCHAParam m_paramGotcha;

    enum{GOTCHA, P_GOTCHA};
    enum{NO_ERR, FILE_IO_ERR, GOTCHA_ERR, LINE_GOTCHA_ERR, P_GOTCHA_ERR};
};
#endif // CDENSIFYPARAM_H
