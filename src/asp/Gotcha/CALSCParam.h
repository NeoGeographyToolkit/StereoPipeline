#ifndef CALSCPARAM_H
#define CALSCPARAM_H

class CALSCParam {

public:
    CALSCParam():m_nMaxIter(10),m_nPatch(30),m_fEigThr(150.f),m_fAffThr(1.5f),m_fDriftThr(0.8f),
                 m_bWeighting(false),m_bIntOffset(true){}

    // matching param
    int m_nMaxIter; // the max num of iterations
    int m_nPatch;   // the size of a matching patch
    float m_fEigThr;
    float m_fAffThr;
    float m_fDriftThr;
    bool m_bWeighting;
    bool m_bIntOffset;
};

#endif // CALSCPARAM_H

