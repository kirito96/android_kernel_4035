


#ifndef _AWB_MGR_H_
#define _AWB_MGR_H_

#include <awb_feature.h>

namespace NS3A
{

typedef struct
{
  MUINT16 u2SensorPreviewWidth;
  MUINT16 u2SensorPreviewHeight;
  MUINT16 u2SensorFullWidth;
  MUINT16 u2SensorFullHeight;
  MUINT16 u2SensorVideoWidth;
  MUINT16 u2SensorVideoHeight;
} SENSOR_RESOLUTION_INFO_T;

class IAwbAlgo;

class AwbMgr
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Ctor/Dtor.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:    ////    Disallowed.
    //  Copy constructor is disallowed.
    AwbMgr(AwbMgr const&);
    //  Copy-assignment operator is disallowed.
    AwbMgr& operator=(AwbMgr const&);

public:  ////
    AwbMgr();
    ~AwbMgr();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Operations.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:
    static AwbMgr& getInstance();
    MRESULT cameraPreviewInit(MINT32 i4SensorDev, Param_T &rParam);
    MRESULT camcorderPreviewInit(MINT32 i4SensorDev, Param_T &rParam);
    MRESULT cameraCaptureInit();
    MRESULT cameraPreviewReinit();
    MRESULT uninit();

    inline MBOOL isAWBLockSupported()
    {
        return MTRUE;
    }

    inline MBOOL isAWBEnable()
    {
        return m_bEnableAWB;
    }

    MRESULT setAWBMode(MINT32 i4NewAWBMode);
    MINT32 getAWBMode() const;
    MRESULT setStrobeMode(MINT32 i4NewStrobeMode);
    MINT32 getStrobeMode() const;
    MRESULT setAWBLock(MBOOL bAWBLock);
    MRESULT enableAWB();
    MRESULT disableAWB();
    MRESULT doPvAWB(MINT32 i4FrameCount, MBOOL bAEStable, MINT32 i4SceneLV, MVOID *pAWBStatBuf);
    MRESULT doAFAWB(MVOID *pAWBStatBuf);
    MRESULT doPreCapAWB(MINT32 i4SceneLV, MVOID *pAWBStatBuf);
    MRESULT doCapAWB(MINT32 i4SceneLV, MVOID *pAWBStatBuf);
    MRESULT getDebugInfo(AWB_DEBUG_INFO_T &rAWBDebugInfo, AWB_DEBUG_DATA_T &rAWBDebugData);
    MINT32 getAWBCCT();
    MRESULT getASDInfo(AWB_ASD_INFO_T &a_rAWBASDInfo);
    MRESULT getAWBOutput(AWB_OUTPUT_T &a_rAWBOutput);

    inline MVOID setAFLV(MINT32 i4AFLV)
    {
         m_i4AFLV = i4AFLV;
    }

    inline MINT32 getAFLV()
    {
         return m_i4AFLV;
    }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Private function
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:
    MRESULT getSensorResolution();
    MRESULT getNvramData();
    MRESULT AWBInit(Param_T &rParam);
    MRESULT getEEPROMData();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  CCT feature
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:
    MRESULT CCTOPAWBEnable();
    MRESULT CCTOPAWBDisable();
    MRESULT CCTOPAWBGetEnableInfo(MINT32 *a_pEnableAWB,MUINT32 *a_pOutLen);
    MRESULT CCTOPAWBGetAWBGain(MVOID *a_pAWBGain, MUINT32 *a_pOutLen);
    MRESULT CCTOPAWBSetAWBGain(MVOID *a_pAWBGain);
    MRESULT CCTOPAWBApplyNVRAMParam(MVOID *a_pAWBNVRAM);
    MRESULT CCTOPAWBGetNVRAMParam(MVOID *a_pAWBNVRAM, MUINT32 *a_pOutLen);
    MRESULT CCTOPAWBSaveNVRAMParam();
    MRESULT CCTOPAWBSetAWBMode(MINT32 a_AWBMode);
    MRESULT CCTOPAWBGetAWBMode(MINT32 *a_pAWBMode, MUINT32 *a_pOutLen);
    MRESULT CCTOPAWBGetLightProb(MVOID *a_pAWBLightProb, MUINT32 *a_pOutLen);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Data member
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:
    IAwbAlgo* m_pIAwbAlgo;
    LIB3A_AWB_MODE_T m_eAWBMode;
    MINT32 m_i4SensorMode;
    MINT32 m_i4StrobeMode;
    MBOOL m_bEnableAWB;
    MBOOL m_bAWBLock;
    MBOOL m_bOneShotAWB;
    MBOOL m_bAWBModeChanged;
    MBOOL m_bStrobeModeChanged;
    MINT32 const* m_pIsAWBActive;
    MINT32 const m_i4AWBCycleNum;
    MINT32 m_i4SensorDev;
    MBOOL m_bDebugEnable;
    MBOOL m_bInitState;
    MINT32 m_i4AFLV;
};

};  //  namespace NS3A
#endif // _AWB_MGR_H_

