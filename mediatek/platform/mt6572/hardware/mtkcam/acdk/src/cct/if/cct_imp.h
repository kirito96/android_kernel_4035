
#ifndef _CCT_IMP_H_
#define _CCT_IMP_H_

#include "nvram_drv.h"
#include "mtkcam/hal/sensor_hal.h"

class NvramDrvBase;
class SensorDrv;
class IspDrv;
class Hal3ABase;


typedef struct {
	CAMERA_DUAL_CAMERA_SENSOR_ENUM m_sen_type;
	MUINT32 m_sen_id;
}cctctrl_sen_prop_t;

typedef struct {
	NvramDrvBase*	m_pnvramdrv;
	NSNvram::BufIF<NVRAM_CAMERA_ISP_PARAM_STRUCT>*	pbufIf_isp;
    NSNvram::BufIF<NVRAM_CAMERA_SHADING_STRUCT>*	pbufIf_shd;
}cctctrl_nvram_prop_t;

typedef struct {
	IspDrv*	m_pispdrv;
}cctctrl_isp_prop_t;

typedef struct {
	cctctrl_sen_prop_t sen_prop;
	cctctrl_nvram_prop_t nvram_prop;
	cctctrl_isp_prop_t isp_prop;
}cctctrl_prop_t;

class CctCtrl
{
public:
    static CctCtrl* createInstance(const cctctrl_prop_t *prop);
    void            destroyInstance();

protected:
    CctCtrl(
        const cctctrl_prop_t *prop,
        NVRAM_CAMERA_ISP_PARAM_STRUCT*const pBuf_ISP,
        NVRAM_CAMERA_SHADING_STRUCT*const   pBuf_SD
    );
    ~CctCtrl();

public:
    virtual MINT32  cctFeatureCtrl_isp  (MUINT32 const a_u4Ioctl, MUINT8*const puParaIn, MUINT32 const u4ParaInLen, MUINT8*const puParaOut, MUINT32 const u4ParaOutLen, MUINT32*const pu4RealParaOutLen);
    virtual MINT32  cctFeatureCtrl_nvram(MUINT32 const a_u4Ioctl, MUINT8*const puParaIn, MUINT32 const u4ParaInLen, MUINT8*const puParaOut, MUINT32 const u4ParaOutLen, MUINT32*const pu4RealParaOutLen);

protected:
    template <MUINT32 ctl_code>
    MINT32 doCctFeatureCtrl(
        MUINT8*const puParaIn,
        MUINT32 const u4ParaInLen,
        MUINT8* const puParaOut,
        MUINT32 const u4ParaOutLen,
        MUINT32*const pu4RealParaOutLen
    );

#define IMP_CCT_CTRL(ctl_cocde) \
    template <> \
    MINT32 CctCtrl::doCctFeatureCtrl<ctl_cocde>( \
        MUINT8* const puParaIn, \
        MUINT32 const u4ParaInLen, \
        MUINT8* const puParaOut, \
        MUINT32 const u4ParaOutLen, \
        MUINT32*const pu4RealParaOutLen \
    )

private:    //// ISP feature control
    MINT32 CCTOPReadIspReg(MVOID *pCCTIspRegInfoIn, MVOID *pCCTIspRegInfoOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPWriteIspReg(MVOID *pCCTIspRegInfoIn);
    MBOOL  updateIspRegs(MUINT32 const u4Category = 0xFFFFFFFF, MUINT32 const u4Index = 0xFFFFFFFF);

    MVOID   dumpIspReg(MUINT32 const u4Addr) const;

    MINT32  setIspOnOff(MUINT32 const u4Category, MBOOL const fgOn);
    MINT32  getIspOnOff(MUINT32 const u4Category, MBOOL& rfgOn) const;

    MVOID   setIspOnOff_OBC(MBOOL const fgOn);
    MVOID   setIspOnOff_CFA(MBOOL const fgOn);
	MVOID   setIspOnOff_BPC(MBOOL const fgOn);
	MVOID   setIspOnOff_NR1(MBOOL const fgOn);
	MVOID   setIspOnOff_CCM(MBOOL const fgOn);
	MVOID   setIspOnOff_GGM(MBOOL const fgOn);
    MVOID   setIspOnOff_ANR(MBOOL const fgOn);
	MVOID   setIspOnOff_CCR(MBOOL const fgOn);
    MVOID   setIspOnOff_EE(MBOOL const fgOn);
	MVOID   setIspOnOff_NR3D(MBOOL const fgOn);
	MVOID   setIspOnOff_MFB(MBOOL const fgOn);

    MBOOL   getIspOnOff_OBC() const;
	MBOOL   getIspOnOff_BPC() const;
    MBOOL   getIspOnOff_NR1() const;
	MBOOL   getIspOnOff_CFA() const;
	MBOOL   getIspOnOff_CCM() const;
    MBOOL   getIspOnOff_GGM() const;
    MBOOL   getIspOnOff_ANR() const;
	MBOOL   getIspOnOff_CCR() const;
    MBOOL   getIspOnOff_EE() const;
	MBOOL   getIspOnOff_NR3D() const;
    MBOOL   getIspOnOff_MFB() const;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Data Members.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:    ////
    CAMERA_DUAL_CAMERA_SENSOR_ENUM const m_eSensorEnum;
    MUINT32 const   m_u4SensorID;

    NvramDrvBase*   m_pNvramDrv;
    IspDrv*         m_pIspDrv;

private:    ////    NVRAM buffer I/F.
    NSNvram::BufIF<NVRAM_CAMERA_ISP_PARAM_STRUCT>&  m_rBufIf_ISP;//isp
    NSNvram::BufIF<NVRAM_CAMERA_SHADING_STRUCT>&    m_rBufIf_SD; //shading

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ISP
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
private:    ////    NVRAM buffer.

    NVRAM_CAMERA_ISP_PARAM_STRUCT&  m_rBuf_ISP;

    ISP_NVRAM_COMMON_STRUCT&        m_rISPComm;
    ISP_NVRAM_REGISTER_STRUCT&      m_rISPRegs;
    ISP_NVRAM_REG_INDEX_T&          m_rISPRegsIdx;
    ISP_NVRAM_PCA_STRUCT&           m_rISPPca;

private:    ////    OB.

    MBOOL   m_fgEnabled_OB;
    MUINT32 m_u4Backup_OB;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Shading
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
private:    ////    NVRAM buffer.

    NVRAM_CAMERA_SHADING_STRUCT&    m_rBuf_SD;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Defect
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
private:    ////    NVRAM buffer.

};

class CctImp : CCTIF
{
    friend class CCTIF;

protected:
    CctImp();
    virtual ~CctImp();

public:

    virtual void destroyInstance();
    virtual MINT32  init(MINT32 sensorType);
    virtual MINT32  uninit();

public:

    virtual MINT32 nvramCCTFeatureControl (MUINT32 a_u4Ioctl, MUINT8 *puParaIn, MUINT32 u4ParaInLen, MUINT8 *puParaOut, MUINT32 u4ParaOutLen, MUINT32 *pu4RealParaOutLen);
    virtual MINT32 sensorCCTFeatureControl(MUINT32 a_u4Ioctl, MUINT8 *puParaIn, MUINT32 u4ParaInLen, MUINT8 *puParaOut, MUINT32 u4ParaOutLen, MUINT32 *pu4RealParaOutLen);
    virtual MINT32 aaaCCTFeatureControl(MUINT32 a_u4Ioctl, MUINT8 *puParaIn, MUINT32 u4ParaInLen, MUINT8 *puParaOut, MUINT32 u4ParaOutLen, MUINT32 *pu4RealParaOutLen);
    virtual MINT32 ispCCTFeatureControl(MUINT32 a_u4Ioctl, MUINT8 *puParaIn, MUINT32 u4ParaInLen, MUINT8 *puParaOut, MUINT32 u4ParaOutLen, MUINT32 *pu4RealParaOutLen);

private:
    // Sensor feature control
    MINT32 CCTOReadSensorReg(MVOID *puParaIn, MVOID *puParaOut, MUINT32 *pu4RealParaOutLen);
    MINT32 CCTOPWriteSensorReg(MVOID *puParaIn);
    MINT32 CCTOPQuerySensor(MVOID *a_pCCTSensorInfoOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPGetSensorRes(MVOID *pCCTSensorResOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPGetLSCSensorRes(MVOID *pCCTSensorResOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPGetEngSensorGroupCount(MUINT32 *pGoupCntOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPGetEngSensorGroupPara(MUINT32 groupIdx, MVOID *pGroupInfoOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPGetEngSensorPara(MVOID *pSensorItemInfoIn, MVOID *pSensorItemInfoOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPSetEngSensorPara(MVOID *pSensorItemInfoIn);
    MINT32 CCTOPGetSensorPregain(MVOID *pSensorItemInfoIn, MVOID *pSensorItemInfoOut, MUINT32 *pRealParaOutLen);
    MINT32 CCTOPSetSensorPregain(MVOID *pSensorItemInfoIn);
    MINT32 CCTOPGetSensorInfo(MVOID *puParaIn, MVOID *puParaOut, MUINT32 *pu4RealParaOutLen);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Data Members.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:    ////

	SensorHal*      m_pSensorHalObj;
	cctctrl_prop_t	m_cctctrl_prop;
    CctCtrl*        m_pCctCtrl;

};

#endif // _CCT_IMP_H_

