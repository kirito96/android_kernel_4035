
#define LOG_TAG "paramctrl_lifetime"

#ifndef ENABLE_MY_LOG
    #define ENABLE_MY_LOG       (1)
#endif

#include <cutils/properties.h>
#include <aaa_types.h>
#include <aaa_error_code.h>
#include <aaa_log.h>
#include <camera_custom_nvram.h>
#include <awb_param.h>
#include <ae_param.h>
#include <af_param.h>
#include <flash_param.h>
#include <isp_tuning.h>
#include <camera_feature.h>
#include <isp_tuning_cam_info.h>
#include <isp_tuning_idx.h>
#include <isp_tuning_custom.h>
#include <nvram_drv_mgr.h>
#include <ispdrv_mgr.h>
#include <isp_mgr.h>
#include <isp_mgr_helper.h>
#include <tdri_mgr.h>
#include <pca_mgr.h>
#include <dynamic_ccm.h>
#include <ccm_mgr.h>
#include <lsc_mgr.h>
#include <dbg_isp_param.h>
#include <mtkcam/hal/sensor_hal.h>
#include "paramctrl_if.h"
#include "paramctrl.h"
#include "CameraProfile.h"

using namespace NSIspTuning;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
IParamctrl*
IParamctrl::createInstance(ESensorDev_T const eSensorDev)
{
    return Paramctrl::getInstance(eSensorDev);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Paramctrl*
Paramctrl::
getInstance(ESensorDev_T const eSensorDev)
{
    Paramctrl* pParamctrl = MNULL;
    NVRAM_CAMERA_ISP_PARAM_STRUCT*  pNvram_Isp = MNULL;
    SensorHal* pSensorHal = MNULL;

    MY_LOG("%s(): eSensorDev = %d\n", __FUNCTION__, eSensorDev);

    // ISP NVRAM
    if (MERR_OK != NvramDrvMgr::getInstance().init(eSensorDev))
    {
        goto lbExit;
    }

    NvramDrvMgr::getInstance().getRefBuf(pNvram_Isp);

    if (! pNvram_Isp)
    {
        MY_ERR("[createInstance] (pNvram_Isp) = (%p)", pNvram_Isp);
        goto lbExit;
    }

#if 0
    MY_LOG("sizeof(pNvram_Isp->Version) = %d\n", sizeof(pNvram_Isp->Version));
    MY_LOG("sizeof(pNvram_Isp->SensorId) = %d\n", sizeof(pNvram_Isp->SensorId));
    MY_LOG("sizeof(pNvram_Isp->ISPComm) = %d\n", sizeof(pNvram_Isp->ISPComm));
    MY_LOG("sizeof(pNvram_Isp->ISPPca) = %d\n", sizeof(pNvram_Isp->ISPPca));
    MY_LOG("sizeof(pNvram_Isp->ISPRegs) = %d\n", sizeof(pNvram_Isp->ISPRegs));
    MY_LOG("sizeof(pNvram_Isp->ISPMfbMixer) = %d\n", sizeof(pNvram_Isp->ISPMfbMixer));
    MY_LOG("sizeof(pNvram_Isp->ISPCcmPoly22) = %d\n", sizeof(pNvram_Isp->ISPCcmPoly22));
    MY_LOG("sizeof(pNvram_Isp) = %d\n", sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
#endif

    //  Sensor ID
    pSensorHal = SensorHal::createInstance();
    if  ( ! pSensorHal )
    {
        MY_ERR("Cannot create Sensor driver");
        goto lbExit;
    }

    //  Query sensor ID
    MUINT32 u4SensorID;

    switch  ( eSensorDev )
    {
    case ESensorDev_Main:
        pSensorHal->sendCommand(SENSOR_DEV_MAIN, SENSOR_CMD_GET_SENSOR_ID, reinterpret_cast<MINT32>(&u4SensorID), 0, 0);
        break;
    case ESensorDev_Sub:
        pSensorHal->sendCommand(SENSOR_DEV_SUB, SENSOR_CMD_GET_SENSOR_ID, reinterpret_cast<MINT32>(&u4SensorID), 0, 0);
        break;
    case ESensorDev_MainSecond:
        pSensorHal->sendCommand(SENSOR_DEV_MAIN_2, SENSOR_CMD_GET_SENSOR_ID, reinterpret_cast<MINT32>(&u4SensorID), 0, 0);
        break;
    default:    //  Shouldn't happen.
        MY_ERR("Invalid sensor device: %d", eSensorDev);
        goto lbExit;
    }

    //  Here, pIspParam must be legal.
    pParamctrl = new Paramctrl(
        eSensorDev, u4SensorID, pNvram_Isp
    );

lbExit:
    NvramDrvMgr::getInstance().uninit();

    if  ( pSensorHal )
        pSensorHal->destroyInstance();

    return  pParamctrl;
}


void
Paramctrl::
destroyInstance()
{
    delete this;
}


Paramctrl::
Paramctrl(
    ESensorDev_T const eSensorDev,
    MUINT32 const u4SensorID,
    NVRAM_CAMERA_ISP_PARAM_STRUCT*const pNvram_Isp
)
    : IParamctrl(eSensorDev)
    , m_u4ParamChangeCount(0)
    , m_fgDynamicTuning(MTRUE)
    , m_fgDynamicBypass(MFALSE)
    , m_fgDynamicCCM(MTRUE)
    , m_eIdx_Effect(MEFFECT_OFF)
    , m_eSensorDev(eSensorDev)
    , m_eOperMode(EOperMode_Normal)
    , m_eSensorMode(ESensorMode_Capture)
    , m_rIspExifDebugInfo()
    , m_IspUsrSelectLevel()
    , m_rIspCamInfo()
    , m_pIspTuningCustom(IspTuningCustom::createInstance(eSensorDev, u4SensorID))
    , m_rIspParam(*pNvram_Isp)
    , m_rIspComm(m_rIspParam.ISPComm)
    , m_IspNvramMgr(&m_rIspParam.ISPRegs)
    , m_pPcaMgr(PcaMgr::createInstance(eSensorDev, m_rIspParam.ISPPca))
    , m_pCcmMgr(CcmMgr::createInstance(eSensorDev, m_rIspParam.ISPRegs, m_rIspParam.ISPCcmPoly22, m_pIspTuningCustom))
    , m_pLscMgr(LscMgr::createInstance(eSensorDev, m_rIspParam.ISPRegs))
    , m_Lock()
    , m_bDebugEnable(MFALSE)
{
}


Paramctrl::
~Paramctrl()
{

}

MERROR_ENUM
Paramctrl::
init()
{
    MERROR_ENUM err = MERR_OK;

    char value[PROPERTY_VALUE_MAX] = {'\0'};
    property_get("debug.paramctrl.enable", value, "0");
    m_bDebugEnable = atoi(value);

    //  (1) Force to assume all params have chagned and different.
    m_u4ParamChangeCount = 1;

    //  (2) Init ISP/T driver manager.
    IspDrvMgr::getInstance().init();
    TdriMgr::getInstance().init();
    LscMgr::getInstance()->init();

    //  (3) validateFrameless() is invoked
    //  when init() or status change, like Camera Mode.
    err = validateFrameless();
    if  ( MERR_OK != err )
    {
        goto lbExit;
    }

    //  (4) however, is it needed to invoke validatePerFrame() in init()?
    //  or just invoke it only when a frame comes.
    err = validatePerFrame(MTRUE);
    if  ( MERR_OK != err )
    {
        goto lbExit;
    }

lbExit:
    if  ( MERR_OK != err )
    {
        uninit();
    }

    MY_LOG("[-ParamctrlRAW::init]err(%X)", err);
    return  err;
}


MERROR_ENUM
Paramctrl::
uninit()
{
    MY_LOG("[+uninit]");

    //  Uninit ISP driver manager.
    IspDrvMgr::getInstance().uninit();

    TdriMgr::getInstance().uninit();

    LscMgr::getInstance()->uninit();
    return  MERR_OK;
}

