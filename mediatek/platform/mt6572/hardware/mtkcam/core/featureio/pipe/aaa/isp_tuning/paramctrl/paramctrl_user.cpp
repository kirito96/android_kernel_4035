
#define LOG_TAG "paramctrl_user"

#ifndef ENABLE_MY_LOG
    #define ENABLE_MY_LOG       (1)
#endif

#include <aaa_types.h>
#include <aaa_log.h>
#include <mtkcam/hal/aaa_hal_base.h>
#include <camera_custom_nvram.h>
#include <isp_tuning.h>
#include <camera_feature.h>
#include <awb_param.h>
#include <ae_param.h>
#include <af_param.h>
#include <flash_param.h>
#include <isp_tuning_cam_info.h>
#include <isp_tuning_idx.h>
#include <isp_tuning_custom.h>
#include <lsc_mgr.h>
#include <dbg_isp_param.h>
#include "paramctrl_if.h"
#include "paramctrl.h"

using namespace android;
using namespace NSIspTuning;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
setIspUserIdx_Edge(EIndex_Isp_Edge_T const eIndex)
{
    Mutex::Autolock lock(m_Lock);

    MY_LOG_IF(m_bDebugEnable, "[+setIspUserIdx_Edge] (old, new)=(%d, %d)", m_IspUsrSelectLevel.eIdx_Edge, eIndex);

    if  ( checkParamChange(m_IspUsrSelectLevel.eIdx_Edge, eIndex) )
        m_IspUsrSelectLevel.eIdx_Edge = eIndex;

    return  MERR_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
setIspUserIdx_Hue(EIndex_Isp_Hue_T const eIndex)
{
    Mutex::Autolock lock(m_Lock);

    MY_LOG_IF(m_bDebugEnable, "[+setIspUserIdx_Hue] (old, new)=(%d, %d)", m_IspUsrSelectLevel.eIdx_Hue, eIndex);

    if  ( checkParamChange(m_IspUsrSelectLevel.eIdx_Hue, eIndex) )
        m_IspUsrSelectLevel.eIdx_Hue = eIndex;

    return  MERR_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
setIspUserIdx_Sat(EIndex_Isp_Saturation_T const eIndex)
{
    Mutex::Autolock lock(m_Lock);

    MY_LOG_IF(m_bDebugEnable, "[+setIspUserIdx_Sat] (old, new)=(%d, %d)", m_IspUsrSelectLevel.eIdx_Sat, eIndex);

    if  ( checkParamChange(m_IspUsrSelectLevel.eIdx_Sat, eIndex) )
        m_IspUsrSelectLevel.eIdx_Sat = eIndex;

    return  MERR_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
setIspUserIdx_Bright(EIndex_Isp_Brightness_T const eIndex)
{
    Mutex::Autolock lock(m_Lock);

    MY_LOG_IF(m_bDebugEnable, "[+setIspUserIdx_Bright] (old, new)=(%d, %d)", m_IspUsrSelectLevel.eIdx_Bright, eIndex);

    if  ( checkParamChange(m_IspUsrSelectLevel.eIdx_Bright, eIndex) )
        m_IspUsrSelectLevel.eIdx_Bright = eIndex;

    return  MERR_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
setIspUserIdx_Contrast(EIndex_Isp_Contrast_T const eIndex)
{
    Mutex::Autolock lock(m_Lock);

    MY_LOG_IF(m_bDebugEnable, "[+setIspUserIdx_Contrast] (old, new)=(%d, %d)", m_IspUsrSelectLevel.eIdx_Contrast, eIndex);

    if  ( checkParamChange(m_IspUsrSelectLevel.eIdx_Contrast, eIndex) )
        m_IspUsrSelectLevel.eIdx_Contrast = eIndex;

    return  MERR_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if 0
MBOOL
Paramctrl::
prepareHw_PerFrame_IspUserIndex()
{
    MBOOL fgRet = MFALSE;

    //  (0) Invoked only when Normal Operation Mode.
    if  ( EOperMode_Normal != getOperMode() )
    {
        fgRet = MTRUE;
        goto lbExit;
    }

    //  Sharpness
    {
        //  (a) Customize the nvram index based on the user setting.
        MUINT8 const u8Idx_EE = m_pIspTuningCustom->
            map_user_setting_to_nvram_index<ISP_NVRAM_EE_T>(
                m_IspNvramMgr.getIdx_EE(),    // The current nvram index.
                getIspUsrSelectLevel()      // Get the user setting.
            );
        //  (b) Overwrite the params member.
        fgRet = m_IspNvramMgr.setIdx_EE(u8Idx_EE);
        if  ( ! fgRet )
        {
            MY_ERR(
                "[ERROR][prepareHw_PerFrame_IspUserIndex]"
                "setIdx_EE: bad idx(%d)", u8Idx_EE
            );
            goto lbExit;
        }
    }

    MY_LOG(
        "[prepareHw_PerFrame_IspUserIndex](ee)=(%d)", m_IspNvramMgr.getIdx_EE()
    );


    fgRet = MTRUE;

lbExit:
    return  fgRet;
}
#endif

