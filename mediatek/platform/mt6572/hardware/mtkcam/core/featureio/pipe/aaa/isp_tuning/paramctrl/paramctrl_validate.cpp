
#define LOG_TAG "paramctrl_validate"

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
#include <tdri_mgr.h>
#include "paramctrl_if.h"
#include "paramctrl.h"
#include "CameraProfile.h"

using namespace android;
using namespace NSIspTuning;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
validate(MBOOL const fgForce)
{
    MERROR_ENUM err = MERR_UNKNOWN;

    // flush turning setting
    switch(m_rIspCamInfo.eIspProfile) {
        case EIspProfile_VideoCapture:
            TdriMgr::getInstance().flushSetting(ISP_DRV_CQ02_SYNC);
            break;
        case EIspProfile_NormalPreview:
        case EIspProfile_ZsdPreview_CC:
        case EIspProfile_ZsdPreview_NCC:
        case EIspProfile_NormalCapture:
        case EIspProfile_VideoPreview:
            TdriMgr::getInstance().flushSetting(ISP_DRV_CQ01_SYNC);
            break;
        default:
            break;
    }


    MBOOL const fgRet = ( MERR_OK == (err = validateFrameless()) )
                    &&  ( MERR_OK == (err = validatePerFrame(fgForce)) )
                        ;

    return  err;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
validateFrameless()
{
    MERROR_ENUM err = MERR_UNKNOWN;

    MY_LOG("[+validateFrameless]");

    Mutex::Autolock lock(m_Lock);

#if 0 // TODO: check if necessary
    //  (1) reinit isp driver manager
    if  ( IspDrvMgr::MERR_OK != IspDrvMgr::getInstance().reinit() )
    {
        err = MERR_BAD_ISP_DRV;
        goto lbExit;
    }
#endif

    //  (2)
    if  ( ! prepareHw_Frameless_All() )
    {
        err = MERR_PREPARE_HW;
        goto lbExit;
    }

    //  (3)
    if  ( ! applyToHw_Frameless_All() )
    {
        err = MERR_APPLY_TO_HW;
        goto lbExit;
    }

    //  (4) Force validatePerFrame() to run.
    m_u4ParamChangeCount++;

    err = MERR_OK;

lbExit:
#if ENABLE_MY_ERR
    if  ( MERR_OK != err )
    {
        MY_ERR("[-validateFrameless]err(%X)", err);
    }
#endif

    return  err;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
validatePerFrame(MBOOL const fgForce)
{
    MERROR_ENUM err = MERR_UNKNOWN;

    MY_LOG_IF(m_bDebugEnable, "[validatePerFrame]");

    Mutex::Autolock lock(m_Lock);

#if 0
    if(getOperMode() == EOperMode_Meta) {
        err = MERR_OK;
        MY_LOG("[validatePerFrame] Meta Mode\n");
        goto lbExit;
    }
#endif

    //  (0) Make sure it's really needed to apply.
    if  ( 0 == getParamChangeCount()  //  no params change
          && !fgForce)                //  not force to apply
    {
        err = MERR_OK;
        goto lbExit;
    }

    MY_LOG_IF(m_bDebugEnable, "[validatePerFrame](ParamChangeCount, fgForce)=(%d, %d)", getParamChangeCount(), fgForce);

    //  (1) Do something.
    err = do_validatePerFrame();
    if  (MERR_OK != err)
    {
        MY_ERR("[validatePerFrame]do_validatePerFrame returns err(%d)", err);
        goto lbExit;
    }

    //  (2) reset to 0 since all params have been applied.
    resetParamChangeCount();

    err = MERR_OK;

lbExit:

    return  err;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MERROR_ENUM
Paramctrl::
do_validatePerFrame()
{
    MERROR_ENUM err = MERR_OK;

    MY_LOG_IF(m_bDebugEnable, "[do_validatePerFrame]");


    //  (1) dynamic tuning
    if (isDynamicTuning())
    {

        IndexMgr idxmgr;

        //  a) Get default index setting.
        INDEX_T const*const pDefaultIndex = m_pIspTuningCustom->getDefaultIndex(
            m_rIspCamInfo.eIspProfile, m_rIspCamInfo.eIdx_Scene, m_rIspCamInfo.eIdx_ISO
        );
        if  ( ! pDefaultIndex )
        {
            MY_ERR("[ERROR][validatePerFrame]pDefaultIndex==NULL");
            err = MERR_CUSTOM_DEFAULT_INDEX_NOT_FOUND;
            goto lbExit;
        }
        idxmgr = *pDefaultIndex;

        MY_LOG_IF(m_bDebugEnable, "[BEFORE][evaluate_nvram_index]");

        if (m_bDebugEnable) {
            idxmgr.dump();
        }

        //  b) Customize the index setting.
        m_pIspTuningCustom->evaluate_nvram_index(m_rIspCamInfo, idxmgr);

        MY_LOG_IF(m_bDebugEnable, "[AFTER][evaluate_nvram_index]");

        if (m_bDebugEnable) {
            idxmgr.dump();
        }

        //  c) Restore customized index set to member.
        m_IspNvramMgr = idxmgr;

    }


    //  (2) Apply Per-Frame Parameters.
    if  (
            ! prepareHw_PerFrame_All()          //  Prepare param members to the ispmgr's buffer.
        ||  ! applyToHw_PerFrame_All()          //  Apply the ispmgr's buffer to H/W.
        )
    {
        err = MERR_SET_ISP_REG;
        goto lbExit;
    }

    //  (3) Save Exif debug info if necessary.
    err = saveDebugInfo();
    if  ( MERR_OK != err )
    {
        goto lbExit;
    }

lbExit:
    if  ( MERR_OK != err )
    {
        MY_ERR("[-do_validatePerFrame]err(%X)", err);
    }

    return  err;
}




#if 0

MERROR_ENUM
ParamctrlComm::
setEnable_Meta_Gamma(MBOOL const fgForceEnable)
{
    MY_LOG(
        "[+setEnable_Meta_Gamma] (fgForceEnable, m_fgForceEnable_Meta_Gamma)=(%d, %d)"
        , fgForceEnable, m_fgForceEnable_Meta_Gamma
    );

    Mutex::Autolock lock(m_Lock);

    checkParamChange(m_fgForceEnable_Meta_Gamma, fgForceEnable);

    m_fgForceEnable_Meta_Gamma = fgForceEnable;

    return  MERR_OK;
}

#endif

