
#define LOG_TAG "aaa_state_camcorder_preview"

#ifndef ENABLE_MY_LOG
    #define ENABLE_MY_LOG       (0)
#endif

#include <aaa_types.h>
#include <aaa_error_code.h>
#include <aaa_log.h>
#include <dbg_aaa_param.h>
#include <aaa_hal.h>
#include "aaa_state.h"
#include <camera_custom_nvram.h>
#include <awb_param.h>
#include <awb_mgr.h>
#include <buf_mgr.h>
#include <mtkcam/hal/sensor_hal.h>
#include <af_param.h>
#include <mcu_drv.h>
#include <isp_reg.h>
#include <af_mgr.h>
#include <ae_param.h>
#include <mtkcam/common.h>
using namespace NSCam;
#include <ae_mgr.h>
#include <flash_mgr.h>
#include <dbg_isp_param.h>
#include <isp_mgr.h>
#include <isp_tuning_mgr.h>
#include <lsc_mgr.h>
#include "CameraProfile.h"  // For CPTLog*()/AutoCPTLog class.


using namespace NS3A;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StateCameraPreview
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StateCamcorderPreview::
StateCamcorderPreview()
    : IState("StateCamcorderPreview")
{
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_Uninit
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_Uninit>)
{
    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_Uninit>");

    // AE uninit
    AeMgr::getInstance().uninit();

    // AWB uninit
    AwbMgr::getInstance().uninit();

    // AAO DMA buffer uninit
    BufMgr::getInstance().uninit();

    // AF uninit
    AfMgr::getInstance().uninit();

    // State transition: eState_CamcorderPreview --> eState_Uninit
    transitState(eState_CamcorderPreview, eState_Uninit);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_CamcorderPreviewStart
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_CamcorderPreviewStart>)
{
    MRESULT err;

    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_CamcorderPreviewStart>");

    // AAO DMA / state enable again
    err = BufMgr::getInstance().DMAInit(camdma2type<ECamDMA_AAO>());
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().DMAInit(ECamDMA_AAO) fail\n");
        return err;
    }

    err = BufMgr::getInstance().AAStatEnable(MTRUE);
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().AAStatEnable(MTRUE) fail\n");
        return err;
    }

    // AFO DMA / state enable again
    err = BufMgr::getInstance().DMAInit(camdma2type<ECamDMA_AFO>());
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().DMAInit(ECamDMA_AFO) fail\n");
        return err;
    }

    err = BufMgr::getInstance().AFStatEnable(MTRUE);
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().AFStatEnable(MTRUE) fail\n");
        return err;
    }

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_CamcorderPreviewEnd
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_CamcorderPreviewEnd>)
{
    MRESULT err;

    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_CamcorderPreviewEnd>");

    // AE uninit
    AeMgr::getInstance().uninit();

    // AWB uninit
    AwbMgr::getInstance().uninit();

    // AF uninit
    AfMgr::getInstance().uninit();

    // AAO DMA / state disable again
    err = BufMgr::getInstance().AAStatEnable(MFALSE);
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().AAStatEnable(MFALSE) fail\n");
        return err;
    }

    err = BufMgr::getInstance().DMAUninit(camdma2type<ECamDMA_AAO>());
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().DMAunInit(ECamDMA_AAO) fail\n");
        return err;
    }

    // AFO DMA / state disable again
    err = BufMgr::getInstance().AFStatEnable(MFALSE);
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().AFStatEnable(MFALSE) fail\n");
        return err;
    }

    err = BufMgr::getInstance().DMAUninit(camdma2type<ECamDMA_AFO>());
    if (FAILED(err)) {
        MY_ERR("BufMgr::getInstance().DMAunInit(ECamDMA_AFO) fail\n");
        return err;
    }

    // State transition: eState_CamcorderPreview --> eState_Init
    transitState(eState_CamcorderPreview, eState_Init);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_VsyncUpdate
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_VsyncUpdate>)
{
    MRESULT err;
    BufInfo_T rBufInfo;

    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_VsyncUpdate>");

    // Update frame count
    updateFrameCount();

    if (getFrameCount() < 0) {// AA statistics is not ready
        // Update AAO DMA base address for next frame
        err = BufMgr::getInstance().updateDMABaseAddr(camdma2type<ECamDMA_AAO>(), BufMgr::getInstance().getNextHwBuf(ECamDMA_AAO));
        IspDebug::getInstance().dumpIspDebugMessage();
        return err;
    }

    // Dequeue AAO DMA buffer
    BufMgr::getInstance().dequeueHwBuf(ECamDMA_AAO, rBufInfo);

    //MTK_SWIP_PROJECT_START
    // F858
    NSIspTuning::LscMgr::getInstance()->updateTSFinput(
            static_cast<NSIspTuning::LscMgr::LSCMGR_TSF_INPUT_SRC>(NSIspTuning::LscMgr::TSF_INPUT_PV),
            AeMgr::getInstance().getLVvalue(), AwbMgr::getInstance().getAWBCCT(),
            reinterpret_cast<MVOID *>(rBufInfo.virtAddr));
    //MTK_SWIP_PROJECT_END

    // AWB
    MINT32 i4SceneLv = AeMgr::getInstance().getLVvalue();
    CPTLog(Event_Pipe_3A_AWB, CPTFlagStart);    // Profiling Start.
    AwbMgr::getInstance().doPvAWB(getFrameCount(), AeMgr::getInstance().IsAEStable(), i4SceneLv, reinterpret_cast<MVOID *>(rBufInfo.virtAddr));
    CPTLog(Event_Pipe_3A_AWB, CPTFlagEnd);     // Profiling End.

    // AE
    CPTLog(Event_Pipe_3A_AE, CPTFlagStart);    // Profiling Start.
    AeMgr::getInstance().doPvAE(reinterpret_cast<MVOID *>(rBufInfo.virtAddr));
    CPTLog(Event_Pipe_3A_AE, CPTFlagEnd);     // Profiling End.

    // Enqueue AAO DMA buffer
    BufMgr::getInstance().enqueueHwBuf(ECamDMA_AAO, rBufInfo);

    // Update AAO DMA base address for next frame
    err = BufMgr::getInstance().updateDMABaseAddr(camdma2type<ECamDMA_AAO>(), BufMgr::getInstance().getNextHwBuf(ECamDMA_AAO));

    return  err;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_AFUpdate
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_AFUpdate>)
{
    MRESULT err = S_3A_OK;
    BufInfo_T rBufInfo;

    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_AFUpdate>");

    // Dequeue AFO DMA buffer
    BufMgr::getInstance().dequeueHwBuf(ECamDMA_AFO, rBufInfo);

    CPTLog(Event_Pipe_3A_Continue_AF, CPTFlagStart);    // Profiling Start.
    AfMgr::getInstance().doAF(reinterpret_cast<MVOID *>(rBufInfo.virtAddr));
    CPTLog(Event_Pipe_3A_Continue_AF, CPTFlagEnd);    // Profiling Start.

    // Enqueue AFO DMA buffer
    BufMgr::getInstance().enqueueHwBuf(ECamDMA_AFO, rBufInfo);

    return  err;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_RecordingStart
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_RecordingStart>)
{
    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_RecordingStart>");

    // Init

    // State transition: eState_CamcorderPreview --> eState_Recording
    transitState(eState_CamcorderPreview, eState_Recording);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_AFStart
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_AFStart>)
{
    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_AFStart>");

    // Init
    if(AeMgr::getInstance().IsDoAEInPreAF() == MTRUE)   {
        transitAFState(eAFState_PreAF);
    }
    else   {
        AfMgr::getInstance().triggerAF();
        transitAFState(eAFState_AF);
    }


    // State transition: eState_CamcorderPreview --> eState_AF
    transitState(eState_CamcorderPreview, eState_AF);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_AFEnd
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_AFEnd>)
{
    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_AFEnd>");

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_PrecaptureStart: for CTS only
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StateCamcorderPreview::
sendIntent(intent2type<eIntent_PrecaptureStart>)
{
    MY_LOG("[StateCamcorderPreview::sendIntent]<eIntent_PrecaptureStart>");

    // State transition: eState_CamcorderPreview --> eState_Precapture
    transitState(eState_CamcorderPreview, eState_Precapture);

    return  S_3A_OK;
}

