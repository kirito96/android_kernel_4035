
#define LOG_TAG "aaa_state_precapture"

#ifndef ENABLE_MY_LOG
    #define ENABLE_MY_LOG       (1)
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
#include <kd_camera_feature.h>
#include <buf_mgr.h>
#include <mtkcam/common.h>
using namespace NSCam;
#include <camera_custom_AEPlinetable.h>
#include <ae_param.h>
#include <ae_mgr.h>
#include <flash_mgr.h>
#include <mtkcam/hal/sensor_hal.h>
#include <af_param.h>
#include <mcu_drv.h>
#include <af_mgr.h>
#include "CameraProfile.h"  // For CPTLog*()/AutoCPTLog class.
#include <cutils/properties.h>
#include <isp_tuning.h>
#include <dbg_isp_param.h>
#include <isp_tuning_mgr.h>
#include <lsc_mgr.h>

using namespace NS3A;
using namespace NSIspTuning;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StatePrecapture
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StatePrecapture::
StatePrecapture()
    : IState("StatePrecapture")
{
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_PrecaptureStart
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_PrecaptureStart>)
{
    MY_LOG("sendIntent(intent2type<eIntent_PrecaptureStart>) line=%d",__LINE__);
    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_PrecaptureEnd
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_PrecaptureEnd>)
{
    MY_LOG("sendIntent(intent2type<eIntent_PrecaptureEnd>) line=%d",__LINE__);

    m_pHal3A->resetReadyToCapture();
    //FlashMgr::getInstance()->turnOffPrecapFlash();
	FlashMgr::getInstance()->endPrecapture();


    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_VsyncUpdate
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_VsyncUpdate>)
{
	MY_LOG("sendIntent(intent2type<eIntent_VsyncUpdate>) line=%d frameCount=%d",__LINE__,getFrameCount());
#define PRE_AE_STATE 0
#define PRE_FLASH_STATE 1

	if(m_pHal3A->isReadyToCapture())
	{
		MY_LOG("VsyncUpdate ReadyToCapture=1, skip");
		return S_3A_OK;
	}


	static int preSyncState = PRE_AE_STATE;
    MRESULT err = S_3A_OK;
    BufInfo_T rBufInfo;
    MBOOL bIsStrobeFired = MFALSE;



    // Dequeue AAO DMA buffer
    BufMgr::getInstance().dequeueHwBuf(ECamDMA_AAO, rBufInfo);

	if(preSyncState==PRE_AE_STATE)
	{
		MY_LOG("preSyncState==PRE_AE_STATE  line=%d",__LINE__);
	    // One-shot AE or strobe AE
		//-------------------------------------------------------
        CPTLog(Event_Pipe_3A_AE, CPTFlagStart);    // Profiling Start.
        AeMgr::getInstance().doPreCapAE(bIsStrobeFired, reinterpret_cast<MVOID *>(rBufInfo.virtAddr));
        CPTLog(Event_Pipe_3A_AE, CPTFlagEnd);     // Profiling End.

	    if(AeMgr::getInstance().IsAEStable() == MTRUE) {    // AE is stable
	        // One-shot AWB without strobe
	        MINT32 i4SceneLV = AeMgr::getInstance().getLVvalue();

            AwbMgr::getInstance().setStrobeMode(AWB_STROBE_MODE_OFF);
	        AwbMgr::getInstance().doPreCapAWB(i4SceneLV, reinterpret_cast<MVOID *>(rBufInfo.virtAddr));
	    }
	    if(AeMgr::getInstance().IsAEStable() == MTRUE) {    // AE is stable
        // FIXME: call notifyReadyToCapture() if both one-shot AE and one-shot AWB are done
        MY_LOG("AE ready  line=%d",__LINE__);
        //m_pHal3A->notifyReadyToCapture();
        preSyncState = PRE_FLASH_STATE;
    	}


	}

	//Note: not "else if ..." because after AE end frame, flash should start to make faster.
	if(preSyncState==PRE_FLASH_STATE)
	{
		MY_LOG("preSyncState==PRE_FLASH_STATE  line=%d",__LINE__);
	    //-------------------------------------
	    // flash
	    CPTLog(Event_Pipe_3A_Strobe, CPTFlagStart);     // Profiling start.
		FlashExePara para;
		FlashExeRep rep;
		para.staBuf =reinterpret_cast<MVOID *>(rBufInfo.virtAddr);
		FlashMgr::getInstance()->doPfOneFrame(&para, &rep);
		if(rep.isEnd==1)
		{
			MY_LOG("vsync-notifyReadyToCapture()- line=%d", __LINE__);
			m_pHal3A->notifyReadyToCapture();
			preSyncState = PRE_AE_STATE;
		}
		CPTLog(Event_Pipe_3A_Strobe, CPTFlagEnd);     // Profiling End.

		//FIXME @@ YawbANU}(precaptureUAWB)

		MINT32 i4SceneLV = AeMgr::getInstance().getLVvalue();
		if(rep.isCurFlashOn==0)
			AwbMgr::getInstance().setStrobeMode(AWB_STROBE_MODE_OFF);
		else
			AwbMgr::getInstance().setStrobeMode(AWB_STROBE_MODE_ON);

		AwbMgr::getInstance().doPreCapAWB(i4SceneLV, reinterpret_cast<MVOID *>(rBufInfo.virtAddr));



	}

	//MTK_SWIP_PROJECT_START
	// F858
	LscMgr::getInstance()->updateTSFinput(
	        static_cast<NSIspTuning::LscMgr::LSCMGR_TSF_INPUT_SRC>(NSIspTuning::LscMgr::TSF_INPUT_PV),
	        AeMgr::getInstance().getLVvalue(), AwbMgr::getInstance().getAWBCCT(),
	        reinterpret_cast<MVOID *>(rBufInfo.virtAddr));
	//MTK_SWIP_PROJECT_END

	//-------------------------------------------------------
	// Enqueue AAO DMA buffer
    BufMgr::getInstance().enqueueHwBuf(ECamDMA_AAO, rBufInfo);
    // Update AAO DMA address
    BufMgr::getInstance().updateDMABaseAddr(camdma2type<ECamDMA_AAO>(), BufMgr::getInstance().getNextHwBuf(ECamDMA_AAO));

    return  err;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_AFUpdate
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_AFUpdate>)
{
    MY_LOG("sendIntent(intent2type<eIntent_AFUpdate>) line=%d",__LINE__);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_CaptureStart
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_CaptureStart>)
{
    MY_LOG("sendIntent(intent2type<eIntent_CaptureStart>) line =%d",__LINE__);




	FlashMgr::getInstance()->capCheckAndFireFlash_Start();
    // Init

    // AWB: update AWB statistics config
    if (FlashMgr::getInstance()->isFlashOnCapture()) {
        AwbMgr::getInstance().setStrobeMode(AWB_STROBE_MODE_ON);
      //  AeMgr::getInstance().setStrobeMode(MTRUE);
    }
    else {
        AwbMgr::getInstance().setStrobeMode(AWB_STROBE_MODE_OFF);
        AeMgr::getInstance().setStrobeMode(MFALSE);
    }

    // AE: update capture parameter
    AeMgr::getInstance().doCapAE();

    AwbMgr::getInstance().cameraCaptureInit();

    // Get operation mode and sensor mode for CCT and EM
    if ((IspTuningMgr::getInstance().getOperMode() == EOperMode_Normal) ||
        (IspTuningMgr::getInstance().getSensorMode() == ESensorMode_Capture)) {

        // AAO DMA / state enable again
        MRESULT err = BufMgr::getInstance().DMAInit(camdma2type<ECamDMA_AAO>());
        if (FAILED(err)) {
            MY_ERR("BufMgr::getInstance().DMAInit(ECamDMA_AAO) fail\n");
            return err;
        }

        err = BufMgr::getInstance().AAStatEnable(MTRUE);
        if (FAILED(err)) {
            MY_ERR("BufMgr::getInstance().AAStatEnable(MTRUE) fail\n");
            return err;
        }

        AfMgr::getInstance().setBestShotConfig();

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

    }

    // State transition: eState_Precapture --> eState_Capture
    transitState(eState_Precapture, eState_Capture);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_CameraPreviewEnd
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_CameraPreviewEnd>)
{
    MY_LOG("sendIntent(intent2type<eIntent_CameraPreviewEnd>) line=%d",__LINE__);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_CamcorderPreviewEnd
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_CamcorderPreviewEnd>)
{
    MY_LOG("sendIntent(intent2type<eIntent_CamcorderPreviewEnd>) line=%d",__LINE__);

    return  S_3A_OK;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  eIntent_AFEnd
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MRESULT
StatePrecapture::
sendIntent(intent2type<eIntent_AFEnd>)
{
    MY_LOG("sendIntent(intent2type<eIntent_AFEnd>) line=%d",__LINE__);

    return  S_3A_OK;
}


