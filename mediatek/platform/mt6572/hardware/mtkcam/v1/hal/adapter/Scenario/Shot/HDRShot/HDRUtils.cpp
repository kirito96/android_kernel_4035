
#include "MyHdr.h"

#include <cstdio>

#include <linux/cache.h>
#include <sys/stat.h>
#include <sys/resource.h>
#include <limits.h>
#include <cutils/properties.h>

#include <camera/MtkCamera.h>

#include <mtkcam/common.h>
#include <common/camutils/CamFormat.h>
#include <mtkcam/v1/camutils/CamInfo.h>
#include <mtkcam/exif/IBaseCamExif.h>
#include <mtkcam/exif/CamExif.h>
#include <common/hw/hwstddef.h>

#include <camshot/_callbacks.h>
#include <camshot/ICamShot.h>
#include <camshot/ISingleShot.h>

#include <drv/imem_drv.h>
#include <mtkcam/hal/sensor_hal.h>

#include <mtkcam/hal/aaa_hal_base.h>

#include <CamUtils.h>
#include <IImgBufQueue.h>

using namespace android;
using namespace MtkCamUtils;
#include <ICamAdapter.h>
#include <ImgBufProvidersManager.h>
#include <BaseCamAdapter.h>


#define MHAL_CAM_THUMB_ADDR_OFFSET  (64 * 1024)

using namespace android;
using namespace NSShot;

MUINT32	HdrShot::mu4RunningNumber = 0;

struct HdrFileInfo {
	String8	filename;
	MUINT32	size;
	MUINT8	*buffer;
};
#if 0
struct HdrMemBufInfo {
	void			*handler;
	IMEM_BUF_INFO	*imemBufInfo;
};
#endif

MBOOL
HdrShot::
updateInfo()
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

	#if 0
	//
	mShotParam.mi4PictureWidth = 4096;
	mShotParam.mi4PictureHeight = 3072;
	mShotParam.mu4ZoomRatio = 200;
	#endif


	//
	char value[PROPERTY_VALUE_MAX] = {'\0'};
	property_get("mediatek.hdr.debug", value, "0");

	mDebugMode = atoi(value);
	MY_DBG("[updateInfo] - mDebugMode=%d", mDebugMode);

    mHdrRoundTotal = 1;

	MY_DBG("[updateInfo] - mHdrRoundTotal=%d", mHdrRoundTotal);

	//capture policy, priority
	GetThreadProp(&mCapturePolicy, &mCapturePriority);
	MY_DBG("[updateInfo] - mCapturePolicy=%d", mCapturePolicy);
	MY_DBG("[updateInfo] - mCapturePriority=%d", mCapturePriority);

	//capture
	mu4W_yuv = mShotParam.mi4PictureWidth;
	mu4H_yuv = mShotParam.mi4PictureHeight;
	mu4SourceSize = mu4W_yuv * mu4H_yuv * 3/2;	//eImgFmt_YV12

	//postview
	mPostviewWidth = mShotParam.mi4PostviewWidth;
	mPostviewHeight = mShotParam.mi4PostviewHeight;
	mPostviewFormat = static_cast<EImageFormat>(android::MtkCamUtils::FmtUtils::queryImageioFormat(mShotParam.ms8PostviewDisplayFormat));

	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
decideCaptureMode()
{
#if 0	//kidd
	//@TODO decide EV at here
	MY_ERR("decideCaptureMode() is a deprecated function");
	MY_ERR("TODO get 3a information");
	return MFALSE;
#else
	FUNCTION_LOG_START;
	MINT32	ret = MTRUE;


	NS3A::Hal3ABase* mpHal3A = NS3A::Hal3ABase::createInstance(MtkCamUtils::DevMetaInfo::queryHalSensorDev(getOpenId()));
	CHECK_OBJECT(mpHal3A);

	NS3A::Hal3A_HDROutputParam_T rHdrOutputParam;

	MINT32	err = 0;
	ret = ( 0 == (err = mpHal3A->getHDRCapInfo(rHdrOutputParam)) );
	mpHal3A->destroyInstance();
	if(err)
	{
		MY_ERR("[decideCaptureMode] mpHal3A->getHDRCapInfo() fail. ec(%x)", err);
		goto lbExit;
	}

	if(mTestMode) {
		rHdrOutputParam.u4OutputFrameNum = 3;
		rHdrOutputParam.u4FinalGainDiff[0] = 4096;
		rHdrOutputParam.u4FinalGainDiff[1] = 256;
		rHdrOutputParam.u4TargetTone = 150;
	}

	MY_DBG("[decideCaptureMode] OutputFrameNum: %d. FinalGainDiff[0/1]: (%d, %d). TargetTone: %d.",
		rHdrOutputParam.u4OutputFrameNum,
		rHdrOutputParam.u4FinalGainDiff[0],
		rHdrOutputParam.u4FinalGainDiff[1],
		rHdrOutputParam.u4TargetTone
	);

	// Record value for later use.
	if(rHdrOutputParam.u4OutputFrameNum) {
		mu4OutputFrameNum	= rHdrOutputParam.u4OutputFrameNum;
		mu4FinalGainDiff[0]	= rHdrOutputParam.u4FinalGainDiff[0];
		mu4FinalGainDiff[1]	= rHdrOutputParam.u4FinalGainDiff[1];
		mu4TargetTone		= rHdrOutputParam.u4TargetTone;
	} else {
		MY_ERR("[decideCaptureMode] u4OutputFrameNum=%d, this should be 2 or 3"
				, rHdrOutputParam.u4OutputFrameNum);
		ret = MFALSE;
	}

lbExit:
	FUNCTION_LOG_END;
	return	ret;
#endif
}


MBOOL
HdrShot::
configureForSingleRun(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	if(mfgIsForceBreak) {MY_DBG("force break at %s", __FUNCTION__); return MFALSE;}

	mHdrRound = 1;

	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
configureForFirstRun(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	if(mfgIsForceBreak) {MY_DBG("force break at %s", __FUNCTION__); return MFALSE;}

	mHdrRound = 1;

	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
configureForSecondRun(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	if(mfgIsForceBreak) {MY_DBG("force break at %s", __FUNCTION__); return MFALSE;}

	mHdrRound = 2;

	ret = ret
	&&	do_SecondRound()
	;

	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
EVBracketCapture(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	if(mfgIsForceBreak) {MY_DBG("force break at %s", __FUNCTION__); return MFALSE;}
	CPTLog(Event_HdrShot_EVCapture, CPTFlagStart);

#if (HDR_PROFILE_CAPTURE2)
	MyDbgTimer DbgTmr("EVBracketCapture");
#endif


	// Increase 4-digit running number (range: 0 ~ 9999).
	if (mu4RunningNumber >= 9999)
		mu4RunningNumber = 0;
	else
		mu4RunningNumber++;

	ret = ret
	&&	decideCaptureMode()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: decideCaptureMode Time")
                                            #endif
  	&&	init()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: init Time")
                                            #endif
#if	HDR_SPEEDUP_MALLOC == 0
      //  ()  Request SmallImg Buffers.
    &&  requestSmallImgBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: requestSmallImgBuf Time")
                                            #endif
	;
#else
	;

	//allocate buffer for first only for speed up capture time
	MUINT32	i = 0;
	if(ret) {
		//extraced from requestSourceImgBuf()
		mpSourceImgBuf[i].size = mu4SourceSize;
		if (allocMem(&mpSourceImgBuf[i]))	// mpSourceImgBuf[i].virtAddr is NULL, allocation fail.
		{
			MY_ERR("[requestBufs] mpSourceImgBuf[%d] fails to request %d bytes.", i, mu4SourceSize);
			ret = MFALSE;
			goto lbExit;
		}
		//extraced from requestFirstRunSourceImgBuf()
		mu4FirstRunSourceSize = mu4W_first * mu4H_first * 3/2;	// I420 Size.
		//MY_VERB("[requestBufs] mu4SourceSize: %d.", mu4FirstRunSourceSize);
		mpFirstRunSourceImgBuf[i].size = mu4FirstRunSourceSize;
		if (allocMem(&mpFirstRunSourceImgBuf[i]))	// mpSourceImgBuf[i].virtAddr is NULL, allocation fail.
		{
			MY_ERR("[requestBufs] mpSourceImgBuf[%d] fails to request %d bytes.", i, mu4SourceSize);
			ret = MFALSE;
			goto lbExit;
		}
		//extraced from requestSmallImgBuf()
		mu4SmallImgSize = mu4W_small * mu4H_small;	// Y800 size.
		mpSmallImgBuf[i].size = mu4SmallImgSize;
		if(allocMem(&mpSmallImgBuf[i]))
		{
			MY_ERR("[requestBufs] mpSmallImgBuf[%d] fails to request %d bytes.", i, mu4SmallImgSize);
			ret = MFALSE;
			goto lbExit;
		}
		//allocate other buffers in thread for time saving
		pthread_create(&mCaptureIMemThread, NULL, HdrShot::allocateCaptureMemoryTask, this);
	}
#endif

	//@TODO refactory
	switch(mHdrRoundTotal) {
		case 1:	//single run
			ret = ret
				// ()	set output as yuv & small
				&& createSourceAndSmallImg();
			break;

		case 2:	//first run
			ret = ret
#if HDR_SPEEDUP_MALLOC == 0
				// ()	allocate memory for first run source buffer
				&& requestFirstRunSourceImgBuf()
#endif
				// ()	set output as yuv & first run
				&& createSourceAndFirstRunSourceImg()
				;

			break;
	}

lbExit:
	CPTLog(Event_HdrShot_EVCapture, CPTFlagEnd);
	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
ImageRegistratoin(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	if(mfgIsForceBreak) {MY_DBG("force break at %s", __FUNCTION__); return MFALSE;}
	CPTLog(Event_HdrShot_ImageRegistration, CPTFlagStart);


#if (HDR_PROFILE_CAPTURE2)
	MyDbgTimer DbgTmr("capture");
#endif

	// (1)	Normalize
    #if HDR_USE_THREAD
    SetHdrState(HDR_STATE_NORMALIZATION);
    ::sem_post(&semHdrThread);
    MY_DBG("[capture] semHdrThread (HDR_STATE_NORMALIZATION) posted.");
    ::sem_wait(&semHdrThreadBack);
    MY_DBG("[capture] semHdrThreadBack (HDR_STATE_NORMALIZATION) received.");
    #else
    ret =
    //  ()  Normalize small images, and put them back to SmallImg[].
		do_Normalization()
		;
    #endif  // HDR_USE_THREAD
                                            #if (HDR_PROFILE_CAPTURE2)
                                            DbgTmr.print("HdrProfiling2:: do_Normalization Time");
                                            #endif

	// (2)	SE
	CPTLog(Event_HdrShot_SE, CPTFlagStart);
	ret = ret
      //  ()  Request SEImg Buffers.
	&&	requestSEImgBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: requestSEImgBuf Time")
                                            #endif
      //  ()  Create SEImg (resize 3 Small Img to 3 SE Img).
    &&  createSEImg()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: createSEImg Time")
                                            #endif
      //  ()  Do SE to get GMV.
    &&  do_SE()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: do_SE Time")
                                            #endif
      //  ()  Release SEImg Buffers.
    &&  releaseSEImgBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseSEImgBuf Time")
                                            #endif
      ;
	CPTLog(Event_HdrShot_SE, CPTFlagEnd);

#if	HDR_SPEEDUP_MALLOC == 1
	if(ret){
		MUINT32	threadRet = 0;
		pthread_join(mProcessIMemThread, (void**)&threadRet);
		mProcessIMemThread = NULL;
		if(!threadRet) {
			MY_ERR("join mProcessIMemThread fail");
			ret = MFALSE;
		}
	}
#else
	ret = ret && requestHdrWorkingBuf();
#endif

	// (3)	MAV
	CPTLog(Event_HdrShot_MAV, CPTFlagStart);
    #if HDR_USE_THREAD
  //  ()  Do Feature Extraciton.
    SetHdrState(HDR_STATE_FEATURE_EXTRACITON);
    ::sem_post(&semHdrThread);
	MY_DBG("[capture] semHdrThread (HDR_STATE_FEATURE_EXTRACITON) posted.");
    ::sem_wait(&semHdrThreadBack);
	MY_DBG("[capture] semHdrThreadBack (HDR_STATE_FEATURE_EXTRACITON) received.");
    #else
	ret = ret
      //  ()  Do Feature Extraciton.
	&&	do_FeatureExtraction()
		;
    #endif  // HDR_USE_THREAD
                                            #if (HDR_PROFILE_CAPTURE2)
                                            DbgTmr.print("HdrProfiling2:: do_FeatureExtraction Time");
                                            #endif

	ret = ret
      //  ()  Release MAV working buffer, because it's not needed anymore.
    &&  releaseSmallImgBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseSmallImgBuf Time")
                                            #endif
      ;

	CPTLog(Event_HdrShot_MAV, CPTFlagEnd);

lbExit:
	CPTLog(Event_HdrShot_ImageRegistration, CPTFlagEnd);
	FUNCTION_LOG_END;
	return  ret;
}


MBOOL
HdrShot::
WeightingMapGeneration(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	if(mfgIsForceBreak) {MY_DBG("force break at %s", __FUNCTION__); return MFALSE;}
	CPTLog(Event_HdrShot_WeightingMapGeneration, CPTFlagStart);

#if (HDR_PROFILE_CAPTURE2)
	MyDbgTimer DbgTmr("capture");
#endif

	if(mHdrRound==1) {
		ret = ret
		&& requestOriWeightMapBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: requestOriWeightingTblBuf Time")
                                            #endif
		;
	}


	ret = ret
      //  ()  Do Alignment (includeing "Feature Matching" and "Weighting Map Generation").
	&&	do_Alignment()
    ;
                                            #if (HDR_PROFILE_CAPTURE2)
                                            DbgTmr.print("HdrProfiling2:: do_Alignment Time");
                                            #endif
	ret = ret
      //  ()  Request Original Weighting Table Buffer.
      //  ()  Get original Weighting Map.
    &&  do_OriWeightMapGet()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: do_OriWeightMapGet Time")
                                            #endif
      // Blur Weighting Map by downsize-then-upscale it.
    &&  requestDownSizedWeightMapBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: requestDownSizedWeightMapBuf Time")
                                            #endif
      //  ()  Down-scale original weighting map, and put into DownSizedWeightMapBuf.
    &&  do_DownScaleWeightMap()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: do_DownScaleWeightMap Time")
                                            #endif
      //  ()  Request Blurred weighting map buffer.
    &&  requestBlurredWeightMapBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: requestBlurredWeightMapBuf Time")
                                            #endif
      //  ()  Up-scale DownSized WeightMap Buf, and put into blurred weighting map.
    &&  do_UpScaleWeightMap()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: do_UpScaleWeightMap Time")
                                            #endif
      // Release DownSizedWeightMapBuf[i] because it's no longer needed.
    &&  releaseDownSizedWeightMapBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseDownSizedWeightMapBuf Time")
                                            #endif
	;

	if(mHdrRoundTotal==1 || mHdrRound==2) {
		ret = ret
      //  ()  Release OriWeightMapBuf, because it's not needed anymore. Note: Some info of OriWeightMap are needed when requestBlurredWeightMapBuf(), so must release it after requestBlurredWeightMapBuf().
	    &&  releaseOriWeightMapBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseOriWeightMapBuf Time")
                                            #endif
		;
	}

	CPTLog(Event_HdrShot_WeightingMapGeneration, CPTFlagEnd);
	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
Blending(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	if(mfgIsForceBreak) {MY_DBG("force break at %s", __FUNCTION__); return MFALSE;}
	CPTLog(Event_HdrShot_Blending, CPTFlagStart);

#if (HDR_PROFILE_CAPTURE2)
	MyDbgTimer DbgTmr("capture");
#endif

	#if HDR_SPEEDUP_JPEG
	if(mHdrRound==2) {
		pthread_create(&mNormalJpegThread, NULL, HdrShot::createNormalJpegImgTask, this);
	}
	#endif


	// (A)	fusion
	CPTLog(Event_HdrShot_Fusion, CPTFlagStart);
	ret = ret
	&&	requestBlendingBuf()	//[ION]
		//  ()  Do Fusion.
	&&	do_Fusion()
                                        #if (HDR_PROFILE_CAPTURE2)
                                        &&	DbgTmr.print("HdrProfiling2:: do_Fusion Time");
                                        #endif
	;
	CPTLog(Event_HdrShot_Fusion, CPTFlagEnd);

	ret = ret
      //  ()  Release Blurred weighting map because it's no longer needed.
    &&	releaseBlurredWeightMapBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseBlurredWeightMapBuf Time")
                                            #endif
      //  ()  Get HDR Cropped Result image.
    &&  do_HdrCroppedResultGet()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: do_HdrResultGet Time")
                                            #endif
	;

	// post view
	if(mHdrRound==1) {
		ret = ret
	      //  ()  Request Postview Image Buffer to put image
	    &&  requestPostviewImgBuf()
	                                            #if (HDR_PROFILE_CAPTURE2)
	                                            &&  DbgTmr.print("HdrProfiling2:: requestPostviewImgBuf Time")
	                                            #endif
	      //  ()  Resized cropped Postview image, and put it into PostviewImgBuf.
	    &&  do_CroppedPostviewResize()
	                                            #if (HDR_PROFILE_CAPTURE2)
	                                            &&  DbgTmr.print("HdrProfiling:: do_CroppedPostviewResize Time")
	                                            #endif
	      //  ()  notify postview was ready.
		&&	handlePostViewData((MUINT8*)mpPostviewImgBuf.virtAddr, mpPostviewImgBuf.size)
	                                            #if (HDR_PROFILE_CAPTURE2)
	                                            &&  DbgTmr.print("HdrProfiling:: handlePostViewData Time")
	                                            #endif
		;
	}

	//	()	encode normal jpeg
	#if ! HDR_SPEEDUP_JPEG
	HdrShot::createNormalJpegImgTask(this);
	#endif
	#if HDR_SPEEDUP_JPEG
	if(mHdrRoundTotal==1) {
		pthread_create(&mNormalJpegThread, NULL, HdrShot::createNormalJpegImgTask, this);
	}
	#endif

	// result image
	if(mHdrRoundTotal==1 || mHdrRound==2) {
#if HDR_SPEEDUP_JPEG
		//	()	wait for saving normal jpeg
		pthread_join(mNormalJpegThread, NULL);
		mNormalJpegThread = NULL;
#endif
		ret = ret
	      //  ()  Request Result Image Buffer to put image (which is resized to original size) for JPG encode.
	    &&  requestResultImgBuf()
	                                            #if (HDR_PROFILE_CAPTURE2)
	                                            &&  DbgTmr.print("HdrProfiling2:: requestResultImgBuf Time")
	                                            #endif
	      //  ()  encode jpeg and call handleJpegData()
		&&	createHdrJpegImgTask(this)
	                                            #if (HDR_PROFILE_CAPTURE2)
	                                            &&  DbgTmr.print("HdrProfiling:: createHdrJpegImg Time")
	                                            #endif
	      //  ()  Release HDR working buffer because it's no longer needed.
	    &&  releaseHdrWorkingBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseHdrWorkingBuf Time")
                                            #endif
		;
	}

	releaseBlendingBuf();	//[ION]

	if(mHdrRoundTotal==1 || mHdrRound==2) {
		ret = ret
	      //  ()  encode jpeg and call handleJpegData()
      //  ()  Release SourceImgBuf[i] because it's no longer needed.
		&& releaseSourceImgBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseSourceImgBuf Time")
                                            #endif
		;
	} else {
		ret = ret
      //  ()  Release FirstRunSourceImgBuf[i] because it's no longer needed.
		&& releaseFirstRunSourceImgBuf()
                                            #if (HDR_PROFILE_CAPTURE2)
                                            &&  DbgTmr.print("HdrProfiling2:: releaseSourceImgBuf Time")
                                            #endif
		;
	}

lbExit:
	if(mHdrRoundTotal==1 || mHdrRound==2) {
	    releasePostviewImgBuf();
	    releaseResultImgBuf();
	}

	CPTLog(Event_HdrShot_Blending, CPTFlagEnd);
	FUNCTION_LOG_END;
	return ret;
}

MBOOL
HdrShot::
requestSourceImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;
	MUINT32 u4OutputFrameNum = OutputFrameNumGet();

	MY_VERB("[requestBufs] mu4SourceSize: %d.", mu4SourceSize);
	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
	{
#if	HDR_SPEEDUP_MALLOC == 1
		if(i==0) {
			//mpSourceImgBuf[0] has been allocated in allocateMemoryTask()
			continue;
		}
#endif

		char szPoolName[100];
		szPoolName[0] = '\0';
		::sprintf(szPoolName, "%s%d", "HdrSrcImgBuf", i);

		mpSourceImgBuf[i].size = mu4SourceSize;
		if (allocMem(&mpSourceImgBuf[i]))	// mpSourceImgBuf[i].virtAddr is NULL, allocation fail.
		{
			MY_ERR("[requestBufs] mpSourceImgBuf[%d] fails to request %d bytes.", i, mu4SourceSize);
			ret = MFALSE;
			goto lbExit;
		}

		MY_VERB("[requestBufs] mpSourceImgBuf[%d].virtAddr: 0x%08X.", i, mpSourceImgBuf[i].virtAddr);
		MY_VERB("[requestBufs] mpSourceImgBuf[%d].phyAddr : 0x%08X.", i, mpSourceImgBuf[i].phyAddr);
		MY_VERB("[requestBufs] mpSourceImgBuf[%d].size: %d.", i, mpSourceImgBuf[i].size);
	}

lbExit:
	if	( ! ret )
	{
		releaseSourceImgBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}

MBOOL
HdrShot::
releaseSourceImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;
	MUINT32 u4OutputFrameNum = OutputFrameNumGet();

	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
	{
		deallocMem(&mpSourceImgBuf[i]);
	}

	FUNCTION_LOG_END;
	return	ret;
}

MBOOL
HdrShot::
requestFirstRunSourceImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	MUINT32 u4OutputFrameNum = OutputFrameNumGet();

	mu4FirstRunSourceSize = mu4W_first * mu4H_first * 3/2;	// I420 Size.

	MY_VERB("[requestBufs] mu4SourceSize: %d.", mu4FirstRunSourceSize);
	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
	{
#if	HDR_SPEEDUP_MALLOC == 1
		if(i==0) {
			//mpFirstRunSourceImgBuf[0] has been allocated in allocateMemoryTask()
			continue;
		}
#endif

		char szPoolName[100];
		szPoolName[0] = '\0';
		::sprintf(szPoolName, "%s%d", "HdrFirstRunSrcImgBuf", i);


		mpFirstRunSourceImgBuf[i].size = mu4FirstRunSourceSize;
		if (allocMem(&mpFirstRunSourceImgBuf[i]))	// mpSourceImgBuf[i].virtAddr is NULL, allocation fail.
		{
			MY_ERR("[requestBufs] mpSourceImgBuf[%d] fails to request %d bytes.", i, mu4SourceSize);
			ret = MFALSE;
			goto lbExit;
		}

		MY_VERB("[requestBufs] mpFirstRunSourceImgBuf[%d].virtAddr: 0x%08X.", i, mpFirstRunSourceImgBuf[i].virtAddr);
		MY_VERB("[requestBufs] mpFirstRunSourceImgBuf[%d].phyAddr : 0x%08X.", i, mpFirstRunSourceImgBuf[i].phyAddr);
		MY_VERB("[requestBufs] mpFirstRunSourceImgBuf[%d].size: %d.", i, mpFirstRunSourceImgBuf[i].size);
	}

lbExit:
	if	( ! ret )
	{
		releaseFirstRunSourceImgBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}

MBOOL
HdrShot::
releaseFirstRunSourceImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;
	MUINT32 u4OutputFrameNum = OutputFrameNumGet();

	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
	{
		deallocMem(&mpFirstRunSourceImgBuf[i]);
	}

	FUNCTION_LOG_END;
	return	ret;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Request Small Image buffers.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestSmallImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	mu4SmallImgSize = mu4W_small * mu4H_small;	// Y800 size.

	switch (mu4OutputFrameNum)	// Allocate buffers according to u4OutputFrameNum, note that there are no "break;" in case 3/case 2.
	{
		case 3:
		mpSmallImgBuf[2].size = mu4SmallImgSize;
		if(allocMem(&mpSmallImgBuf[2]))
			ret = MFALSE;
		case 2:
		mpSmallImgBuf[1].size = mu4SmallImgSize;
		if(allocMem(&mpSmallImgBuf[1]))
			ret = MFALSE;

#if	HDR_SPEEDUP_MALLOC == 0
		case 1:
		mpSmallImgBuf[0].size = mu4SmallImgSize;
		if(allocMem(&mpSmallImgBuf[0]))
			ret = MFALSE;
		break;
#endif
	}

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		MY_VERB("[requestSmallImgBuf] mu4SmallImgSize: %d.", mu4SmallImgSize);
		MY_VERB("[requestSmallImgBuf] mpSmallImgBuf[%d].virtAddr: 0x%08X.",	i, mpSmallImgBuf[i].virtAddr);
		MY_VERB("[requestSmallImgBuf] mpSmallImgBuf[%d].phyAddr : 0x%08X.",	i, mpSmallImgBuf[i].phyAddr);
		MY_VERB("[requestSmallImgBuf] mpSmallImgBuf[%d].size: %d.",		i, mpSmallImgBuf[i].size);
	}

lbExit:
	if	( ! ret )
	{
		releaseSmallImgBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseSmallImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		// For SmallImg Buffer.
		deallocMem(&mpSmallImgBuf[i]);
	}

	FUNCTION_LOG_END;
	return	ret;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Request SE Image buffers.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestSEImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;
	MY_DBG("[requestSEImgBuf] - E. u4OutputFrameNum: %d.", mu4OutputFrameNum);

	mu4SEImgSize = mu4W_se * mu4H_se;	// Y800 size.

	switch (mu4OutputFrameNum)	// Allocate buffers according to u4OutputFrameNum, note that there are no "break;" in case 3/case 2.
	{
		case 3:
		MY_DBG("[requestSEImgBuf] - alloc mpSEImgBuf[2].");
		mpSEImgBuf[2].size = mu4SEImgSize;
		if(allocMem(&mpSEImgBuf[2]))
			ret = MFALSE;
		case 2:
		MY_DBG("[requestSEImgBuf] - alloc mpSEImgBuf[1].");
		mpSEImgBuf[1].size = mu4SEImgSize;
		if(allocMem(&mpSEImgBuf[1]))
			ret = MFALSE;
		case 1:
		MY_DBG("[requestSEImgBuf] - alloc mpSEImgBuf[0].");
		mpSEImgBuf[0].size = mu4SEImgSize;
		if(allocMem(&mpSEImgBuf[0]))
			ret = MFALSE;
		break;
	}

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		MY_VERB("[requestSEImgBuf] mu4SEImgSize: %d.", mu4SEImgSize);
		MY_VERB("[requestSEImgBuf] mpSEImgBuf[%d].virtAddr: 0x%08X.",	i, mpSEImgBuf[i].virtAddr);
		MY_VERB("[requestSEImgBuf] mpSEImgBuf[%d].phyAddr : 0x%08X.",	i, mpSEImgBuf[i].phyAddr);
		MY_VERB("[requestSEImgBuf] mpSEImgBuf[%d].size(): %d.",		i, mpSEImgBuf[i].size);
	}

lbExit:
	if	( ! ret )
	{
		releaseSEImgBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseSEImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		// For SE Image Buffer.
		deallocMem(&mpSEImgBuf[i]);
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
requestNormalJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	mNormalJpegBuf.size = mu4W_yuv * mu4H_yuv;
	if(allocMem(&mNormalJpegBuf))
		ret = MFALSE;

	MY_DBG("[requestNormalJpegBuf] mNormalJpegBuf.virtAddr: 0x%08X.",	mNormalJpegBuf.virtAddr);
	MY_DBG("[requestNormalJpegBuf] mNormalJpegBuf.phyAddr : 0x%08X.",	mNormalJpegBuf.phyAddr);
	MY_DBG("[requestNormalJpegBuf] mNormalJpegBuf.size(): %d.",			mNormalJpegBuf.size);

lbExit:
	if	( ! ret )
	{
		releaseNormalJpegBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseNormalJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	deallocMem(&mNormalJpegBuf);

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
requestNormalThumbnailJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	mNormalThumbnailJpegBuf.size = mJpegParam.mi4JpegThumbWidth * mJpegParam.mi4JpegThumbHeight * 2;
	if(allocMem(&mNormalThumbnailJpegBuf))
		ret = MFALSE;

	MY_DBG("[requestNormalThumbnailJpegBuf] mNormalThumbnailJpegBuf.virtAddr: 0x%08X.",	mNormalThumbnailJpegBuf.virtAddr);
	MY_DBG("[requestNormalThumbnailJpegBuf] mNormalThumbnailJpegBuf.phyAddr : 0x%08X.",	mNormalThumbnailJpegBuf.phyAddr);
	MY_DBG("[requestNormalThumbnailJpegBuf] mNormalThumbnailJpegBuf.size(): %d.",			mNormalThumbnailJpegBuf.size);

lbExit:
	if	( ! ret )
	{
		releaseNormalThumbnailJpegBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseNormalThumbnailJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	deallocMem(&mNormalThumbnailJpegBuf);

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
requestHdrJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	mHdrJpegBuf.size = mu4W_yuv * mu4H_yuv;
	if(allocMem(&mHdrJpegBuf))
		ret = MFALSE;

	MY_DBG("[requestHdrJpegBuf] mHdrJpegBuf.virtAddr: 0x%08X.",	mHdrJpegBuf.virtAddr);
	MY_DBG("[requestHdrJpegBuf] mHdrJpegBuf.phyAddr : 0x%08X.",	mHdrJpegBuf.phyAddr);
	MY_DBG("[requestHdrJpegBuf] mHdrJpegBuf.size(): %d.",			mHdrJpegBuf.size);

lbExit:
	if	( ! ret )
	{
		releaseHdrJpegBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseHdrJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	deallocMem(&mHdrJpegBuf);

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
requestHdrThumbnailJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	mHdrThumbnailJpegBuf.size = mJpegParam.mi4JpegThumbWidth * mJpegParam.mi4JpegThumbHeight * 2;
	if(allocMem(&mHdrThumbnailJpegBuf))
		ret = MFALSE;

	MY_DBG("[requestHdrThumbnailJpegBuf] mHdrThumbnailJpegBuf.virtAddr: 0x%08X.",	mHdrThumbnailJpegBuf.virtAddr);
	MY_DBG("[requestHdrThumbnailJpegBuf] mHdrThumbnailJpegBuf.phyAddr : 0x%08X.",	mHdrThumbnailJpegBuf.phyAddr);
	MY_DBG("[requestHdrThumbnailJpegBuf] mHdrThumbnailJpegBuf.size(): %d.",			mHdrThumbnailJpegBuf.size);

lbExit:
	if	( ! ret )
	{
		releaseHdrThumbnailJpegBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseHdrThumbnailJpegBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	deallocMem(&mHdrThumbnailJpegBuf);

	FUNCTION_LOG_END;
	return	ret;
}


///////////////////////////////////////////////////////////////////////////
/// @brief Request HDR working buffers.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestHdrWorkingBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	// compare the size of HDR & MAV
	// a)	HDR
	MUINT32 mavBufferSize = 0;
	MUINT32 hdrBufferSize = 0;
	hdrBufferSize = mpHdrHal->HdrWorkingBuffSizeGet();
	MY_DBG("hdrBufferSize=%d", hdrBufferSize);

	// b)	MAV
	MY_DBG("mu4W_small=%d mu4H_small=%d", mu4W_small, mu4H_small);
	ret = ret && mpHdrHal->MavWorkingBuffSizeGet(mu4W_small, mu4H_small, &mavBufferSize);
	if(!ret) {
		MY_ERR("can't get mav working buffer size");
		goto lbExit;
	}
	MY_DBG("mavBufferSize=%d", mavBufferSize);

	// c) use large one as common buffer
	mu4HdrWorkingBufSize = (hdrBufferSize>mavBufferSize) ? hdrBufferSize : mavBufferSize;
	MY_DBG("mu4HdrWorkingBufSize=%d", mu4HdrWorkingBufSize);

	// d) For HDR Working Buffer.
	mpHdrWorkingBuf.size = mu4HdrWorkingBufSize;
#if 0
	if(mpIMemDrv->allocVirtBuf(&mpHdrWorkingBuf)) {
		ret = MFALSE;
	}
#else
	SetThreadProp(SCHED_OTHER, -20);
	mpHdrWorkingBuf.virtAddr = (MUINT32)malloc(mpHdrWorkingBuf.size);
	touchVirtualMemory((MUINT8*)mpHdrWorkingBuf.virtAddr, mpHdrWorkingBuf.size);
	SetThreadProp(mCapturePolicy, mCapturePriority);
	if(!mpHdrWorkingBuf.virtAddr) {
		ret = MFALSE;
	}
#endif
	mTotalBufferSize += mpHdrWorkingBuf.size;
	MY_DBG("allocMem total=%d\n", mTotalBufferSize);

	MY_DBG("[requestHdrWorkingBuf] mu4HdrWorkingBufSize    : %d.", mu4HdrWorkingBufSize);
	MY_DBG("[requestHdrWorkingBuf] mpHdrWorkingBuf.virtAddr: 0x%08X.",	mpHdrWorkingBuf.virtAddr);
	MY_DBG("[requestHdrWorkingBuf] mpHdrWorkingBuf.phyAddr : 0x%08X.",	mpHdrWorkingBuf.phyAddr);
	MY_DBG("[requestHdrWorkingBuf] mpHdrWorkingBuf.size    : %d.",		mpHdrWorkingBuf.size);

lbExit:
	if	( ! ret )
	{
		releaseHdrWorkingBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseHdrWorkingBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	// For HDR Working Buffer.
	if(mpHdrWorkingBuf.virtAddr) {
#if 0
		mpIMemDrv->freeVirtBuf(&mpHdrWorkingBuf);
#else
		free((void*)mpHdrWorkingBuf.virtAddr);
#endif
		mTotalBufferSize -= mpHdrWorkingBuf.size;
		MY_DBG("deallocMem total=%d\n", mTotalBufferSize);
	}
	mpHdrWorkingBuf.virtAddr = NULL;

	FUNCTION_LOG_END;
	return	ret;
}

MUINT32
HdrShot::
getAlignedSize(MUINT32 const u4Size)
{
    return (u4Size + (L1_CACHE_BYTES)) & ~(L1_CACHE_BYTES-1);
}


///////////////////////////////////////////////////////////////////////////
/// @brief Request Original WeightingTable buffers.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestOriWeightMapBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	//	   Allocate memory for original and blurred Weighting Map.
	MUINT32 u4Size = sizeof(HDR_PIPE_WEIGHT_TBL_INFO*) * mu4OutputFrameNum;
	MUINT32 u4AlignedSize = getAlignedSize(u4Size);
	MUINT32 u4TableSize = sizeof(HDR_PIPE_WEIGHT_TBL_INFO);
	MUINT32 u4AlignedTableSize = getAlignedSize(u4TableSize);
	MY_VERB("[requestOriWeightMapBuf] u4Size: %d. u4AlignedSize: %d. u4TableSize: %d. u4AlignedTableSize: %d.", u4Size, u4AlignedSize, u4TableSize, u4AlignedTableSize);

	OriWeight = (HDR_PIPE_WEIGHT_TBL_INFO**)memalign(L1_CACHE_BYTES, u4AlignedSize);
	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
		OriWeight[i] = (HDR_PIPE_WEIGHT_TBL_INFO*)memalign(L1_CACHE_BYTES, u4AlignedTableSize);

	//[ION]
	mHdrSetBmapInfo.bmap_width = mu4W_yuv / 2;
	mHdrSetBmapInfo.bmap_height = mu4H_yuv / 2;
	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		mWeightingBuf[i].size = mu4W_yuv * mu4H_yuv / 4;
		if(allocMem(&mWeightingBuf[i])) {
			ret = MFALSE;
			goto lbExit;
		}
		mHdrSetBmapInfo.bmap_image_addr[i] = mWeightingBuf[i].virtAddr;
		MY_DBG("[requestOriWeightMapBuf] addr[%d] = 0x%x", i, mWeightingBuf[i].virtAddr);
	}
	mHdrSetBmapInfo.bmap_image_size = mWeightingBuf[0].size * mu4OutputFrameNum;

lbExit:
	if	( ! ret )
	{
		releaseOriWeightMapBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


///////////////////////////////////////////////////////////////////////////
/// @brief Release Original WeightingTable buffers.
///
///	Note: Some info of OriWeightMap are needed when requestBlurredWeightMapBuf(),
///		  so must release it after requestBlurredWeightMapBuf().
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
releaseOriWeightMapBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

    if (OriWeight)
    {
		for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
		{
			deallocMem(&mWeightingBuf[i]);
		}

    	delete [] OriWeight;
    	OriWeight = NULL;
    }

	FUNCTION_LOG_END;
	return	ret;

}


///////////////////////////////////////////////////////////////////////////
/// @brief Request DownSizedWeightMap buffers.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestDownSizedWeightMapBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	if(mHdrRound == 1) {
		mu4W_dsmap = (OriWeight[0]->weight_table_width+31)  / 32;
		mu4H_dsmap = (OriWeight[0]->weight_table_height+31) / 32;
	} else {
		mu4W_dsmap = (OriWeight[0]->weight_table_width+39)  / 40;
		mu4H_dsmap = (OriWeight[0]->weight_table_height+39) / 40;
	}
	//MT6589 only accept odd output
	mu4W_dsmap = (mu4W_dsmap+1)&~1;
	mu4H_dsmap = (mu4H_dsmap+1)&~1;

	//	   Calculate width/height of Down-sized Weighting Map.
	mu4DownSizedWeightMapSize = mu4W_dsmap * mu4H_dsmap;	// Y800 size.
	//	   Allocate memory for Down-sized Weighting Map.
	switch (mu4OutputFrameNum)	// Allocate buffers according to u4OutputFrameNum, note that there are no "break;" in case 3/case 2.
	{
		case 3:
		mpDownSizedWeightMapBuf[2].size = mu4DownSizedWeightMapSize;
		if(allocMem(&mpDownSizedWeightMapBuf[2]))
			ret = MFALSE;
		case 2:
		mpDownSizedWeightMapBuf[1].size = mu4DownSizedWeightMapSize;
		if(allocMem(&mpDownSizedWeightMapBuf[1]))
			ret = MFALSE;
		case 1:
		mpDownSizedWeightMapBuf[0].size = mu4DownSizedWeightMapSize;
		if(allocMem(&mpDownSizedWeightMapBuf[0]))
			ret= MFALSE;
		break;
	}

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		MY_VERB("[requestDownSizedWeightMapBuf] mu4DownSizedWeightMapSize: %d.", mu4DownSizedWeightMapSize);
		MY_VERB("[requestDownSizedWeightMapBuf] mpDownSizedWeightMapBuf[%d].virtAddr: 0x%08X.",	i, mpDownSizedWeightMapBuf[i].virtAddr);
		MY_VERB("[requestDownSizedWeightMapBuf] mpDownSizedWeightMapBuf[%d].phyAddr : 0x%08X.",	i, mpDownSizedWeightMapBuf[i].phyAddr);
		MY_VERB("[requestDownSizedWeightMapBuf] mpDownSizedWeightMapBuf[%d].size: %d.",		i, mpDownSizedWeightMapBuf[i].size);
	}

lbExit:
	if	( ! ret )
	{
		releaseDownSizedWeightMapBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseDownSizedWeightMapBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		// For DownSized Weight Map Buffer.
		deallocMem(&mpDownSizedWeightMapBuf[i]);
	}

	FUNCTION_LOG_END;
	return	ret;
}


///////////////////////////////////////////////////////////////////////////
/// @brief Request Blurred WeightingTable buffers. Must execute after OriWeightTbl is gottn.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestBlurredWeightMapBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	//	   Allocate memory for original and blurred Weighting Map.
	MUINT32 u4Size = sizeof(HDR_PIPE_WEIGHT_TBL_INFO*) * mu4OutputFrameNum;
	MUINT32 u4AlignedSize = getAlignedSize(u4Size);
	MUINT32 u4TableSize = sizeof(HDR_PIPE_WEIGHT_TBL_INFO);
	MUINT32 u4AlignedTableSize = getAlignedSize(u4TableSize);
	MY_VERB("[requestBlurredWeightMapBuf] u4Size: %d. u4AlignedSize: %d. u4TableSize: %d. u4AlignedTableSize: %d.", u4Size, u4AlignedSize, u4TableSize, u4AlignedTableSize);

	BlurredWeight = (HDR_PIPE_WEIGHT_TBL_INFO**)memalign(L1_CACHE_BYTES, u4AlignedSize);

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		BlurredWeight[i] = (HDR_PIPE_WEIGHT_TBL_INFO*)memalign(L1_CACHE_BYTES, u4AlignedTableSize);

		// Init BlurredWeight[i], and allocate memory for Blurred Weighting Map.
		BlurredWeight[i]->weight_table_width  = OriWeight[i]->weight_table_width;
		BlurredWeight[i]->weight_table_height = OriWeight[i]->weight_table_height;
		mpBlurredWeightMapBuf[i].size = (BlurredWeight[i]->weight_table_width * BlurredWeight[i]->weight_table_height);
		if(allocMem(&mpBlurredWeightMapBuf[i])) {
			ret = MFALSE;
			goto lbExit;
		}
		BlurredWeight[i]->weight_table_data = (MUINT8*)mpBlurredWeightMapBuf[i].virtAddr;

	}
lbExit:
	if	( ! ret )
	{
		releaseBlurredWeightMapBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseBlurredWeightMapBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;
	MUINT32 u4OutputFrameNum = OutputFrameNumGet();

	// Free allocated memory
    if (BlurredWeight)
    {
    	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
    	{
    		deallocMem(&mpBlurredWeightMapBuf[i]);
    	}
    	delete [] BlurredWeight;
    	BlurredWeight = NULL;
    }

	FUNCTION_LOG_END;
	return	ret;

}


///////////////////////////////////////////////////////////////////////////
/// @brief Request PostviewImg buffer.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestPostviewImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

// Jason tmp solution
    //MUINT32 u4Stride[3];
	//GetStride(mPostviewWidth, mPostviewFormat, u4Stride);
	MUINT32 StrideW = mPostviewWidth;
    if(mPostviewWidth%32)
        StrideW = mPostviewWidth- (mPostviewWidth%32) + 32;

	//mu4PostviewImgSize = mPostviewWidth * mPostviewHeight * 3/2;	//queryImgBufSize(mPostviewFormat, mPostviewWidth, mPostviewHeight);
	mu4PostviewImgSize = StrideW * mPostviewHeight * 3/2;	//queryImgBufSize(mPostviewFormat, mPostviewWidth, mPostviewHeight);
    MY_DBG("Jason tmp: %d * %d = %d", StrideW, mPostviewHeight, mu4PostviewImgSize);
// Jason tmp solution	
	mpPostviewImgBuf.size = mu4PostviewImgSize;
	if(allocMem(&mpPostviewImgBuf))
		ret = MFALSE;

	MY_VERB("[requestPostviewImgBuf] mu4PostviewImgSize       : %d.", mu4PostviewImgSize);
	MY_VERB("[requestPostviewImgBuf] mpPostviewImgBuf.virtAddr: 0x%08X.",	mpPostviewImgBuf.virtAddr);
	MY_VERB("[requestPostviewImgBuf] mpPostviewImgBuf.phyAddr : 0x%08X.",	mpPostviewImgBuf.phyAddr);
	MY_VERB("[requestPostviewImgBuf] mpPostviewImgBuf.size    : %d.",		mpPostviewImgBuf.size);

lbExit:
	if	( ! ret )
	{
		releasePostviewImgBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releasePostviewImgBuf()
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	deallocMem(&mpPostviewImgBuf);

	FUNCTION_LOG_END;
	return	ret;
}



///////////////////////////////////////////////////////////////////////////
/// @brief Request ResultImg buffer.
///
/// @return SUCCDSS (TRUE) or Fail (FALSE).
///////////////////////////////////////////////////////////////////////////
MBOOL
HdrShot::
requestResultImgBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	mu4ResultImgSize = mu4W_yuv * mu4H_yuv * 3/2;	// * 3/2: I420 size.
	mpResultImgBuf.size = mu4ResultImgSize;
	if(allocMem(&mpResultImgBuf))
		ret = MFALSE;

	MY_VERB("[requestResultImgBuf] mu4ResultImgSize       : %d.", mu4ResultImgSize);
	MY_VERB("[requestResultImgBuf] mpResultImgBuf.virtAddr: 0x%08X.",	mpResultImgBuf.virtAddr);
	MY_VERB("[requestResultImgBuf] mpResultImgBuf.phyAddr : 0x%08X.",	mpResultImgBuf.phyAddr);
	MY_VERB("[requestResultImgBuf] mpResultImgBuf.size    : %d.",		mpResultImgBuf.size);

lbExit:
	if	( ! ret )
	{
		releaseResultImgBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseResultImgBuf()
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	deallocMem(&mpResultImgBuf);

	FUNCTION_LOG_END;
	return	ret;
}


#if 0
MBOOL
HdrShot::
requestWeightingBuf(void)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
	{
		mWeightingBuf[i].size = mu4W_small * mu4H_small / 4;
		allocMem(&mWeightingBuf[i]);
	}

lbExit:
	if	( ! ret )
	{
		releaseWeightingBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseWeightingBuf()
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	MUINT32 u4OutputFrameNum = OutputFrameNumGet();
	for (MUINT32 i = 0; i < u4OutputFrameNum; i++) {
		deallocMem(&mWeightingBuf[i]);
	}

	FUNCTION_LOG_END;
	return	ret;
}
#endif


#if 1
MBOOL
HdrShot::
requestBlendingBuf()
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	mBlendingBuf.size = mu4W_yuv * mu4H_yuv * 3/2;
	if(allocMem(&mBlendingBuf)) {
		ret = MFALSE;
		goto lbExit;
	}

lbExit:
	if	( ! ret )
	{
		releaseBlendingBuf();
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
releaseBlendingBuf()
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

	deallocMem(&mBlendingBuf);

	FUNCTION_LOG_END;
	return	ret;
}
#endif

unsigned int
HdrShot::
dumpToFile(
    char const *fname,
    unsigned char *pbuf,
    unsigned int size
)
{
    int nw, cnt = 0;
    unsigned int written = 0;

    MY_DBG("opening file [%s]\n", fname);
    int fd = open(fname, O_RDWR | O_CREAT, S_IRWXU);
    if (fd < 0) {
        MY_ERR("failed to create file [%s]: %s", fname, strerror(errno));
        return 0x80000000;
    }

    MY_DBG("writing %d bytes to file [%s]\n", size, fname);
    while (written < size) {
        nw = ::write(fd, pbuf + written, size - written);
        if (nw < 0) {
            MY_ERR("failed to write to file [%s]: %s", fname, strerror(errno));
            break;
        }
        written += nw;
        cnt++;
    }
    MY_DBG("done writing %d bytes to file [%s] in %d passes\n", size, fname, cnt);
    ::close(fd);

    return 0;
}


#if 0
static bool
saveBufToFile(char const*const fname, MUINT8 *const buf, MUINT32 const size)
{
    int nw, cnt = 0;
    uint32_t written = 0;

    MY_DBG("(name, buf, size) = (%s, %x, %d)", fname, buf, size);
    MY_DBG("opening file [%s]\n", fname);
    int fd = ::open(fname, O_RDWR | O_CREAT | O_TRUNC);
    if (fd < 0) {
        MY_ERR("failed to create file [%s]: %s", fname, ::strerror(errno));
        return false;
    }

    MY_DBG("writing %d bytes to file [%s]\n", size, fname);
    while (written < size) {
        nw = ::write(fd,
                     buf + written,
                     size - written);
        if (nw < 0) {
            MY_ERR("failed to write to file [%s]: %s", fname, ::strerror(errno));
            break;
        }
        written += nw;
        cnt++;
    }
    MY_DBG("done writing %d bytes to file [%s] in %d passes\n", size, fname, cnt);
    ::close(fd);
    return true;
}
#endif


static uint32_t
loadFileToBuf(char const*const fname, uint8_t*const buf, uint32_t size)
{
    int nr, cnt = 0;
    uint32_t readCnt = 0;

    MY_DBG("opening file [%s]\n", fname);
    int fd = ::open(fname, O_RDONLY);
    if (fd < 0) {
        MY_ERR("failed to create file [%s]: %s", fname, strerror(errno));
        return readCnt;
    }
    //
    if (size == 0) {
        size = ::lseek(fd, 0, SEEK_END);
        ::lseek(fd, 0, SEEK_SET);
    }
    //
    MY_DBG("read %d bytes from file [%s]\n", size, fname);
    while (readCnt < size) {
	    MY_DBG("readCnt=%d size=%d\n", readCnt, size);
        //nr = ::read(fd, buf + readCnt, 102400);
        nr = ::read(fd, buf + readCnt, size - readCnt);
        if (nr < 0) {
            MY_ERR("failed to read from file [%s]: %s",
                        fname, strerror(errno));
            break;
        }
        readCnt += nr;
        cnt++;
    }
    MY_DBG("done reading %d bytes to file [%s] in %d passes\n", size, fname, cnt);
    ::close(fd);

    return readCnt;
}

MBOOL
HdrShot::
fgCamShotNotifyCb(MVOID* user, NSCamShot::CamShotNotifyInfo const msg)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

    HdrShot *pHdrShot = reinterpret_cast <HdrShot *>(user);
    if (NULL != pHdrShot)
    {
        if (NSCamShot::ECamShot_NOTIFY_MSG_EOF == msg.msgType)
        {
            pHdrShot->mpShotCallback->onCB_Shutter(true,0);
			pHdrShot->mShutterCBDone = MTRUE;
        }
    }

	FUNCTION_LOG_END;
    return ret;
}


MBOOL
HdrShot::
fgCamShotDataCb(MVOID* user, NSCamShot::CamShotDataInfo  const msg)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

#if 0
    HdrShot *pHdrShot = reinterpret_cast<HdrShot *>(user);
    if (NULL != pHdrShot)
    {
		switch(msg.msgType) {
			case NSCamShot::ECamShot_DATA_MSG_BAYER:
	            pHdrShot->handleBayerData( msg.puData, msg.u4Size);
				break;
			#if 0	//@TODO deprecated
			case NSCamShot::ECamShot_DATA_MSG_YUV:
	            pHdrShot->handleYuvData( msg.puData, msg.u4Size);
				break;
			case NSCamShot::ECamShot_DATA_MSG_POSTVIEW:
	            pHdrShot->handlePostViewData( msg.puData, msg.u4Size);
				break;
			case NSCamShot::ECamShot_DATA_MSG_JPEG:
	            pHdrShot->handleJpegData(msg.puData, msg.u4Size, reinterpret_cast<MUINT8*>(msg.ext1), msg.ext2);
				break;
			#endif
		}
    }
#endif

	FUNCTION_LOG_END;
    return ret;
}


MBOOL
HdrShot::
handleYuvData(MUINT8* const puBuf, MUINT32 const u4Size)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
    MY_DBG("[handleYuvData] (puBuf, size) = (%p, %d)", puBuf, u4Size);

	if(mTestMode) {
	    MY_ERR("[%s] mTestMode", __FUNCTION__);
		return MTRUE;
	}

#if !HDR_DEBUG_SKIP_HANDLER
#endif

	FUNCTION_LOG_END;
    return ret;
}


MBOOL
HdrShot::
handlePostViewData(MUINT8* const puBuf, MUINT32 const u4Size)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
    MY_DBG("[handlePostViewData] + (puBuf, size) = (%p, %d)", puBuf, u4Size);

	if(mTestMode) {
	    MY_ERR("[%s] mTestMode", __FUNCTION__);
		return MTRUE;
	}

#if !HDR_DEBUG_SKIP_HANDLER
    mpShotCallback->onCB_PostviewDisplay(0,
                                         u4Size,
                                         reinterpret_cast<uint8_t const*>(puBuf)
                                        );
#endif

	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
handleJpegData(MUINT8* const puJpegBuf, MUINT32 const u4JpegSize, MUINT8* const puThumbBuf, MUINT32 const u4ThumbSize, MUINT32 const u4Index, MBOOL bFinal)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
    MY_DBG("[handleJpegData] + (puJpgBuf, jpgSize, puThumbBuf, thumbSize) = (%p, %d, %p, %d)", puJpegBuf, u4JpegSize, puThumbBuf, u4ThumbSize);

	if(mTestMode) {
	    MY_ERR("[%s] mTestMode", __FUNCTION__);
		return MTRUE;
	}

#if !HDR_DEBUG_SKIP_HANDLER
    MUINT8 *puExifHeaderBuf = new MUINT8[128 * 1024];
    MUINT32 u4ExifHeaderSize = 0;

	//@TODO make EXIF
    //makeExifHeader(eAppMode_PhotoMode, puThumbBuf, u4ThumbSize, puExifHeaderBuf, u4ExifHeaderSize);
    updateThumbnailExif(mpCamExif[1], puThumbBuf, u4ThumbSize, puExifHeaderBuf, u4ExifHeaderSize);
    MY_DBG("[handleJpegData] (thumbbuf, size, exifHeaderBuf, size) = (%p, %d, %p, %d)",
                      puThumbBuf, u4ThumbSize, puExifHeaderBuf, u4ExifHeaderSize);
    // Jpeg callback
    mpShotCallback->onCB_CompressedImage(0
                                         , u4JpegSize
                                         , reinterpret_cast<uint8_t const*>(puJpegBuf)
                                         , u4ExifHeaderSize	//header size
                                         , puExifHeaderBuf	//header buf
                                         , u4Index			//callback index
                                         , bFinal			//final image
                                         , bFinal ? MTK_CAMERA_MSG_EXT_DATA_COMPRESSED_IMAGE : MTK_CAMERA_MSG_EXT_DATA_HDR
                                         );
	mJpegCBDone = MTRUE;
    delete [] puExifHeaderBuf;
#endif

	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
createSourceAndSmallImg(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE; 

#if HDR_DEBUG_OFFLINE_SOURCE_IMAGE
	if(mTestMode) {
		MY_DBG("[createSourceAndSmallImg] - offline");

		//source
	    for(int i=0; i<mu4OutputFrameNum; i++) {
#if	HDR_SPEEDUP_MALLOC == 1
			if(i==1) {
				MUINT32	threadRet = 0;
				pthread_join(mCaptureIMemThread, (void**)&threadRet);
				mCaptureIMemThread = NULL;
				if(!threadRet) {
					MY_ERR("join mCaptureIMemThread fail");
					ret = MFALSE;
					break;
				}
				pthread_create(&mProcessIMemThread, NULL, HdrShot::allocateProcessMemoryTask, this);
			}
#endif
			char yuvfile[128];
			#if HDR_DEBUG_FORCE_SINGLE_RUN
			sprintf(yuvfile, HDR_DEBUG_OUTPUT_FOLDER"hdr_sample_1600x1200_%d.i420", i);
			#else
			//sprintf(yuvfile, HDR_DEBUG_OUTPUT_FOLDER"hdr_sample_4000x3000_%d.i420", i);
			sprintf(yuvfile, HDR_DEBUG_OUTPUT_FOLDER"hdr_sample_3264x2448_%d.i420", i);
			#endif

			uint32_t nReadSize = loadFileToBuf(yuvfile, (uint8_t*)mpSourceImgBuf[i].virtAddr, mu4SourceSize);
			if(nReadSize < mpSourceImgBuf[i].size) {
				MY_DBG("[createSourceAndSmallImg] can't read enought data");
				ret = MFALSE;
			}
			mpCamExif[i] = new CamExif;
	    }

		//small
		ret = ret && createSmallImg();

		goto lbCaptureDone;
	}
#endif

{
	MY_DBG("[createSourceAndSmallImg] - online");
    NSCamShot::ISingleShot *pSingleShot = NSCamShot::ISingleShot::createInstance(eShotMode_HdrShot, "testshot");
    //
    pSingleShot->init();

    // shot param
    NSCamShot::ShotParam rShotParam(
    					//mpSourceImgBuf
						eImgFmt_I420,			//yuv format
                        mu4W_yuv,				//picutre width
                        mu4H_yuv,				//picture height
                        //mpSmallImgBuf
                        0,						//postview rotation
                        0,						//postview flip
                        eImgFmt_Y800,			//postview format
                        mu4W_small,				//postview width
                        mu4H_small,				//postview height
                        0,						//postview rotation
                        0,						//postview flip
                        mShotParam.mu4ZoomRatio	//zoom
                        );

    // sensor param
	NSCamShot::SensorParam rSensorParam(static_cast<MUINT32>(MtkCamUtils::DevMetaInfo::queryHalSensorDev(getOpenId())),                             //Device ID
                             ACDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG,         //Scenaio
                             10,                                       //bit depth
                             //@TODO MFALSE,                                   //bypass delay
                             MFALSE,                                   //bypass delay
                             MFALSE,                                   //bypass scenario
                             0,                                       //RawType
                             MFALSE                                   //bypass AE capture shutter delay                                        
                            );
	//
	//pSingleShot->setCallbacks(fgCamShotNotifyCb, fgCamShotDataCb, this);
	MY_DBG("pSingleShot->setCallbacks");
	pSingleShot->setCallbacks(fgCamShotNotifyCb, NULL, this);
	//
    pSingleShot->setShotParam(rShotParam);
    //
    //pSingleShot->setJpegParam(rJpegParam);

    //u4OutputFrameNum
#if !HDR_DEBUG_SKIP_3A
    NS3A::Hal3ABase* p3AHal = NS3A::Hal3ABase::createInstance(MtkCamUtils::DevMetaInfo::queryHalSensorDev(getOpenId()));
    //NS3A::CaptureParam_T rCap3AParam;
    //for 6572 YUV imgsensor
    NS3A::Param_T cam3aParam;
#if HDR_SWITCH_CAPTURE_SEQUENCE
    MUINT32 u4EVIndex[3] = {0, -2, 2};
#else
    MUINT32 u4EVIndex[3] = {-2, 0, 2};
#endif
    if (2 == mu4OutputFrameNum)
    {
        u4EVIndex[0] = -2;
        u4EVIndex[1] = 2;
    }
#endif

    for(int i=0; i<mu4OutputFrameNum; i++) {
#if	HDR_SPEEDUP_MALLOC == 1
		if(i==1) {
			MUINT32	threadRet = 0;
			pthread_join(mCaptureIMemThread, (void**)&threadRet);
			mCaptureIMemThread = NULL;
			if(!threadRet) {
				MY_ERR("join mCaptureIMemThread fail");
				ret = MFALSE;
				break;
			}
			pthread_create(&mProcessIMemThread, NULL, HdrShot::allocateProcessMemoryTask, this);
		}
#endif
		// (1)	notify
		pSingleShot->disableNotifyMsg(0xFFFFFFFF);
		if(0 == i) {
			//shutter sound
			pSingleShot->enableNotifyMsg(NSCamShot::ECamShot_NOTIFY_MSG_EOF);
		}

		// (2)	data message
		pSingleShot->disableDataMsg(0xFFFFFFFF);
		pSingleShot->enableDataMsg(	NSCamShot::ECamShot_DATA_MSG_YUV
										| NSCamShot::ECamShot_DATA_MSG_POSTVIEW);

		MUINT32	stride[3];
		// (3)	regist mpSourceImgBuf
		GetStride(mu4W_yuv, eImgFmt_I420, stride);
		NSCamHW::ImgInfo	sourceImgInfo(eImgFmt_I420, mu4W_yuv, mu4H_yuv);
		NSCamHW::BufInfo	sourceBufInfo(mpSourceImgBuf[i].size, mpSourceImgBuf[i].virtAddr, mpSourceImgBuf[i].phyAddr, mpSourceImgBuf[i].memID);
		NSCamHW::ImgBufInfo	sourceImgBufInfo(sourceImgInfo, sourceBufInfo, stride);
		pSingleShot->registerImgBufInfo(NSCamShot::ECamShot_BUF_TYPE_YUV, sourceImgBufInfo);

		// (4)	regist mpSmallImgBuf
		GetStride(mu4W_small, eImgFmt_Y800, stride);
		NSCamHW::ImgInfo	smallImgInfo(eImgFmt_Y800, mu4W_small, mu4H_small);
		NSCamHW::BufInfo	smallBufInfo(mpSmallImgBuf[i].size, mpSmallImgBuf[i].virtAddr, mpSmallImgBuf[i].phyAddr, mpSmallImgBuf[i].memID);
		NSCamHW::ImgBufInfo	smallImgBufInfo(smallImgInfo, smallBufInfo, stride);
		pSingleShot->registerImgBufInfo(NSCamShot::ECamShot_BUF_TYPE_POSTVIEW, smallImgBufInfo);

		// (5)	capture
#if !HDR_DEBUG_SKIP_3A
		//p3AHal->getCaptureParams(i, 0, rCap3AParam);
		//p3AHal->updateCaptureParams(rCap3AParam);
        
        //for 6572 YUV imgsensor
#if HDR_SWITCH_CAPTURE_SEQUENCE
        rSensorParam.fgBypassAEShutterDelay= (0 == i) ? MTRUE : MFALSE;
#else
        rSensorParam.fgBypassAEShutterDelay = MFALSE;
#endif

        if ( ! p3AHal->getParams(cam3aParam) )
        {
            MY_ERR("[createSourceAndSmallImg] getParams fail");
        }
        cam3aParam.i4ExpIndex = u4EVIndex[i];
        MY_DBG("[createSourceAndSmallImg] cam3aParam.i4ExpIndex=%d",cam3aParam.i4ExpIndex);
        if ( ! p3AHal->setParams(cam3aParam) )
        {
            MY_ERR("[createSourceAndSmallImg] setParams fail");
        }
#endif
		rSensorParam.fgBypassScenaio= (0 != i) ? MTRUE : MFALSE;
		CPTLog(Event_HdrShot_SingleCapture, CPTFlagStart);
		pSingleShot->startOne(rSensorParam);
		CPTLog(Event_HdrShot_SingleCapture, CPTFlagEnd);

		// (6)	save Exif
		mpCamExif[i] = new CamExif;
		update3AExif(mpCamExif[i]);	//@TODO release mpCamExif[i]
	}

#if HDR_SWITCH_CAPTURE_SEQUENCE
    MINT32 u4tempAddr = 0;
    if (mu4OutputFrameNum == 3)
    {
        u4tempAddr = mpSourceImgBuf[0].virtAddr;
        mpSourceImgBuf[0].virtAddr = mpSourceImgBuf[1].virtAddr;
        mpSourceImgBuf[1].virtAddr = u4tempAddr;
        
        u4tempAddr = mpSourceImgBuf[0].phyAddr;
        mpSourceImgBuf[0].phyAddr = mpSourceImgBuf[1].phyAddr;
        mpSourceImgBuf[1].phyAddr = u4tempAddr;

        u4tempAddr = mpSmallImgBuf[0].virtAddr;
        mpSmallImgBuf[0].virtAddr = mpSmallImgBuf[1].virtAddr;
        mpSmallImgBuf[1].virtAddr = u4tempAddr;

        u4tempAddr = mpSmallImgBuf[0].phyAddr;
        mpSmallImgBuf[0].phyAddr = mpSmallImgBuf[1].phyAddr;
        mpSmallImgBuf[1].phyAddr = u4tempAddr;
    }
#endif

#if !HDR_DEBUG_SKIP_3A
	p3AHal->destroyInstance();
#endif
    //
    pSingleShot->uninit();
    //
    pSingleShot->destroyInstance();
}


lbCaptureDone:
	if(!ret) goto lbExit;

	//@TODO dump raw when mDebugMode==1
	if(CUST_HDR_DEBUG || HDR_DEBUG_SAVE_SOURCE_IMAGE || mDebugMode) {
		for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
		{
			char szFileName[100];
			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_1_mpSourceImgBuf[%d]_%dx%d_r%d.i420", mu4RunningNumber, i, mu4W_yuv, mu4H_yuv, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)mpSourceImgBuf[i].virtAddr, mu4SourceSize);
		}
	}

	if(HDR_DEBUG_SAVE_SMALL_IMAGE || mDebugMode) {
		for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
		{
			char szFileName[100];

			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_2_mpSmallImgBuf[%d]_%dx%d_r%d.y", mu4RunningNumber, i, mu4W_small, mu4H_small, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)mpSmallImgBuf[i].virtAddr, mpSmallImgBuf[i].size);
		}
	}

lbExit:
	FUNCTION_LOG_END;
	return ret;
}

MBOOL
HdrShot::
createSourceAndFirstRunSourceImg(void)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

#if HDR_DEBUG_OFFLINE_SOURCE_IMAGE
	if(mTestMode) {
		MY_DBG("[createSourceAndFirstRunSourceImg] - offline");

		//source
	    for(int i=0; i<mu4OutputFrameNum; i++) {
#if	HDR_SPEEDUP_MALLOC == 1
			if(i==1) {
				MUINT32	threadRet = 0;
				pthread_join(mCaptureIMemThread, (void**)&threadRet);
				mCaptureIMemThread = NULL;
				if(!threadRet) {
					MY_ERR("join mCaptureIMemThread fail");
					ret = MFALSE;
					break;
				}
				pthread_create(&mProcessIMemThread, NULL, HdrShot::allocateProcessMemoryTask, this);
			}
#endif

			char yuvfile[128];
			#if HDR_DEBUG_FORCE_SINGLE_RUN
			sprintf(yuvfile, HDR_DEBUG_OUTPUT_FOLDER"hdr_sample_1600x1200_%d.i420", i);
			#else
			//sprintf(yuvfile, HDR_DEBUG_OUTPUT_FOLDER"hdr_sample_4000x3000_%d.i420", i);
			sprintf(yuvfile, HDR_DEBUG_OUTPUT_FOLDER"hdr_sample_3264x2448_%d.i420", i);
			#endif

			uint32_t nReadSize = loadFileToBuf(yuvfile, (uint8_t*)mpSourceImgBuf[i].virtAddr, mu4SourceSize);
			if(nReadSize < mpSourceImgBuf[i].size) {
				MY_DBG("[createSourceAndFirstRunSourceImg] can't read enought data");
				ret = MFALSE;
			}
			mpCamExif[i] = new CamExif;
	    }

		//first run source
		ret = ret && createFirstRunSourceImg();

		goto lbCaptureDone;
	}
#endif

{
	MY_DBG("[createSourceAndFirstRunSourceImg] - online");
    NSCamShot::ISingleShot *pSingleShot = NSCamShot::ISingleShot::createInstance(eShotMode_HdrShot, "testshot");
    //
    pSingleShot->init();

    // shot param
    NSCamShot::ShotParam rShotParam(
    					//mpSourceImgBuf
						eImgFmt_I420,			//yuv format
                        mu4W_yuv,				//picutre width
                        mu4H_yuv,				//picture height
                        //mpFirstRunSourceImgBuf
                        0,						//postview rotation
                        0,						//postview flip
                        eImgFmt_I420,			//postview format
                        mu4W_first,				//postview width
                        mu4H_first,				//postview height
                        0,						//postview rotation
                        0,						//postview flip
                        mShotParam.mu4ZoomRatio	//zoom
                        );

    // sensor param
	NSCamShot::SensorParam rSensorParam(static_cast<MUINT32>(MtkCamUtils::DevMetaInfo::queryHalSensorDev(getOpenId())),                             //Device ID
                             ACDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG,         //Scenaio
                             10,                                    //bit depth
                             //@TODO MFALSE,                                     //bypass delay
                             MFALSE,                                //bypass delay
                             MFALSE,                                //bypass scenario
                             0,                                     //RawType
                             MFALSE                                 //bypass AE capture shutter delay   
                            );
	//
	pSingleShot->setCallbacks(fgCamShotNotifyCb, fgCamShotDataCb, this);
	//
    pSingleShot->setShotParam(rShotParam);
    //
    //pSingleShot->setJpegParam(rJpegParam);

#if !HDR_DEBUG_SKIP_3A
    NS3A::Hal3ABase* p3AHal = NS3A::Hal3ABase::createInstance(MtkCamUtils::DevMetaInfo::queryHalSensorDev(getOpenId()));
    //NS3A::CaptureParam_T rCap3AParam;
    //for 6572 YUV imgsensor
    NS3A::Param_T cam3aParam;
#if HDR_SWITCH_CAPTURE_SEQUENCE
    MUINT32 u4EVIndex[3] = {0, -2, 2};
#else
    MUINT32 u4EVIndex[3] = {-2, 0, 2};
#endif
    if (2 == mu4OutputFrameNum)
    {
        u4EVIndex[0] = -2;
        u4EVIndex[1] = 2;
    }
#endif

    for(int i=0; i<mu4OutputFrameNum; i++) {
#if	HDR_SPEEDUP_MALLOC == 1
		if(i==1) {
			MUINT32	threadRet = 0;
			pthread_join(mCaptureIMemThread, (void**)&threadRet);
			mCaptureIMemThread = NULL;
			if(!threadRet) {
				MY_ERR("join mCaptureIMemThread fail");
				ret = MFALSE;
				break;
			}
			pthread_create(&mProcessIMemThread, NULL, HdrShot::allocateProcessMemoryTask, this);
		}
#endif

		// (1)	notify
		pSingleShot->disableNotifyMsg(0xFFFFFFFF);
		//if(i==0 || i==mu4OutputFrameNum-1) {
		if(i==0) {
			//shutter sound
			pSingleShot->enableNotifyMsg(NSCamShot::ECamShot_NOTIFY_MSG_EOF);
		}

		// (2)	data message
		pSingleShot->disableDataMsg(0xFFFFFFFF);
		pSingleShot->enableDataMsg(	NSCamShot::ECamShot_DATA_MSG_YUV
										| NSCamShot::ECamShot_DATA_MSG_POSTVIEW);

		MUINT32	stride[3];
		// (3)	regist mpSourceImgBuf
		GetStride(mu4W_yuv, eImgFmt_I420, stride);
		NSCamHW::ImgInfo	sourceImgInfo(eImgFmt_I420, mu4W_yuv, mu4H_yuv);
		NSCamHW::BufInfo	sourceBufInfo(mpSourceImgBuf[i].size, mpSourceImgBuf[i].virtAddr, mpSourceImgBuf[i].phyAddr, mpSourceImgBuf[i].memID);
		NSCamHW::ImgBufInfo	sourceImgBufInfo(sourceImgInfo, sourceBufInfo, stride);
		pSingleShot->registerImgBufInfo(NSCamShot::ECamShot_BUF_TYPE_YUV, sourceImgBufInfo);

		// (4)	regist mpFirstRunSourceImgBuf
		GetStride(mu4W_first, eImgFmt_I420, stride);
		NSCamHW::ImgInfo	firstImgInfo(eImgFmt_I420, mu4W_first, mu4H_first);
		NSCamHW::BufInfo	firstBufInfo(mpFirstRunSourceImgBuf[i].size, mpFirstRunSourceImgBuf[i].virtAddr, mpFirstRunSourceImgBuf[i].phyAddr, mpFirstRunSourceImgBuf[i].memID);
		NSCamHW::ImgBufInfo	firstImgBufInfo(firstImgInfo, firstBufInfo, stride);
		pSingleShot->registerImgBufInfo(NSCamShot::ECamShot_BUF_TYPE_POSTVIEW, firstImgBufInfo);

		// (5)	update 3A & capture
#if !HDR_DEBUG_SKIP_3A
		//p3AHal->getCaptureParams(i, 0, rCap3AParam);
		//p3AHal->updateCaptureParams(rCap3AParam);
        
        //for 6572 YUV imgsensor
#if HDR_SWITCH_CAPTURE_SEQUENCE
        rSensorParam.fgBypassAEShutterDelay= (0 == i) ? MTRUE : MFALSE;
#else
        rSensorParam.fgBypassAEShutterDelay = MFALSE;
#endif

        if ( ! p3AHal->getParams(cam3aParam) )
        {
             MY_ERR("[createSourceAndFirstRunSourceImg] getParams fail");
        }
        cam3aParam.i4ExpIndex = u4EVIndex[i];
        MY_DBG("[createSourceAndFirstRunSourceImg] cam3aParam.i4ExpIndex=%d",cam3aParam.i4ExpIndex);

        if ( ! p3AHal->setParams(cam3aParam) )
        {
            MY_ERR("[createSourceAndFirstRunSourceImg] setParams fail");
        }
#endif
		rSensorParam.fgBypassScenaio= (0 != i) ? MTRUE : MFALSE;
		CPTLog(Event_HdrShot_SingleCapture, CPTFlagStart);
		pSingleShot->startOne(rSensorParam);
		CPTLog(Event_HdrShot_SingleCapture, CPTFlagEnd);


		// (6)	save Exif
		mpCamExif[i] = new CamExif;
		update3AExif(mpCamExif[i]);	//@TODO release mpCamExif[i]
	}

#if HDR_SWITCH_CAPTURE_SEQUENCE
    MINT32 u4tempAddr = 0;
    if (mu4OutputFrameNum == 3)
    {
        u4tempAddr = mpSourceImgBuf[0].virtAddr;
        mpSourceImgBuf[0].virtAddr = mpSourceImgBuf[1].virtAddr;
        mpSourceImgBuf[1].virtAddr = u4tempAddr;
        
        u4tempAddr = mpSourceImgBuf[0].phyAddr;
        mpSourceImgBuf[0].phyAddr = mpSourceImgBuf[1].phyAddr;
        mpSourceImgBuf[1].phyAddr = u4tempAddr;

        u4tempAddr = mpSmallImgBuf[0].virtAddr;
        mpSmallImgBuf[0].virtAddr = mpSmallImgBuf[1].virtAddr;
        mpSmallImgBuf[1].virtAddr = u4tempAddr;

        u4tempAddr = mpSmallImgBuf[0].phyAddr;
        mpSmallImgBuf[0].phyAddr = mpSmallImgBuf[1].phyAddr;
        mpSmallImgBuf[1].phyAddr = u4tempAddr;
    }
#endif

#if !HDR_DEBUG_SKIP_3A
	p3AHal->destroyInstance();
#endif
    //
    pSingleShot->uninit();
    //
    pSingleShot->destroyInstance();
}

lbCaptureDone:

	if(!ret) goto lbExit;

	//@TODO dump raw when mDebugMode==1
	if(CUST_HDR_DEBUG || HDR_DEBUG_SAVE_SOURCE_IMAGE || mDebugMode) {
		for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
		{
			char szFileName[100];

			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_1_mpSourceImgBuf[%d]_%dx%d_r%d.i420", mu4RunningNumber, i, mu4W_yuv, mu4H_yuv, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)mpSourceImgBuf[i].virtAddr, mu4SourceSize);

			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_1_mpFirstRunSourceImgBuf[%d]_%dx%d_r%d.i420", mu4RunningNumber, i, mu4W_first, mu4H_first, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)mpFirstRunSourceImgBuf[i].virtAddr, mu4FirstRunSourceSize);
		}
	}

	// ()	copy y from first run to small
	for(int i=0; i<mu4OutputFrameNum; i++) {
		MY_DBG("[createSourceAndFirstRunSourceImg] copy y from first(%d) to small(%d)", mpFirstRunSourceImgBuf[i].size, mpSmallImgBuf[i].size);
		memcpy((void*)mpSmallImgBuf[i].virtAddr
				, (void*)mpFirstRunSourceImgBuf[i].virtAddr
				, mpSmallImgBuf[i].size);
		MY_DBG("[createSourceAndFirstRunSourceImg] copy y from first(%d) to small(%d) done", mpFirstRunSourceImgBuf[i].size, mpSmallImgBuf[i].size);
	}

	if(HDR_DEBUG_SAVE_SMALL_IMAGE || mDebugMode) {
		for (MUINT32 i = 0; i < mu4OutputFrameNum; i++)
		{
			char szFileName[100];

			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_2_mpSmallImgBuf[%d]_%dx%d_r%d.y", mu4RunningNumber, i, mu4W_small, mu4H_small, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)mpSmallImgBuf[i].virtAddr, mpSmallImgBuf[i].size);
		}
	}

lbExit:
	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
GetStride(MUINT32 srcWidth, EImageFormat srcFormat, MUINT32 *pStride)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

	switch(srcFormat) {
		case eImgFmt_NV12:
		case eImgFmt_NV21:
		case eImgFmt_YUY2:
			pStride[0] = pStride[1] = pStride[2] = srcWidth;
			break;
		case eImgFmt_I420:
		case eImgFmt_YV12:
			pStride[0] = srcWidth;
			pStride[1] = pStride[2] = srcWidth / 2;
			break;
		case eImgFmt_Y800:
			pStride[0] = srcWidth;
			pStride[1] = pStride[2] = 0;
			break;
		default:
			pStride[0] = pStride[1] = pStride[2] = srcWidth;
			MY_ERR("GetStride: unspported format %d", srcFormat);
			ret = MFALSE;
	}

	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
CDPResize(IMEM_BUF_INFO *srcMem, MUINT32 srcWidth, MUINT32 srcHeight, EImageFormat srcFormat,
	IMEM_BUF_INFO *desMem, MUINT32 desWidth, MUINT32 desHeight, EImageFormat desFormat,
	MUINT32 rotation)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

	IMEM_BUF_INFO tempInfo[2];
	MUINT32 tempWidth[2];
	MUINT32 tempHeight[2];
	MFLOAT tempSizeMulti = (MFLOAT)srcMem->size / (srcWidth*srcHeight);

 	//init
	if((desWidth>32*srcWidth)
			|| (desHeight>32*srcHeight)) {
		//prepare source
		tempWidth[0] = srcWidth;
		tempHeight[0] = srcHeight;
		MBOOL isFirstRun = MTRUE;
		tempInfo[0] = *srcMem;

		while(1) {
			//prepare target
			MY_DBG("[CDPResize] - prepare target");
			tempWidth[1] = desWidth;
			tempHeight[1] = desHeight;

			while(tempWidth[1] > tempWidth[0]*32)
				tempWidth[1] = (tempWidth[1]+31)/32;
			while(tempHeight[1] > tempHeight[0]*32)
				tempHeight[1] = (tempHeight[1]+31)/32;
			tempWidth[1] = (tempWidth[1]+1)&~1;
			tempHeight[1] = (tempHeight[1]+1)&~1;
			MY_DBG("[CDPResize] - desWidth=%d desHeight=%d", desWidth, desHeight);
			MY_DBG("[CDPResize] - tempWidth[0]=%d tempHeight[0]=%d", tempWidth[0], tempHeight[0]);
			MY_DBG("[CDPResize] - tempWidth[1]=%d tempHeight[1]=%d", tempWidth[1], tempHeight[1]);

			//scale up - last round
			if(tempWidth[1]==desWidth && tempHeight[1]==desHeight) {
				MY_DBG("[CDPResize] - scale up - final round");
				MBOOL ret;
				ret = CDPResize_simple(&tempInfo[0], tempWidth[0], tempHeight[0], srcFormat
					, desMem, desWidth, desHeight, desFormat
					, rotation);
				#if 1
				deallocMem(&tempInfo[0]);
				#else
				if(!isFirstRun)
					deallocMem(&tempInfo[0]);
				#endif
				return ret;
			}

			//scale up
			MY_DBG("[CDPResize] - scale up");
			tempInfo[1].size = tempWidth[1] * tempHeight[1] * tempSizeMulti;
			if(allocMem(&tempInfo[1])) {
				ret = MFALSE;
				goto lbExit;
			}
			CDPResize_simple(&tempInfo[0], tempWidth[0], tempHeight[0], srcFormat
				, &tempInfo[1], tempWidth[1], tempHeight[1], srcFormat
				, rotation);
			if(!isFirstRun)
				deallocMem(&tempInfo[0]);
			tempWidth[0] = tempWidth[1];
			tempHeight[0] = tempHeight[1];
			tempInfo[0] = tempInfo[1];

			isFirstRun = MFALSE;
		}

	}

	ret = CDPResize_simple(srcMem, srcWidth, srcHeight, srcFormat,
			desMem, desWidth, desHeight, desFormat, rotation);
lbExit:
	FUNCTION_LOG_END;
	return ret;
}


MBOOL
HdrShot::
CDPResize_simple(IMEM_BUF_INFO *srcMem, MUINT32 srcWidth, MUINT32 srcHeight, EImageFormat srcFormat,
	IMEM_BUF_INFO *desMem, MUINT32 desWidth, MUINT32 desHeight, EImageFormat desFormat,
	MUINT32 rotation)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

	MY_DBG("[CDPResize] - srcMem=%x", srcMem);
	MY_DBG("[CDPResize] - srcWidth=%d, srcHeight=%d", srcWidth, srcHeight);
	MY_DBG("[CDPResize] - srcFormat=%d", srcFormat);
	MY_DBG("[CDPResize] - desMem=%x", desMem);
	MY_DBG("[CDPResize] - desWidth=%d, desHeight=%d", desWidth, desHeight);
	MY_DBG("[CDPResize] - desFormat=%d", desFormat);

	MUINT32 u4Stride[3] = {0, 0, 0};

    // (1). Create Instance
	MY_DBG("[CDPResize] - (1). Create Instance");
	NSCamHW::ImgInfo rSourceImgInfo(srcFormat, srcWidth, srcHeight);
    NSCamHW::BufInfo rSourceBufInfo(srcMem->size, srcMem->virtAddr, srcMem->phyAddr, srcMem->memID);
	GetStride(srcWidth, srcFormat, u4Stride);
	NSCamHW::ImgBufInfo rSourceImgBufInfo(rSourceImgInfo, rSourceBufInfo, u4Stride);
	MY_DBG("[CDPResize] - source stride %d,%d,%d", u4Stride[0], u4Stride[1], u4Stride[2]);

    NSCamShot::ISImager *pISImager = NSCamShot::ISImager::createInstance(rSourceImgBufInfo);
    if(!pISImager) {
		return MFALSE;
    }

	// (2). Set Output info for small image
	MY_DBG("[CDPResize] - (2). Set Output info for small image");
    NSCamHW::BufInfo rTargetBufInfo(desMem->size, desMem->virtAddr, desMem->phyAddr, desMem->memID);

    // (3). init setting
	MY_DBG("[CDPResize] - (3). init setting");
    pISImager->setTargetBufInfo(rTargetBufInfo);
    pISImager->setFormat(desFormat);
    pISImager->setRotation(rotation);
    pISImager->setFlip(0);
    pISImager->setResize(desWidth, desHeight);
    pISImager->setEncodeParam(1, 90);
    pISImager->setROI(Rect(0, 0, srcWidth, srcHeight));
	MY_DBG("[CDPResize] - before execute() t5");
    ret = pISImager->execute();
	MY_DBG("[CDPResize] - after execute()");
    pISImager->destroyInstance();
	MY_DBG("[CDPResize] - finish");

lbExit:
	FUNCTION_LOG_END;
	return ret;
}


#if 1
MBOOL
HdrShot::
createFirstRunSourceImg(void)
{
	FUNCTION_LOG_START;
	MBOOL  ret = MTRUE;
	MUINT32 u4OutputFrameNum = OutputFrameNumGet();
	MY_DBG("[createFirstRunSourceImg] - E. u4OutputFrameNum: %d.", u4OutputFrameNum);

#if (HDR_PROFILE_CAPTURE)
    MyDbgTimer DbgTmr("createFirstRunSourceImg");
#endif

	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
	{
		ret = CDPResize(
			&mpSourceImgBuf[i], mu4W_yuv, mu4H_yuv, eImgFmt_I420,
			&mpFirstRunSourceImgBuf[i], mu4W_first, mu4H_first, eImgFmt_I420, 0);
	}


#if (HDR_PROFILE_CAPTURE)
	DbgTmr.print("HdrProfiling:: createFirstRunSourceImg Time");
#endif

	FUNCTION_LOG_END;
	return	ret;
}
#endif

MBOOL
HdrShot::
createSmallImg(void)
{
	FUNCTION_LOG_START;
	MBOOL  ret = MTRUE;
	MUINT32 u4OutputFrameNum = OutputFrameNumGet();

#if (HDR_PROFILE_CAPTURE)
    MyDbgTimer DbgTmr("createSmallImg");
#endif

	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
	{
		ret = CDPResize(
			&mpSourceImgBuf[i], mu4W_yuv, mu4H_yuv, eImgFmt_I420,
			&mpSmallImgBuf[i], mu4W_small, mu4H_small, eImgFmt_Y800, 0);
	}


#if (HDR_PROFILE_CAPTURE)
	DbgTmr.print("HdrProfiling:: createSmallImg Time");
#endif

	FUNCTION_LOG_END;
	return	ret;
}

MBOOL
HdrShot::
createSEImg(void)
{
	FUNCTION_LOG_START;
	MBOOL  ret = MTRUE;
	MUINT32 u4OutputFrameNum = OutputFrameNumGet();

#if (HDR_PROFILE_CAPTURE)
	MyDbgTimer DbgTmr("createSEImg");
#endif

	for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
	{
		MY_DBG("[createSEImg] - CDP %d/%d.", i, u4OutputFrameNum);
		ret = ret
		&&	CDPResize(&mpSmallImgBuf[i], mu4W_small, mu4H_small, eImgFmt_Y800,
						&mpSEImgBuf[i], mu4W_se, mu4H_se, eImgFmt_Y800, 0);
	}

#if (HDR_PROFILE_CAPTURE)
	DbgTmr.print("HdrProfiling:: createSEImg Time");
#endif

	if(HDR_DEBUG_SAVE_SE_IMAGE || mDebugMode) {
		for (MUINT32 i = 0; i < u4OutputFrameNum; i++)
		{
			char szFileName[100];
			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_4_mpSEImgBuf[%d]_%dx%d_r%d.y", mu4RunningNumber, i, mu4W_se, mu4H_se, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)mpSEImgBuf[i].virtAddr, mu4SEImgSize);
		}
	}

	FUNCTION_LOG_END;
	return	ret;
}


MBOOL
HdrShot::
createJpegImg(NSCamHW::ImgBufInfo const & rSrcImgBufInfo
			, NSCamShot::JpegParam const & rJpgParm
			, MUINT32 const u4Rot
			, MUINT32 const u4Flip
			, NSCamHW::ImgBufInfo const & rJpgImgBufInfo
			, MUINT32 & u4JpegSize)
{
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

    // (0). debug
    MY_DBG("[createJpegImg] - rSrcImgBufInfo.eImgFmt=0x%x", rSrcImgBufInfo.eImgFmt);
    MY_DBG("[createJpegImg] - u4Rot=%d", u4Rot);
    MY_DBG("[createJpegImg] - u4Flip=%d", u4Flip);
    //
    // (1). Create Instance
    NSCamShot::ISImager *pISImager = NSCamShot::ISImager::createInstance(rSrcImgBufInfo);
    if(!pISImager) {
		MY_ERR("HdrShot::createJpegImg can't get ISImager instance.");
		return MFALSE;
    }

    // init setting
    NSCamHW::BufInfo rBufInfo(rJpgImgBufInfo.u4BufSize, rJpgImgBufInfo.u4BufVA, rJpgImgBufInfo.u4BufPA, rJpgImgBufInfo.i4MemID);
    //
    pISImager->setTargetBufInfo(rBufInfo);
    //
    pISImager->setFormat(eImgFmt_JPEG);
    //
    pISImager->setRotation(u4Rot);
    //
    pISImager->setFlip(u4Flip);
    //
    pISImager->setResize(rJpgImgBufInfo.u4ImgWidth, rJpgImgBufInfo.u4ImgHeight);
    //
    pISImager->setEncodeParam(rJpgParm.fgIsSOI, rJpgParm.u4Quality);
    //
    pISImager->setROI(Rect(0, 0, rSrcImgBufInfo.u4ImgWidth, rSrcImgBufInfo.u4ImgHeight));
    //
    pISImager->execute();
    //
    u4JpegSize = pISImager->getJpegSize();

    pISImager->destroyInstance();


	FUNCTION_LOG_END;
    return ret;
}


NSCamHW::ImgBufInfo
HdrShot::
imem2ImgBuf(IMEM_BUF_INFO imembufinfo, EImageFormat format
			, MUINT32 widht, MUINT32 height)
{
	MUINT32 stride[3];
	NSCamHW::ImgInfo	imgInfo(format, widht, height);
	NSCamHW::BufInfo	bufInfo(imembufinfo.size, imembufinfo.virtAddr, imembufinfo.phyAddr, imembufinfo.memID);
	NSCamHW::ImgBufInfo	ImgBufInfo(imgInfo, bufInfo, stride);
	return ImgBufInfo;
}


MVOID*
HdrShot::
createNormalJpegImgTask(MVOID* arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	//non-real-time policy
	SetThreadProp(SCHED_OTHER, -20);

	HdrShot *self = (HdrShot*)arg;

	//HDR
	if(!self->mfgIsForceBreak) {
		ret = ret && (MBOOL)self->encodeHdrThumbnailJpeg(arg);
	}

	//NORMAL
	if(!self->mfgIsForceBreak) {
		ret = ret && (MBOOL)self->encodeNormalThumbnailJpeg(arg);
	}
	if(!self->mfgIsForceBreak) {
		ret = ret && (MBOOL)self->encodeNormalJpeg(arg);
	}
	if(!self->mfgIsForceBreak) {
		ret = ret && (MBOOL)self->saveNormalJpeg(arg);
	}
	self->releaseNormalJpegBuf();
	self->releaseNormalThumbnailJpegBuf();

	FUNCTION_LOG_END;
    return (MVOID*)ret;
}


MVOID*
HdrShot::
allocateCaptureMemoryTask(MVOID* arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	HdrShot *self = (HdrShot*)arg;

	//allocate buffers for 2rd & 3nd capture
#if 1
	ret = ret
		&&	self->requestSourceImgBuf()
		&&	self->requestFirstRunSourceImgBuf()
		&&	self->requestSmallImgBuf()
		;
#else
	ret = MFALSE;
#endif

	if(!ret) {
		MY_ERR("can't alloc memory");
	}
lbExit:
	FUNCTION_LOG_END;
    return (MVOID*)ret;
}


MVOID*
HdrShot::
allocateProcessMemoryTask(MVOID* arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	HdrShot *self = (HdrShot*)arg;

	//allocate buffers for MAV & HDR Core
	ret = ret
		&&	self->requestHdrWorkingBuf()
		;

	if(!ret) {
		MY_ERR("can't alloc memory");
	}
lbExit:
	FUNCTION_LOG_END;
    return (MVOID*)ret;
}


MVOID*
HdrShot::
encodeNormalThumbnailJpeg(MVOID *arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	MUINT32 stride[3];
	MUINT32	i = OutputFrameNumGet()/2;

	GetStride(mu4W_yuv, eImgFmt_I420, stride);
	NSCamHW::ImgInfo	sourceImgInfo(eImgFmt_I420, mu4W_yuv, mu4H_yuv);
	NSCamHW::BufInfo	sourceBufInfo(mpSourceImgBuf[i].size, mpSourceImgBuf[i].virtAddr, mpSourceImgBuf[i].phyAddr, mpSourceImgBuf[i].memID);
	NSCamHW::ImgBufInfo	sourceImgBufInfo(sourceImgInfo, sourceBufInfo, stride);

	//rThumbImgBufInfo
	requestNormalThumbnailJpegBuf();
	NSCamHW::ImgInfo		rThumbImgInfo(eImgFmt_JPEG, mJpegParam.mi4JpegThumbWidth, mJpegParam.mi4JpegThumbHeight);
	NSCamHW::BufInfo		rThumbBufInfo(mNormalThumbnailJpegBuf.size, mNormalThumbnailJpegBuf.virtAddr, mNormalThumbnailJpegBuf.phyAddr, mNormalThumbnailJpegBuf.memID);
	NSCamHW::ImgBufInfo		rThumbImgBufInfo(rThumbImgInfo, rThumbBufInfo, stride);

    MUINT32 u4ThumbSize = 0;
    if (0 != mJpegParam.mi4JpegThumbWidth && 0 != mJpegParam.mi4JpegThumbHeight)
    {
    	//EXIF add SOI tag
        NSCamShot::JpegParam rParam(mJpegParam.mu4JpegThumbQuality, MFALSE);
        ret = ret && createJpegImg(sourceImgBufInfo, rParam, mShotParam.mi4Rotation, 0, rThumbImgBufInfo, mNormalThumbnailJpegBufSize);
    }

	FUNCTION_LOG_END;
	return (MVOID*)ret;
}


MVOID*
HdrShot::
encodeNormalJpeg(MVOID *arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	MUINT32 stride[3];
	MUINT32	i = OutputFrameNumGet()/2;

	GetStride(mu4W_yuv, eImgFmt_I420, stride);
	NSCamHW::ImgInfo	sourceImgInfo(eImgFmt_I420, mu4W_yuv, mu4H_yuv);
	NSCamHW::BufInfo	sourceBufInfo(mpSourceImgBuf[i].size, mpSourceImgBuf[i].virtAddr, mpSourceImgBuf[i].phyAddr, mpSourceImgBuf[i].memID);
	NSCamHW::ImgBufInfo	sourceImgBufInfo(sourceImgInfo, sourceBufInfo, stride);

	//rJpegImgBufInfo
	//IMEM_BUF_INFO	mNormalJpegBuf;
	requestNormalJpegBuf();
	NSCamHW::ImgInfo		rJpegImgInfo(eImgFmt_JPEG, mu4W_yuv, mu4H_yuv);
	NSCamHW::BufInfo		rJpegBufInfo(mNormalJpegBuf.size, mNormalJpegBuf.virtAddr, mNormalJpegBuf.phyAddr, mNormalJpegBuf.memID);
	NSCamHW::ImgBufInfo		rJpegImgBufInfo(rJpegImgInfo, rJpegBufInfo, stride);

    MUINT32 u4JpegSize = 0;
    NSCamShot::JpegParam yuvJpegParam(mJpegParam.mu4JpegQuality, MFALSE);
    ret = ret && createJpegImg(sourceImgBufInfo, yuvJpegParam, mShotParam.mi4Rotation, 0 , rJpegImgBufInfo, mNormalJpegBufSize);

	FUNCTION_LOG_END;
	return (MVOID*)ret;
}


MVOID*
HdrShot::
saveNormalJpeg(MVOID *arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	MUINT32	u4Index = 0;
	MBOOL	bFinal = MTRUE;

    // Jpeg callback, it contains thumbnail in ext1, ext2.
    handleJpegData((MUINT8*)mNormalJpegBuf.virtAddr, mNormalJpegBufSize
    				, (MUINT8*)mNormalThumbnailJpegBuf.virtAddr, mNormalThumbnailJpegBufSize
    				, 0, 0);

	FUNCTION_LOG_END;
	return (MVOID*)ret;
}


MVOID*
HdrShot::
saveFileTask(MVOID* arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	HdrFileInfo *pFileInfo = (HdrFileInfo*)arg;
	dumpToFile(pFileInfo->filename.string()
				, pFileInfo->buffer
				, pFileInfo->size
				);
	delete pFileInfo->buffer;
	delete pFileInfo;

	pthread_detach(pthread_self());

	FUNCTION_LOG_END;
	return (MVOID*)ret;
}


MVOID*
HdrShot::
createHdrJpegImgTask(MVOID* arg)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

	//non-real-time policy
	SetThreadProp(SCHED_OTHER, -20);

	HdrShot *self = (HdrShot*)arg;

	//ret = ret && (MBOOL)self->encodeHdrThumbnailJpeg(arg);
	if(!self->mfgIsForceBreak ) {
		ret = ret && (MBOOL)self->encodeHdrJpeg(arg);
	}
	if(!self->mfgIsForceBreak ) {
		ret = ret && (MBOOL)self->saveHdrJpeg(arg);
	}
	self->releaseHdrJpegBuf();
	self->releaseHdrThumbnailJpegBuf();

	FUNCTION_LOG_END;
    return (MVOID*)ret;
}


MVOID*
HdrShot::
encodeHdrThumbnailJpeg(MVOID *arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	MUINT32 	u4Stride[3];
	MUINT32		stride[3];

	//mpPostviewImgBuf as rPostViewBufInfo
	GetStride(mPostviewWidth, mPostviewFormat, u4Stride);
	NSCamHW::ImgInfo		rPostViewImgInfo(mPostviewFormat, mPostviewWidth, mPostviewHeight);
	NSCamHW::BufInfo		rPostViewBufInfo(mpPostviewImgBuf.size, mpPostviewImgBuf.virtAddr, mpPostviewImgBuf.phyAddr, mpPostviewImgBuf.memID);
	NSCamHW::ImgBufInfo		rPostViewImgBufInfo(rPostViewImgInfo, rPostViewBufInfo, u4Stride);

	//rThumbImgBufInfo
	requestHdrThumbnailJpegBuf();
	NSCamHW::ImgInfo		rThumbImgInfo(eImgFmt_JPEG, mJpegParam.mi4JpegThumbWidth, mJpegParam.mi4JpegThumbHeight);
	NSCamHW::BufInfo		rThumbBufInfo(mHdrThumbnailJpegBuf.size, mHdrThumbnailJpegBuf.virtAddr, mHdrThumbnailJpegBuf.phyAddr, mHdrThumbnailJpegBuf.memID);
	NSCamHW::ImgBufInfo		rThumbImgBufInfo(rThumbImgInfo, rThumbBufInfo, stride);

    //MUINT32 mHdrThumbnailJpegBufSize = 0;
    if (0 != mJpegParam.mi4JpegThumbWidth && 0 != mJpegParam.mi4JpegThumbHeight)
    {
    	//EXIF add SOI tag
        NSCamShot::JpegParam rParam(mJpegParam.mu4JpegThumbQuality, MFALSE);
        ret = ret && createJpegImg(rPostViewImgBufInfo, rParam, mShotParam.mi4Rotation, 0, rThumbImgBufInfo, mHdrThumbnailJpegBufSize);
    }

	FUNCTION_LOG_END;
	return (MVOID*)ret;
}


MVOID*
HdrShot::
encodeHdrJpeg(MVOID *arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	MUINT32 	u4Stride[3];
	MUINT32		stride[3];

	//mrHdrCroppedResult as rYuvImgBufInfo
	MUINT32 				u4HdrCroppedResultSize = mrHdrCroppedResult.output_image_width * mrHdrCroppedResult.output_image_height * 3 / 2;
	MUINT32					rYuvSize = mrHdrCroppedResult.output_image_width * mrHdrCroppedResult.output_image_height * 3/2;	//NV21
	GetStride(mrHdrCroppedResult.output_image_width, eImgFmt_I420, u4Stride);
	NSCamHW::ImgInfo		rYuvImgInfo(eImgFmt_I420, mrHdrCroppedResult.output_image_width , mrHdrCroppedResult.output_image_height);
	//NSCamHW::BufInfo		rYuvBufInfo(u4HdrCroppedResultSize, (MUINT32)mrHdrCroppedResult.output_image_addr, 0, mpHdrWorkingBuf.memID);
	NSCamHW::BufInfo		rYuvBufInfo(u4HdrCroppedResultSize
										, (MUINT32)mBlendingBuf.virtAddr
										, mBlendingBuf.phyAddr
										, mBlendingBuf.memID);
	NSCamHW::ImgBufInfo		rYuvImgBufInfo(rYuvImgInfo, rYuvBufInfo, u4Stride);

	//rJpegImgBufInfo
	requestHdrJpegBuf();
	NSCamHW::ImgInfo		rJpegImgInfo(eImgFmt_JPEG, mu4W_yuv, mu4H_yuv);
	NSCamHW::BufInfo		rJpegBufInfo(mHdrJpegBuf.size, mHdrJpegBuf.virtAddr, mHdrJpegBuf.phyAddr, mHdrJpegBuf.memID);
	NSCamHW::ImgBufInfo		rJpegImgBufInfo(rJpegImgInfo, rJpegBufInfo, stride);


    //MUINT32 mHdrJpegBufSize = 0;
    // create jpeg bitstreawm
    NSCamShot::JpegParam yuvJpegParam(mJpegParam.mu4JpegQuality, MFALSE);
    ret = ret && createJpegImg(rYuvImgBufInfo, yuvJpegParam, mShotParam.mi4Rotation, 0 , rJpegImgBufInfo, mHdrJpegBufSize);

	FUNCTION_LOG_END;
	return (MVOID*)ret;
}


MVOID*
HdrShot::
saveHdrJpeg(MVOID *arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	MUINT32	u4Index = 0;
	MBOOL	bFinal = MTRUE;

#if	0
	system("mkdir /sdcard/meminfo");
	system("cat /proc/meminfo >/sdcard/meminfo/proc_meminfo.log");
	system("procrank >/sdcard/meminfo/procrank.log");
#endif

    // Jpeg callback, it contains thumbnail in ext1, ext2.
    handleJpegData((MUINT8*)mHdrJpegBuf.virtAddr, mHdrJpegBufSize
    				, (MUINT8*)mHdrThumbnailJpegBuf.virtAddr, mHdrThumbnailJpegBufSize
    				, u4Index, bFinal);

	FUNCTION_LOG_END;
	return (MVOID*)ret;
}


MBOOL
HdrShot::
createJpegImgWithThumbnail(NSCamHW::ImgBufInfo const &rYuvImgBufInfo, NSCamHW::ImgBufInfo const &rPostViewImgBufInfo, MUINT32 const u4Index, MBOOL bFinal)
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

#if 1	//convert into ImgBufInfo
	MUINT32	stride[3];

	//rJpegImgBufInfo
	IMEM_BUF_INFO	jpegBuf;
	jpegBuf.size = mu4W_yuv * mu4H_yuv;
	#if 1	//test
	if(allocMem(&jpegBuf)) {
		ret = MFALSE;
	}
	#else
	mpIMemDrv->allocVirtBuf(&jpegBuf);
	#endif
	NSCamHW::ImgInfo		rJpegImgInfo(eImgFmt_JPEG, mu4W_yuv, mu4H_yuv);
	NSCamHW::BufInfo		rJpegBufInfo(jpegBuf.size, jpegBuf.virtAddr, jpegBuf.phyAddr, jpegBuf.memID);
	NSCamHW::ImgBufInfo		rJpegImgBufInfo(rJpegImgInfo, rJpegBufInfo, stride);

	//rThumbImgBufInfo
	IMEM_BUF_INFO	thumbBuf;
	thumbBuf.size = mJpegParam.mi4JpegThumbWidth * mJpegParam.mi4JpegThumbHeight;
	#if 1	//test
	if(allocMem(&thumbBuf)) {
		ret = MFALSE;
	}
	#else
	mpIMemDrv->allocVirtBuf(&thumbBuf);
	#endif
	NSCamHW::ImgInfo		rThumbImgInfo(eImgFmt_JPEG, mJpegParam.mi4JpegThumbWidth, mJpegParam.mi4JpegThumbHeight);
	NSCamHW::BufInfo		rThumbBufInfo(thumbBuf.size, thumbBuf.virtAddr, thumbBuf.phyAddr, thumbBuf.memID);
	NSCamHW::ImgBufInfo		rThumbImgBufInfo(rThumbImgInfo, rThumbBufInfo, stride);
#endif

    MUINT32 u4JpegSize = 0;
    MUINT32 u4ThumbSize = 0;
    // create jpeg bitstreawm
    //ImgBufInfo rJpegImgBufInfo = queryJpegImgBufInfo();
    //ImgBufInfo rThumbImgBufInfo = queryThumbImgBufInfo();
        // jpeg param
    	//EXIF add SOI tag
    NSCamShot::JpegParam yuvJpegParam(mJpegParam.mu4JpegQuality, MFALSE);
    ret = ret && createJpegImg(rYuvImgBufInfo, yuvJpegParam, mShotParam.mi4Rotation, 0 , rJpegImgBufInfo, u4JpegSize);

    // (3.1) create thumbnail
    // If postview is enable, use postview buffer,
    // else use yuv buffer to do thumbnail
    if (0 != mJpegParam.mi4JpegThumbWidth && 0 != mJpegParam.mi4JpegThumbHeight)
    {
    	//EXIF add SOI tag
        NSCamShot::JpegParam rParam(mJpegParam.mu4JpegThumbQuality, MFALSE);
        ret = ret && createJpegImg(rPostViewImgBufInfo, rParam, mShotParam.mi4Rotation, 0, rThumbImgBufInfo, u4ThumbSize);
    }

	if(HDR_DEBUG_SAVE_HDR_JPEG || mDebugMode) {
		if(bFinal) {
			char szFileName[100];

			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_a_rJpegImgBufInfo_%dx%d_r%d.jpg", mu4RunningNumber, mu4W_yuv, mu4H_yuv, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)jpegBuf.virtAddr, u4JpegSize);

			::sprintf(szFileName, HDR_DEBUG_OUTPUT_FOLDER "%04d_b_rThumbImgBufInfo_%dx%d_r%d.jpg", mu4RunningNumber, mJpegParam.mi4JpegThumbWidth, mJpegParam.mi4JpegThumbHeight, mHdrRound);
			dumpToFile(szFileName, (MUINT8*)thumbBuf.virtAddr, u4ThumbSize);
		}
	}


	if(bFinal) {
	    // Jpeg callback, it contains thumbnail in ext1, ext2.
	    handleJpegData((MUINT8*)rJpegImgBufInfo.u4BufVA, u4JpegSize, (MUINT8*)rThumbImgBufInfo.u4BufVA, u4ThumbSize, u4Index, bFinal);
	} else {
	#if 1
		// prepare exif buffer
	    MUINT8 *puExifHeaderBuf = new MUINT8[128 * 1024];
	    MUINT32 u4ExifHeaderSize = 0;
		MUINT8	*puThumbBuf = (MUINT8*)rThumbImgBufInfo.u4BufVA;
	    updateThumbnailExif(mpCamExif[1]
							, puThumbBuf, u4ThumbSize
	    					, puExifHeaderBuf, u4ExifHeaderSize);

		//combine exif & bit stream
		MUINT8 *puImageBuffer = new MUINT8[u4ExifHeaderSize+u4JpegSize];
		memcpy(puImageBuffer, puExifHeaderBuf, u4ExifHeaderSize);
		memcpy(puImageBuffer+u4ExifHeaderSize, (MUINT8*)jpegBuf.virtAddr, u4JpegSize);

	    MY_DBG("[handleJpegData] (thumbbuf, size, exifHeaderBuf, size) = (%p, %d, %p, %d)",
	                      puThumbBuf, u4ThumbSize, puExifHeaderBuf, u4ExifHeaderSize);

	    MY_DBG("[handleJpegData] (size: thumb=%d, header=%d, main=%d, all=%d)",
	                      u4ThumbSize, puExifHeaderBuf, u4JpegSize, u4ExifHeaderSize+u4JpegSize);
		dumpToFile(mShotParam.ms8ShotFileName, puImageBuffer, u4ExifHeaderSize+u4JpegSize);

		#if 1
		//for MR1
	    mpShotCallback->onCB_RawImage(0
	    							, u4ExifHeaderSize+u4JpegSize
	                                , reinterpret_cast<uint8_t const*>(puImageBuffer)
	                                );
		mRawCBDone = MTRUE;
		#endif

		delete puImageBuffer;
	    delete [] puExifHeaderBuf;

	#else
		dumpToFile(mShotParam.ms8ShotFileName, (MUINT8*)jpegBuf.virtAddr, u4JpegSize);
	#endif
	}

lbExit:
	#if 1	//test
	deallocMem(&jpegBuf);
	deallocMem(&thumbBuf);
	#else
	mpIMemDrv->freeVirtBuf(&jpegBuf);
	mpIMemDrv->freeVirtBuf(&thumbBuf);
	#endif

	FUNCTION_LOG_END;
    return ret;
}

MBOOL
HdrShot::
createHdrJpegImg()
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;
	MUINT32 		u4Stride[3];

	//mrHdrCroppedResult as rYuvImgBufInfo
	MUINT32 				u4HdrCroppedResultSize = mrHdrCroppedResult.output_image_width * mrHdrCroppedResult.output_image_height * 3 / 2;
	MUINT32					rYuvSize = mrHdrCroppedResult.output_image_width * mrHdrCroppedResult.output_image_height * 3/2;	//NV21
	GetStride(mrHdrCroppedResult.output_image_width, eImgFmt_I420, u4Stride);
	NSCamHW::ImgInfo		rYuvImgInfo(eImgFmt_I420, mrHdrCroppedResult.output_image_width , mrHdrCroppedResult.output_image_height);
	NSCamHW::BufInfo		rYuvBufInfo(u4HdrCroppedResultSize, (MUINT32)mrHdrCroppedResult.output_image_addr, 0, mpHdrWorkingBuf.memID);
	NSCamHW::ImgBufInfo		rYuvImgBufInfo(rYuvImgInfo, rYuvBufInfo, u4Stride);

	//mpPostviewImgBuf as rPostViewBufInfo
	GetStride(mPostviewWidth, mPostviewFormat, u4Stride);
	NSCamHW::ImgInfo		rPostViewImgInfo(mPostviewFormat, mPostviewWidth, mPostviewHeight);
	NSCamHW::BufInfo		rPostViewBufInfo(mpPostviewImgBuf.size, mpPostviewImgBuf.virtAddr, mpPostviewImgBuf.phyAddr, mpPostviewImgBuf.memID);
	NSCamHW::ImgBufInfo		rPostViewImgBufInfo(rPostViewImgInfo, rPostViewBufInfo, u4Stride);

	CPTLog(Event_HdrShot_SaveHdr, CPTFlagStart);
	ret = createJpegImgWithThumbnail(rYuvImgBufInfo
									, rPostViewImgBufInfo
									//, 1			//index
									, 0			//index
									, MTRUE);	//is final jpeg?
	CPTLog(Event_HdrShot_SaveHdr, CPTFlagEnd);

	FUNCTION_LOG_END;
    return ret;
}


MBOOL
HdrShot::
createNormalJpegImg()
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	MUINT32 stride[3];
	MUINT32	i = OutputFrameNumGet()/2;

	GetStride(mu4W_yuv, eImgFmt_I420, stride);
	NSCamHW::ImgInfo	sourceImgInfo(eImgFmt_I420, mu4W_yuv, mu4H_yuv);
	NSCamHW::BufInfo	sourceBufInfo(mpSourceImgBuf[i].size, mpSourceImgBuf[i].virtAddr, mpSourceImgBuf[i].phyAddr, mpSourceImgBuf[i].memID);
	NSCamHW::ImgBufInfo	sourceImgBufInfo(sourceImgInfo, sourceBufInfo, stride);

	CPTLog(Event_HdrShot_SaveNormal, CPTFlagStart);
	ret = createJpegImgWithThumbnail(sourceImgBufInfo
									, sourceImgBufInfo
									, 0			//index
									, MFALSE);	//is final jpeg?
	CPTLog(Event_HdrShot_SaveNormal, CPTFlagEnd);

	FUNCTION_LOG_END;
    return ret;
}


//@deprecated
MBOOL
HdrShot::
saveSourceJpg(void)
{
	FUNCTION_LOG_START;

    MBOOL ret = MTRUE;

	FUNCTION_LOG_END;
    return  ret;

}

#if 0	//@TODO deprecated
MBOOL
HdrShot::
saveRaw(void)
{
	FUNCTION_LOG_START;
    MBOOL ret = MTRUE;

	FUNCTION_LOG_END;
    return  ret;

}
#endif

//kidd check those
HdrState_e
HdrShot::
GetHdrState(void)
{
	return mHdrState;
}

void
HdrShot::
SetHdrState(HdrState_e eHdrState)
{
	mHdrState = eHdrState;
}

MINT32
HdrShot::
mHalCamHdrProc(HdrState_e eHdrState)
{
	return 0;
}

MBOOL
HdrShot::
init()
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

	MINT32  err = 0;  // Error code. 0: no error. others: error.
	if(mTestMode) {
		mu4OutputFrameNum = 3;
	}

	//1. init IMem
	mpIMemDrv = IMemDrv::createInstance();
	if(!mpIMemDrv) {
		MY_ERR("HdrShot::init can't alloc mpIMemDrv");
		goto lbExit;
	}
	mpIMemDrv->init();	//check this, see fd
#if HDR_SPEEDUP_MALLOC == 0
	ret = requestSourceImgBuf();
	if  ( ! ret )
	{
		goto lbExit;
	}
#endif

	//2. get information for HDR Hal
	MY_DBG("[init] - HDR Pipe Init");
	// Config HDR Pipe init info structure.
	HDR_PIPE_INIT_INFO rHdrPipeInitInfo;
	rHdrPipeInitInfo.u4ImgW       = mu4W_yuv;
	rHdrPipeInitInfo.u4ImgH       = mu4H_yuv;
	rHdrPipeInitInfo.u4OutputFrameNum = OutputFrameNumGet();
	rHdrPipeInitInfo.u4FinalGainDiff0 = mu4FinalGainDiff[0];
	rHdrPipeInitInfo.u4FinalGainDiff1 = mu4FinalGainDiff[1];
	rHdrPipeInitInfo.u4TargetTone   = mu4TargetTone;
	for (MUINT32 i = 0; i < OutputFrameNumGet(); i++)
	{
		rHdrPipeInitInfo.pSourceImgBufAddr[i] = (MUINT32)(mpSourceImgBuf[i].virtAddr);
	}
	// Create HDR hal object.
	MY_DBG("[init] - Create HDR hal object");
	mpHdrHal = HdrHalBase::createInstance();
	if  ( ! mpHdrHal )
	{
		MY_ERR("HdrHalBase::createInstance fail.");
		goto lbExit;
	}
	// Init HDR hal object.
	ret = mpHdrHal->init((void*)(&rHdrPipeInitInfo));
	if  ( ! ret )
	{
		MY_ERR("mpHdrHal->init fail.");
		goto lbExit;
	}
	ret = ret && mpHdrHal->ConfigMavParam();
	if  ( ! ret )
	{
		MY_ERR("mpHdrHal->ConfigMavParam fail.");
		goto lbExit;
	}


	// For SmallImg Buffer.
	MY_DBG("[init] - QuerySmallImgResolution");
	mpHdrHal->QuerySmallImgResolution(mu4W_small, mu4H_small);
	// For FirstRunSourceImg Buffer.
	mu4W_first = mu4W_small;
	mu4H_first = mu4H_small;
	// For SE Image Buffer.
	MY_DBG("[init] - QuerySEImgResolution");
	mpHdrHal->QuerySEImgResolution(mu4W_se, mu4H_se);


#if HDR_USE_THREAD
	// Create HDR thread.
	SetHdrState(HDR_STATE_INIT);
	::sem_init(&semHdrThread, 0, 0);
	::sem_init(&semHdrThreadBack, 0, 0);
	::sem_init(&semHdrThreadEnd, 0, 0);
	pthread_create(&threadHdrNormal, NULL, mHalCamHdrThread, NULL);
	MY_DBG("[init] threadHdrNormal: %d.", threadHdrNormal);
#endif  // HDR_USE_THREAD


lbExit:
	if  ( ! ret )
	{
		uninit();
	}

	FUNCTION_LOG_END;
	return  ret;
}


MBOOL
HdrShot::
uninit()
{
	FUNCTION_LOG_START;
	MBOOL ret = MTRUE;

	if (mpHdrHal)
	{
		mpHdrHal->uninit();
		mpHdrHal->destroyInstance();
		mpHdrHal = NULL;
	}

	if(mpIMemDrv) {
		mpIMemDrv->uninit();
		mpIMemDrv->destroyInstance();
		mpIMemDrv = NULL;
	}

	for (MUINT32 i = 0; i < OutputFrameNumGet(); i++) {
		if(mpCamExif[i]) {
			delete mpCamExif[i];
			mpCamExif[i] = 0;
		}
	}

	mu4W_yuv = mu4H_yuv = 0;

#if HDR_USE_THREAD
	SetHdrState(HDR_STATE_UNINIT);
	::sem_post(&semHdrThread);
	::sem_wait(&semHdrThreadEnd);
	MY_DBG("semHdrThreadEnd received.");
#endif  // HDR_USE_THREAD

#if 0	//kidd
	//ret = ShotBase::uninit();
	//ImpShot::onDestroy();
#endif


	FUNCTION_LOG_END;
	return  ret;
}


MUINT32
HdrShot::
allocMem_Blocking(IMEM_BUF_INFO *memBuf)
{
	FUNCTION_LOG_START;
	MBOOL ret = 0;

	mTotalBufferSize += memBuf->size;
	MY_DBG("allocMem size=%d\n", memBuf->size);
	MY_DBG("allocMem total=%d\n", mTotalBufferSize);

	ret = mpIMemDrv->allocVirtBuf(memBuf);
    if (ret) {
        MY_ERR("g_pIMemDrv->allocVirtBuf() error");
		goto lbExit;
    }
#if	HDR_SPEEDUP_MALLOC == 0
    memset((void*)memBuf->virtAddr, 0 , memBuf->size);
#endif
	ret = mpIMemDrv->mapPhyAddr(memBuf);
    if (ret) {
        MY_ERR("mpIMemDrv->mapPhyAddr() error");
		goto lbExit;
    }

lbExit:
	FUNCTION_LOG_END;
	return ret;
}


#if 0
void*
HdrShot::
allocMemTask(void *arg)
{
	FUNCTION_LOG_START;
	MBOOL 	ret = MTRUE;

	//non-real-time policy
	SetThreadProp(SCHED_OTHER, -20);

	HdrMemBufInfo *memBufInfo = (HdrMemBufInfo*)arg;
	HdrShot *self = (HdrShot*)memBufInfo->handler;
	ret = self->allocMem_Blocking(memBufInfo->imemBufInfo);
lbExit:
	FUNCTION_LOG_END;
    return (MVOID*)ret;
}
#endif


MUINT32
HdrShot::
allocMem(IMEM_BUF_INFO *memBuf)
{
#if HDR_DEBUG_SKIP_MODIFY_POLICY
	return allocMem_Blocking(memBuf);
#else
	FUNCTION_LOG_START;

	MBOOL 	ret = MTRUE;
	#if 0
	MUINT32	threadRet = 0;

	HdrMemBufInfo memBufInfo;
	memBufInfo.handler = this;
	memBufInfo.imemBufInfo = memBuf;

	pthread_t tempThread = NULL;
	pthread_create(&tempThread, NULL, HdrShot::allocMemTask, &memBufInfo);
	pthread_join(tempThread, (void**)&threadRet);
	ret = threadRet;
	#else
	SetThreadProp(SCHED_OTHER, -20);
	ret = allocMem_Blocking(memBuf);
	SetThreadProp(mCapturePolicy, mCapturePriority);
	#endif

lbExit:
	FUNCTION_LOG_END;
	return ret;
#endif
}

MBOOL
HdrShot::
deallocMem(IMEM_BUF_INFO *memBuf)
{
	if(memBuf->virtAddr == 0) {
		return MTRUE;
	}

	mTotalBufferSize -= memBuf->size;
	MY_DBG("deallocMem total=%d\n", mTotalBufferSize);

    if (mpIMemDrv->unmapPhyAddr(memBuf)) {
        MY_ERR("m_pIMemDrv->unmapPhyAddr() error");
    }

    if (mpIMemDrv->freeVirtBuf(memBuf)) {
        MY_ERR("m_pIMemDrv->freeVirtBuf() error");
    }
	memBuf->virtAddr = 0;

	return MTRUE;
}


MBOOL
HdrShot::

update3AExif(CamExif *pCamExif)
{
    //
	FUNCTION_LOG_START;
    MBOOL ret = MTRUE;

    //MY_DBG("+ (u4CamMode) = (%d)", u4CamMode);

    CamExifParam rExifParam;
    CamDbgParam rDbgParam;

    // ExifParam (for Gps)
    if (! mJpegParam.ms8GpsLatitude.isEmpty() && !mJpegParam.ms8GpsLongitude.isEmpty())
    {
        rExifParam.u4GpsIsOn = 1;
        ::strncpy(reinterpret_cast<char*>(rExifParam.uGPSLatitude), mJpegParam.ms8GpsLatitude.string(), mJpegParam.ms8GpsLatitude.length());
        ::strncpy(reinterpret_cast<char*>(rExifParam.uGPSLongitude), mJpegParam.ms8GpsLongitude.string(), mJpegParam.ms8GpsLongitude.length());
        ::strncpy(reinterpret_cast<char*>(rExifParam.uGPSTimeStamp), mJpegParam.ms8GpsTimestamp.string(), mJpegParam.ms8GpsTimestamp.length());
        ::strncpy(reinterpret_cast<char*>(rExifParam.uGPSProcessingMethod), mJpegParam.ms8GpsMethod.string(), mJpegParam.ms8GpsMethod.length());
        rExifParam.u4GPSAltitude = ::atoi(mJpegParam.ms8GpsAltitude.string());
    }
    // the bitstream already rotated. rotation should be 0
    rExifParam.u4Orientation = 0;
    rExifParam.u4ZoomRatio = mShotParam.mu4ZoomRatio;
    //
    camera_info rCameraInfo = MtkCamUtils::DevMetaInfo::queryCameraInfo(getOpenId());
    rExifParam.u4Facing = rCameraInfo.facing;

	#if 0
    //
    //! CamDbgParam (for camMode, shotMode)
    rDbgParam.u4CamMode = u4CamMode;
    rDbgParam.u4ShotMode = getShotMode();
	#endif

    //
    pCamExif->init(rExifParam,  rDbgParam);
    //
    NS3A::Hal3ABase* p3AHal = NS3A::Hal3ABase::createInstance(MtkCamUtils::DevMetaInfo::queryHalSensorDev(getOpenId()));

    p3AHal->set3AEXIFInfo(pCamExif);
    p3AHal->setDebugInfo(pCamExif);
	p3AHal->destroyInstance();

	FUNCTION_LOG_END;
    return ret;
}


MBOOL
HdrShot::
updateThumbnailExif(CamExif *pCamExif, MUINT8* const puThumbBuf, MUINT32 const u4ThumbSize, MUINT8* puExifBuf, MUINT32 &u4FinalExifSize) {
	FUNCTION_LOG_START;
	MBOOL	ret = MTRUE;

    uint32_t u4App1HeaderSize = 0;
    uint32_t u4AppnHeaderSize = 0;
    uint32_t exifHeaderSize = 0;

    MY_DBG("pCamExif=0x%x, puThumbBuf=0x%x, u4ThumbSize=%d, puExifBuf=0x%x, u4FinalExifSize=%d",
		pCamExif, puThumbBuf, u4ThumbSize, puExifBuf, u4FinalExifSize);

	CHECK_OBJECT(pCamExif);
	CHECK_OBJECT(puThumbBuf);
	CHECK_OBJECT(puExifBuf);

	//	`APP1` | THUMBNAIL
    // the bitstream already rotated. it need to swap the width/height
    if (90 == mShotParam.mi4Rotation || 270 == mShotParam.mi4Rotation)
    {
        pCamExif->makeExifApp1(mShotParam.mi4PictureHeight,  mShotParam.mi4PictureWidth
								, u4ThumbSize, puExifBuf,  &u4App1HeaderSize);
    }
    else
    {
        pCamExif->makeExifApp1(mShotParam.mi4PictureWidth, mShotParam.mi4PictureHeight
								, u4ThumbSize, puExifBuf,  &u4App1HeaderSize);
    }

	//	APP1 | `THUMBNAIL`
    // copy thumbnail image after APP1
    MUINT8 *pdest = puExifBuf + u4App1HeaderSize;
    ::memcpy(pdest, puThumbBuf, u4ThumbSize) ;
    //
    // Sensor Debug Info
    SensorHal* pSensorHal = SensorHal::createInstance();
	CHECK_OBJECT(pSensorHal);
    pSensorHal->setDebugInfo(pCamExif);
    pSensorHal->destroyInstance();
    pdest = puExifBuf + u4App1HeaderSize + u4ThumbSize;
    pCamExif->appendDebugExif(pdest, &u4AppnHeaderSize);
    u4FinalExifSize = u4App1HeaderSize + u4ThumbSize + u4AppnHeaderSize;

    MY_DBG("- (app1Size, appnSize, exifSize) = (%d, %d, %d)",
                          u4App1HeaderSize, u4AppnHeaderSize, u4FinalExifSize);

	FUNCTION_LOG_END;
    return ret;
}


MBOOL
HdrShot::
touchVirtualMemory(MUINT8* vm, MUINT32 size)
{
	MBOOL	ret = MTRUE;
	for(MUINT32 i=0; i<size; i+=4096) {
		vm[i] = 0;
	}
	return MTRUE;
}


MBOOL
HdrShot::
SetThreadProp(int policy, int priority)
{
#if !HDR_DEBUG_SKIP_MODIFY_POLICY
    //@see http://www.kernel.org/doc/man-pages/online/pages/man2/sched_setscheduler.2.html
    //int const policy    = pthreadAttr_ptr->sched_policy;
    //int const priority  = pthreadAttr_ptr->sched_priority;
	//MY_DBG("policy=%d, priority=%d", policy, priority);

    struct sched_param sched_p;
    ::sched_getparam(0, &sched_p);

	switch(policy)
	{
		//non-real-time
		case SCHED_OTHER:
		    sched_p.sched_priority = 0;
			sched_setscheduler(0, policy, &sched_p);
		    setpriority(PRIO_PROCESS, 0, priority);	//-20(high)~19(low)
			break;

		//real-time
		case SCHED_FIFO:
		case SCHED_RR:
		default:
		  	sched_p.sched_priority = priority;	//1(low)~99(high)
			sched_setscheduler(0, policy, &sched_p);
	}
    //
    #if 0
    ::sched_getparam(0, &sched_p);
    XLOGD(
        "policy:(expect, result)=(%d, %d), priority:(expect, result)=(%d, %d-%d)"
        , policy, ::sched_getscheduler(0)
        , priority, getpriority(PRIO_PROCESS, 0), sched_p.sched_priority
    );
	#endif
#endif

    return MTRUE;
}

MBOOL
HdrShot::
GetThreadProp(int *policy, int *priority)
{
#if !HDR_DEBUG_SKIP_MODIFY_POLICY
    //@see http://www.kernel.org/doc/man-pages/online/pages/man2/sched_setscheduler.2.html
    struct sched_param sched_p;
    *policy = ::sched_getscheduler(0);

	switch(*policy)
	{
		//non-real-time
		case SCHED_OTHER:
		    *priority = getpriority(PRIO_PROCESS, 0);	//-20(high)~19(low)
			break;

		//real-time
		case SCHED_FIFO:
		case SCHED_RR:
		default:
		    struct sched_param sched_p;
		    ::sched_getparam(0, &sched_p);
			*priority = sched_p.sched_priority;
	}
#endif

    return MTRUE;
}



