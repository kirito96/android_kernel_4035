
///////////////////////////////////////////////////////////////////////////////
// No Warranty
// Except as may be otherwise agreed to in writing, no warranties of any
// kind, whether express or implied, are given by MTK with respect to any MTK
// Deliverables or any use thereof, and MTK Deliverables are provided on an
// "AS IS" basis.  MTK hereby expressly disclaims all such warranties,
// including any implied warranties of merchantability, non-infringement and
// fitness for a particular purpose and any warranties arising out of course
// of performance, course of dealing or usage of trade.  Parties further
// acknowledge that Company may, either presently and/or in the future,
// instruct MTK to assist it in the development and the implementation, in
// accordance with Company's designs, of certain softwares relating to
// Company's product(s) (the "Services").  Except as may be otherwise agreed
// to in writing, no warranties of any kind, whether express or implied, are
// given by MTK with respect to the Services provided, and the Services are
// provided on an "AS IS" basis.  Company further acknowledges that the
// Services may contain errors, that testing is important and Company is
// solely responsible for fully testing the Services and/or derivatives
// thereof before they are used, sublicensed or distributed.  Should there be
// any third party action brought against MTK, arising out of or relating to
// the Services, Company agree to fully indemnify and hold MTK harmless.
// If the parties mutually agree to enter into or continue a business
// relationship or other arrangement, the terms and conditions set forth
// hereunder shall remain effective and, unless explicitly stated otherwise,
// shall prevail in the event of a conflict in the terms in any agreements
// entered into between the parties.
////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, MediaTek Inc.
// All rights reserved.
//
// Unauthorized use, practice, perform, copy, distribution, reproduction,
// or disclosure of this information in whole or in part is prohibited.
////////////////////////////////////////////////////////////////////////////////

//! \file  AcdkIF.cpp

#define LOG_TAG "ACDK_IF"

#define CCT_SUPPORT 0

extern "C" {
#include <linux/fb.h>
#include <linux/kd.h>
#include "mtkfb.h"
}

#include "AcdkLog.h"
#include "mtkcam/acdk/AcdkTypes.h"
#include "AcdkErrCode.h"
#include "mtkcam/acdk/AcdkCommon.h"
#include "AcdkBase.h"
#include "mtkcam/acdk/AcdkIF.h"

#if (CCT_SUPPORT)
#include "cct_feature.h"
#include "cct_calibration.h"
#endif

static MBOOL g_bAcdkOpend = MFALSE;

using namespace NSACDK; 

static AcdkBase *g_pAcdkBaseObj = NULL; 

#if (CCT_SUPPORT)
static AcdkCalibration *g_pAcdkCalibrationObj = NULL;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/////////////////////////////////////////////////////////////////////////
//
//   MDK_Open () -
//!  brief ACDK I/F MDK_Open()
//!
/////////////////////////////////////////////////////////////////////////
MBOOL MDK_Open()
{
    ACDK_LOGD("+");
    
    if (g_bAcdkOpend == MTRUE)
    {
        ACDK_LOGE("ACDK device already opened"); 
        return MFALSE; 
    }    

    g_pAcdkBaseObj = AcdkBase::createInstance();    

#if (CCT_SUPPORT)

    g_pAcdkCalibrationObj = new AcdkCalibration();

#endif

    g_bAcdkOpend = MTRUE; 

    ACDK_LOGD("-");
    return MTRUE;
}

/////////////////////////////////////////////////////////////////////////
//
//   MDK_Close () -
//!  brief ACDK I/F MDK_Close()
//!
/////////////////////////////////////////////////////////////////////////
MBOOL MDK_Close()
{
    ACDK_LOGD("+");
    
    if (g_bAcdkOpend == MFALSE)
    {
        ACDK_LOGE("Acdk device is not opened"); 
        return MFALSE; 
    }

    g_pAcdkBaseObj->destroyInstance(); 

#if (CCT_SUPPORT)
    
    delete g_pAcdkCalibrationObj;
	g_pAcdkCalibrationObj = NULL;

#endif

    g_bAcdkOpend = MFALSE; 

    ACDK_LOGD("-");
    return MTRUE; 
}

/////////////////////////////////////////////////////////////////////////
//
//   MDK_Init () -
//!  brief ACDK I/F MDK_Init()
//!
/////////////////////////////////////////////////////////////////////////
MBOOL MDK_Init()
{
    ACDK_LOGD("+");
    
    if (g_bAcdkOpend == MFALSE)
    {
        ACDK_LOGE("ACDK device is not opened"); 
        return MFALSE; 
    }

    MINT32 mrRet;   

    mrRet = g_pAcdkBaseObj->init(); 
    if(mrRet != 0)
    {
        ACDK_LOGE("Fail to init AcdkCCTCtrl"); 
        return MFALSE; 
    }

#if (CCT_SUPPORT)
    
    if(!g_pAcdkCalibrationObj)
    {
        ACDK_LOGE("g_pAcdkCalibrationObj is NULL"); 
        return MFALSE; 
    }
    
    mrRet = g_pAcdkCalibrationObj->init(g_pAcdkBaseObj);

    if (FAILED(mrRet))
    {
        ACDK_LOGE("Fail to enable AcdkCalibration");
        return MFALSE;
    }

#endif
    
    ACDK_LOGD("-");
    return MTRUE; 
}

/////////////////////////////////////////////////////////////////////////
//
//   MDK_DeInit () -
//!  brief ACDK I/F MDK_DeInit()
//!
/////////////////////////////////////////////////////////////////////////
MBOOL MDK_DeInit()
{
    ACDK_LOGD("+");
    
    MINT32 err; 
    
    if (g_bAcdkOpend == MFALSE) 
    {
        ACDK_LOGE("ACDK device is not opened"); 
        return MFALSE; 
    }
      
    err = g_pAcdkBaseObj->uninit(); 
    if (err != 0)
    {
        ACDK_LOGE("Fail to disable ACDK CamCtrl  err(0x%x)", err); 
        return MFALSE; 
    }

#if (CCT_SUPPORT)

    err = g_pAcdkCalibrationObj->uninit(); 
    if (FAILED(err))
    {
        ACDK_LOGE("Fail to disable AcdkCalibration");
        return MFALSE; 
    }

#endif

    ACDK_LOGD("-");
    return MTRUE; 
}

/////////////////////////////////////////////////////////////////////////
//
//   MDK_IOControl () -
//!  brief ACDK I/F MDK_IOControl()
//!
/////////////////////////////////////////////////////////////////////////
MBOOL MDK_IOControl(MUINT32 a_u4Ioctl, ACDK_FEATURE_INFO_STRUCT *a_prAcdkFeatureInfo)
{
    ACDK_LOGD("+");
    
    if (g_bAcdkOpend == MFALSE)
    {
        ACDK_LOGE("ACDK device is not opened"); 
        return MFALSE; 
    }

    MBOOL bRet = MTRUE;

    ACDK_LOGD("Reveive IOControl Code:0x%x", a_u4Ioctl); 
    
#if (CCT_SUPPORT)

    if(a_u4Ioctl >= ACDK_CCT_CDVT_START && a_u4Ioctl < ACDK_CCT_CDVT_END)
    {
        ACDK_LOGD("ACDK_CCT_CDVT");
        bRet = g_pAcdkCalibrationObj->sendcommand(a_u4Ioctl, 
                                                  a_prAcdkFeatureInfo->puParaIn, 
                                                  a_prAcdkFeatureInfo->u4ParaInLen, 
                                                  a_prAcdkFeatureInfo->puParaOut, 
                                                  a_prAcdkFeatureInfo->u4ParaOutLen, 
                                                  a_prAcdkFeatureInfo->pu4RealParaOutLen); 

    }
    else if(a_u4Ioctl == ACDK_CCT_V2_OP_SHADING_CAL)
    {
        ACDK_LOGD("ACDK_CCT_V2_OP_SHADING_CAL");
        bRet = g_pAcdkCalibrationObj->sendcommand(a_u4Ioctl, 
                                                  a_prAcdkFeatureInfo->puParaIn, 
                                                  a_prAcdkFeatureInfo->u4ParaInLen, 
                                                  a_prAcdkFeatureInfo->puParaOut, 
                                                  a_prAcdkFeatureInfo->u4ParaOutLen, 
                                                  a_prAcdkFeatureInfo->pu4RealParaOutLen); 
    }

#endif

    if(a_u4Ioctl > ACDK_COMMAND_START && a_u4Ioctl < ACDK_COMMAND_END)
    {
        ACDK_LOGD("ACDK_COMMAND");
        bRet = g_pAcdkBaseObj->sendcommand (a_u4Ioctl, 
                                            a_prAcdkFeatureInfo->puParaIn, 
                                            a_prAcdkFeatureInfo->u4ParaInLen, 
                                            a_prAcdkFeatureInfo->puParaOut, 
                                            a_prAcdkFeatureInfo->u4ParaOutLen, 
                                            a_prAcdkFeatureInfo->pu4RealParaOutLen); 
    }    

    ACDK_LOGD("-");
    return MTRUE; 
}

#ifdef __cplusplus
} // extern "C"
#endif
