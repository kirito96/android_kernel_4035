
#define LOG_TAG "CamShot/ImageTranform"
//
#include <mtkcam/Log.h>
#define MY_LOGV(fmt, arg...)    CAM_LOGV("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGD(fmt, arg...)    CAM_LOGD("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGI(fmt, arg...)    CAM_LOGI("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGW(fmt, arg...)    CAM_LOGW("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGE(fmt, arg...)    CAM_LOGE("[%s] "fmt, __FUNCTION__, ##arg)
#define FUNCTION_LOG_START      MY_LOGD("+");
#define FUNCTION_LOG_END        MY_LOGD("-");

//
#include <mtkcam/common.h>
#include <common/hw/hwstddef.h>
//
#include <mtkcam/v1/camutils/CamProfile.h>

// cdp 
#include <campipe/IPipe.h>
#include <campipe/ICdpPipe.h>
//
#include <drv/res_mgr_drv.h>
#include <campipe/pipe_mgr_drv.h>
//
#include "./inc/ImageTransform.h"

#define CHECK_OBJECT(x)  { if (x == NULL) { MY_LOGE("Null %s Object", #x); return MFALSE;}}

using namespace android; 
using namespace NSCamPipe; 

namespace NSCamShot {
////////////////////////////////////////////////////////////////////////////////


ImageTransform::
ImageTransform(   
)
    : mi4ErrorCode(0)
    , mpPipeMgrDrv(NULL)
    , mpResMgrDrv(NULL)
{
}


MBOOL
ImageTransform::
execute(
    ImgBufInfo const rSrcBufInfo, 
    ImgBufInfo const rDstBufInfo, 
    Rect const rROI, 
    MUINT32 const u4Rotation, 
    MUINT32 const u4Flip, 
    MUINT32 const u4TimeOutInMs
)
{
    FUNCTION_LOG_START;
    MtkCamUtils::CamProfile profile("execute", "ImageTransform");
    if (!lock(u4TimeOutInMs)) 
    {
        MY_LOGE("[execute] lock fail "); 
        return MFALSE; 
    }
    // (1). Create Instance 
#warning [TODO] sensor type ??? 
    ICdpPipe    *pCdpPipe = ICdpPipe::createInstance(eSWScenarioID_CAPTURE_NORMAL, eScenarioFmt_RAW); 
    CHECK_OBJECT(pCdpPipe); 
    //
    if (pCdpPipe != NULL) 
    {
        MY_LOGD("Pipe (Name, ID) = (%s, %d)", pCdpPipe->getPipeName(), pCdpPipe->getPipeId()); 
    }
    
    // (2). Query port property
    vector<PortProperty> rInPortProperty; 
    vector<PortProperty> rOutPortProperty;     
    if (pCdpPipe->queryPipeProperty(rInPortProperty,rOutPortProperty))
    {
        MY_LOGD("Port Property (IN, OUT): (%d, %d)", rInPortProperty.size(), rOutPortProperty.size()); 
        for (MUINT32 i = 0; i < rInPortProperty.size(); i++)
        {
            MY_LOGD("IN: (type, index, inout, fmt, rot, flip) = (%d, %d, %d, %d, %d, %d)", 
                         rInPortProperty.at(i).type, rInPortProperty.at(i).index, rInPortProperty.at(i).inout,  
                         rInPortProperty.at(i).u4SupportFmt, rInPortProperty.at(i).fgIsSupportRotate, rInPortProperty.at(i).fgIsSupportFlip); 
        }       
        for (MUINT32 i = 0; i < rOutPortProperty.size(); i++)
        {
            MY_LOGD("IN: (type, index, inout, fmt, rot, flip) = (%d, %d, %d, %d, %d, %d)", 
                         rOutPortProperty.at(i).type, rOutPortProperty.at(i).index, rOutPortProperty.at(i).inout,  
                         rOutPortProperty.at(i).u4SupportFmt, rOutPortProperty.at(i).fgIsSupportRotate, rOutPortProperty.at(i).fgIsSupportFlip); 
        } 
    }    

    // (3). init 
    pCdpPipe->init(); 

    // (4). setCallback
    pCdpPipe->setCallbacks(NULL, NULL, NULL); 

    // (5). Config pipe 
    // 
    MemoryInPortInfo rMemInPort(ImgInfo(rSrcBufInfo.eImgFmt, rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight), 
                                0, rSrcBufInfo.u4Stride, Rect(rROI.x, rROI.y, rROI.w, rROI.h)); 
    //   
    MemoryOutPortInfo rVdoPort(ImgInfo(rDstBufInfo.eImgFmt, rDstBufInfo.u4ImgWidth, rDstBufInfo.u4ImgHeight), 
                               rDstBufInfo.u4Stride, u4Rotation, u4Flip);   
    rVdoPort.index = 1;   
    //
    vector<PortInfo const*> vCdpInPorts;  
    vector<PortInfo const*> vCdpOutPorts; 
    //
    vCdpInPorts.push_back(&rMemInPort); 
    vCdpOutPorts.push_back(&rVdoPort); 
    //
    pCdpPipe->configPipe(vCdpInPorts, vCdpOutPorts); 

    // (6). Enqueue, In buf
    // 
    QBufInfo rInBuf; 
    rInBuf.vBufInfo.clear(); 
    BufInfo rBufInfo(rSrcBufInfo.u4BufSize, rSrcBufInfo.u4BufVA, rSrcBufInfo.u4BufPA, rSrcBufInfo.i4MemID);  
    rInBuf.vBufInfo.push_back(rBufInfo); 
    pCdpPipe->enqueBuf(PortID(EPortType_MemoryIn, 0, 0), rInBuf); 
    // 
    QBufInfo rOutBuf; 
    rOutBuf.vBufInfo.clear(); 
    rBufInfo.u4BufSize = rDstBufInfo.u4BufSize; 
    rBufInfo.u4BufVA = rDstBufInfo.u4BufVA; 
    rBufInfo.u4BufPA = rDstBufInfo.u4BufPA; 
    rBufInfo.i4MemID = rDstBufInfo.i4MemID; 
    //rBufInfo.eMemType = rDstBufInfo.eMemType;
    rOutBuf.vBufInfo.push_back(rBufInfo); 
    pCdpPipe->enqueBuf(PortID(EPortType_MemoryOut, 1, 1), rOutBuf); 
    // 
    profile.print(); 
    // (7). start
    pCdpPipe->start(); 

    // (8). Dequeue Vdo Out Buf 
    QTimeStampBufInfo rQVdoOutBuf; 
    pCdpPipe->dequeBuf(PortID(EPortType_MemoryOut, 1, 1), rQVdoOutBuf); 
    // (8.1) Dequeue In Buf 
    QTimeStampBufInfo rQInBUf; 
    pCdpPipe->dequeBuf(PortID(EPortType_MemoryIn, 0, 0), rQInBUf); 
 
    // (9). Stop 
    pCdpPipe->stop();
    // (10). uninit 
    pCdpPipe->uninit(); 
    // (11). destory instance 
    pCdpPipe->destroyInstance(); 
    unlock(); 
    profile.print(); 
    //
    return 0; 
}


MBOOL    
ImageTransform::
lock(MUINT32 const u4TimeOutInMs)
{
    //
    mpPipeMgrDrv = PipeMgrDrv::CreateInstance();
    CHECK_OBJECT(mpPipeMgrDrv); 
    mpPipeMgrDrv->Init();    
    // 
    mpResMgrDrv = ResMgrDrv::CreateInstance();
    CHECK_OBJECT(mpResMgrDrv); 
    mpResMgrDrv->Init();
    //
    RES_MGR_DRV_MODE_STRUCT rResMgrMode; 
    rResMgrMode.Dev = RES_MGR_DRV_DEV_CAM; 
    rResMgrMode.ScenSw = RES_MGR_DRV_SCEN_SW_CAM_CAP; 
    rResMgrMode.ScenHw = RES_MGR_DRV_SCEN_HW_ZSD; 
    if (!mpResMgrDrv->SetMode(&rResMgrMode))
    {
        MY_LOGE("fail to set resource mode"); 
        return MFALSE; 
    }
    //
    PIPE_MGR_DRV_LOCK_STRUCT rPipeMgrMode; 
    rPipeMgrMode.Timeout = u4TimeOutInMs; 
    rPipeMgrMode.PipeMask = PIPE_MGR_DRV_PIPE_MASK_CDP_CAM; 
    if (!mpPipeMgrDrv->Lock(&rPipeMgrMode))
    {
        MY_LOGE("fail to lock pipe"); 
        return MFALSE; 
    }

    return MTRUE; 
}

MBOOL
ImageTransform::
unlock()
{
    CHECK_OBJECT(mpPipeMgrDrv); 
    CHECK_OBJECT(mpResMgrDrv); 
    //
    PIPE_MGR_DRV_UNLOCK_STRUCT rPipeMgrMode; 
    rPipeMgrMode.PipeMask = PIPE_MGR_DRV_PIPE_MASK_CDP_CAM; 
    //    
    if (!mpPipeMgrDrv->Unlock(&rPipeMgrMode))
    {
        MY_LOGE("fail to unlock pipe"); 
        return MFALSE;      
    }
    //
    mpPipeMgrDrv->Uninit(); 
    mpPipeMgrDrv->DestroyInstance(); 
    mpPipeMgrDrv = NULL; 
    //
    mpResMgrDrv->Uninit(); 
    mpResMgrDrv->DestroyInstance(); 
    mpResMgrDrv = NULL; 
    return MTRUE; 
}




////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamShot

