
#define LOG_TAG "CamShot/JpegCodec"
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
#include <cutils/properties.h>
#include <stdlib.h>

//
#include <mtkcam/common.h>
#include <common/hw/hwstddef.h>
// 
#include <mtkcam/v1/camutils/CamMisc.h>
#include <mtkcam/v1/camutils/CamProfile.h>
//
using namespace NSCamHW; 
// jpeg encoder 
#include <jpeg_hal.h>
//
// image transform 
#include <IImageTransform.h>
//
#include "../inc/ImageUtils.h"
#include "./inc/JpegCodec.h"

#define MEDIA_PATH "/sdcard/"


#define CHECK_OBJECT(x)  { if (x == NULL) { MY_LOGE("Null %s Object", #x); return MFALSE;}}
using namespace android; 

namespace NSCamShot {
////////////////////////////////////////////////////////////////////////////////


JpegCodec::
JpegCodec(   
)
    : mi4ErrorCode(0)
{
    char value[PROPERTY_VALUE_MAX] = {'\0'}; 
    property_get("debug.camera.dump", value, "0"); 
    mu4DumpFlag = ::atoi(value); 
}



MBOOL
JpegCodec::
encode(
    ImgBufInfo const rSrcBufInfo, 
    ImgBufInfo const rDstBufInfo, 
    Rect const rROI, 
    MUINT32 const u4Rotation, 
    MUINT32 const u4Flip, 
    MUINT32 const u4Quality, 
    MUINT32 const u4IsSOI, 
    MUINT32 &u4EncSize
)
{
    MBOOL ret = MTRUE; 
    if (checkIfNeedImgTransform(rSrcBufInfo, rDstBufInfo, rROI, u4Rotation, u4Flip))
    {
        // 
        IMEM_BUF_INFO rTempMemBuf; 
        // Jpeg code width/height should align to 16x 
        MUINT32 u4AlignedWidth = (~0xf) & (0xf + rDstBufInfo.u4ImgWidth);
        MUINT32 u4AlignedHeight = (~0xf) & (0xf + rDstBufInfo.u4ImgHeight); 
        MY_LOGD("[encode] Ori (width, height) = (%d, %d), Aligned (width, height) = (%d, %d)", 
                          rDstBufInfo.u4ImgWidth, rDstBufInfo.u4ImgHeight, u4AlignedWidth, u4AlignedHeight); 
        rTempMemBuf.size = u4AlignedWidth *u4AlignedHeight* 2; 
        allocMem(rTempMemBuf); 
        // (1). Image transform 
        MUINT32 u4Stride[3] = {u4AlignedWidth, 0, 0}; 
        ImgBufInfo rTempImgBuf(ImgInfo(eImgFmt_YUY2, u4AlignedWidth, u4AlignedHeight), 
                          BufInfo(rTempMemBuf.size, rTempMemBuf.virtAddr, rTempMemBuf.phyAddr, rTempMemBuf.memID), u4Stride); 
        // 
        IImageTransform *pImgTransform = IImageTransform::createInstance(); 
        
        ret =  pImgTransform->execute(rSrcBufInfo, rTempImgBuf, rROI, u4Rotation, u4Flip, 10 * 1000);  //10s timeout
        if (mu4DumpFlag)
        {
            char fileName[256]; 
            sprintf(fileName, "/%s/trans_%dx%d.yuv", MEDIA_PATH, u4AlignedWidth, u4AlignedHeight); 
            MtkCamUtils::saveBufToFile(fileName, reinterpret_cast<MUINT8*>(rTempMemBuf.virtAddr), rTempMemBuf.size); 
        }

        pImgTransform->destroyInstance();
        // (2). Encode
        encode(rTempImgBuf, rDstBufInfo, u4Quality, u4IsSOI, u4EncSize); 
        // 
        deallocMem(rTempMemBuf); 
    }    
    else 
    {
        ret = encode(rSrcBufInfo, rDstBufInfo, u4Quality, u4IsSOI, u4EncSize); 
    }
    return ret;  
} 


MBOOL
JpegCodec::
encode(ImgBufInfo const rSrcBufInfo, ImgBufInfo const rDstBufInfo, MUINT32 const u4Quality, MUINT32 const u4IsSOI, MUINT32 &u4EncSize)
{
    FUNCTION_LOG_START;
    MtkCamUtils::CamProfile profile("encode", "JpegCodec");
    MBOOL ret = MFALSE; 
    JpgEncHal* pJpgEncoder = new JpgEncHal();
    // (1). Lock 
    if(!pJpgEncoder->lock())
    {
        MY_LOGE("can't lock jpeg resource");        
        goto EXIT; 
    }
    // (2). size, format, addr 
    if (eImgFmt_YUY2 == rSrcBufInfo.eImgFmt)
    {
        pJpgEncoder->setEncSize(rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight, 
                                JpgEncHal:: kENC_YUY2_Format); 
        pJpgEncoder->setSrcAddr((void *)rSrcBufInfo.u4BufVA, (void *)NULL);
        pJpgEncoder->setSrcBufSize(pJpgEncoder->getSrcBufMinStride() ,rSrcBufInfo.u4BufSize, 0);
    }
    else if (eImgFmt_NV21 == rSrcBufInfo.eImgFmt) 
    {
        pJpgEncoder->setEncSize(rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight, 
                                JpgEncHal:: kENC_NV21_Format);   
        pJpgEncoder->setSrcAddr((void *)rSrcBufInfo.u4BufVA, (void *)(rSrcBufInfo.u4BufVA + rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight));
        pJpgEncoder->setSrcBufSize(pJpgEncoder->getSrcBufMinStride(), rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight, 
                                                 rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight / 2);
    }
    else if (eImgFmt_NV12 == rSrcBufInfo.eImgFmt) 
    {
        pJpgEncoder->setEncSize(rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight, 
                                JpgEncHal:: kENC_NV12_Format);   
        pJpgEncoder->setSrcAddr((void *)rSrcBufInfo.u4BufVA, (void *)(rSrcBufInfo.u4BufVA + rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight));
        pJpgEncoder->setSrcBufSize(pJpgEncoder->getSrcBufMinStride(), rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight, 
                                                 rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight / 2);
    } 
    else if (eImgFmt_UYVY == rSrcBufInfo.eImgFmt) 
    {
        pJpgEncoder->setEncSize(rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight, 
                                JpgEncHal:: kENC_UYVY_Format);   
        pJpgEncoder->setSrcAddr((void *)rSrcBufInfo.u4BufVA, (void *)NULL);
        pJpgEncoder->setSrcBufSize(pJpgEncoder->getSrcBufMinStride(), rSrcBufInfo.u4BufSize, 0);
    } 
    else if (eImgFmt_YV12 == rSrcBufInfo.eImgFmt) 
    {
        pJpgEncoder->setEncSize(rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight, 
                                JpgEncHal:: kENC_YV12_Format);   
        pJpgEncoder->setSrcAddr((void *)rSrcBufInfo.u4BufVA, (void *)(rSrcBufInfo.u4BufVA + rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight),
        	                                                                               (void *)(rSrcBufInfo.u4BufVA + rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight * 5/4));
        pJpgEncoder->setSrcBufSize(pJpgEncoder->getSrcBufMinStride(), rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight, 
                                                 rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight / 4, rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight / 4);
    } 
    else if (eImgFmt_I420 == rSrcBufInfo.eImgFmt) 
    {
        pJpgEncoder->setEncSize(rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight, 
                                JpgEncHal:: kENC_YV12_Format);   
        pJpgEncoder->setSrcAddr((void *)rSrcBufInfo.u4BufVA, (void *)(rSrcBufInfo.u4BufVA + rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight * 5/4),
        	                                                                               (void *)(rSrcBufInfo.u4BufVA + rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight));
        pJpgEncoder->setSrcBufSize(pJpgEncoder->getSrcBufMinStride(), rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight, 
                                                 rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight / 4, rSrcBufInfo.u4ImgWidth * rSrcBufInfo.u4ImgHeight / 4);
    }  
    else 
    {
        MY_LOGE("Not support image format:0x%x", rSrcBufInfo.eImgFmt); 
        goto EXIT; 
    }
    // (3). set quality
    pJpgEncoder->setQuality(u4Quality);     
    // (4). dst addr, size 
    pJpgEncoder->setDstAddr((void *)rDstBufInfo.u4BufVA);
    pJpgEncoder->setDstSize(rDstBufInfo.u4BufSize);
    // (6). set SOI 
    pJpgEncoder->enableSOI((u4IsSOI > 0) ? 1 : 0);     
    // (7). ION mode 
    //if (eMemoryType_ION == rSrcBufInfo.eMemType)
    if ( rSrcBufInfo.i4MemID > 0 )
    {
        pJpgEncoder->setIonMode(1); 
        pJpgEncoder->setSrcFD(rSrcBufInfo.i4MemID, rSrcBufInfo.i4MemID); 
        pJpgEncoder->setDstFD(rDstBufInfo.i4MemID); 
    }

    // (8).  Start 
    if (pJpgEncoder->start(&u4EncSize))
    {
        MY_LOGD("Jpeg encode done, size = %d", u4EncSize); 
        ret = MTRUE; 
    }
    else 
    {
        pJpgEncoder->unlock(); 
        goto EXIT; 
    }
  
    pJpgEncoder->unlock();
    profile.print();
    
EXIT:
    delete pJpgEncoder;
    FUNCTION_LOG_END;
    return ret;
}

MBOOL
JpegCodec::
isSupportedFormat(EImageFormat const eFmt)
{
    FUNCTION_LOG_START;
    MY_LOGD("Format:0x%x", eFmt); 
    if (eFmt ==  eImgFmt_YUY2) 
    {
        return MTRUE; 
    }
#if 0 
    else if (eFmt == eImgFmt_NV21)
    {
        return MTRUE; 
    }
#endif
    else 
    {
        return MFALSE; 
    }
    
    FUNCTION_LOG_END;
}

MBOOL
JpegCodec::
checkIfNeedImgTransform(
    ImgBufInfo const rSrcBufInfo, 
    ImgBufInfo const rDstBufInfo, 
    Rect const rROI, 
    MUINT32 const u4Rotation, 
    MUINT32 const u4Flip 
)
{
    FUNCTION_LOG_START;
    // format 
    // resize
    if (rDstBufInfo.u4ImgWidth != rSrcBufInfo.u4ImgWidth ||
         rDstBufInfo.u4ImgHeight != rSrcBufInfo.u4ImgHeight) 
    {
        MY_LOGD("Resize src =(%d,%d), dst=(%d,%d)", 
                 rSrcBufInfo.u4ImgWidth, rSrcBufInfo.u4ImgHeight, 
                 rDstBufInfo.u4ImgWidth, rDstBufInfo.u4ImgHeight); 
        return MTRUE; 
    }
    // roi 
    if (rROI.x != 0 || rROI.y != 0 
        || rROI.w != rSrcBufInfo.u4ImgWidth 
        || rROI.h != rSrcBufInfo.u4ImgHeight)
    {
        MY_LOGD("Crop , roi = (%d, %d, %d, %d)", rROI.x, rROI.y, 
                                                 rROI.w, rROI.h); 
        return MTRUE; 
    } 
    // rotation 
    if (0 != u4Rotation)
    {
        MY_LOGD("rotation: %d", u4Rotation); 
        return MTRUE; 
    }
    // flip 
    if (0 != u4Flip)
    {
        MY_LOGD("flip:%d", u4Flip); 
        return MTRUE; 
    }
    // JPEG format but source format not support 
    if (!isSupportedFormat(rSrcBufInfo.eImgFmt))
    {
        MY_LOGD("Not JPEG codec support fmt:0x%x", rSrcBufInfo.eImgFmt); 
        return MTRUE; 
    }
    //
    MY_LOGD("No need to do image transform"); 

    FUNCTION_LOG_END;
    return MFALSE; 
}

MBOOL
JpegCodec::
allocMem(IMEM_BUF_INFO & rMemBuf) 
{
    // 
    IMemDrv *pIMemDrv = IMemDrv::createInstance(); 
    CHECK_OBJECT(pIMemDrv); 
    //
    pIMemDrv->init(); 
    // 
    if (pIMemDrv->allocVirtBuf(&rMemBuf)) {
        MY_LOGE("g_pIMemDrv->allocVirtBuf() error");
        return MFALSE;              
    }
    ::memset((void*)rMemBuf.virtAddr, 0 , rMemBuf.size);
#if 1
    if (pIMemDrv->mapPhyAddr(&rMemBuf)) {
        MY_LOGE("mpIMemDrv->mapPhyAddr() error");
        return MFALSE;        
    }
#endif 
    //
    pIMemDrv->uninit(); 
    pIMemDrv->destroyInstance(); 
    return MTRUE; 
}

MBOOL
JpegCodec::
deallocMem(IMEM_BUF_INFO & rMemBuf)
{
    IMemDrv *pIMemDrv = IMemDrv::createInstance(); 
    CHECK_OBJECT(pIMemDrv); 
    //
    pIMemDrv->init(); 
    //
#if 1
    if (pIMemDrv->unmapPhyAddr(&rMemBuf)) 
    {
        MY_LOGE("m_pIMemDrv->unmapPhyAddr() error");
        return MFALSE;              
    }
#endif 
    //
    if (pIMemDrv->freeVirtBuf(&rMemBuf)) 
    {
        MY_LOGE("m_pIMemDrv->freeVirtBuf() error");
        return MFALSE;        
    }        
    rMemBuf.size = 0; 
    //
    pIMemDrv->uninit(); 
    pIMemDrv->destroyInstance(); 
    return MTRUE; 
}


////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamShot

