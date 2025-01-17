
#ifndef _MAHL_FB_H_
#define _MAHL_FB_H_

#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <linux/cache.h>
#include <CameraProfile.h>

#include <common/faces.h>
#include <mtkcam/Log.h>
#include <mtkcam/common.h>
#include <common/hw/hwstddef.h>
#include <common/camutils/CamFormat.h>
#include <mtkcam/v1/camutils/CamInfo.h>

#include <drv/imem_drv.h>
#include <mtkcam/hal/sensor_hal.h>

#include <Shot/IShot.h>

#include <camshot/ICamShot.h>
#include <camshot/ISingleShot.h>
#include <camshot/ISImager.h>
#include <camshot/_callbacks.h>
#include <camshot/_params.h>

#include "ImpShot.h"
#include "featureio/facebeautify_hal_base.h"

using namespace android;
using namespace NSShot;
using namespace NSCamShot; 
using namespace NSCamHW;

namespace android {
namespace NSShot {
    
class Mhal_facebeauty : public ImpShot
{
public:    
    MUINT32         mu4W_yuv;       //  YUV Width
    MUINT32         mu4H_yuv;       //  YUV Height
    MUINT32         mDSWidth;
    MUINT32         mDSHeight;
    
public:  ////    Buffers.
    //    
    IMEM_BUF_INFO mpWorkingBuferr;
    MUINT32 FBWorkingBufferSize;
    //  Source.
    IMEM_BUF_INFO mpSource;
    MUINT32       mu4SourceSize;
    //  alpha map.
    IMEM_BUF_INFO mpAmap;
    //  Blurred image
    IMEM_BUF_INFO mpBlurImg;
           
    //  Postview image
    IMEM_BUF_INFO mpPostviewImgBuf;
           
    MTKPipeFaceBeautyResultInfo msFaceBeautyResultInfo;
public:  
    halFACEBEAUTIFYBase*     mpFb;
    IMemDrv*        mpIMemDrv;
            
protected:  ////    Resolutions.    
    MUINT32         mPostviewWidth;
    MUINT32         mPostviewHeight;
    MUINT32         mStep1Width;
    MUINT32         mStep1Height;       

protected:  ////    Info.    
    halSensorType_e meSensorType;
    MtkCameraFace FBFaceInfo[15];
    MtkFaceInfo MTKPoseInfo[15];
    MtkCameraFaceMetadata  FBmetadata;
 
public:  ////    Tuning parameter.
    MINT32          mSmoothLevel;
    MINT32          mWarpLevel;
    MINT32          mBrightLevel;
    MINT32          mRuddyLevel;
    MINT32          mContrastLevel;   
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Attributes.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	  
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:     ////    Interfaces.

    Mhal_facebeauty(char const*const pszShotName, uint32_t const u4ShotMode, int32_t const i4OpenId);
    virtual ~Mhal_facebeauty()  {}
    virtual bool    onCreate(MtkCameraFaceMetadata* FaceInfo);    
    virtual void    onDestroy();    
    virtual bool    sendCommand(uint32_t const  cmd, uint32_t const  arg1, uint32_t const  arg2);    
    //virtual bool    setCallback(sp<IShotCallback>& rpShotCallback);
    virtual MBOOL   doCapture(); 
    virtual MBOOL   SaveJpg();
    virtual MBOOL   WaitSaveDone();
protected:  ////    Capture command.
    virtual MBOOL   onCmd_capture();
    virtual MBOOL   onCmd_reset();
    virtual MBOOL   onCmd_cancel();

protected:  ////                    callbacks 
    static MBOOL fgCamShotNotifyCb(MVOID* user, NSCamShot::CamShotNotifyInfo const msg);
    static MBOOL fgCamShotDataCb(MVOID* user, NSCamShot::CamShotDataInfo const msg); 

protected:
    MBOOL handlePostViewData(MUINT8* const puBuf, MUINT32 const u4Size);
    MBOOL handleJpegData(MUINT8* const puJpegBuf, MUINT32 const u4JpegSize, MUINT8* const puThumbBuf, MUINT32 const u4ThumbSize, MUINT32 const Mode);    
    MBOOL handleYuvDataCallback(MUINT8* const puBuf, MUINT32 const u4Size);
            
public:  ////    Invoked by capture().    
    virtual MBOOL   createJpegImg(NSCamHW::ImgBufInfo const & rSrcImgBufInfo, NSCamShot::JpegParam const & rJpgParm, MUINT32 const u4Rot, MUINT32 const u4Flip, NSCamHW::ImgBufInfo const & rJpgImgBufInfo, MUINT32 & u4JpegSize);
    virtual MBOOL   createJpegImgWithThumbnail(NSCamHW::ImgBufInfo const &rYuvImgBufInfo, NSCamHW::ImgBufInfo const &rPostViewBufInfo, MUINT32 const Mode);
    virtual MBOOL   createFBJpegImg(IMEM_BUF_INFO Srcbufinfo, int u4SrcWidth, int u4SrcHeight, MUINT32 const Mode);
    virtual MBOOL   createFullFrame(IMEM_BUF_INFO Srcbufinfo);
        
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Utilities.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:  ////    Buffers.
    virtual MBOOL   requestBufs();
    virtual MBOOL   releaseBufs();
    virtual MBOOL   allocMem(IMEM_BUF_INFO &memBuf);
    virtual MBOOL   deallocMem(IMEM_BUF_INFO &memBuf);
     
protected:  ////    Misc.
    virtual MBOOL   InitialAlgorithm(MUINT32 srcWidth, MUINT32 srcHeight, MINT32 gBlurLevel, MINT32 FBTargetColor);
    virtual MBOOL   ImgProcess(IMEM_BUF_INFO Srcbufinfo, MUINT32 srcWidth, MUINT32 srcHeight, EImageFormat srctype, IMEM_BUF_INFO Desbufinfo, MUINT32 desWidth, MUINT32 desHeight, EImageFormat destype) const;
    virtual MBOOL   STEP2(IMEM_BUF_INFO Srcbufinfo, MUINT32 srcWidth, MUINT32 srcHeight, IMEM_BUF_INFO DSbufinfo, MtkCameraFaceMetadata* FaceInfo, void* FaceBeautyResultInfo);
    virtual MBOOL   STEP3(IMEM_BUF_INFO Srcbufinfo, void* FaceBeautyResultInfo) const;
    virtual MBOOL   STEP1(IMEM_BUF_INFO Srcbufinfo, MUINT32 srcWidth, MUINT32 srcHeight, IMEM_BUF_INFO Desbufinfo, IMEM_BUF_INFO NRbufinfo, void* FaceBeautyResultInfo);
    virtual MBOOL   STEP4(IMEM_BUF_INFO Srcbufinfo, MUINT32 srcWidth, MUINT32 srcHeight, IMEM_BUF_INFO Blurbufinfo, IMEM_BUF_INFO Alphabufinfo, void* FaceBeautyResultInfo);
    virtual MBOOL   STEP5(IMEM_BUF_INFO Srcbufinfo, MUINT32 srcWidth, MUINT32 srcHeight, IMEM_BUF_INFO Alphabufinfo, void* FaceBeautyResultInfo);
    virtual MBOOL   STEP6(IMEM_BUF_INFO Srcbufinfo, MUINT32 srcWidth, MUINT32 srcHeight, IMEM_BUF_INFO tmpbufinfo, void* FaceBeautyResultInfo) const;
     
};
}; // namespace NSShot
}; // namespace android

#endif  //  _MAHL_FB_H_

