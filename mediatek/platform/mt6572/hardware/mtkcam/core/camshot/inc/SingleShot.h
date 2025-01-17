
#ifndef _MTK_CAMERA_CORE_CAMSHOT_INC_CAMSHOT_SINGLESHOT_H_
#define _MTK_CAMERA_CORE_CAMSHOT_INC_CAMSHOT_SINGLESHOT_H_
//
//

namespace NSCamPipe
{
    class ICamIOPipe;
    class IPostProcPipe; 
    struct PipeNotifyInfo;
};

class ISImager; 

class ResMgrDrv; 
class PipeMgrDrv; 

namespace NSCamShot {
////////////////////////////////////////////////////////////////////////////////



class CamShotImp;


class SingleShot : public CamShotImp
{
public:     ////    Constructor/Destructor.
                    SingleShot(
                        EShotMode const eShotMode,
                        char const*const szCamShotName
                    );

public:     ////    Instantiation.
    virtual MBOOL   init();
    virtual MBOOL   uninit();

public:     ////    Operations.
    virtual MBOOL   start(SensorParam const & rSensorParam);
    virtual MBOOL   startAsync(SensorParam const & rSensorParam);    
    virtual MBOOL   startOne(SensorParam const  & rSensorParam); 
    virtual MBOOL   startOne(ImgBufInfo const  & rImgBufInfo); 
    virtual MBOOL   startOne(SensorParam const & rSensorParam, ImgBufInfo const & rImgBufInfo);
    virtual MBOOL   stop();

public:     ////    Settings.
    virtual MBOOL   setShotParam(ShotParam const & rParam); 
    virtual MBOOL   setJpegParam(JpegParam const & rParam); 

public:     ////    buffer setting. 
    virtual MBOOL   registerImgBufInfo(ECamShotImgBufType, ImgBufInfo const &rImgBuf); 

public:     ////    Info.

public:     ////    Old style commnad.
    virtual MBOOL   sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3);
protected:  ////    callback
    static  MBOOL   fgPipeNotifyCb(MVOID* user, NSCamPipe::PipeNotifyInfo const msg);

protected: 
    MBOOL     startOne(ImgBufInfo const & rImgRawBufInfo, ImgBufInfo const & rJpegImgBufInfo);

    MBOOL     createSensorRawImg(SensorParam const & rSensorParam, ImgBufInfo const & rRawImgBufInfo); 
    MBOOL     createSensorRawImg(SensorParam const & rSensorParam, ImgBufInfo const & rRawImgBufInfo, ImgBufInfo const & rJpegImgBufInfo);
    MBOOL     createYuvRawImg(ImgBufInfo const & rSrcImgBufInfo, Rect const rSrcCropRect, MUINT32 const u4Img1Rot, MUINT32 const u4Img1Flip, ImgBufInfo const & rDstImgBufInfo1,  ImgBufInfo const &rDstImgBufInfo2); 
    MBOOL     createYuvRawImg(ImgBufInfo const & rSrcImgBufInfo, Rect const rSrcCropRect, MUINT32 const u4Rot, MUINT32 const u4Flip, ImgBufInfo const & rDstImgBufInfo);     
    MBOOL     createJpegImg(ImgBufInfo const & rSrcImgBufInfo, JpegParam const & rJpgParm, MUINT32 const u4Rot, MUINT32 const u4Flip, ImgBufInfo const & rJpgImgBufInfo, MUINT32 & u4JpegSize); 
    MBOOL     decodeJpegImg(ImgBufInfo const & rJpegImgBufInfo, MUINT32 fileSize, ImgBufInfo const & rDstImgBufInfo, EImageFormat eImgFmt);
    MBOOL     checkIfFireFlash(MUINT32 countdown);
protected:    //// helper function 
    MBOOL     allocMem(IMEM_BUF_INFO & rMemBuf); 
    MBOOL     deallocMem(IMEM_BUF_INFO & rMemBuf); 
    MBOOL     reallocMem(IMEM_BUF_INFO & rMemBuf, MUINT32 const u4Size); 
    MBOOL     allocImgMem(char const* const pszName, EImageFormat const eFmt, MUINT32 const u4Width, MUINT32 const u4Height, IMEM_BUF_INFO & rMem);
    // 
    inline    MVOID     setImageBuf(EImageFormat const eFmt, MUINT32 const u4Width, MUINT32 const u4Height, ImgBufInfo & rBuf, IMEM_BUF_INFO & rMem); 
    inline    MVOID     freeShotMem(); 
    //
    ImgBufInfo querySensorRawImgBufInfo(); 
    ImgBufInfo queryYuvRawImgBufInfo(); 
    ImgBufInfo queryJpegImgBufInfo(); 
    ImgBufInfo queryPostViewImgInfo(); 
    ImgBufInfo queryThumbImgBufInfo(); 
    inline    MVOID     getPictureDimension(MUINT32 & u4Width,  MUINT32 & u4Height); 
    inline    MUINT32    mapScenarioType(EImageFormat const eFmt);     
    //
    MBOOL        lock(MUINT32 u4HWScenario, MUINT32 u4PipeType,  MUINT32 const u4TimeOutInMs);
    MBOOL        unlock(MUINT32 u4PipeType);
    
private:
    pthread_t mPreAllocMemThreadHandle;

private:
    static void*    _preAllocMemThread(void* arg);	
    MBOOL    preAllocMem(); 
    MBOOL    waitPreAllocMemDone(); 
    MBOOL    _preAllocMem(); 

private:      //// data members
    ISImager *mpISImager; 
    ICamIOPipe    *mpCamIOPipe;
    IMemDrv *mpMemDrv; 
    // 
    MUINT32   mu4DumpFlag; 

    ////      parameters
    SensorParam mSensorParam; 
    ShotParam   mShotParam; 
    JpegParam   mJpegParam;     
    ////      buffer 
    ImgBufInfo mRawImgBufInfo; 
    ImgBufInfo mYuvImgBufInfo; 
    ImgBufInfo mPostViewImgBufInfo; 
    ImgBufInfo mJpegImgBufInfo; 
    ImgBufInfo mThumbImgBufInfo; 
    ////
    IMEM_BUF_INFO mRawMem; 
    IMEM_BUF_INFO mYuvMem; 
    IMEM_BUF_INFO mPostViewMem; 
    IMEM_BUF_INFO mJpegMem; 
    IMEM_BUF_INFO mThumbnailMem; 

    PipeMgrDrv   *mpPipeMgrDrv; 
    ResMgrDrv    *mpResMgrDrv; 

    MBOOL         isCaptureJPEGFile;
    IMEM_BUF_INFO mMemOfCaptureJPEGFile; 

    MINT32       mu4FlashCountDown;
};


////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamShot
#endif  //  _MTK_CAMERA_CORE_CAMSHOT_INC_CAMSHOT_SINGLESHOT_H_

