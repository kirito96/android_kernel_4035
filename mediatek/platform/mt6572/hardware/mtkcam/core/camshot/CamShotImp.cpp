#define LOG_TAG "CamShot/CamShotimp"
//
#include <mtkcam/Log.h>
#define MY_LOGV(fmt, arg...)    CAM_LOGV("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGD(fmt, arg...)    CAM_LOGD("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGI(fmt, arg...)    CAM_LOGI("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGW(fmt, arg...)    CAM_LOGW("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGE(fmt, arg...)    CAM_LOGE("[%s] "fmt, __FUNCTION__, ##arg)

#include <mtkcam/common.h>
#include <common/hw/hwstddef.h>
//
#include <campipe/_ports.h>
//
#include <camshot/_callbacks.h>
#include <camshot/_params.h>
//
#include "./inc/CamShotImp.h"
//


namespace NSCamShot {

////////////////////////////////////////////////////////////////////////////////


CamShotImp::
CamShotImp(
    EShotMode const eShotMode,
    char const*const szCamShotName
)
    : meShotMode(eShotMode)
    , mszCamShotName(szCamShotName)
    //
    , mi4ErrorCode(0)
    //
    , mpCbUser(NULL)
    , mi4NotifyMsgSet(0)
    , mNotifyCb(NULL)
    , mi4DataMsgSet(0)
    , mDataCb(NULL)
    //
{
}

MVOID
CamShotImp::
setCallbacks(CamShotNotifyCallback_t notify_cb, CamShotDataCallback_t data_cb, MVOID* user)
{
    MY_LOGV("(notify_cb, data_cb, user)=(%p, %p, %p)", notify_cb, data_cb, user);
    mpCbUser = user;
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
}


MBOOL
CamShotImp::
onNotifyCallback(CamShotNotifyInfo const& msg) const
{
    MBOOL   ret = MTRUE;
    //
    if  ( mNotifyCb )
    {
        mNotifyCb(mpCbUser, msg);
        ret = MTRUE;
    }
    else
    {
        MY_LOGW("Notify Callback is NULL");
        ret = MFALSE;
    }
    return  ret;
}


MBOOL
CamShotImp::
onDataCallback(CamShotDataInfo const& msg) const
{
    MBOOL   ret = MTRUE;
    //
    if  ( mDataCb )
    {
        mDataCb(mpCbUser, msg);
        ret = MTRUE;
    }
    else
    {
        MY_LOGW("Data Callback is NULL");
        ret = MFALSE;
    }
    return  ret;
}


MVOID 
CamShotImp::
dumpPipeProperty(vector<PortProperty> const &vInPorts, vector<PortProperty> const &vOutPorts)
{
    //
    MY_LOGD("PipeProperty In Ports:"); 
    for (MUINT32 i = 0 ; i < vInPorts.size(); i++) 
    {
        MY_LOGD("(type, index, inout, format, rotate, flip): (%d, %d, %d, 0x%x, %d, %d)", 
                               vInPorts.at(i).type, vInPorts.at(i).index, vInPorts.at(i).inout, 
                               vInPorts.at(i).u4SupportFmt,vInPorts.at(i).fgIsSupportRotate,vInPorts.at(i).fgIsSupportFlip);                                                        
    }
    //
    MY_LOGD("PipeProperty Out Ports:"); 
    for (MUINT32 i = 0 ; i < vOutPorts.size(); i++) 
    {
        MY_LOGD("(type, index, inout, format, rotate, flip): (%d, %d, %d, 0x%x, %d, %d)", 
                               vOutPorts.at(i).type, vOutPorts.at(i).index, vOutPorts.at(i).inout, 
                               vOutPorts.at(i).u4SupportFmt,vOutPorts.at(i).fgIsSupportRotate,vOutPorts.at(i).fgIsSupportFlip);                                                        
    }
}


MVOID  
CamShotImp::
dumpSensorParam(SensorParam const & rParam)
{
    MY_LOGD("+"); 
    MY_LOGD("(id, scenario, bitdepth) = (%d, %d, %d)", rParam.u4DeviceID, rParam.u4Scenario, rParam.u4Bitdepth); 
    MY_LOGD("(fgBypassDelay, fgBypassScenaio) = (%d, %d)", rParam.fgBypassDelay, rParam.fgBypassScenaio); 
    MY_LOGD("-"); 

}

MVOID  
CamShotImp::
dumpShotParam(ShotParam const & rParam)
{
    MY_LOGD("+"); 
    MY_LOGD("Picture (fmt, width, height, rotation, flip) = (%x, %d, %d, %d, %d)", 
                       rParam.ePictureFmt, rParam.u4PictureWidth, rParam.u4PictureHeight, 
                       rParam.u4PictureRotation, rParam.u4PictureFlip); 
    MY_LOGD("PostView (fmt, width, height, rotation, flip) = (%x, %d, %d, %d, %d)", 
                       rParam.ePostViewFmt, rParam.u4PostViewWidth, rParam.u4PostViewHeight, 
                       rParam.u4PostViewRotation, rParam.u4PostViewFlip); 
    MY_LOGD("ZoomRatoio = %d", rParam.u4ZoomRatio);    

    MY_LOGD("-"); 

}
 
MVOID  
CamShotImp::
dumpJpegParam(JpegParam const & rParam)
{
    MY_LOGD("+"); 
    MY_LOGD("Jpeg (Quality, fgIsSOI) = (%d, %d)", rParam.u4Quality, rParam.fgIsSOI); 
    MY_LOGD("Thumbnail (Width, Height, Quality, fgIsSOI) = (%d, %d, %d, %d)", rParam.u4ThumbWidth, rParam.u4ThumbHeight,
                                                                     rParam.u4ThumbQuality, rParam.fgThumbIsSOI); 
 
    MY_LOGD("-"); 
}


MBOOL      
CamShotImp::
handleNotifyCallback(MINT32 const i4Msg, MUINT32 const ext1, MUINT32 const ext2)
{
    MBOOL ret = MTRUE; 
    if (isNotifyMsgEnabled(i4Msg)) 
    {
        CamShotNotifyInfo rCbNotifyInfo(i4Msg, ext1, ext2); 
        ret = onNotifyCallback(rCbNotifyInfo);         
    }
    return ret; 
}

MBOOL      
CamShotImp::
handleDataCallback(MINT32 const i4Msg, MUINT32 const ext1, MUINT32 const ext2, MUINT8* puData, MUINT32 const u4Size)
{
    MBOOL ret = MTRUE; 
    if (isDataMsgEnabled(i4Msg)) 
    {
        CamShotDataInfo rCbDataInfo(i4Msg, 
                                    ext1, 
                                    ext2, 
                                    puData, 
                                    u4Size
                                   ); 
        ret = onDataCallback(rCbDataInfo); 
    }
    return ret; 
}



////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamShot


