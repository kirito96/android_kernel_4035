
#define LOG_TAG "CamShot/Sample"
//
#include <utils/threads.h>
//
#include <mtkcam/Log.h>
#include <mtkcam/common.h>

#define MY_LOGV(fmt, arg...)    CAM_LOGV("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGD(fmt, arg...)    CAM_LOGD("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGI(fmt, arg...)    CAM_LOGI("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGW(fmt, arg...)    CAM_LOGW("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGE(fmt, arg...)    CAM_LOGE("[%s] "fmt, __FUNCTION__, ##arg)
//
//
#include <mtkcam/common.h>
using namespace NSCam;
#include <common/hw/hwstddef.h>
//
#include <campipe/_ports.h>
//
#include <camshot/_callbacks.h>
#include <camshot/_params.h>

#include <camshot/ICamShot.h>
#include <camshot/ISamleSingleShot.h>
//
#include "../inc/SingleShotImp.h"
#include "../inc/SampleSingleShot.h"
//
using namespace android;


namespace NSCamShot {
////////////////////////////////////////////////////////////////////////////////


class ISampleSingleShotBridge : public ISampleSingleShot
{
    friend  class   ISampleSingleShot;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Implementation.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
protected:  ////    
    mutable android::Mutex      mLock;
    android::Mutex&             getLockRef()    { return mLock; }
    MUINT32                     mu4InitRefCount;

protected:  ////    Implementor.
    SampleSingleShot*const            mpSampleSingleShotImp;
    inline  SampleSingleShot const*   getImp() const  { return mpSampleSingleShotImp; }
    inline  SampleSingleShot*         getImp()        { return mpSampleSingleShotImp; }

protected:  ////    Constructor/Destructor.
                    ISampleSingleShotBridge(SampleSingleShot*const pSampleSingleShot);
                    ~ISampleSingleShotBridge();

private:    ////    Disallowed.
                    ISampleSingleShotBridge(ISampleSingleShotBridge const& obj);
    ISampleSingleShotBridge&  operator=(ISampleSingleShotBridge const& obj);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Interfaces.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:     ////    Instantiation.
    virtual MVOID   destroyInstance();
    virtual MBOOL   init();
    virtual MBOOL   uninit();

public:     ////    Attributes.
    virtual char const* getCamShotName() const;
    virtual EShotMode     getShotMode() const; 
    virtual MINT32      getLastErrorCode() const;

public:     ////    Callbacks.
    virtual MVOID   setCallbacks(CamShotNotifyCallback_t notify_cb, CamShotDataCallback_t data_cb, MVOID* user);
    //
    //  notify callback
    virtual MBOOL   isNotifyMsgEnabled(MINT32 const i4MsgTypes) const;
    virtual MVOID   enableNotifyMsg(MINT32 const i4MsgTypes);
    virtual MVOID   disableNotifyMsg(MINT32 const i4MsgTypes);
    //
    //  data callback
    virtual MBOOL   isDataMsgEnabled(MINT32 const i4MsgTypes) const;
    virtual MVOID   enableDataMsg(MINT32 const i4MsgTypes);
    virtual MVOID   disableDataMsg(MINT32 const i4MsgTypes);

public:     ////    Operations.
    virtual MBOOL   start(SensorParam const & rSensorParam, MUINT32 u4ShotCount=0xFFFFFFFF);
    virtual MBOOL   startAsync(SensorParam const & rSensorParam) ;   
    virtual MBOOL   startOne(SensorParam const & rSensorParam); 
    virtual MBOOL   startOne(ImgBufInfo const & rImgBufInfo); 
    virtual MBOOL   startOne(SensorParam const & rSensorParam,ImgBufInfo const & rImgBufInfo);
    virtual MBOOL   stop();

public:     ////    Settings.
    virtual MBOOL   setShotParam(ShotParam const & rParam); 
    virtual MBOOL   setJpegParam(JpegParam const & rParam); 

public:     ////    buffer setting. 
    virtual MBOOL   registerImgBufInfo(ECamShotImgBufType, ImgBufInfo const &rImgBuf); 

public:     ////    Old style commnad.
    virtual MBOOL   sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3);

};


ISampleSingleShot*
ISampleSingleShot::
createInstance(EShotMode const eShotMode, char const*const szCamShotName)
{
    SampleSingleShot* pSampleSingleShotImp = new SampleSingleShot(eShotMode, "SampleSingleShot");
    if  ( ! pSampleSingleShotImp )
    {
        MY_LOGE("[ISampleSingleShot] fail to new SampleSingleShot");
        return  NULL;
    }
    //
    ISampleSingleShotBridge*  pISampleSingleShot = new ISampleSingleShotBridge(pSampleSingleShotImp);
    if  ( ! pISampleSingleShot )
    {
        MY_LOGE("[ISampleSingleShot] fail to new ISampleSingleShotBridge");
        delete  pSampleSingleShotImp;
        return  NULL;
    }
    //
    return  pISampleSingleShot;
}


MVOID
ISampleSingleShotBridge::
destroyInstance()
{
    delete  mpSampleSingleShotImp;  //  Firstly, delete the implementor here instead of destructor.
    delete  this;       //  Finally, delete myself.
}


ISampleSingleShotBridge::
ISampleSingleShotBridge(SampleSingleShot*const pSampleSingleShot)
    : ISampleSingleShot()
    , mLock()
    , mu4InitRefCount(0)
    , mpSampleSingleShotImp(pSampleSingleShot)
{
}


ISampleSingleShotBridge::
~ISampleSingleShotBridge()
{
}


char const*
ISampleSingleShotBridge::
getCamShotName() const
{
    return  getImp()->getCamShotName();
}


EShotMode
ISampleSingleShotBridge::
getShotMode() const
{
    return  getImp()->getShotMode();
}


MINT32
ISampleSingleShotBridge::
getLastErrorCode() const
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->getLastErrorCode();
}


MBOOL
ISampleSingleShotBridge::
init()
{
    MBOOL   ret = MTRUE;
    Mutex::Autolock _lock(mLock);

    if  ( 0 != mu4InitRefCount )
    {
        mu4InitRefCount++;
    }
    else if ( (ret = getImp()->init()) )
    {
        mu4InitRefCount = 1;
    }
    MY_LOGD("- mu4InitRefCount(%d), ret(%d)", mu4InitRefCount, ret);
    return  ret;
}


MBOOL
ISampleSingleShotBridge::
uninit()
{
    MBOOL   ret = MTRUE;
    Mutex::Autolock _lock(mLock);

    if  ( 0 < mu4InitRefCount )
    {
        mu4InitRefCount--;
        if  ( 0 == mu4InitRefCount )
        {
            ret = getImp()->uninit();
        }
    }
    MY_LOGD("- mu4InitRefCount(%d), ret(%d)", mu4InitRefCount, ret);
    return  ret;
}


MVOID
ISampleSingleShotBridge::
setCallbacks(CamShotNotifyCallback_t notify_cb, CamShotDataCallback_t data_cb, MVOID* user)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->setCallbacks(notify_cb, data_cb, user);
}


MBOOL
ISampleSingleShotBridge::
isNotifyMsgEnabled(MINT32 const i4MsgTypes) const
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->isNotifyMsgEnabled(i4MsgTypes);
}


MVOID
ISampleSingleShotBridge::
enableNotifyMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->enableNotifyMsg(i4MsgTypes);
}


MVOID
ISampleSingleShotBridge::
disableNotifyMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->disableNotifyMsg(i4MsgTypes);
}


MBOOL
ISampleSingleShotBridge::
isDataMsgEnabled(MINT32 const i4MsgTypes) const
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->isDataMsgEnabled(i4MsgTypes);
}


MVOID
ISampleSingleShotBridge::
enableDataMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->enableDataMsg(i4MsgTypes);
}


MVOID
ISampleSingleShotBridge::
disableDataMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->disableDataMsg(i4MsgTypes);
}


MBOOL
ISampleSingleShotBridge::
start(SensorParam const & rSensorParam, MUINT32 u4ShotCount)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->start(rSensorParam);
}

MBOOL
ISampleSingleShotBridge::
startAsync(SensorParam const & rSensorParam)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->startAsync(rSensorParam);
}

MBOOL
ISampleSingleShotBridge::
startOne(SensorParam const & rSensorParam)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->startOne(rSensorParam);
}


MBOOL
ISampleSingleShotBridge::
startOne(ImgBufInfo const & rImgBufInfo)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->startOne(rImgBufInfo);
}

MBOOL
ISampleSingleShotBridge::
startOne(SensorParam const & rSensorParam, ImgBufInfo const & rImgBufInfo)
{
    Mutex::Autolock _lock(mLock);
    return  MTRUE;
}

MBOOL
ISampleSingleShotBridge::
stop()
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->stop();
}

MBOOL
ISampleSingleShotBridge::
setShotParam(ShotParam const & rParam)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->setShotParam(rParam);
}

MBOOL
ISampleSingleShotBridge::
setJpegParam(JpegParam const & rParam)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->setJpegParam(rParam);
}



MBOOL   
ISampleSingleShotBridge::
registerImgBufInfo(ECamShotImgBufType const eBufType, ImgBufInfo const &rImgBuf)
{
    Mutex::Autolock _lock(mLock); 
    return  getImp()->registerImgBufInfo(eBufType, rImgBuf); 
}


MBOOL   
ISampleSingleShotBridge::
sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->sendCommand(cmd, arg1, arg2, arg3);
} 


////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamShot

