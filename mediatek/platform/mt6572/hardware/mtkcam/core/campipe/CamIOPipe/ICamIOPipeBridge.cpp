
#define LOG_TAG "campipe/camio"
//
#include <utils/threads.h>
//
#include <mtkcam/Log.h>
#define MY_LOGV(fmt, arg...)    CAM_LOGV("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGD(fmt, arg...)    CAM_LOGD("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGI(fmt, arg...)    CAM_LOGI("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGW(fmt, arg...)    CAM_LOGW("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGE(fmt, arg...)    CAM_LOGE("[%s] "fmt, __FUNCTION__, ##arg)
//
#include <mtkcam/common.h>
#include <common/hw/hwstddef.h>
//
#include <campipe/IPipe.h>
#include <campipe/ICamIOPipe.h>
//
#include "../inc/PipeImp.h"
#include "../inc/CamIOPipe.h"
//
using namespace android;


namespace NSCamPipe {
////////////////////////////////////////////////////////////////////////////////


class ICamIOPipeBridge : public ICamIOPipe
{
    friend  class   ICamIOPipe;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Implementation.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
protected:  ////    
    mutable android::Mutex      mLock;
    android::Mutex&             getLockRef()    { return mLock; }
    MUINT32                     mu4InitRefCount;

protected:  ////    Implementor.
    CamIOPipe*const            mpPipeImp;
    inline  CamIOPipe const*   getImp() const  { return mpPipeImp; }
    inline  CamIOPipe*         getImp()        { return mpPipeImp; }

protected:  ////    Constructor/Destructor.
                    ICamIOPipeBridge(CamIOPipe*const pCamIOPipe);
                    ~ICamIOPipeBridge();

private:    ////    Disallowed.
                    ICamIOPipeBridge(ICamIOPipeBridge const& obj);
    ICamIOPipeBridge&  operator=(ICamIOPipeBridge const& obj);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Interfaces.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:     ////    Instantiation.
    virtual MVOID   destroyInstance();
    virtual MBOOL   init();
    virtual MBOOL   uninit();

public:     ////    Attributes.
    virtual char const* getPipeName() const;
    virtual EPipeID     getPipeId() const;
    virtual MINT32      getLastErrorCode() const;

public:     ////    Callbacks.
    virtual MVOID   setCallbacks(PipeNotifyCallback_t notify_cb, PipeDataCallback_t data_cb, MVOID* user);
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
    virtual MBOOL   start();
    virtual MBOOL   startOne(); 
    virtual MBOOL   stop();

public:     ////    Buffer Quening.
    virtual MBOOL   enqueBuf(PortID const ePortID, QBufInfo const& rQBufInfo);
    virtual MBOOL   dequeBuf(PortID const ePortID, QTimeStampBufInfo& rQBufInfo, MUINT32 const u4TimeoutMs = 0xFFFFFFFF);    

public:     ////    Settings.
    virtual MBOOL   configPipe(vector<PortInfo const*>const& vInPorts, vector<PortInfo const*>const& vOutPorts);
    virtual MBOOL   sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3); 
public:     ////    info 
    virtual MBOOL   queryPipeProperty(vector<PortProperty> &vInPorts, vector<PortProperty> &vOutPorts); 
    
public:
    virtual MVOID   waitSignal(EPipeSignal ePipeSignal, MUINT32 const u4TimeoutMs = 0xFFFFFFFF); 

};


ICamIOPipe*
ICamIOPipe::
createInstance(ESWScenarioID const eSWScenarioID, EScenarioFmt const eScenarioFmt)
{
    CamIOPipe* pPipeImp = new CamIOPipe("CamIOPipe", ICamIOPipe::ePipeID, eSWScenarioID, eScenarioFmt);
    if  ( ! pPipeImp )
    {
        MY_LOGE("[ICamIOPipe] fail to new CamIOPipe");
        return  NULL;
    }
    //
    ICamIOPipeBridge*  pIPipe = new ICamIOPipeBridge(pPipeImp);
    if  ( ! pIPipe )
    {
        MY_LOGE("[ICamIOPipe] fail to new ICamIOPipeBridge");
        delete  pPipeImp;
        return  NULL;
    }
    //
    return  pIPipe;
}


MVOID
ICamIOPipeBridge::
destroyInstance()
{
    delete  mpPipeImp;  //  Firstly, delete the implementor here instead of destructor.
    delete  this;       //  Finally, delete myself.
}


ICamIOPipeBridge::
ICamIOPipeBridge(CamIOPipe*const pCamIOPipe)
    : ICamIOPipe()
    , mLock()
    , mu4InitRefCount(0)
    , mpPipeImp(pCamIOPipe)
{
}


ICamIOPipeBridge::
~ICamIOPipeBridge()
{
}


char const*
ICamIOPipeBridge::
getPipeName() const
{
    return  getImp()->getPipeName();
}


EPipeID
ICamIOPipeBridge::
getPipeId() const
{
    return  getImp()->getPipeId();
}

MINT32
ICamIOPipeBridge::
getLastErrorCode() const
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->getLastErrorCode();
}


MBOOL
ICamIOPipeBridge::
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
ICamIOPipeBridge::
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
ICamIOPipeBridge::
setCallbacks(PipeNotifyCallback_t notify_cb, PipeDataCallback_t data_cb, MVOID* user)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->setCallbacks(notify_cb, data_cb, user);
}


MBOOL
ICamIOPipeBridge::
isNotifyMsgEnabled(MINT32 const i4MsgTypes) const
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->isNotifyMsgEnabled(i4MsgTypes);
}


MVOID
ICamIOPipeBridge::
enableNotifyMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->enableNotifyMsg(i4MsgTypes);
}


MVOID
ICamIOPipeBridge::
disableNotifyMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->disableNotifyMsg(i4MsgTypes);
}


MBOOL
ICamIOPipeBridge::
isDataMsgEnabled(MINT32 const i4MsgTypes) const
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->isDataMsgEnabled(i4MsgTypes);
}


MVOID
ICamIOPipeBridge::
enableDataMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->enableDataMsg(i4MsgTypes);
}


MVOID
ICamIOPipeBridge::
disableDataMsg(MINT32 const i4MsgTypes)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->disableDataMsg(i4MsgTypes);
}


MBOOL
ICamIOPipeBridge::
start()
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->start();
}

MBOOL
ICamIOPipeBridge::
startOne()
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->startOne();
}


MBOOL
ICamIOPipeBridge::
stop()
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->stop();
}


MBOOL
ICamIOPipeBridge::
configPipe(vector<PortInfo const*>const& vInPorts, vector<PortInfo const*>const& vOutPorts)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->configPipe(vInPorts, vOutPorts);
}

MBOOL   
ICamIOPipeBridge::
sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->sendCommand(cmd, arg1, arg2, arg3);
} 

MBOOL
ICamIOPipeBridge::
queryPipeProperty(vector<PortProperty> &vInPorts, vector<PortProperty> &vOutPorts)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->queryPipeProperty(vInPorts, vOutPorts);
}


MBOOL
ICamIOPipeBridge::
enqueBuf(PortID const ePortID, QBufInfo const& rQBufInfo)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->enqueBuf(ePortID, rQBufInfo);
}


MBOOL
ICamIOPipeBridge::
dequeBuf(PortID const ePortID, QTimeStampBufInfo& rQBufInfo, MUINT32 const u4TimeoutMs /*= 0xFFFFFFFF*/)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->dequeBuf(ePortID, rQBufInfo, u4TimeoutMs);
}

MVOID
ICamIOPipeBridge::
waitSignal(EPipeSignal ePipeSignal, MUINT32 const u4TimeoutMs)
{
    Mutex::Autolock _lock(mLock);
    return  getImp()->waitSignal(ePipeSignal, u4TimeoutMs);
}

////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamPipe

