
#define LOG_TAG "MtkCam/CamAdapter"
//
#include <inc/CamUtils.h>
using namespace android;
using namespace MtkCamUtils;
//
#include <inc/IState.h>
#include "State.h"
using namespace NSMtkDefaultCamAdapter;
//


#define MY_LOGV(fmt, arg...)        CAM_LOGV("(%d)(%s)[%s] "fmt, ::gettid(), getName(), __FUNCTION__, ##arg)
#define MY_LOGD(fmt, arg...)        CAM_LOGD("(%d)(%s)[%s] "fmt, ::gettid(), getName(), __FUNCTION__, ##arg)
#define MY_LOGI(fmt, arg...)        CAM_LOGI("(%d)(%s)[%s] "fmt, ::gettid(), getName(), __FUNCTION__, ##arg)
#define MY_LOGW(fmt, arg...)        CAM_LOGW("(%d)(%s)[%s] "fmt, ::gettid(), getName(), __FUNCTION__, ##arg)
#define MY_LOGE(fmt, arg...)        CAM_LOGE("(%d)(%s)[%s] "fmt, ::gettid(), getName(), __FUNCTION__, ##arg)
#define MY_LOGA(fmt, arg...)        CAM_LOGA("(%d)(%s)[%s] "fmt, ::gettid(), getName(), __FUNCTION__, ##arg)
#define MY_LOGF(fmt, arg...)        CAM_LOGF("(%d)(%s)[%s] "fmt, ::gettid(), getName(), __FUNCTION__, ##arg)
//
#define MY_LOGV_IF(cond, ...)       do { if ( (cond) ) { MY_LOGV(__VA_ARGS__); } }while(0)
#define MY_LOGD_IF(cond, ...)       do { if ( (cond) ) { MY_LOGD(__VA_ARGS__); } }while(0)
#define MY_LOGI_IF(cond, ...)       do { if ( (cond) ) { MY_LOGI(__VA_ARGS__); } }while(0)
#define MY_LOGW_IF(cond, ...)       do { if ( (cond) ) { MY_LOGW(__VA_ARGS__); } }while(0)
#define MY_LOGE_IF(cond, ...)       do { if ( (cond) ) { MY_LOGE(__VA_ARGS__); } }while(0)
#define MY_LOGA_IF(cond, ...)       do { if ( (cond) ) { MY_LOGA(__VA_ARGS__); } }while(0)
#define MY_LOGF_IF(cond, ...)       do { if ( (cond) ) { MY_LOGF(__VA_ARGS__); } }while(0)


#define ENABLE_OP_LOG               (1)


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StateBase
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


StateBase::
StateBase(char const*const pcszName, ENState const eState)
    : IState()
    , m_pcszName(pcszName)
    , m_eState(eState)
    , m_pStateManager(IStateManager::inst())
{
}


status_t
StateBase::
op_UnSupport(char const*const pcszDbgText)
{
    MY_LOGW("%s", pcszDbgText);
    //
    return  INVALID_OPERATION;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StateIdle
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StateIdle::
StateIdle(ENState const eState)
    : StateBase(__FUNCTION__, eState)
{
}


status_t
StateIdle::
onStartPreview(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStartPreview();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Preview);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}



status_t
StateIdle::
onCapture(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleCapture();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
#if 1
    status = stateWaiter.waitState(eState_Capture);
    if  ( OK != status ) {
        goto lbExit;
    }
#endif
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StatePreview
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StatePreview::
StatePreview(ENState const eState)
    : StateBase(__FUNCTION__, eState)
{
}


status_t
StatePreview::
onStopPreview(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStopPreview();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Idle);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


status_t
StatePreview::
onPreCapture(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandlePreCapture();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_PreCapture);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


status_t
StatePreview::
onStartRecording(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStartRecording();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Recording);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StatePreCapture
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StatePreCapture::
StatePreCapture(ENState const eState)
    : StateBase(__FUNCTION__, eState)
{
}


status_t
StatePreCapture::
onStopPreview(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStopPreview();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Idle);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StateCapture
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StateCapture::
StateCapture(ENState const eState)
    : StateBase(__FUNCTION__, eState)
{
}


status_t
StateCapture::
onCaptureDone(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleCaptureDone();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Idle);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


status_t
StateCapture::
onCancelCapture(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleCancelCapture();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
#if 1
    status = stateWaiter.waitState(eState_Idle);
    if  ( OK != status ) {
        goto lbExit;
    }
#endif
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StateRecording
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StateRecording::
StateRecording(ENState const eState)
    : StateBase(__FUNCTION__, eState)
{
}


status_t
StateRecording::
onPreCapture(IStateHandler* pHandler)
{
    return  OK;
}


status_t
StateRecording::
onStopPreview(IStateHandler* pHandler)
{
    return  OK;
#if 0
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStopPreview();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Idle);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
#endif
}


status_t
StateRecording::
onStopRecording(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStopRecording();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Preview);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


status_t
StateRecording::
onCapture(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleVideoSnapshot();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_VideoSnapshot);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  StateRecording
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
StateVideoSnapshot::
StateVideoSnapshot(ENState const eState)
    : StateBase(__FUNCTION__, eState)
{
}


status_t
StateVideoSnapshot::
onStartRecording(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    getStateManager()->registerOneShotObserver(&stateWaiter);
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStartRecording();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    status = stateWaiter.waitState(eState_Recording);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}

#if 0
status_t
StateVideoSnapshot::
onStopRecording(IStateHandler* pHandler)
{
    IStateManager::StateObserver stateWaiter(getStateManager());
    //
    //
    MY_LOGD_IF(ENABLE_OP_LOG, "+");
    //
    status_t status = OK;
    //
    status = pHandler->onHandleStopRecording();
    if  ( OK != status ) {
        goto lbExit;
    }
    //
    getStateManager()->registerOneShotObserver(&stateWaiter);
    status = stateWaiter.waitState(eState_Preview);
    if  ( OK != status ) {
        goto lbExit;
    }
    //
lbExit:
    MY_LOGD_IF(ENABLE_OP_LOG, "- status(%d)", status);
    return  status;
}
#endif
