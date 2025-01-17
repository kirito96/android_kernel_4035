
#ifndef _ISPIO_I_PIPE_H_
#define _ISPIO_I_PIPE_H_

#include <vector>
using namespace std;

#include <mtkcam/common.h>

#include <inc/imageio/ispio_pipe_scenario.h>
#include <inc/imageio/ispio_pipe_identity.h>
#include <inc/imageio/ispio_pipe_callbacks.h>
#include <inc/imageio/ispio_pipe_ports.h>
#include <inc/imageio/ispio_pipe_buffer.h>


namespace NSImageio {
namespace NSIspio   {
////////////////////////////////////////////////////////////////////////////////


class IPipeCommand
{
public:
    virtual ~IPipeCommand() {}
    virtual MBOOL   execute() = 0;
};


class IPipe
{
public:     ////    Attributes.
    virtual char const* getPipeName() const = 0;
    virtual EPipeID     getPipeId() const = 0;
    virtual MINT32      getLastErrorCode() const = 0;

protected:  ////    Constructor/Destructor.
    virtual         ~IPipe() {}

public:     ////    Instantiation.
    virtual MVOID   destroyInstance() = 0;
    virtual MBOOL   init() = 0;
    virtual MBOOL   uninit() = 0;

public:     ////    Callbacks.
    virtual MVOID   setCallbacks(PipeNotifyCallback_t notify_cb, PipeDataCallback_t data_cb, MVOID* user) = 0;
    //
    //  notify callback
    virtual MBOOL   isNotifyMsgEnabled(MINT32 const i4MsgTypes) const   = 0;
    virtual MVOID   enableNotifyMsg(MINT32 const i4MsgTypes)            = 0;
    virtual MVOID   disableNotifyMsg(MINT32 const i4MsgTypes)           = 0;
    //
    //  data callback
    virtual MBOOL   isDataMsgEnabled(MINT32 const i4MsgTypes) const     = 0;
    virtual MVOID   enableDataMsg(MINT32 const i4MsgTypes)              = 0;
    virtual MVOID   disableDataMsg(MINT32 const i4MsgTypes)             = 0;

public:     ////    Operations.
    virtual MBOOL   start() = 0;
    virtual MBOOL   stop()  = 0;
public:     ////    Buffer Quening.
    /*
     *  Add a buffer into the specified non-sensor input port.
     *
     *  Params:
     *      PortID
     *      [I] The input port ID to add.
     *          The port ID must be one of which specified when configPipe().
     *
     *      rQBufInfo
     *      [I] Reference to a QBufInfo structure containing a buffer 
     *          information to add.
     *
     *  Return:
     *      MTRUE indicates success; MFALSE indicates failure, and an error code
     *      can be retrived by getLastErrorCode().
     */
    virtual MBOOL   enqueInBuf(PortID const portID, QBufInfo const& rQBufInfo)      = 0;
    /*
     *  De-queue a buffer information for a specified output port ID.
     *
     *  Params:
     *      PortID
     *      [I] The output port ID to dequeue.
     *
     *      rQBufInfo
     *      [O] Reference to a QTimeStampBufInfo structure to store the de-queued
     *          buffer information.
     *
     *      u4TimeoutMS
     *      [I] timeout in milliseconds
     *          If u4TimeoutMS > 0, a timeout is specified, and this call will 
     *          be blocked until any buffer is ready.
     *          If u4TimeoutMS = 0, this call must return immediately no matter
     *          whether any buffer is ready or not.
     *
     *  Return:
     *      MTRUE indicates success; MFALSE indicates failure, and an error code
     *      can be retrived by getLastErrorCode().
     */
    virtual MBOOL   dequeInBuf(
                        PortID const portID, 
                        QTimeStampBufInfo& rQBufInfo, 
                        MUINT32 const u4TimeoutMs = 0xFFFFFFFF
                    )  = 0;
    /*
     *  En-queue a buffer into the specified output port.
     *
     *  Params:
     *      PortID
     *      [I] The output port ID to enqueue.
     *          The port ID must be one of which specified when configPipe().
     *
     *      rQBufInfo
     *      [I] Reference to a QBufInfo structure containing a buffer 
     *          information to enqueue.
     *
     *  Return:
     *      MTRUE indicates success; MFALSE indicates failure, and an error code
     *      can be retrived by getLastErrorCode().
     */
    virtual MBOOL   enqueOutBuf(PortID const portID, QBufInfo const& rQBufInfo)     = 0;

    /*
     *  De-queue a buffer information for a specified output port ID.
     *
     *  Params:
     *      PortID
     *      [I] The output port ID to dequeue.
     *
     *      rQBufInfo
     *      [O] Reference to a QTimeStampBufInfo structure to store the de-queued
     *          buffer information.
     *
     *      u4TimeoutMS
     *      [I] timeout in milliseconds
     *          If u4TimeoutMS > 0, a timeout is specified, and this call will 
     *          be blocked until any buffer is ready.
     *          If u4TimeoutMS = 0, this call must return immediately no matter
     *          whether any buffer is ready or not.
     *
     *  Return:
     *      MTRUE indicates success; MFALSE indicates failure, and an error code
     *      can be retrived by getLastErrorCode().
     */
    virtual MBOOL   dequeOutBuf(
                        PortID const portID, 
                        QTimeStampBufInfo& rQBufInfo, 
                        MUINT32 const u4TimeoutMs = 0xFFFFFFFF
                    )  = 0;

public:     ////    Settings.
    /*
     *  Configure in/out ports of this pipe.
     *
     *  Params:
     *      vInPorts
     *      [I] Reference to the vector of input ports.
     *
     *      vOutPorts
     *      [I] Reference to the vector of output ports.
     *
     *  Return:
     *      MTRUE indicates success; MFALSE indicates failure, and an error code
     *      can be retrived by getLastErrorCode().
     */
    virtual MBOOL   configPipe(vector<PortInfo const*>const& vInPorts, vector<PortInfo const*>const& vOutPorts) = 0;

virtual MBOOL   configPipeUpdate(vector<PortInfo const*>const& vInPorts, vector<PortInfo const*>const& vOutPorts) = 0;

public:
    ////    Interrupt handling
    virtual MBOOL   irq(EPipePass pass, EPipeIRQ irq_int) = 0;
    ////    original style sendCommand method
    virtual MBOOL   sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3) = 0;
};


////////////////////////////////////////////////////////////////////////////////
};  //namespace NSIspio
};  //namespace NSImageio
#endif  //  _ISPIO_I_PIPE_H_

