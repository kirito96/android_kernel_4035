

#ifndef _ISPIO_CDP_PIPE_H_
#define _ISPIO_CDP_PIPE_H_

#include <vector>

using namespace std;


//#include <ispio_pipe_ports.h>
//#include <ispio_pipe_buffer.h>
//
#include "res_mgr_drv.h"

#include <DpBlitStream.h> // Fro DpFramework

#if (PLATFORM_VERSION_MAJOR == 2)
#include <utils/threads.h>         	// For android::Mutex.
#else
#include <utils/Mutex.h>          	// For android::Mutex.
#endif

using namespace android;
namespace NSImageio {
namespace NSIspio   {

//TODO:remove later
//int scenario_pmem_alloc_sync(unsigned int size,int *memId,unsigned char **vAddr,unsigned int *pAddr);
//int scenario_pmem_free(unsigned char *vAddr,unsigned int size,int memId);

//int scenario_pmem_alloc_sync_2(unsigned int size,int *memId,unsigned char **vAddr,unsigned int *pAddr);
//int scenario_pmem_free_2(unsigned char *vAddr,unsigned int size,int memId);


////////////////////////////////////////////////////////////////////////////////

//Tpipe Driver
//#define CDP_MAX_TDRI_HEX_SIZE           (ISP_MAX_TDRI_HEX_SIZE)




class PipeImp;


class CdpPipe : public PipeImp
{
public:     ////    Constructor/Destructor.
                    CdpPipe(
                        char const*const szPipeName,
                        EPipeID const ePipeID,
                        EScenarioID const eScenarioID,
                        EScenarioFmt const eScenarioFmt
                    );

                    virtual ~CdpPipe();

public:     ////    Instantiation.
    virtual MBOOL   init();
    virtual MBOOL   uninit();

public:     ////    Operations.
    virtual MBOOL   start();
    virtual MBOOL   syncJpegPass2C(); // sync jpeg ring buffer pass2C
    virtual MBOOL   startFmt();
    virtual MBOOL   stop();

public:     ////    Buffer Quening.
    virtual MBOOL   enqueInBuf(PortID const portID, QBufInfo const& rQBufInfo);
    virtual MBOOL   dequeInBuf(PortID const portID, QTimeStampBufInfo& rQBufInfo, MUINT32 const u4TimeoutMs = 0xFFFFFFFF);
    //
    virtual MBOOL   enqueOutBuf(PortID const portID, QBufInfo const& rQBufInfo);
    virtual MBOOL   dequeOutBuf(PortID const portID, QTimeStampBufInfo& rQBufInfo, MUINT32 const u4TimeoutMs = 0xFFFFFFFF);

public:     ////    Settings.
    virtual MBOOL   configPipe(vector<PortInfo const*>const& vInPorts, vector<PortInfo const*>const& vOutPorts);
    virtual MBOOL   configPipeUpdate(vector<PortInfo const*>const& vInPorts, vector<PortInfo const*>const& vOutPorts);
public:     ////    Commands.
    virtual MBOOL   onSet2Params(MUINT32 const u4Param1, MUINT32 const u4Param2);
    virtual MBOOL   onGet1ParamBasedOn1Input(MUINT32 const u4InParam, MUINT32*const pu4OutParam);

public:     ////    Interrupt handling
	virtual MBOOL   irq(EPipePass pass, EPipeIRQ irq_int);

public:     ////    original style sendCommand method
    virtual MBOOL   sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3);

//
private:
    ResMgrDrv*               m_resMgr;
    vector<BufInfo>          m_vBufImgi;
    vector<BufInfo>          m_vBufDispo;
    vector<BufInfo>          m_vBufVido;

    BufInfo mImgiBuf;
    BufInfo mDispoBuf;
    BufInfo mVidoBuf;
    //
    EPipePass   m_pipePass;
    //
    mutable Mutex       mLock;
    //
    EConfigSettingStage m_settingStage;
    //
    PortInfo m_portInfo_imgi;
    PortInfo m_portInfo_dispo;
    PortInfo m_portInfo_vido;
    //
    MBOOL   m_isImgPlaneByImgi;
    //
    RES_MGR_DRV_MODE_STRUCT resMgrMode;

    DpBlitStream m_dpStream;
};


////////////////////////////////////////////////////////////////////////////////
};  //namespace NSIspio
};  //namespace NSImageio
#endif  //  _ISPIO_CDP_PIPE_H_

