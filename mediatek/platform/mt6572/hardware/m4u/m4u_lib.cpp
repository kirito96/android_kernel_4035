
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <cutils/log.h>
#include "m4u_lib.h"
#include <string.h>
#include <errno.h>

#undef LOG_TAG
#define LOG_TAG "M4U_L"

#define MTKM4UMSG
#ifdef MTKM4UMSG
  #define M4UMSG(...) \
        do { \
            ALOGI(__VA_ARGS__); \
        } while (0)
#else
  #define M4UMSG(...)
#endif


#define __M4U_WRAP_LAYER_EN__

#ifdef __M4U_WRAP_LAYER_EN__
    bool m4u_en[M4U_CLNTMOD_MAX] = {false};
#endif



//#define __DUMP_BACKTRACE_ON_ERROR__
#ifdef __DUMP_BACKTRACE_ON_ERROR__
extern "C" void rtt_dump_caller_backtrace(const char* tag);
extern "C" int rtt_dump_backtrace(pid_t pid, pid_t tid, const char* file_path); 
static void m4u_dump_backtrace(void)
{
    char name[35] = "/data/m4u_dump";
    //sprintf(name, "/data/m4u_dump_%d_%d.txt", getpid(), gettid());
    M4UMSG("m4u dump back trace when error============> \n");
    rtt_dump_backtrace(getpid(), gettid(), name);

    FILE *fp = fopen(name, "r");
    if(fp)
    {
        char tmp[101] = "";
        int cnt=0;
        while( (fgets(tmp, 100, fp) != NULL) && (cnt<500) )
        {
            cnt++;
            M4UMSG("[%d]: %s", cnt, tmp);
        }
       fclose(fp);
    }
    //unlink(name);
}
#else
static void m4u_dump_backtrace(void)
{
}
#endif


MTKM4UDrv::MTKM4UDrv(void)
{

    mFileDescriptor = -1;
 
    mFileDescriptor = open("/proc/M4U_device", O_RDONLY);
    if(mFileDescriptor<0)
    {
        M4UMSG("Open file failed mFileDescriptor=%d, error=%d :%s", mFileDescriptor, errno, strerror(errno));
        m4u_dump_backtrace();
    }
    else
    {
        if(ioctl(mFileDescriptor, MTK_M4U_T_M4UDrv_CONSTRUCT, NULL))
        {
            M4UMSG(" ioctl MTK_M4U_T_M4UDrv_CONSTRUCT fail! %d, %s \n", errno, strerror(errno));
            m4u_dump_backtrace();
        }
    }

}

MTKM4UDrv::~MTKM4UDrv(void)
{    
    if(-1!=mFileDescriptor)
    {
        if(ioctl(mFileDescriptor, MTK_M4U_T_M4UDrv_DECONSTRUCT, NULL))
        {
            M4UMSG(" ioctl MTK_M4U_T_M4UDrv_DECONSTRUCT fail! %d, %s \n", errno, strerror(errno));
            m4u_dump_backtrace();
        }
    	close(mFileDescriptor);
    }
    mFileDescriptor = -1;
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_power_on(M4U_MODULE_ID_ENUM eModuleID)
{

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }
    
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_power_on failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }

    if(ioctl(mFileDescriptor, MTK_M4U_T_POWER_ON, &eModuleID))
    {
        M4UMSG(" ioctl MTK_M4U_T_POWER_ON fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_power_off(M4U_MODULE_ID_ENUM eModuleID)
{

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }    
    
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_power_off failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }

    if(ioctl(mFileDescriptor, MTK_M4U_T_POWER_OFF, &eModuleID))
    {
        M4UMSG(" ioctl MTK_M4U_T_POWER_OFF fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_alloc_mva(M4U_MODULE_ID_ENUM eModuleID, 
								  const unsigned int BufAddr, 
								  const unsigned int BufSize,
								  int security,
								  int cache_coherent,
								  unsigned int *pRetMVAAddr)
{
    M4U_MOUDLE_STRUCT m4u_module;
    m4u_module.eModuleID = eModuleID;
    m4u_module.BufAddr = BufAddr;
    m4u_module.BufSize = BufSize;
    m4u_module.security = security;
    m4u_module.cache_coherent = cache_coherent;


    if(!m4u_check_m4u_en(eModuleID))
    {
        //just work around, tell kernel it's pmem, and use wrap layer!!
        m4u_module.MVAStart = -1;
        if(mFileDescriptor<0 )
        {
            M4UMSG("m4u_alloc_mva failed \n");
            m4u_dump_backtrace();
            return M4U_STATUS_INVALID_HANDLE;
        }
        if(ioctl(mFileDescriptor, MTK_M4U_T_ALLOC_MVA, &m4u_module))
        {
            M4UMSG(" ioctl MTK_M4U_T_ALLOC_MVA fail! %d, %s \n", errno, strerror(errno));
            m4u_dump_backtrace();
            return M4U_STATUS_KERNEL_FAULT;
        }
        else
        {
            M4UMSG(" ioctl MTK_M4U_T_ALLOC_MVA pmem! VA:0x%x, MVA:0x%x \n", m4u_module.BufAddr, m4u_module.MVAStart);
            *pRetMVAAddr = m4u_module.MVAStart;
            return M4U_STATUS_OK;
        }

        return M4U_STATUS_OK;
    }
    else
    {
        m4u_module.MVAStart = 0;

        if(mFileDescriptor<0 )
        {
            M4UMSG("m4u_alloc_mva failed \n");
            m4u_dump_backtrace();
            return M4U_STATUS_INVALID_HANDLE;
        }
        if(ioctl(mFileDescriptor, MTK_M4U_T_ALLOC_MVA, &m4u_module))
        {
            M4UMSG(" ioctl MTK_M4U_T_ALLOC_MVA fail! %d, %s \n", errno, strerror(errno));
            m4u_dump_backtrace();
            return M4U_STATUS_KERNEL_FAULT;
        }
        else
        {
            M4UMSG(" ioctl MTK_M4U_T_ALLOC_MVA! VA:0x%x, MVA:0x%x \n", m4u_module.BufAddr, m4u_module.MVAStart);        
            *pRetMVAAddr = m4u_module.MVAStart;
            return M4U_STATUS_OK;
        }
    }
}

 	        

M4U_STATUS_ENUM MTKM4UDrv::m4u_query_mva(M4U_MODULE_ID_ENUM eModuleID, 
								  const unsigned int BufAddr, 
								  const unsigned int BufSize, 
								  unsigned int *pRetMVAAddr)
{
    M4U_MOUDLE_STRUCT m4u_module;
    m4u_module.eModuleID = eModuleID;
    m4u_module.BufAddr = BufAddr;
    m4u_module.BufSize = BufSize;

    *pRetMVAAddr = 0;

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }
 	        
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_query_mva failed \n");
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_QUERY_MVA, &m4u_module))
    {
        M4UMSG(" ioctl MTK_M4U_T_QUERY_MVA fail! %d, %s \n", errno, strerror(errno));
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        *pRetMVAAddr = m4u_module.MVAStart;
        return M4U_STATUS_OK;
    }
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_dealloc_mva(M4U_MODULE_ID_ENUM eModuleID, 
									const unsigned int BufAddr, 
									const unsigned int BufSize,
                                    const unsigned int MVAStart)
{
    M4U_MOUDLE_STRUCT m4u_module;
    m4u_module.eModuleID = eModuleID;
    m4u_module.BufAddr = BufAddr;
    m4u_module.BufSize = BufSize;
    m4u_module.MVAStart = MVAStart;



    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }
	        
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_dealloc_mva failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_DEALLOC_MVA, &m4u_module))
    {
        M4UMSG(" ioctl MTK_M4U_T_DEALLOC_MVA fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }

}


M4U_STATUS_ENUM MTKM4UDrv::m4u_insert_wrapped_range(M4U_MODULE_ID_ENUM eModuleID, 
                                  M4U_PORT_ID_ENUM portID, 
								  const unsigned int MVAStart, 
								  const unsigned int MVAEnd)
{
    M4U_WRAP_DES_T m4u_wrap;
    m4u_wrap.eModuleID = eModuleID;
    m4u_wrap.ePortID = portID;
    m4u_wrap.MVAStart = MVAStart;
    m4u_wrap.MVAEnd = MVAEnd;
	      

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_insert_wrapped_range failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_INSERT_WRAP_RANGE, &m4u_wrap))
    {
        M4UMSG(" ioctl MTK_M4U_T_INSERT_TLB_RANGE fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }	
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_insert_tlb_range(M4U_MODULE_ID_ENUM eModuleID, 
										unsigned int MVAStart, 
										const unsigned int MVAEnd, 
										unsigned int entryCount) 
{
    M4U_MOUDLE_STRUCT m4u_module;
    m4u_module.eModuleID = eModuleID;
    m4u_module.MVAStart = MVAStart;
    m4u_module.MVAEnd = MVAEnd;
    m4u_module.entryCount = entryCount;  


    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }
	      
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_insert_tlb_range failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_INSERT_TLB_RANGE, &m4u_module))
    {
        M4UMSG(" ioctl MTK_M4U_T_INSERT_TLB_RANGE fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }

}

M4U_STATUS_ENUM MTKM4UDrv::m4u_insert_tlb_range(M4U_MODULE_ID_ENUM eModuleID, 
										unsigned int MVAStart, 
										const unsigned int MVAEnd, 
										M4U_RANGE_PRIORITY_ENUM ePriority,
										unsigned int entryCount) 
{
    return m4u_insert_tlb_range(eModuleID, MVAStart, MVAEnd, entryCount);
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_invalid_tlb_range(M4U_MODULE_ID_ENUM eModuleID,
										  unsigned int MVAStart, 
										  unsigned int MVAEnd)
{
    M4U_MOUDLE_STRUCT m4u_module;
    m4u_module.eModuleID = eModuleID;
    m4u_module.MVAStart = MVAStart;
    m4u_module.MVAEnd = MVAEnd;


    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_invalid_tlb_range failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_INVALID_TLB_RANGE, &m4u_module))
    {
        M4UMSG(" ioctl MTK_M4U_T_INVALID_TLB_RANGE fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_invalid_tlb_all(M4U_MODULE_ID_ENUM eModuleID)
{

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }
	        
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_invalid_tlb_all failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_INVALID_TLB_ALL, &eModuleID))
    {
        M4UMSG(" ioctl MTK_M4U_T_INVALID_TLB_ALL fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_manual_insert_entry(M4U_MODULE_ID_ENUM eModuleID,
									unsigned int EntryMVA, 
									bool Lock)	
{


    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    M4U_MOUDLE_STRUCT m4u_module;

    if(0)
    {
        m4u_module.eModuleID = eModuleID;
        m4u_module.EntryMVA = EntryMVA;
        m4u_module.Lock = Lock;
    	        
        if(mFileDescriptor<0 )
        {
            M4UMSG("m4u_insert_entry failed \n");
            m4u_dump_backtrace();
            return M4U_STATUS_INVALID_HANDLE;
        }
        if(ioctl(mFileDescriptor, MTK_M4U_T_MANUAL_INSERT_ENTRY, &m4u_module))
        {
            M4UMSG(" ioctl MTK_M4U_T_MANUAL_INSERT_ENTRY fail! %d, %s \n", errno, strerror(errno));
            m4u_dump_backtrace();
            return M4U_STATUS_KERNEL_FAULT;
        }
        else
        {
            return M4U_STATUS_OK;
        }	
    }
    else
    {
        return M4U_STATUS_OK;
    }
}


M4U_MODULE_ID_ENUM m4u_get_module_by_port(M4U_PORT_ID_ENUM portID)
{
    M4U_MODULE_ID_ENUM moduleID = M4U_CLNTMOD_UNKNOWN;
    switch(portID)
    {
        case M4U_PORT_MDP_RDMA               :
        case M4U_PORT_MDP_WDMA               :
        case M4U_PORT_MDP_ROTO               :
        case M4U_PORT_MDP_ROTCO              :
        case M4U_PORT_MDP_ROTVO              :
            moduleID = M4U_CLNTMOD_MDP;
            break;
        
        case M4U_PORT_LCD_OVL                :
        case M4U_PORT_LCD_R                  :
        case M4U_PORT_LCD_W                  :
        case M4U_PORT_LCD_DBI                :
            moduleID = M4U_CLNTMOD_DISP;
            break;

        
        case M4U_PORT_VENCMC                 :
        case M4U_PORT_VENC_REC_VDEC_WDMA     :
        case M4U_PORT_VENC_CDMA_VDEC_CDMA    :
        case M4U_PORT_VENC_MVQP              :
        case M4U_PORT_VENC_BSDMA_VDEC_POST0  :			
            moduleID = M4U_CLNTMOD_VIDEO;
            break;

        case M4U_PORT_CAM_WDMA               :
            moduleID = M4U_CLNTMOD_CAM;
            break;

        case M4U_PORT_CMDQ                :
            moduleID = M4U_CLNTMOD_CMDQ;
            break;
        default:
        	M4UMSG("m4u_get_module_by_port() fail, invalid portID=%d", portID);
    }	
  
    return moduleID;    
}


///> native
M4U_STATUS_ENUM MTKM4UDrv::m4u_config_port(M4U_PORT_STRUCT* pM4uPort)
{

    M4U_MODULE_ID_ENUM eModuleID = m4u_get_module_by_port(pM4uPort->ePortID);

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    if(NULL==pM4uPort)
    {
        M4UMSG("m4u_config_port failed, input M4U_PORT_STRUCT* is null! \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
	        	        
    if(mFileDescriptor<0)
    {
        M4UMSG("m4u_config_port failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_CONFIG_PORT, pM4uPort))
    {
        M4UMSG(" ioctl MTK_M4U_T_CONFIG_PORT fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }	
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_config_port_rotator(M4U_PORT_STRUCT_ROTATOR* pM4uPort)
{

    M4U_MODULE_ID_ENUM eModuleID = m4u_get_module_by_port(pM4uPort->ePortID);
    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    if(NULL==pM4uPort)
    {
        M4UMSG("m4u_config_port_rotator failed, input M4U_PORT_STRUCT_ROTATOR* is null! \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
	        	        
    if(mFileDescriptor<0)
    {
        M4UMSG("m4u_config_port_rotator failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_CONFIG_PORT_ROTATOR, pM4uPort))
    {
        M4UMSG(" ioctl MTK_M4U_T_CONFIG_PORT_ROTATOR fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }	
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_reset_mva_release_tlb(M4U_MODULE_ID_ENUM eModuleID)
{

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_reset_mva_release_tlb failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }

    if(ioctl(mFileDescriptor, MTK_M4U_T_RESET_MVA_RELEASE_TLB, &eModuleID))
    {
        M4UMSG(" ioctl MTK_M4U_T_RESET_MVA_RELEASE_TLB fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}

///> ------------------- helper function -----------------------------------------------------
M4U_STATUS_ENUM MTKM4UDrv::m4u_monitor_start(M4U_PORT_ID_ENUM PortID)
{

    M4U_MODULE_ID_ENUM eModuleID = m4u_get_module_by_port(PortID);

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_monitor_start failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }

    if(ioctl(mFileDescriptor, MTK_M4U_T_MONITOR_START, &PortID))
    {
        M4UMSG(" ioctl MTK_M4U_T_MONITOR_START fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_monitor_stop(M4U_PORT_ID_ENUM PortID)
{
    M4U_MODULE_ID_ENUM eModuleID = m4u_get_module_by_port(PortID);

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }


    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_monitor_stop failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }

    if(ioctl(mFileDescriptor, MTK_M4U_T_MONITOR_STOP, &PortID))
    {
        M4UMSG(" ioctl MTK_M4U_T_MONITOR_STOP fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_dump_reg(M4U_MODULE_ID_ENUM eModuleID)
{

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_dump_reg failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }

    //M4UMSG("before m4u_dump_reg, module=%d", eModuleID);
    if(ioctl(mFileDescriptor, MTK_M4U_T_DUMP_REG, &eModuleID))
    {
        M4UMSG(" ioctl MTK_M4U_T_DUMP_REG fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        //M4UMSG("after m4u_dump_reg, module=%d", eModuleID);
        return M4U_STATUS_OK;
    }
}

M4U_STATUS_ENUM MTKM4UDrv::m4u_dump_info(M4U_MODULE_ID_ENUM eModuleID)
{

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }


    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_dump_info failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }

    if(ioctl(mFileDescriptor, MTK_M4U_T_DUMP_INFO, &eModuleID))
    {
        M4UMSG(" ioctl MTK_M4U_T_DUMP_INFO fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }

}

M4U_STATUS_ENUM MTKM4UDrv::m4u_cache_sync(M4U_MODULE_ID_ENUM eModuleID,
	                                    M4U_CACHE_SYNC_ENUM eCacheSync,
		                                  unsigned int BufAddr, 
		                                  unsigned int BufSize)
{

    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }


    M4U_CACHE_STRUCT m4u_cache;
    
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_cache_sync failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
	        
    m4u_cache.eModuleID = eModuleID;
    m4u_cache.eCacheSync = eCacheSync;
    m4u_cache.BufAddr = BufAddr;
    m4u_cache.BufSize = BufSize;
    if(ioctl(mFileDescriptor, MTK_M4U_T_CACHE_SYNC, &m4u_cache))
    {
        M4UMSG(" ioctl MTK_M4U_T_CACHE_SYNC fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_dump_pagetable(M4U_MODULE_ID_ENUM eModuleID, 
								  const unsigned int BufAddr, 
								  const unsigned int BufSize, 
								  unsigned int MVAStart)
{


    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    M4U_MOUDLE_STRUCT m4u_module;
    m4u_module.eModuleID = eModuleID;
    m4u_module.BufAddr = BufAddr;
    m4u_module.BufSize = BufSize;
    m4u_module.MVAStart = MVAStart;
 	        
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_dump_pagetable failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_DUMP_PAGETABLE, &m4u_module))
    {
        M4UMSG(" ioctl MTK_M4U_T_DUMP_PAGETABLE fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_register_buffer(M4U_MODULE_ID_ENUM eModuleID, 
								  const unsigned int BufAddr, 
								  const unsigned int BufSize,
								  int security,
								  int cache_coherent,
								  unsigned int *pRetMVAAddr)
{


    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    M4U_MOUDLE_STRUCT m4u_module;
    m4u_module.eModuleID = eModuleID;
    m4u_module.BufAddr = BufAddr;
    m4u_module.BufSize = BufSize;
    m4u_module.security = security;
    m4u_module.cache_coherent = cache_coherent;
 	        
    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_register_buffer failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
    if(ioctl(mFileDescriptor, MTK_M4U_T_REGISTER_BUFFER, &m4u_module))
    {
        M4UMSG(" ioctl MTK_M4U_T_REGISTER_BUFFER fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        *pRetMVAAddr = 0;
        return M4U_STATUS_OK;
    }
}


M4U_STATUS_ENUM MTKM4UDrv::m4u_cache_flush_all(M4U_MODULE_ID_ENUM eModuleID)
{


    if(!m4u_check_m4u_en(eModuleID))
    {
        return M4U_STATUS_OK;
    }

    if(mFileDescriptor<0 )
    {
        M4UMSG("m4u_cache_sync failed \n");
        m4u_dump_backtrace();
        return M4U_STATUS_INVALID_HANDLE;
    }
	        
    if(ioctl(mFileDescriptor, MTK_M4U_T_CACHE_FLUSH_ALL, NULL))
    {
        M4UMSG(" ioctl MTK_M4U_T_CACHE_FLUSH_ALL fail! %d, %s \n", errno, strerror(errno));
        m4u_dump_backtrace();
        return M4U_STATUS_KERNEL_FAULT;
    }
    else
    {
        return M4U_STATUS_OK;
    }
}

#ifdef __PMEM_WRAP_LAYER_EN__

    bool MTKM4UDrv::mUseM4U[M4U_CLNTMOD_MAX] = {false};

    bool MTKM4UDrv::m4u_enable_m4u_func(M4U_MODULE_ID_ENUM eModuleID)
    {
        bool ret = mUseM4U[eModuleID];
        mUseM4U[eModuleID] = true;
        return ret;
    }
    
    bool MTKM4UDrv::m4u_disable_m4u_func(M4U_MODULE_ID_ENUM eModuleID)
    {
        bool ret = mUseM4U[eModuleID];
        mUseM4U[eModuleID] = false;
        return ret;
    }
    bool MTKM4UDrv::m4u_print_m4u_enable_status()
    {
        for(int i=0; i<M4U_CLNTMOD_MAX; i++)
        {
            M4UMSG("module(%d)\t:\t%d\n", i, mUseM4U[i]);
        }
        return true;
    }
    bool MTKM4UDrv::m4u_check_m4u_en(M4U_MODULE_ID_ENUM eModuleID)
    {
        if(!mUseM4U[eModuleID])
        {
            M4UMSG("warning: call m4u function but didnot enable m4u function. ID: %d\n", eModuleID);
        }
        return mUseM4U[eModuleID];
    }
#endif 




