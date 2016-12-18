
#ifndef __DISP_DRV_LOG_H__
#define __DISP_DRV_LOG_H__

#include <platform/mt_typedefs.h>

// global debug macro for display
//#define DISP_DRV_DBG


extern unsigned int disp_log_on;
extern unsigned int dbi_log_on;

#define DISP_LOG_PRINT(level, sub_module, fmt, arg...)      \
    do {                                                    \
        printk("DISP/"fmt, ##arg);  \
    }while(0)
    
#define LOG_PRINT(level, module, fmt, arg...)               \
    do {                                                    \
        printk(fmt, ##arg);             \
    }while(0)

#define DISP_LOG(fmt, arg...) \
    do { \
        if (disp_log_on) DISP_LOG_PRINT(ANDROID_LOG_WARN, "COMMON", fmt, ##arg); \
    }while (0)

#define DISP_FUNC()	\
    do { \
        if(disp_log_on) DISP_LOG_PRINT(ANDROID_LOG_INFO, "COMMON", "[Func]%s\n", __func__); \
    }while (0)

#define DBI_LOG(fmt, arg...) \
    do { \
        if (dbi_log_on) DISP_LOG_PRINT(ANDROID_LOG_WARN, "LCD", fmt, ##arg);    \
    }while (0)

#define DBI_FUNC()	\
    do { \
        if(dbi_log_on) DISP_LOG_PRINT(ANDROID_LOG_INFO, "LCD", "[Func]%s\n", __func__);  \
    }while (0)
        

#endif // __DISP_DRV_LOG_H__
