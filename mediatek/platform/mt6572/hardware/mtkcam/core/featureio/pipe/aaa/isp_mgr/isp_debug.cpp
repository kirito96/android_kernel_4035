
#define LOG_TAG "isp_debug"

#ifndef ENABLE_MY_LOG
    #define ENABLE_MY_LOG       (1)
#endif

#include <cutils/properties.h>
#include <aaa_types.h>
#include <aaa_error_code.h>
#include <aaa_log.h>
#include <camera_custom_nvram.h>
#include <awb_feature.h>
#include <awb_param.h>
#include <ae_feature.h>
#include <ae_param.h>
#include <asm/arch/mt6589_sync_write.h> // For dsb() in isp_reg.h.
#include <isp_drv.h>
#include "isp_mgr.h"
#include <tdri_mgr.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
IspDebug&
IspDebug::
getInstance()
{
    static  IspDebug singleton;
    return  singleton;
}

IspDebug::
IspDebug()
    : m_pIspDrv(MNULL)
    , m_pIspReg(MNULL)
    , m_Users(0)
    , m_Lock()
    , m_bDebugEnable(MFALSE)
{
}

MBOOL
IspDebug::
init()
{
    char value[PROPERTY_VALUE_MAX] = {'\0'};
    property_get("debug.isp_debug.enable", value, "1");
    m_bDebugEnable = atoi(value);

	Mutex::Autolock lock(m_Lock);

	if (m_Users > 0)
	{
		MY_LOG("%d has created \n", m_Users);
		android_atomic_inc(&m_Users);
		return MTRUE;
	}

    m_pIspDrv = IspDrv::createInstance();

    if (!m_pIspDrv) {
        MY_ERR("IspDrv::createInstance() fail \n");
        return MFALSE;
    }

    m_pIspDrv->init();

    m_pIspReg = (isp_reg_t*)m_pIspDrv->getRegAddr();

	android_atomic_inc(&m_Users);

    return MTRUE;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MBOOL
IspDebug::
uninit()
{
	Mutex::Autolock lock(m_Lock);

	// If no more users, return directly and do nothing.
	if (m_Users <= 0)
	{
		return MTRUE;
	}

	// More than one user, so decrease one User.
	android_atomic_dec(&m_Users);

	if (m_Users == 0) // There is no more User after decrease one User
	{
        if (m_pIspDrv) {
            m_pIspDrv->uninit();
            }

        m_pIspReg = MNULL;
        m_pIspDrv = MNULL;
    }
	else	// There are still some users.
	{
		MY_LOG("Still %d users \n", m_Users);
	}

    return MTRUE;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MBOOL
IspDebug::
dumpIspDebugMessage()
{
    MY_LOG_IF("%s()\n", __FUNCTION__);

    if (!m_pIspReg) {
        MY_ERR("m_pIspReg is NULL\n");
        return MFALSE;
    }

    // DMA status check
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_DMA_INT, AAO_DONE_ST) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_DMA_INT, AAO_DONE_ST));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, AA_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, AA_EN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_DMA_EN, AAO_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_DMA_EN, AAO_EN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_MUX_SEL, AA_SEL_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_MUX_SEL, AA_SEL_EN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_MUX_SEL, AA_SEL) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_MUX_SEL, AA_SEL));

    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AAO_OFST_ADDR, OFFSET_ADDR) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AAO_OFST_ADDR, OFFSET_ADDR));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AAO_XSIZE, XSIZE) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AAO_XSIZE, XSIZE));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AAO_YSIZE, YSIZE) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AAO_YSIZE, YSIZE));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AAO_STRIDE, BUS_SIZE) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AAO_STRIDE, BUS_SIZE));

    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST0_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST0_EN));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST1_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST1_EN));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST2_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST2_EN));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST3_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AE_HST_CTL, AE_HST3_EN));

#if 0
    // CAM_AWB_WIN_ORG
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_ORG, AWB_WIN_ORIGIN_X) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_ORG, AWB_W_ORIGIN_X));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_ORG, AWB_WIN_ORIGIN_Y) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_ORG, AWB_W_ORIGIN_Y));

    // CAM_AWB_WIN_SIZE
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_SIZE, AWB_WIN_SIZE_X) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_SIZE, AWB_W_SIZE_X));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_SIZE, AWB_WIN_SIZE_Y) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_SIZE, AWB_W_SIZE_Y));

    // CAM_AWB_WIN_PITCH
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_PITCH, AWB_WIN_PITCH_X) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_PITCH, AWB_W_PITCH_X));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_PITCH, AWB_WIN_PITCH_Y) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_PITCH, AWB_W_PITCH_Y));

    // CAM_AWB_WIN_NUM
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_NUM, AWB_WIN_NUM_X) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_NUM, AWB_W_NUM_X));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_NUM, AWB_WIN_NUM_Y) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_WIN_NUM, AWB_W_NUM_Y));

    // CAM_AWB_RAWPREGAIN1_0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWPREGAIN1_0, RAWPREGAIN1_R) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWPREGAIN1_0, AWB_GAIN1_R));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWPREGAIN1_0, RAWPREGAIN1_G) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWPREGAIN1_0, AWB_GAIN1_G));

    // CAM_AWB_RAWPREGAIN1_1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWPREGAIN1_1, RAWPREGAIN1_B) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWPREGAIN1_1, AWB_GAIN1_B));

    // CAM_AWB_RAWLIMIT1_0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWLIMIT1_0, RAWLIMIT1_R) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWLIMIT1_0, AWB_LIMIT1_R));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWLIMIT1_0, RAWLIMIT1_G) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWLIMIT1_0, AWB_LIMIT1_G));

    // CAM_AWB_RAWLIMIT1_1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWLIMIT1_1, RAWLIMIT1_B) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_RAWLIMIT1_1, AWB_LIMIT1_B));

    // CAM_AWB_LOW_THR
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_LOW_THR, AWB_LOW_THR0) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_LOW_THR, AWB_LOW_THR0));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_LOW_THR, AWB_LOW_THR1) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_LOW_THR, AWB_LOW_THR1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_LOW_THR, AWB_LOW_THR2) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_LOW_THR, AWB_LOW_THR2));

    // CAM_AWB_HI_THR
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_HI_THR, AWB_HI_THR0) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_HI_THR, AWB_HI_THR0));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_HI_THR, AWB_HI_THR1) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_HI_THR, AWB_HI_THR1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_HI_THR, AWB_HI_THR2) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_HI_THR, AWB_HI_THR2));

    // CAM_AWB_PIXEL_CNT0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_PIXEL_CNT0, PIXEL_CNT0) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_PIXEL_CNT0, PIXEL_CNT0));

    // CAM_AWB_PIXEL_CNT1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_PIXEL_CNT1, PIXEL_CNT1) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_PIXEL_CNT1, PIXEL_CNT1));

    // CAM_AWB_PIXEL_CNT2
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_PIXEL_CNT2, PIXEL_CNT2) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_PIXEL_CNT2, PIXEL_CNT2));

    // CAM_AWB_ERR_THR
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_ERR_THR, AWB_ERR_THR) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_ERR_THR, AWB_ERR_THR));

    // CAM_AWB_ROT
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_ROT, AWB_COS) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_ROT, AWB_C));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_ROT, AWB_SIN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_ROT, AWB_S));

    // CAM_AWB_L0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_X, AWB_L0_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_X, AWB_L0_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_X, AWB_L0_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_X, AWB_L0_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_Y, AWB_L0_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_Y, AWB_L0_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_Y, AWB_L0_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L0_Y, AWB_L0_Y_UP));

    // CAM_AWB_L1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_X, AWB_L1_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_X, AWB_L1_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_X, AWB_L1_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_X, AWB_L1_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_Y, AWB_L1_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_Y, AWB_L1_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_Y, AWB_L1_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L1_Y, AWB_L1_Y_UP));

    // CAM_AWB_L2
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_X, AWB_L2_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_X, AWB_L2_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_X, AWB_L2_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_X, AWB_L2_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_Y, AWB_L2_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_Y, AWB_L2_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_Y, AWB_L2_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L2_Y, AWB_L2_Y_UP));

    // CAM_AWB_L3
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_X, AWB_L3_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_X, AWB_L3_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_X, AWB_L3_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_X, AWB_L3_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_Y, AWB_L3_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_Y, AWB_L3_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_Y, AWB_L3_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L3_Y, AWB_L3_Y_UP));

    // CAM_AWB_L4
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_X, AWB_L4_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_X, AWB_L4_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_X, AWB_L4_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_X, AWB_L4_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_Y, AWB_L4_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_Y, AWB_L4_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_Y, AWB_L4_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L4_Y, AWB_L4_Y_UP));

    // CAM_AWB_L5
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_X, AWB_L5_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_X, AWB_L5_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_X, AWB_L5_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_X, AWB_L5_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_Y, AWB_L5_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_Y, AWB_L5_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_Y, AWB_L5_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L5_Y, AWB_L5_Y_UP));

    // CAM_AWB_L6
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_X, AWB_L6_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_X, AWB_L6_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_X, AWB_L6_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_X, AWB_L6_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_Y, AWB_L6_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_Y, AWB_L6_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_Y, AWB_L6_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L6_Y, AWB_L6_Y_UP));

    // CAM_AWB_L7
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_X, AWB_L7_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_X, AWB_L7_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_X, AWB_L7_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_X, AWB_L7_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_Y, AWB_L7_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_Y, AWB_L7_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_Y, AWB_L7_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L7_Y, AWB_L7_Y_UP));

    // CAM_AWB_L8
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_X, AWB_L8_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_X, AWB_L8_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_X, AWB_L8_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_X, AWB_L8_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_Y, AWB_L8_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_Y, AWB_L8_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_Y, AWB_L8_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L8_Y, AWB_L8_Y_UP));

    // CAM_AWB_L9
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_X, AWB_L9_X_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_X, AWB_L9_X_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_X, AWB_L9_X_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_X, AWB_L9_X_UP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_Y, AWB_L9_Y_LOW) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_Y, AWB_L9_Y_LOW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_Y, AWB_L9_Y_UP) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AWB_L9_Y, AWB_L9_Y_UP));

    // CAM_AE_RAWPREGAIN2_0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AE_RAWPREGAIN2_0, RAWPREGAIN2_R) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AE_RAWPREGAIN2_0, AE_GAIN2_R));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AE_RAWPREGAIN2_0, RAWPREGAIN2_G) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AE_RAWPREGAIN2_0, AE_GAIN2_G));

    // CAM_AE_RAWPREGAIN2_1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_AE_RAWPREGAIN2_1, RAWPREGAIN2_B) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_AE_RAWPREGAIN2_1, AE_GAIN2_B));
#endif


    // OBC
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, OB_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, OB_EN));
    #if 1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST0, OBOFFSET0) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST0, OBOFFSET0));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST1, OBOFFSET1) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST1, OBOFFSET1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST2, OBOFFSET2) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST2, OBOFFSET2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST3, OBOFFSET3) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_OFFST3, OBOFFSET3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN0, OBGAIN0) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN0, OBGAIN0));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN1, OBGAIN1) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN1, OBGAIN1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN2, OBGAIN2) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN2, OBGAIN2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN3, OBGAIN3) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_OBC_GAIN3, OBGAIN3));
    #endif

    // BPC/NR1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, BNR_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, BNR_EN));
    #if 0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CON) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CON));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_4) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_4));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_5) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_5));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_6) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD1_6));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD2_1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD2_1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD2_2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD2_2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD2_3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD2_3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_CD0) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_CD0));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_DET) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_DET));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_COR) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_COR));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_TBLI1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_TBLI1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_BPC_TBLI2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_BPC_TBLI2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR1_CON) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR1_CON));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR1_CT_CON) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR1_CT_CON));
    #endif

    // LSC
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, LSC_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, LSC_EN));

    // PGN
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, PGN_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, PGN_EN));
    #if 1
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU01, PGN_CH0_SATU) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU01, PGN_CH0_SATU));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU01, PGN_CH1_SATU) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU01, PGN_CH1_SATU));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU23, PGN_CH2_SATU) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU23, PGN_CH2_SATU));
    //MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU23, PGN_CH3_SATU) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_SATU23, PGN_CH3_SATU));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN01, PGN_CH0_GAIN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN01, PGN_CH0_GAIN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN01, PGN_CH1_GAIN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN01, PGN_CH1_GAIN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN23, PGN_CH2_GAIN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN23, PGN_CH2_GAIN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN23, PGN_CH3_GAIN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_GAIN23, PGN_CH3_GAIN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS01, PGN_CH0_OFFS) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS01, PGN_CH0_OFFS));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS01, PGN_CH1_OFFS) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS01, PGN_CH1_OFFS));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS23, PGN_CH2_OFFS) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS23, PGN_CH2_OFFS));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS23, PGN_CH3_OFFS) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_PGN_OFFS23, PGN_CH3_OFFS));
    #endif

    // CFA
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, CFA_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, CFA_EN));
    #if 0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_BYPASS) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_BYPASS));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_ED_F) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_ED_F));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_ED_NYQ) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_ED_NYQ));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_ED_STEP) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_ED_STEP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_RGB_HF) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_RGB_HF));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_BW) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_BW));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F1_ACT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F1_ACT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F2_ACT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F2_ACT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F3_ACT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F3_ACT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F4_ACT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F4_ACT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F1_L) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F1_L));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F2_L) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F2_L));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F3_L) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F3_L));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_F4_L) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_F4_L));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_HF_RB) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_HF_RB));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_HF_GAIN) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_HF_GAIN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_HF_COMP) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_HF_COMP));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_HF_CORING_TH) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_HF_CORING_TH));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_ACT_LUT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_ACT_LUT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_SPARE) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_SPARE));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CFA_BB) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CFA_BB));
    #endif

    // CCM
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, G2G_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, G2G_EN));
    #if 1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV0A, G2G_CNV_00) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV0A, G2G_CNV_00));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV0A, G2G_CNV_01) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV0A, G2G_CNV_01));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV0B, G2G_CNV_02) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV0B, G2G_CNV_02));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV1A, G2G_CNV_10) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV1A, G2G_CNV_10));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV1A, G2G_CNV_11) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV1A, G2G_CNV_11));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV1B, G2G_CNV_12) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV1B, G2G_CNV_12));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV2A, G2G_CNV_20) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV2A, G2G_CNV_20));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV2A, G2G_CNV_21) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV2A, G2G_CNV_21));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV2B, G2G_CNV_22) = 0x%8x\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_CONV2B, G2G_CNV_22));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_G2G_ACC, G2G_ACC) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_G2G_ACC, G2G_ACC));
    #endif

    // GGM
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, GGM_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN1, GGM_EN));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_GGM_CTRL, GAMMA_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_GGM_CTRL, GAMMA_EN));
    #if 0
    //for (MINT32 i = 0; i < 144; i++) {
        //MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_GGM_RB_GMT[%d]) = 0x%8x\n", i, ISP_READ_REG(m_pIspReg , CAM_GGM_RB_GMT[i]));
        //MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_GGM_G_GMT[%d]) = 0x%8x\n", i, ISP_READ_REG(m_pIspReg , CAM_GGM_G_GMT[i]));
    //}
    #endif

    // G2C
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, G2C_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, G2C_EN));
    #if 1
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_0A) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_0A));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_0B) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_0B));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_1A) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_1A));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_1B) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_1B));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_2A) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_2A));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_2B) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_G2C_CONV_2B));
    #endif

    // NBC
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, NBC_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, NBC_EN));
    #if 0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_CON1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_CON1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_CON2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_CON2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_CON3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_CON3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_YAD1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_YAD1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_YAD2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_YAD2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_4LUT1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_4LUT1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_4LUT2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_4LUT2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_4LUT3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_4LUT3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_PTY) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_PTY));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_CAD) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_CAD));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_PTC) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_PTC));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_LCE1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_LCE1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_LCE2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_LCE2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_HP1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_HP1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_HP2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_HP2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_HP3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_HP3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_ACTY) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_ACTY));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_ANR_ACTC) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_ANR_ACTC));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CCR_CON) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CCR_CON));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CCR_YLUT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CCR_YLUT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CCR_UVLUT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CCR_UVLUT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_CCR_YLUT2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_CCR_YLUT2));
    #endif

    // PCA
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, PCA_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, PCA_EN));
    #if 0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_PCA_CON1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_PCA_CON1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_PCA_CON2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_PCA_CON2));

    for (MINT32 i = 0; i < 179; i++) {
        MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_PCA_TBL[%d]) = 0x%8x\n", i, ISP_READ_REG(m_pIspReg , CAM_PCA_TBL[i]));
    }
#endif

    // SEEE
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, SEEE_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, SEEE_EN));
    #if 0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_SRK_CTRL) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_SRK_CTRL));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_CLIP_CTRL) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_CLIP_CTRL));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_HP_CTRL1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_HP_CTRL1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_HP_CTRL2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_HP_CTRL2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL4) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL4));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL5) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL5));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL6) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL6));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL7) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_ED_CTRL7));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_Y_CTRL) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_Y_CTRL));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EDGE_CTRL3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_SPECIAL_CTRL) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_SPECIAL_CTRL));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_CORE_CTRL1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_CORE_CTRL1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_CORE_CTRL2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_CORE_CTRL2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK4) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK4));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK5) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_SEEE_EE_LINK5));
    #endif

    // NR3D
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, NR3D_EN) = %d\n", ISP_READ_BITS(m_pIspReg , CAM_CTL_EN2, NR3D_EN));
    #if 0
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_BLEND) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_BLEND));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_SKIP_KEY) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_SKIP_KEY));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_FBCNT_OFF) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_FBCNT_OFF));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_FBCNT_SIZ) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_FBCNT_SIZ));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_FB_COUNT) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_FB_COUNT));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_CPX) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_CPX));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_Y_CON1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_Y_CON1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_Y_CON2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_Y_CON2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_Y_CON3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_Y_CON3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_U_CON1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_U_CON1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_U_CON2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_U_CON2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_U_CON3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_U_CON3));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_V_CON1) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_V_CON1));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_V_CON2) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_V_CON2));
    MY_LOG_IF(m_bDebugEnable,"ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_V_CON3) = 0x%8x\n", ISP_READ_REG(m_pIspReg , CAM_NR3D_LIMIT_V_CON3));
    #endif

    return MTRUE;
}
  
