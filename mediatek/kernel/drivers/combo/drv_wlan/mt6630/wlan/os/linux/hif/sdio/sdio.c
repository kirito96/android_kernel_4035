





#include "gl_os.h"

#if MTK_WCN_HIF_SDIO
#include "hif_sdio.h"
#else
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h> /* sdio_readl(), etc */
#include <linux/mmc/sdio_ids.h>
#endif

#include <linux/mm.h>
#ifndef CONFIG_X86
#include <asm/memory.h>
#endif

#if defined(MT6630)
    #include "mt6630_reg.h"
#endif

#if CFG_DBG_GPIO_PINS/* FIXME: move to platform or custom header */
#include <mach/mt6516_gpio.h>
#endif


#define HIF_SDIO_ERR_TITLE_STR              "["CHIP_NAME"] SDIO Access Error!"
#define HIF_SDIO_ERR_DESC_STR               "**SDIO Access Error**\n"

#if MTK_WCN_HIF_SDIO



static INT32
mtk_sdio_probe(MTK_WCN_HIF_SDIO_CLTCTX, const MTK_WCN_HIF_SDIO_FUNCINFO *);

static INT32
mtk_sdio_remove(MTK_WCN_HIF_SDIO_CLTCTX);
static INT32 mtk_sdio_interrupt(MTK_WCN_HIF_SDIO_CLTCTX);


static MTK_WCN_HIF_SDIO_FUNCINFO funcInfo[] = {
#if defined(MT6630)
    { MTK_WCN_HIF_SDIO_FUNC(0x037a, 0x6630, 0x1, 512) },
#endif
};


static MTK_WCN_HIF_SDIO_CLTINFO cltInfo = {
    .func_tbl = funcInfo,
    .func_tbl_size = sizeof(funcInfo)/sizeof(MTK_WCN_HIF_SDIO_FUNCINFO),
    .hif_clt_probe = mtk_sdio_probe,
    .hif_clt_remove = mtk_sdio_remove,
    .hif_clt_irq = mtk_sdio_interrupt,
};

#else

static const struct sdio_device_id mtk_sdio_ids[] = {
#if defined(MT6630)
	{ SDIO_DEVICE(0x037a, 0x6630) }, /* Not an SDIO standard class device */
#endif
	{ /* end: all zeroes */ },
};

MODULE_DEVICE_TABLE(sdio, mtk_sdio_ids);

#endif




static probe_card pfWlanProbe = NULL;
static remove_card pfWlanRemove = NULL;


#if (MTK_WCN_HIF_SDIO == 0)
static struct sdio_driver mtk_sdio_driver = {
	.name		= "wlan", /* "MTK SDIO WLAN Driver" */
	.id_table	= mtk_sdio_ids,
	.probe		= NULL,
	.remove		= NULL,
};
#endif


#if CFG_DBG_GPIO_PINS

/* debug pins */
UINT_32 dbgPinSTP[] = {
    GPIO_PLATFORM(33)/* CMFLASH, IDX_ERR J613 */
    , GPIO_PLATFORM(62)/* EINT3, IDX_TX_THREAD */
    , GPIO_PLATFORM(80)/* SPI_CS_N, IDX_TX_REQ J613 */
    , GPIO_PLATFORM(81)/* SPI_SCK, IDX_TX_PORT_WRITE J613 */
    , GPIO_PLATFORM(17) /* CMRST, IDX_STP_MTX_BT J618 */
    , GPIO_PLATFORM(18) /* CMPDN, IDX_STP_MTX_FM J613 */
    , GPIO_PLATFORM(19) /* CMVREF,IDX_STP_MTX_GPS J613 */
    , GPIO_INVALID /* REMOVED, IDX_STP_MTX_WIFI */
    , GPIO_INVALID /* REMOVED, IDX_STP_MTX_WMT */
    , GPIO_PLATFORM(135) /* SCL2, IDX_LOOP_CNT  J616 */
    , GPIO_PLATFORM(136) /* SDA2, IDX_NO_BUF J616 */
    , GPIO_PLATFORM(30) /* CAM_MECHSH0, IDX_BT_TX, J613 low-active */
    , GPIO_PLATFORM(31) /* CAM_MECHSH1, IDX_BT_RX, J613 low-active */
    , GPIO_PLATFORM(124) /* GPS_PWR_EN, ThreadDSPIn [GPS] */
    , GPIO_PLATFORM(125) /* GPS_SYNC, mtk_sys_msg_recv [GPS] */
    , GPIO_PLATFORM(21) /* GPS_EINT8, dump_nmea_data [GPS] */
    , GPIO_PLATFORM(29) /* CAM_STROBE, IDX_GPS_TX, J613 low-active */
    , GPIO_PLATFORM(20) /*CMHREF, J613 UNUSED */
//    , GPIO_6516(64) /* EINT5, REMOVED!!! for MT6620-Wi-Fi Int */
//    , GPIO_6516(122) /* BT_PWR_EN, REMOVED!!! for MT6620-PMU_EN */
//    , GPIO_6516(123) /* BT_RESET, REMOVED!!! for MT6620-RST */
};
#endif


#if CFG_DBG_GPIO_PINS
void debug_gpio_init(void)
{
    int i;

    for (i = 0; i < sizeof(dbgPinSTP)/sizeof(dbgPinSTP[0]); ++i) {
        if (GPIO_INVALID == dbgPinSTP[i]) {
            continue;
        }
        //printk(KERN_INFO "[%s] %ld \n", __FUNCTION__, dbgPinSTP[i]);
        mt_set_gpio_pull_enable(dbgPinSTP[i], 0); /* disable pull */
        mt_set_gpio_dir(dbgPinSTP[i], GPIO_DIR_OUT); /* set output */
        mt_set_gpio_mode(dbgPinSTP[i], GPIO_MODE_00); /* set gpio mode */

        /* toggle twice to check if ok: */
        mt_set_gpio_out(dbgPinSTP[i], GPIO_OUT_ZERO); /* tie low */
        mt_set_gpio_out(dbgPinSTP[i], GPIO_OUT_ONE); /* tie high*/
        mt_set_gpio_out(dbgPinSTP[i], GPIO_OUT_ZERO); /* tie low */
        mt_set_gpio_out(dbgPinSTP[i], GPIO_OUT_ONE); /* tie high*/
    }
    //printk(KERN_INFO "[%s] initialization ok \n", __FUNCTION__);
}

void debug_gpio_deinit(void)
{
    int i;
    for (i = 0; i < sizeof(dbgPinSTP)/sizeof(dbgPinSTP[0]); ++i) {
        if (GPIO_INVALID == dbgPinSTP[i]) {
            continue;
        }
        //printk(KERN_INFO "[%s] %ld \n", __FUNCTION__, dbgPinSTP[i]);
        mt_set_gpio_dir(dbgPinSTP[i], GPIO_DIR_IN);
    }

    //printk(KERN_INFO "[%s] k\n", __FUNCTION__);
}

void mtk_wcn_stp_debug_gpio_assert(UINT_32 dwIndex, UINT_32 dwMethod)
{
    unsigned int i;

    if (dwIndex >= (sizeof(dbgPinSTP)/sizeof(dbgPinSTP[0]))) {
        //printk(KERN_INFO "[%s] invalid dwIndex(%ld) \n", __FUNCTION__, dwIndex);
        return;
    }

    if (dwIndex > IDX_STP_MAX) {
        //printk(KERN_INFO "[%s] dwIndex(%ld) > IDX_STP_MAX(%d) \n", __FUNCTION__, dwIndex, IDX_STP_MAX);
    }

    if (GPIO_INVALID == dbgPinSTP[dwIndex]) {
        return;
    }

    if (dwMethod & DBG_TIE_DIR) {
        if (dwMethod & DBG_HIGH) {
            mt_set_gpio_out(dbgPinSTP[dwIndex], GPIO_OUT_ONE);
        }
        else {
            mt_set_gpio_out(dbgPinSTP[dwIndex], GPIO_OUT_ZERO);
        }
        return;
    }

    if (dwMethod & DBG_TOGGLE(0)) {
        for (i = 0; i < DBG_TOGGLE_NUM(dwMethod); ++i) {
            mt_set_gpio_out(dbgPinSTP[dwIndex], GPIO_OUT_ZERO);
            mt_set_gpio_out(dbgPinSTP[dwIndex], GPIO_OUT_ONE);
        }
        return;
    }

    return;
}
#endif

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#if MTK_WCN_HIF_SDIO

static INT32 mtk_sdio_interrupt(MTK_WCN_HIF_SDIO_CLTCTX cltCtx)
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    INT32 ret = 0;

    prGlueInfo = mtk_wcn_hif_sdio_get_drvdata(cltCtx);

    //ASSERT(prGlueInfo);

    if (!prGlueInfo) {
        //printk(KERN_INFO DRV_NAME"No glue info in mtk_sdio_interrupt()\n");
        return (-HIF_SDIO_ERR_FAIL);
    }

    if (prGlueInfo->u4Flag & GLUE_FLAG_HALT) {
        //printk(KERN_INFO DRV_NAME"GLUE_FLAG_HALT skip INT\n");
        ret = mtk_wcn_hif_sdio_writel(cltCtx, MCR_WHLPCR, WHLPCR_INT_EN_CLR);
        return ret;
    }

    ret = mtk_wcn_hif_sdio_writel(cltCtx, MCR_WHLPCR, WHLPCR_INT_EN_CLR);

    set_bit (GLUE_FLAG_INT_BIT, &prGlueInfo->u4Flag);

    /* when we got sdio interrupt, we wake up the tx servie thread*/
#if CFG_SUPPORT_MULTITHREAD    
    wake_up_interruptible(&prGlueInfo->waitq_hif);
#else     
    wake_up_interruptible(&prGlueInfo->waitq);
#endif

    return ret;
}

#else

static unsigned int in_interrupt = 0;

static void mtk_sdio_interrupt(struct sdio_func *func)
{
    P_GLUE_INFO_T prGlueInfo = NULL;

    int ret = 0;

    prGlueInfo = sdio_get_drvdata(func);
    //ASSERT(prGlueInfo);

    if (!prGlueInfo) {
        //printk(KERN_INFO DRV_NAME"No glue info in mtk_sdio_interrupt()\n");
        return;
    }

    if (prGlueInfo->u4Flag & GLUE_FLAG_HALT) {
        sdio_writel(prGlueInfo->rHifInfo.func, WHLPCR_INT_EN_CLR, MCR_WHLPCR, &ret);
        //printk(KERN_INFO DRV_NAME"GLUE_FLAG_HALT skip INT\n");
        return;
    }

    sdio_writel(prGlueInfo->rHifInfo.func, WHLPCR_INT_EN_CLR, MCR_WHLPCR, &ret);

    #if 0
    wlanISR(prGlueInfo->prAdapter, TRUE);

    if (prGlueInfo->u4Flag & GLUE_FLAG_HALT) {
        /* Should stop now... skip pending interrupt */
        //printk(KERN_INFO DRV_NAME"ignore pending interrupt\n");
    }
    else {
        wlanIST(prGlueInfo->prAdapter);
    }
    #endif

    set_bit (GLUE_FLAG_INT_BIT, &prGlueInfo->u4Flag);

    /* when we got sdio interrupt, we wake up the tx servie thread*/
#if CFG_SUPPORT_MULTITHREAD    
    wake_up_interruptible(&prGlueInfo->waitq_hif);
#else     
    wake_up_interruptible(&prGlueInfo->waitq);
#endif
}
#endif

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#if MTK_WCN_HIF_SDIO

// FIXME: global variable
static const MTK_WCN_HIF_SDIO_FUNCINFO *prFunc;


static INT32
mtk_sdio_probe(MTK_WCN_HIF_SDIO_CLTCTX cltCtx, const MTK_WCN_HIF_SDIO_FUNCINFO *prFuncInfo)
{
    INT32 ret = HIF_SDIO_ERR_SUCCESS;

    prFunc = prFuncInfo;

    if (pfWlanProbe((PVOID) &cltCtx) != WLAN_STATUS_SUCCESS) {
        //printk(KERN_WARNING DRV_NAME"pfWlanProbe fail!call pfWlanRemove()\n");
        pfWlanRemove();
        ret = -(HIF_SDIO_ERR_FAIL);
    } else {
        //printk(KERN_INFO DRV_NAME"mtk_wifi_sdio_probe() done(%d)\n", ret);
    }
    return ret;
}
#else
static int mtk_sdio_probe (
    struct sdio_func *func,
    const struct sdio_device_id *id
    )
{
    int ret = 0;
    int i = 0;

    //printk(KERN_INFO DRV_NAME "mtk_sdio_probe()\n");

    ASSERT(func);
    ASSERT(id);

    //printk(KERN_INFO DRV_NAME "Basic struct size checking...\n");
    //printk(KERN_INFO DRV_NAME "sizeof(struct device) = %d\n", sizeof(struct device));
    //printk(KERN_INFO DRV_NAME "sizeof(struct mmc_host) = %d\n", sizeof(struct mmc_host));
    //printk(KERN_INFO DRV_NAME "sizeof(struct mmc_card) = %d\n", sizeof(struct mmc_card));
    //printk(KERN_INFO DRV_NAME "sizeof(struct mmc_driver) = %d\n", sizeof(struct mmc_driver));
    //printk(KERN_INFO DRV_NAME "sizeof(struct mmc_data) = %d\n", sizeof(struct mmc_data));
    //printk(KERN_INFO DRV_NAME "sizeof(struct mmc_command) = %d\n", sizeof(struct mmc_command));
    //printk(KERN_INFO DRV_NAME "sizeof(struct mmc_request) = %d\n", sizeof(struct mmc_request));
    //printk(KERN_INFO DRV_NAME "sizeof(struct sdio_func) = %d\n", sizeof(struct sdio_func));

    //printk(KERN_INFO DRV_NAME "Card information checking...\n");
    //printk(KERN_INFO DRV_NAME "func = 0x%p\n", func);
    //printk(KERN_INFO DRV_NAME "Number of info = %d:\n", func->card->num_info);

    for (i = 0; i < func->card->num_info; i++) {
        //printk(KERN_INFO DRV_NAME "info[%d]: %s\n", i, func->card->info[i]);
    }

    sdio_claim_host(func);
    ret = sdio_enable_func(func);
    sdio_release_host(func);

    if (ret) {
        //printk(KERN_INFO DRV_NAME"sdio_enable_func failed!\n");
        goto out;
    }
    //printk(KERN_INFO DRV_NAME"sdio_enable_func done!\n");

    if (pfWlanProbe((PVOID)func) != WLAN_STATUS_SUCCESS) {
        //printk(KERN_WARNING DRV_NAME"pfWlanProbe fail!call pfWlanRemove()\n");
        pfWlanRemove();
        ret = -1;
    }
    else {
#if CFG_DBG_GPIO_PINS
    //printk(KERN_INFO "[%s] init debug gpio, 20100815 \n", __FUNCTION__);
    /* Debug pins initialization */
    debug_gpio_init();
#endif
    }

out:
    //printk(KERN_INFO DRV_NAME"mtk_sdio_probe() done(%d)\n", ret);
    return ret;
}
#endif


#if MTK_WCN_HIF_SDIO
static INT32
mtk_sdio_remove(MTK_WCN_HIF_SDIO_CLTCTX cltCtx)
{
    INT32 ret = HIF_SDIO_ERR_SUCCESS;
    //printk(KERN_INFO DRV_NAME"pfWlanRemove done\n");
    pfWlanRemove();

    return ret;
}
#else
static void
mtk_sdio_remove (
    struct sdio_func *func
    )
{
    //printk(KERN_INFO DRV_NAME"mtk_sdio_remove()\n");

#if CFG_DBG_GPIO_PINS
    //printk(KERN_INFO "[%s] deinit debug gpio \n", __FUNCTION__);
    debug_gpio_deinit();
#endif

    ASSERT(func);
    //printk(KERN_INFO DRV_NAME"pfWlanRemove done\n");
    pfWlanRemove();

    sdio_claim_host(func);
    sdio_disable_func(func);
    //printk(KERN_INFO DRV_NAME"sdio_disable_func() done\n");
    sdio_release_host(func);

    //printk(KERN_INFO DRV_NAME"mtk_sdio_remove() done\n");
}
#endif

#if (MTK_WCN_HIF_SDIO == 0)
static int
mtk_sdio_suspend (
    struct device * pDev,
    pm_message_t state
    )
{
    //printk(KERN_INFO "mtk_sdio: mtk_sdio_suspend dev(0x%p)\n", pDev);
    //printk(KERN_INFO "mtk_sdio: MediaTek SDIO WLAN driver\n");

    return 0;
}

int mtk_sdio_resume (
    struct device * pDev
    )
{
    //printk(KERN_INFO "mtk_sdio: mtk_sdio_resume dev(0x%p)\n", pDev);

    return 0;
}
#endif


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
WLAN_STATUS
glRegisterBus (
    probe_card pfProbe,
    remove_card pfRemove
    )
{
    int ret = 0;

    ASSERT(pfProbe);
    ASSERT(pfRemove);

    //printk(KERN_INFO "mtk_sdio: MediaTek SDIO WLAN driver\n");
    //printk(KERN_INFO "mtk_sdio: Copyright MediaTek Inc.\n");

    pfWlanProbe = pfProbe;
    pfWlanRemove = pfRemove;

#if MTK_WCN_HIF_SDIO
    /* register MTK sdio client */
    ret = ((mtk_wcn_hif_sdio_client_reg(&cltInfo) == HIF_SDIO_ERR_SUCCESS) ? WLAN_STATUS_SUCCESS : WLAN_STATUS_FAILURE);
#else
    mtk_sdio_driver.probe = mtk_sdio_probe;
    mtk_sdio_driver.remove = mtk_sdio_remove;

    mtk_sdio_driver.drv.suspend = mtk_sdio_suspend;
    mtk_sdio_driver.drv.resume = mtk_sdio_resume;

    ret = (sdio_register_driver(&mtk_sdio_driver) == 0) ? WLAN_STATUS_SUCCESS : WLAN_STATUS_FAILURE;
#endif

    return ret;
} /* end of glRegisterBus() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glUnregisterBus(
    remove_card pfRemove
    )
{
    ASSERT(pfRemove);
    pfRemove();

#if MTK_WCN_HIF_SDIO
    /* unregister MTK sdio client */
    mtk_wcn_hif_sdio_client_unreg(&cltInfo);
#else
    sdio_unregister_driver(&mtk_sdio_driver);
#endif

    return;
} /* end of glUnregisterBus() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glSetHifInfo (
    P_GLUE_INFO_T prGlueInfo,
    UINT_32 u4Cookie
    )
{
    P_GL_HIF_INFO_T prHif = NULL;

    prHif = &prGlueInfo->rHifInfo;

#if MTK_WCN_HIF_SDIO
    //prHif->prFuncInfo = ((MTK_WCN_HIF_SDIO_FUNCINFO *) u4Cookie);
    prHif->prFuncInfo = prFunc;
    prHif->cltCtx = *((MTK_WCN_HIF_SDIO_CLTCTX *) u4Cookie);
    mtk_wcn_hif_sdio_set_drvdata(prHif->cltCtx, prGlueInfo);

#else
    prHif->func = (struct sdio_func *) u4Cookie;

    //printk(KERN_INFO DRV_NAME"prHif->func->dev = 0x%p\n", &prHif->func->dev);
    //printk(KERN_INFO DRV_NAME"prHif->func->vendor = 0x%04X\n", prHif->func->vendor);
    //printk(KERN_INFO DRV_NAME"prHif->func->device = 0x%04X\n", prHif->func->device);
    //printk(KERN_INFO DRV_NAME"prHif->func->func = 0x%04X\n", prHif->func->num);

    sdio_set_drvdata(prHif->func, prGlueInfo);

    SET_NETDEV_DEV(prGlueInfo->prDevHandler, &prHif->func->dev);
#endif

    return;
} /* end of glSetHifInfo() */

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glClearHifInfo (
    P_GLUE_INFO_T prGlueInfo
    )
{
    //P_GL_HIF_INFO_T prHif = NULL;
    //ASSERT(prGlueInfo);
    //prHif = &prGlueInfo->rHifInfo;

    return;
} /* end of glClearHifInfo() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOL
glBusInit (
    PVOID pvData
    )
{
#if (MTK_WCN_HIF_SDIO == 0)
    int ret = 0;
    struct sdio_func *func = NULL;

    ASSERT(pvData);

    func = (struct sdio_func *) pvData;

    sdio_claim_host(func);
    ret = sdio_set_block_size(func, 512);
    sdio_release_host(func);

    if (ret) {
        //printk(KERN_INFO DRV_NAME"sdio_set_block_size 512 failed!\n");
    }
    else {
        //printk(KERN_INFO DRV_NAME"sdio_set_block_size 512 done!\n");
    }

    //printk(KERN_INFO DRV_NAME"param: func->cur_blksize(%d)\n", func->cur_blksize);
    //printk(KERN_INFO DRV_NAME"param: func->max_blksize(%d)\n", func->max_blksize);
    //printk(KERN_INFO DRV_NAME"param: func->card->host->max_blk_size(%d)\n", func->card->host->max_blk_size);
    //printk(KERN_INFO DRV_NAME"param: func->card->host->max_blk_count(%d)\n", func->card->host->max_blk_count);
#endif
    return TRUE;
} /* end of glBusInit() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glBusRelease (
    PVOID pvData
    )
{

    return;
} /* end of glBusRelease() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
INT_32
glBusSetIrq (
    PVOID pvData,
    PVOID pfnIsr,
    PVOID pvCookie
    )
{
    int ret = 0;

    struct net_device *prNetDevice = NULL;
    P_GLUE_INFO_T prGlueInfo = NULL;
    P_GL_HIF_INFO_T prHifInfo = NULL;

    ASSERT(pvData);
    if (!pvData) {
        return -1;
    }
    prNetDevice = (struct net_device *) pvData;
    prGlueInfo = (P_GLUE_INFO_T) pvCookie;
    ASSERT(prGlueInfo);
    if (!prGlueInfo) {
        return -1;
    }

    prHifInfo = &prGlueInfo->rHifInfo;

#if (MTK_WCN_HIF_SDIO == 0)
    sdio_claim_host(prHifInfo->func);
    ret = sdio_claim_irq(prHifInfo->func, mtk_sdio_interrupt);
    sdio_release_host(prHifInfo->func);
#else
    mtk_wcn_hif_sdio_enable_irq(prHifInfo->cltCtx, TRUE);
#endif

    return ret;
} /* end of glBusSetIrq() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
glBusFreeIrq (
    PVOID pvData,
    PVOID pvCookie
    )
{
    struct net_device *prNetDevice = NULL;
    P_GLUE_INFO_T prGlueInfo = NULL;
    P_GL_HIF_INFO_T prHifInfo = NULL;

    ASSERT(pvData);
    if (!pvData) {
        //printk(KERN_INFO DRV_NAME"%s null pvData\n", __FUNCTION__);
        return;
    }
    prNetDevice = (struct net_device *) pvData;
    prGlueInfo = (P_GLUE_INFO_T) pvCookie;
    ASSERT(prGlueInfo);
    if (!prGlueInfo) {
        //printk(KERN_INFO DRV_NAME"%s no glue info\n", __FUNCTION__);
        return;
    }

    prHifInfo = &prGlueInfo->rHifInfo;
#if (MTK_WCN_HIF_SDIO == 0)
    sdio_claim_host(prHifInfo->func);
    sdio_release_irq(prHifInfo->func);
    sdio_release_host(prHifInfo->func);
#else
    mtk_wcn_hif_sdio_enable_irq(prHifInfo->cltCtx, FALSE);
#endif

    return;
} /* end of glBusreeIrq() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOL
kalDevRegRead (
    IN  P_GLUE_INFO_T   prGlueInfo,
    IN  UINT_32         u4Register,
    OUT PUINT_32        pu4Value
    )
{
    int ret = 0;

    ASSERT(prGlueInfo);
    ASSERT(pu4Value);

#if MTK_WCN_HIF_SDIO
    ret = mtk_wcn_hif_sdio_readl(prGlueInfo->rHifInfo.cltCtx, u4Register, (PUINT32) pu4Value);
#else
    if (!in_interrupt) {
        sdio_claim_host(prGlueInfo->rHifInfo.func);
    }

    *pu4Value = sdio_readl(prGlueInfo->rHifInfo.func, u4Register, &ret);

    if (!in_interrupt) {
        sdio_release_host(prGlueInfo->rHifInfo.func);
    }
#endif

    if (ret) {
        kalSendAeeWarning(HIF_SDIO_ERR_TITLE_STR, HIF_SDIO_ERR_DESC_STR "sdio_readl() reports error: %x", ret);
        DBGLOG(HAL, ERROR, ("sdio_readl() reports error: %x", ret));
    }

    return (ret) ? FALSE : TRUE;
} /* end of kalDevRegRead() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOL
kalDevRegWrite (
    IN P_GLUE_INFO_T  prGlueInfo,
    IN UINT_32        u4Register,
    IN UINT_32        u4Value
    )
{
    int ret = 0;

    ASSERT(prGlueInfo);

#if MTK_WCN_HIF_SDIO
    ret = mtk_wcn_hif_sdio_writel(prGlueInfo->rHifInfo.cltCtx, u4Register, u4Value);
#else
    if (!in_interrupt) {
        sdio_claim_host(prGlueInfo->rHifInfo.func);
    }

    sdio_writel(prGlueInfo->rHifInfo.func, u4Value, u4Register, &ret);

    if (!in_interrupt) {
        sdio_release_host(prGlueInfo->rHifInfo.func);
    }
#endif

    if (ret) {
        kalSendAeeWarning(HIF_SDIO_ERR_TITLE_STR, HIF_SDIO_ERR_DESC_STR "sdio_writel() reports error: %x", ret);
        DBGLOG(HAL, ERROR, ("sdio_writel() reports error: %x", ret));
    }

    return (ret) ? FALSE : TRUE;
} /* end of kalDevRegWrite() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOL
kalDevPortRead (
    IN  P_GLUE_INFO_T   prGlueInfo,
    IN  UINT_16         u2Port,
    IN  UINT_32         u4Len,
    OUT PUINT_8         pucBuf,
    IN  UINT_32         u4ValidOutBufSize
    )
{
    P_GL_HIF_INFO_T prHifInfo = NULL;
    PUINT_8 pucDst = NULL;
    int count = u4Len;
    int ret = 0;
    int bNum = 0;

#if (MTK_WCN_HIF_SDIO == 0)
    struct sdio_func *prSdioFunc = NULL;
#endif

    #if DBG
    //printk(KERN_INFO DRV_NAME"++kalDevPortRead++ buf:0x%p, port:0x%x, length:%d\n", pucBuf, u2Port, u4Len);
    #endif

    ASSERT(prGlueInfo);
    prHifInfo = &prGlueInfo->rHifInfo;

    ASSERT(pucBuf);
    pucDst = pucBuf;

    ASSERT(u4Len <= u4ValidOutBufSize);

#if (MTK_WCN_HIF_SDIO == 0)
    prSdioFunc = prHifInfo->func;

    ASSERT(prSdioFunc->cur_blksize > 0);

    if (!in_interrupt) {
        sdio_claim_host(prSdioFunc);
    }

    /* Split buffer into multiple single block to workaround hifsys */
    while (count >= prSdioFunc->cur_blksize) {
        count -= prSdioFunc->cur_blksize;
        bNum++;
    }
    if (count > 0 && bNum > 0) {
        bNum++;
    }

    if (bNum > 0) {
        ret = sdio_readsb(prSdioFunc, pucDst, u2Port, prSdioFunc->cur_blksize * bNum);

#ifdef CONFIG_X86
        /* ENE workaround */
        {
            int tmp;
            sdio_writel(prSdioFunc, 0x0, SDIO_X86_WORKAROUND_WRITE_MCR, &tmp);
        }
#endif

    }
    else {
        ret = sdio_readsb(prSdioFunc, pucDst, u2Port, count);
    }

    if (!in_interrupt) {
        sdio_release_host(prSdioFunc);
    }
#else

    /* Split buffer into multiple single block to workaround hifsys */
    while (count >= (prGlueInfo->rHifInfo).prFuncInfo->blk_sz) {
        count -= ((prGlueInfo->rHifInfo).prFuncInfo->blk_sz);
        bNum++;
    }
    if (count > 0 && bNum > 0) {
        bNum++;
    }

    if (bNum > 0) {
        ret = mtk_wcn_hif_sdio_read_buf(prGlueInfo->rHifInfo.cltCtx, u2Port, (PUINT32) pucDst,
                 ((prGlueInfo->rHifInfo).prFuncInfo->blk_sz) * bNum);
    }
    else {
        ret = mtk_wcn_hif_sdio_read_buf(prGlueInfo->rHifInfo.cltCtx, u2Port, (PUINT32) pucDst, count);
    }
#endif

    if (ret) {
        kalSendAeeWarning(HIF_SDIO_ERR_TITLE_STR, HIF_SDIO_ERR_DESC_STR "sdio_readsb() reports error: %x", ret);
        DBGLOG(HAL, ERROR, ("sdio_readsb() reports error: %x", ret));
    }

    return (ret) ? FALSE : TRUE;
} /* end of kalDevPortRead() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOL
kalDevPortWrite (
    IN P_GLUE_INFO_T  prGlueInfo,
    IN UINT_16        u2Port,
    IN UINT_32        u4Len,
    IN PUINT_8        pucBuf,
    IN UINT_32        u4ValidInBufSize
    )
{
    P_GL_HIF_INFO_T prHifInfo = NULL;
    PUINT_8 pucSrc = NULL;
    int count = u4Len;
    int ret = 0;
    int bNum = 0;

#if (MTK_WCN_HIF_SDIO == 0)
    struct sdio_func *prSdioFunc = NULL;
#endif

    #if DBG
    //printk(KERN_INFO DRV_NAME"++kalDevPortWrite++ buf:0x%p, port:0x%x, length:%d\n", pucBuf, u2Port, u2Len);
    #endif

    ASSERT(prGlueInfo);
    prHifInfo = &prGlueInfo->rHifInfo;

    ASSERT(pucBuf);
    pucSrc = pucBuf;

    ASSERT(u4Len <= u4ValidInBufSize);

#if (MTK_WCN_HIF_SDIO == 0)
    prSdioFunc = prHifInfo->func;
    ASSERT(prSdioFunc->cur_blksize > 0);

    if (!in_interrupt) {
        sdio_claim_host(prSdioFunc);
    }

    /* Split buffer into multiple single block to workaround hifsys */
    while (count >= prSdioFunc->cur_blksize) {
        count -= prSdioFunc->cur_blksize;
        bNum++;
    }
    if (count > 0 && bNum > 0) {
        bNum++;
    }

    if (bNum > 0) { // block mode
        ret = sdio_writesb(prSdioFunc, u2Port, pucSrc, prSdioFunc->cur_blksize * bNum);

#ifdef CONFIG_X86
        /* ENE workaround */
        {
            int tmp;
            sdio_writel(prSdioFunc, 0x0, SDIO_X86_WORKAROUND_WRITE_MCR, &tmp);
        }
#endif

    }
    else {  // byte mode

        ret = sdio_writesb(prSdioFunc, u2Port, pucSrc, count);
    }

    if (!in_interrupt) {
        sdio_release_host(prSdioFunc);
    }
#else
    /* Split buffer into multiple single block to workaround hifsys */
    while (count >= ((prGlueInfo->rHifInfo).prFuncInfo->blk_sz)) {
        count -= ((prGlueInfo->rHifInfo).prFuncInfo->blk_sz);
        bNum++;
    }
    if (count > 0 && bNum > 0) {
        bNum++;
    }

    if (bNum > 0) { // block mode
        ret = mtk_wcn_hif_sdio_write_buf(prGlueInfo->rHifInfo.cltCtx, u2Port, (PUINT32) pucSrc,
                             ((prGlueInfo->rHifInfo).prFuncInfo->blk_sz) * bNum);
    }
    else {  // byte mode
        ret = mtk_wcn_hif_sdio_write_buf(prGlueInfo->rHifInfo.cltCtx, u2Port, (PUINT32) pucSrc, count);
    }
#endif

    if (ret) {
        kalSendAeeWarning(HIF_SDIO_ERR_TITLE_STR, HIF_SDIO_ERR_DESC_STR "sdio_writesb() reports error: %x", ret);
        DBGLOG(HAL, ERROR, ("sdio_writesb() reports error: %x", ret));
    }

    return (ret) ? FALSE : TRUE;
} /* end of kalDevPortWrite() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOL
kalDevWriteWithSdioCmd52 (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN UINT_32          u4Addr,
    IN UINT_8           ucData
    )
{
    int ret = 0;

#if (MTK_WCN_HIF_SDIO == 0)
    if (!in_interrupt) {
        sdio_claim_host(prGlueInfo->rHifInfo.func);
    }

    sdio_writeb(prGlueInfo->rHifInfo.func, ucData, u4Addr, &ret);

    if (!in_interrupt) {
        sdio_release_host(prGlueInfo->rHifInfo.func);
    }
#else
    ret = mtk_wcn_hif_sdio_writeb(prGlueInfo->rHifInfo.cltCtx, u4Addr, ucData);
#endif

    if (ret) {
        kalSendAeeWarning(HIF_SDIO_ERR_TITLE_STR, HIF_SDIO_ERR_DESC_STR "sdio_writeb() reports error: %x", ret);
        DBGLOG(HAL, ERROR, ("sdio_writeb() reports error: %x", ret));
    }

    return (ret) ? FALSE : TRUE;

} /* end of kalDevWriteWithSdioCmd52() */


VOID
glSetPowerState (
    IN P_GLUE_INFO_T  prGlueInfo,
    IN UINT_32 ePowerMode
    )
{
    return;
}

