





#include "precomp.h"



#define P2P_MODE_INF_NAME "p2p%d";
#define AP_MODE_INF_NAME "ap%d";
//#define MAX_INF_NAME_LEN 15
//#define MIN_INF_NAME_LEN 1

#define RUNNING_P2P_MODE 0
#define RUNNING_AP_MODE 1






static PUCHAR ifname = P2P_MODE_INF_NAME;
static UINT_16 mode = RUNNING_P2P_MODE;


#if defined(CONFIG_HAS_EARLYSUSPEND) && 0
extern int glRegisterEarlySuspend(
    struct early_suspend        *prDesc,
    early_suspend_callback      wlanSuspend,
    late_resume_callback        wlanResume);

extern int glUnregisterEarlySuspend(struct early_suspend *prDesc);
#endif



/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VOID
p2pCheckInterfaceName(
    VOID
    )
{

    if(mode) {
        mode = RUNNING_AP_MODE;
        ifname = AP_MODE_INF_NAME;
    }
#if 0
    UINT_32 ifLen = 0;

    if(ifname) {
        ifLen = strlen(ifname);

        if(ifLen > MAX_INF_NAME_LEN) {
            ifname[MAX_INF_NAME_LEN] = '\0';
        }
        else if( ifLen < MIN_INF_NAME_LEN  ) {
            ifname = P2P_MODE_INF_NAME;
        }
    } else {
        ifname = P2P_MODE_INF_NAME;
    }
#endif
}


extern UINT_8 g_aucBufIpAddr[32];

#if 0
static void wlanP2PEarlySuspend(void)
{
    struct net_device *prDev = NULL;
    P_GLUE_INFO_T prGlueInfo = NULL;
    UINT_8  ip[4] = { 0 };
    UINT_32 u4NumIPv4 = 0;
#ifdef  CONFIG_IPV6
    UINT_8  ip6[16] = { 0 };     // FIX ME: avoid to allocate large memory in stack
    UINT_32 u4NumIPv6 = 0;
#endif
    UINT_32 i;
	P_PARAM_NETWORK_ADDRESS_IP prParamIpAddr;

    printk(KERN_INFO "*********p2pEarlySuspend************\n");

    if(!wlanExportGlueInfo(&prGlueInfo)) {
        printk(KERN_INFO "*********p2pEarlySuspend ignored************\n");
        return;
    }

    ASSERT(prGlueInfo);
    // <1> Sanity check and acquire the net_device
    prDev = prGlueInfo->prP2PInfo->prDevHandler;
    ASSERT(prDev);

    // <3> get the IPv4 address
    if(!prDev || !(prDev->ip_ptr)||\
        !((struct in_device *)(prDev->ip_ptr))->ifa_list||\
        !(&(((struct in_device *)(prDev->ip_ptr))->ifa_list->ifa_local))){
        printk(KERN_INFO "ip is not avaliable.\n");
        return;
    }

    // <4> copy the IPv4 address
    kalMemCopy(ip, &(((struct in_device *)(prDev->ip_ptr))->ifa_list->ifa_local), sizeof(ip));
    printk(KERN_INFO"ip is %d.%d.%d.%d\n",
            ip[0],ip[1],ip[2],ip[3]);

   // todo: traverse between list to find whole sets of IPv4 addresses
    if (!((ip[0] == 0) &&
         (ip[1] == 0) &&
         (ip[2] == 0) &&
         (ip[3] == 0))) {
        u4NumIPv4++;
    }

#ifdef  CONFIG_IPV6
    // <5> get the IPv6 address
    if(!prDev || !(prDev->ip6_ptr)||\
        !((struct in_device *)(prDev->ip6_ptr))->ifa_list||\
        !(&(((struct in_device *)(prDev->ip6_ptr))->ifa_list->ifa_local))){
        printk(KERN_INFO "ipv6 is not avaliable.\n");
        return;
    }
    // <6> copy the IPv6 address
    kalMemCopy(ip6, &(((struct in_device *)(prDev->ip6_ptr))->ifa_list->ifa_local), sizeof(ip6));
    printk(KERN_INFO"ipv6 is %d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d\n",
            ip6[0],ip6[1],ip6[2],ip6[3],
            ip6[4],ip6[5],ip6[6],ip6[7],
            ip6[8],ip6[9],ip6[10],ip6[11],
            ip6[12],ip6[13],ip6[14],ip6[15]
            );
    // todo: traverse between list to find whole sets of IPv6 addresses

    if (!((ip6[0] == 0) &&
         (ip6[1] == 0) &&
         (ip6[2] == 0) &&
         (ip6[3] == 0) &&
         (ip6[4] == 0) &&
         (ip6[5] == 0))) {
    }

#endif
    // <7> set up the ARP filter
    {
        WLAN_STATUS rStatus = WLAN_STATUS_FAILURE;
        UINT_32 u4SetInfoLen = 0;
//        UINT_8 aucBuf[32] = {0};
        UINT_32 u4Len = OFFSET_OF(PARAM_NETWORK_ADDRESS_LIST, arAddress);
        P_PARAM_NETWORK_ADDRESS_LIST prParamNetAddrList = (P_PARAM_NETWORK_ADDRESS_LIST)g_aucBufIpAddr;//aucBuf;
        P_PARAM_NETWORK_ADDRESS prParamNetAddr = prParamNetAddrList->arAddress;

        kalMemZero(g_aucBufIpAddr, sizeof(g_aucBufIpAddr));

        prParamNetAddrList->u4AddressCount = u4NumIPv4 + u4NumIPv6;
        prParamNetAddrList->u2AddressType = PARAM_PROTOCOL_ID_TCP_IP;
        for (i = 0; i < u4NumIPv4; i++) {
            prParamNetAddr->u2AddressLength = sizeof(PARAM_NETWORK_ADDRESS_IP);//4;;
            prParamNetAddr->u2AddressType = PARAM_PROTOCOL_ID_TCP_IP;;
#if 0
            kalMemCopy(prParamNetAddr->aucAddress, ip, sizeof(ip));
            prParamNetAddr = (P_PARAM_NETWORK_ADDRESS)((UINT_32)prParamNetAddr + sizeof(ip));
            u4Len += OFFSET_OF(PARAM_NETWORK_ADDRESS, aucAddress) + sizeof(ip);
#else
            prParamIpAddr = (P_PARAM_NETWORK_ADDRESS_IP)prParamNetAddr->aucAddress;
            kalMemCopy(&prParamIpAddr->in_addr, ip, sizeof(ip));

//            prParamNetAddr = (P_PARAM_NETWORK_ADDRESS)((UINT_32)prParamNetAddr + sizeof(PARAM_NETWORK_ADDRESS));    // TODO: frog. The pointer is not right.

            prParamNetAddr = (P_PARAM_NETWORK_ADDRESS) ((UINT_32) prParamNetAddr +
            (UINT_32) (prParamNetAddr->u2AddressLength + OFFSET_OF(PARAM_NETWORK_ADDRESS, aucAddress)));

            u4Len += OFFSET_OF(PARAM_NETWORK_ADDRESS, aucAddress) + sizeof(PARAM_NETWORK_ADDRESS_IP);
#endif
        }
#ifdef  CONFIG_IPV6
        for (i = 0; i < u4NumIPv6; i++) {
            prParamNetAddr->u2AddressLength = 6;;
            prParamNetAddr->u2AddressType = PARAM_PROTOCOL_ID_TCP_IP;;
            kalMemCopy(prParamNetAddr->aucAddress, ip6, sizeof(ip6));
//            prParamNetAddr = (P_PARAM_NETWORK_ADDRESS)((UINT_32)prParamNetAddr + sizeof(ip6));

            prParamNetAddr = (P_PARAM_NETWORK_ADDRESS) ((UINT_32) prParamNetAddr +
            (UINT_32) (prParamNetAddr->u2AddressLength + OFFSET_OF(PARAM_NETWORK_ADDRESS, aucAddress)));

            u4Len += OFFSET_OF(PARAM_NETWORK_ADDRESS, aucAddress) + sizeof(ip6);
       }
#endif
        ASSERT(u4Len <= sizeof(g_aucBufIpAddr/*aucBuf*/));

        rStatus = kalIoctl(prGlueInfo,
                wlanoidSetP2pSetNetworkAddress,
                (PVOID)prParamNetAddrList,
                u4Len,
                FALSE,
                FALSE,
                TRUE,
                TRUE,
                &u4SetInfoLen);

        if (rStatus != WLAN_STATUS_SUCCESS) {
            printk(KERN_INFO DRV_NAME"set HW pattern filter fail 0x%lx\n", rStatus);
        }
    }
}


static void wlanP2PLateResume(void)
{
    struct net_device *prDev = NULL;
    P_GLUE_INFO_T prGlueInfo = NULL;
    UINT_8  ip[4] = { 0 };
#ifdef  CONFIG_IPV6
    UINT_8  ip6[16] = { 0 };     // FIX ME: avoid to allocate large memory in stack
#endif

    printk(KERN_INFO "*********wlanP2PLateResume************\n");
    if(!wlanExportGlueInfo(&prGlueInfo)) {
        printk(KERN_INFO "*********p2pLateResume ignored************\n");
        return;
    }

    ASSERT(prGlueInfo);
    // <1> Sanity check and acquire the net_device
    prDev = prGlueInfo->prP2PInfo->prDevHandler;
    ASSERT(prDev);

   // <3> get the IPv4 address
    if(!prDev || !(prDev->ip_ptr)||\
        !((struct in_device *)(prDev->ip_ptr))->ifa_list||\
        !(&(((struct in_device *)(prDev->ip_ptr))->ifa_list->ifa_local))){
        printk(KERN_INFO "ip is not avaliable.\n");
        return;
    }

    // <4> copy the IPv4 address
    kalMemCopy(ip, &(((struct in_device *)(prDev->ip_ptr))->ifa_list->ifa_local), sizeof(ip));
    printk(KERN_INFO"ip is %d.%d.%d.%d\n",
            ip[0],ip[1],ip[2],ip[3]);

#ifdef  CONFIG_IPV6
    // <5> get the IPv6 address
    if(!prDev || !(prDev->ip6_ptr)||\
        !((struct in_device *)(prDev->ip6_ptr))->ifa_list||\
        !(&(((struct in_device *)(prDev->ip6_ptr))->ifa_list->ifa_local))){
        printk(KERN_INFO "ipv6 is not avaliable.\n");
        return;
    }
    // <6> copy the IPv6 address
    kalMemCopy(ip6, &(((struct in_device *)(prDev->ip6_ptr))->ifa_list->ifa_local), sizeof(ip6));
    printk(KERN_INFO"ipv6 is %d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d\n",
            ip6[0],ip6[1],ip6[2],ip6[3],
            ip6[4],ip6[5],ip6[6],ip6[7],
            ip6[8],ip6[9],ip6[10],ip6[11],
            ip6[12],ip6[13],ip6[14],ip6[15]
            );
#endif
    // <7> clear the ARP filter
    {
        WLAN_STATUS rStatus = WLAN_STATUS_FAILURE;
        UINT_32 u4SetInfoLen = 0;
//        UINT_8 aucBuf[32] = {0};
        UINT_32 u4Len = sizeof(PARAM_NETWORK_ADDRESS_LIST);
        P_PARAM_NETWORK_ADDRESS_LIST prParamNetAddrList = (P_PARAM_NETWORK_ADDRESS_LIST)g_aucBufIpAddr;//aucBuf;

        kalMemZero(g_aucBufIpAddr, sizeof(g_aucBufIpAddr));

        prParamNetAddrList->u4AddressCount = 0;
        prParamNetAddrList->u2AddressType = PARAM_PROTOCOL_ID_TCP_IP;

        ASSERT(u4Len <= sizeof(g_aucBufIpAddr/*aucBuf*/));
        rStatus = kalIoctl(prGlueInfo,
                wlanoidSetP2pSetNetworkAddress,
                (PVOID)prParamNetAddrList,
                u4Len,
                FALSE,
                FALSE,
                TRUE,
                TRUE,
                &u4SetInfoLen);

        if (rStatus != WLAN_STATUS_SUCCESS) {
            printk(KERN_INFO DRV_NAME"set HW pattern filter fail 0x%lx\n", rStatus);
        }
    }
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND) && 0
static struct early_suspend mt6620_p2p_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
};

static void p2p_early_suspend(struct early_suspend *h)
{
    printk(KERN_INFO "*********wlanP2P_early_suspend************\n");
    wlanP2PEarlySuspend();
}

static void p2p_late_resume(struct early_suspend *h)
{
    printk(KERN_INFO "*********wlanP2P_late_resume************\n");
    wlanP2PLateResume();
}
#endif

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOLEAN
p2pLaunch(
    P_GLUE_INFO_T prGlueInfo
    )
{
    printk("p2p Launch\n");

    if(prGlueInfo->prAdapter->fgIsP2PRegistered == TRUE) {
        printk("p2p already registered\n");
        return FALSE;
    }
    else if(glRegisterP2P(prGlueInfo, ifname, (BOOLEAN)mode)) {
        prGlueInfo->prAdapter->fgIsP2PRegistered = TRUE;
        printk("Launch success, fgIsP2PRegistered TRUE.\n");

        return TRUE;
    }
    else {
        printk("Launch Fail\n");
    }

    return FALSE;
}


VOID
p2pSetMode (
    IN BOOLEAN fgIsAPMOde
    )
{
    if (fgIsAPMOde) {
        mode = RUNNING_AP_MODE;
        ifname = AP_MODE_INF_NAME;
    }
    else {
        mode = RUNNING_P2P_MODE;
        ifname = P2P_MODE_INF_NAME;
    }

    return;
} /* p2pSetMode */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
BOOLEAN
p2pRemove(
    P_GLUE_INFO_T prGlueInfo
    )
{
    printk("p2p Remove\n");

    if(prGlueInfo->prAdapter->fgIsP2PRegistered == FALSE) {
        printk("p2p is not Registered.\n");
        return FALSE;
    }
    else {
        prGlueInfo->prAdapter->fgIsP2PRegistered = FALSE;
        glUnregisterP2P(prGlueInfo);
        /*p2p is removed successfully*/
        return TRUE;
    }
    
    return FALSE;
}

#if 0
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int initP2P(void)
{
    P_GLUE_INFO_T prGlueInfo;

    /*check interface name validation*/
    p2pCheckInterfaceName();

    printk( KERN_INFO DRV_NAME "InitP2P, Ifname: %s, Mode: %s\n", ifname, mode ? "AP":"P2P");

    /*register p2p init & exit function to wlan sub module handler*/
    wlanSubModRegisterInitExit(p2pLaunch, p2pRemove, P2P_MODULE);

    /*if wlan is not start yet, do nothing
        * p2pLaunch will be called by txthread while wlan start
        */
    /*if wlan is not started yet, return FALSE*/
    if(wlanExportGlueInfo(&prGlueInfo)) {
        wlanSubModInit(prGlueInfo);
        return ( prGlueInfo->prAdapter->fgIsP2PRegistered? 0: -EIO);
    }

    return 0;
} /* end of initP2P() */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//1 Module Leave Point
static VOID __exit exitP2P(void)
{
    P_GLUE_INFO_T prGlueInfo;

    printk( KERN_INFO DRV_NAME "ExitP2P\n");

    /*if wlan is not started yet, return FALSE*/
    if(wlanExportGlueInfo(&prGlueInfo)) {
        wlanSubModExit(prGlueInfo);
    }
    /*UNregister p2p init & exit function to wlan sub module handler*/
    wlanSubModRegisterInitExit(NULL, NULL, P2P_MODULE);
} /* end of exitP2P() */
#endif

