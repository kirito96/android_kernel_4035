






#include "config.h"

#if CFG_ENABLE_WIFI_DIRECT && CFG_ENABLE_WIFI_DIRECT_CFG_80211
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>

#include "precomp.h"








BOOLEAN
mtk_p2p_cfg80211func_channel_format_switch(
    IN struct ieee80211_channel *channel,
    IN enum nl80211_channel_type channel_type,
    IN P_RF_CHANNEL_INFO_T prRfChnlInfo,
    IN P_ENUM_CHNL_EXT_T prChnlSco
    );


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)


INT_32 
mtk_Netdev_To_RoleIdx (
    P_GL_P2P_INFO_T prGlP2pInfo,
    struct net_device *ndev,
    PUINT_8 pucRoleIdx
    )
{
    INT_32 i4Ret = -1;
    UINT_32 u4Idx = 0;

    if (pucRoleIdx == NULL) {
        return i4Ret;
    }
    
    for (u4Idx = 0; u4Idx < KAL_P2P_NUM; u4Idx++) {
        if (prGlP2pInfo->aprRoleHandler[u4Idx] == ndev) {
            *pucRoleIdx = (UINT_8)u4Idx;
            i4Ret = 0;
        }
    }

    return i4Ret;
    
} /* mtk_Netdev_To_RoleIdx */

#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
extern const struct net_device_ops p2p_netdev_ops;
#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
struct wireless_dev *
#else
struct net_device *
#endif
mtk_p2p_cfg80211_add_iface (
    struct wiphy *wiphy,
    const char *name,
    enum nl80211_iftype type,
    u32 *flags,
    struct vif_params *params
    )
{
    //2 TODO: Fit kernel 3.10 modification
#if 0
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    struct net_device* prNewNetDevice = NULL;
    UINT_32 u4Idx = 0;
    P_GL_P2P_INFO_T prP2pInfo = (P_GL_P2P_INFO_T)NULL;


    do {
        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
        if (prGlueInfo == NULL) {
            break;
        }

        prP2pInfo = prGlueInfo->prP2PInfo;

        for (u4Idx = 0; u4Idx < KAL_P2P_NUM; u4Idx++) {
            if (prP2pInfo->aprRoleHandler[u4Idx] == NULL) {
                p2pRoleFsmInit(prGlueInfo->prAdapter, u4Idx);
            }
            else if (prP2pInfo->aprRoleHandler[u4Idx] == prP2pInfo->prDevHandler) {
                break;
            }
        }

        if (u4Idx == 2) {
            /* Role port full. */
            break;
        }

        prNewNetDevice = alloc_netdev_mq(sizeof(P_GLUE_INFO_T), name, ether_setup, CFG_MAX_TXQ_NUM);

        if (prNewNetDevice == NULL) {
            break;
        }
        
        prP2pInfo->aprRoleHandler[u4Idx] = prNewNetDevice;

        *((P_GLUE_INFO_T *)netdev_priv(prNewNetDevice)) = prGlueInfo;

        memcpy(prNewNetDevice->perm_addr, prGlueInfo->prAdapter->rWifiVar.aucInterfaceAddress, ETH_ALEN);
        prNewNetDevice->dev_addr = prNewNetDevice->perm_addr;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
        prNewNetDevice->netdev_ops           = &p2p_netdev_ops;
#else
        ASSERT(FALSE);
#endif

#if (MTK_WCN_HIF_SDIO == 0)
        SET_NETDEV_DEV(prNewNetDevice, &(prGlueInfo->rHifInfo.func->dev));
#endif


#if CFG_ENABLE_WIFI_DIRECT_CFG_80211
        prNewNetDevice->ieee80211_ptr        = &(prP2pInfo->wdev);
#endif

#if CFG_TCP_IP_CHKSUM_OFFLOAD
        prNewNetDevice->features = NETIF_F_IP_CSUM;
#endif /* CFG_TCP_IP_CHKSUM_OFFLOAD */

        
    } while (FALSE);


    return prNewNetDevice;

#else
    return NULL;
#endif
} /* mtk_p2p_cfg80211_add_iface */


int
mtk_p2p_cfg80211_del_iface (
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
    struct wireless_dev *wdev
#else
    struct net_device *dev
#endif    
    )
{
    // TODO:
    return 0;
} /* mtk_p2p_cfg80211_del_iface */


int mtk_p2p_cfg80211_add_key(
    struct wiphy *wiphy,
    struct net_device *ndev,
    u8 key_index,
    bool pairwise,
    const u8 *mac_addr,
    struct key_params *params
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    INT_32 i4Rslt = -EINVAL;
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
    UINT_32 u4BufLen = 0;
    P2P_PARAM_KEY_T rKey;
    UINT_8 ucRoleIdx = 0;
    const UINT_8 aucBCAddr[] = BC_MAC_ADDR;
    const UINT_8 aucZeroMacAddr[] = NULL_MAC_ADDR;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, ndev, &ucRoleIdx) != 0) {
        return -EINVAL;
    }
    
#if DBG
    DBGLOG(RSN, TRACE, ("mtk_p2p_cfg80211_add_key\n"));
    if (mac_addr) {
        DBGLOG(RSN, TRACE, ("keyIdx = %d pairwise = %d mac = "MACSTR"\n", key_index, pairwise, MAC2STR(mac_addr)));
    } else {
        DBGLOG(RSN, TRACE, ("keyIdx = %d pairwise = %d null mac\n", key_index, pairwise));
    }
    DBGLOG(RSN, TRACE, ("Cipher = %x\n", params->cipher));
    DBGLOG_MEM8(RSN, TRACE, params->key, params->key_len);
#endif

    //Todo:: By Cipher to set the key

    kalMemZero(&rKey, sizeof(P2P_PARAM_KEY_T));

    if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
                                        ucRoleIdx,
                                        &rKey.ucBssIdx) != WLAN_STATUS_SUCCESS) {
        return -EINVAL;                                
    }
    
    rKey.u4KeyIndex = key_index;
    
    if (mac_addr) {
        if (EQUAL_MAC_ADDR(mac_addr, aucZeroMacAddr))
            COPY_MAC_ADDR(rKey.arBSSID, aucBCAddr);
        else 
            COPY_MAC_ADDR(rKey.arBSSID, mac_addr);

        if (pairwise) {
            //if (!((rKey.arBSSID[0] & rKey.arBSSID[1] & rKey.arBSSID[2] & rKey.arBSSID[3] & rKey.arBSSID[4] & rKey.arBSSID[5]) == 0xFF)) {
            //    rKey.u4KeyIndex |= BIT(31);
            //}
            rKey.u4KeyIndex |= BIT(31);
            rKey.u4KeyIndex |= BIT(30);
        }
    }
    else {
        COPY_MAC_ADDR(rKey.arBSSID, aucBCAddr);
    }
    
    if (params->key)
    {
        kalMemCopy(rKey.aucKeyMaterial, params->key, params->key_len);
    }
    rKey.u4KeyLength = params->key_len;
    rKey.u4Length = ((UINT_32)&(((P_P2P_PARAM_KEY_T)0)->aucKeyMaterial)) + rKey.u4KeyLength;

    rStatus = kalIoctl(prGlueInfo,
            wlanoidSetAddP2PKey,
            &rKey,
            rKey.u4Length,
            FALSE,
            FALSE,
            TRUE,
            &u4BufLen);
    if (rStatus == WLAN_STATUS_SUCCESS)
        i4Rslt = 0;

    return i4Rslt;
}


int mtk_p2p_cfg80211_get_key(
    struct wiphy *wiphy,
    struct net_device *ndev,
    u8 key_index,
    bool pairwise,
    const u8 *mac_addr,
    void *cookie,
    void (*callback)(void *cookie, struct key_params*)
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    // not implemented yet

    return -EINVAL;
}


int mtk_p2p_cfg80211_del_key(
    struct wiphy *wiphy,
    struct net_device *ndev,
    u8 key_index,
    bool pairwise,
    const u8 *mac_addr
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    PARAM_REMOVE_KEY_T rRemoveKey;
    INT_32 i4Rslt = -EINVAL;
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
    UINT_32 u4BufLen = 0;
    UINT_8 ucRoleIdx = 0;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));


    if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo,
                                ndev,
                                &ucRoleIdx) < 0) {
        return -EINVAL;
    }

#if DBG
    DBGLOG(RSN, TRACE, ("mtk_p2p_cfg80211_del_key\n"));
    if (mac_addr) {
        DBGLOG(RSN, TRACE, ("keyIdx = %d pairwise = %d mac = "MACSTR"\n", key_index, pairwise, MAC2STR(mac_addr)));
    }
    else {
        DBGLOG(RSN, TRACE, ("keyIdx = %d pairwise = %d null mac\n", key_index, pairwise));      
    }
#endif
    
    kalMemZero(&rRemoveKey, sizeof(PARAM_REMOVE_KEY_T));

    if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
                        ucRoleIdx,
                        &rRemoveKey.ucBssIdx) != WLAN_STATUS_SUCCESS) {
        return -EINVAL;
    }
    
    if (mac_addr)
        COPY_MAC_ADDR(rRemoveKey.arBSSID, mac_addr);
    rRemoveKey.u4KeyIndex = key_index;
    rRemoveKey.u4Length = sizeof(PARAM_REMOVE_KEY_T);

    rStatus = kalIoctl(prGlueInfo,
            wlanoidSetRemoveP2PKey,
            &rRemoveKey,
            rRemoveKey.u4Length,
            FALSE,
            FALSE,
            TRUE,
            &u4BufLen);

    if (rStatus == WLAN_STATUS_SUCCESS)
        i4Rslt = 0;

    return i4Rslt;
}


int
mtk_p2p_cfg80211_set_default_key (
    struct wiphy *wiphy,
    struct net_device *netdev,
    u8 key_index,
    bool unicast,
    bool multicast
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    //PARAM_DEFAULT_KEY_T rDefaultKey;
    UINT_8 ucRoleIdx = 0;
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
    INT_32 i4Rst = -EINVAL;
    //UINT_32 u4BufLen = 0;
    //BOOLEAN fgDef = FALSE, fgMgtDef = FALSE;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, netdev, &ucRoleIdx) != 0) {
        return -EINVAL;
    }

#if DBG
    DBGLOG(RSN, TRACE, ("mtk_p2p_cfg80211_set_default_key\n"));
    DBGLOG(RSN, TRACE, ("keyIdx = %d unicast = %d multicast = %d\n", key_index, unicast, multicast));
#endif
#if 0
    if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
                                        ucRoleIdx,
                                        &rDefaultKey.ucBssIdx) != WLAN_STATUS_SUCCESS) {
        return -EINVAL;                                
    }

    rDefaultKey.ucKeyID = key_index;
    rDefaultKey.ucUnicast = unicast;
    rDefaultKey.ucMulticast = multicast;
    if (rDefaultKey.ucUnicast && !rDefaultKey.ucMulticast)
        return -EINVAL;

    if (rDefaultKey.ucUnicast && rDefaultKey.ucMulticast)
        fgDef = TRUE;

    if (!rDefaultKey.ucUnicast && rDefaultKey.ucMulticast)
        fgMgtDef = TRUE;
#endif
#if 0   
    rStatus = kalIoctl(prGlueInfo,
        wlanoidSetDefaultP2PKey,
        &rDefaultKey,
        sizeof(PARAM_DEFAULT_KEY_T),
        FALSE,
        FALSE,
        TRUE,
        FALSE,
        &u4BufLen);
#endif

    if (rStatus == WLAN_STATUS_SUCCESS)
        i4Rst = 0;

    return i4Rst;
}

int mtk_p2p_cfg80211_get_station(
    struct wiphy *wiphy,
    struct net_device *ndev,
    u8 *mac,
    struct station_info *sinfo
    )
{
    INT_32 i4RetRslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    P_GL_P2P_INFO_T prP2pGlueInfo = (P_GL_P2P_INFO_T)NULL;
    P2P_STATION_INFO_T rP2pStaInfo;

    ASSERT(wiphy);

    do {
        if ((wiphy == NULL) ||
                (ndev == NULL) ||
                (sinfo == NULL) ||
                (mac == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_get_station\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
        prP2pGlueInfo = prGlueInfo->prP2PInfo;

        sinfo->filled = 0;

        /* Get station information. */
        /* 1. Inactive time? */
        p2pFuncGetStationInfo(prGlueInfo->prAdapter,
                                                mac,
                                                &rP2pStaInfo);

        /* Inactive time. */
        sinfo->filled |= STATION_INFO_INACTIVE_TIME;
        sinfo->inactive_time = rP2pStaInfo.u4InactiveTime;
        sinfo->generation = prP2pGlueInfo->i4Generation;

        i4RetRslt = 0;
    } while (FALSE);

    return i4RetRslt;
}

int
mtk_p2p_cfg80211_scan (
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
    struct net_device *ndev,
#endif
    struct cfg80211_scan_request *request
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    P_GL_P2P_INFO_T prP2pGlueInfo = (P_GL_P2P_INFO_T)NULL;
    P_MSG_P2P_SCAN_REQUEST_T prMsgScanRequest = (P_MSG_P2P_SCAN_REQUEST_T)NULL;
    UINT_32 u4MsgSize = 0, u4Idx = 0;
    INT_32 i4RetRslt = -EINVAL;
    P_RF_CHANNEL_INFO_T prRfChannelInfo = (P_RF_CHANNEL_INFO_T)NULL;
    P_P2P_SSID_STRUCT_T prSsidStruct = (P_P2P_SSID_STRUCT_T)NULL;
    struct ieee80211_channel *prChannel = NULL;
    struct cfg80211_ssid *prSsid = NULL;
    UINT_8 ucBssIdx = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)  
    UINT_8 ucRoleIdx = 0;
#endif

    /* [---------Channel---------] [---------SSID---------][---------IE---------] */


    do {
        if ((wiphy == NULL) || (request == NULL)) {
            break;
        }

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        prP2pGlueInfo = prGlueInfo->prP2PInfo;

        if (prP2pGlueInfo == NULL) {
            ASSERT(FALSE);
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_scan.\n"));


        if (prP2pGlueInfo->prScanRequest != NULL) {
            /* There have been a scan request on-going processing. */
            DBGLOG(P2P, TRACE, ("There have been a scan request on-going processing.\n"));
            break;
        }

        prP2pGlueInfo->prScanRequest = request;

        /* Should find out why the n_channels so many? */
        if (request->n_channels > MAXIMUM_OPERATION_CHANNEL_LIST) {
            request->n_channels = MAXIMUM_OPERATION_CHANNEL_LIST;
            DBGLOG(P2P, TRACE, ("Channel list exceed the maximun support.\n"));
        }

        // TODO:
        /* Find a way to distinct DEV port scan & ROLE port scan.
                */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)                
        if (ndev == prGlueInfo->prP2PInfo->prDevHandler) {
            ucBssIdx = P2P_DEV_BSS_INDEX;
            DBGLOG(P2P, TRACE, ("Device Port Scan.\n"));
        }
        else if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo,
                                    ndev,
                                    &ucRoleIdx) < 0) {
            DBGLOG(P2P, TRACE, ("Can not find role port.\n"));
            break;
        }
        else {
            if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
                        ucRoleIdx,
                        &ucBssIdx) != WLAN_STATUS_SUCCESS) {
                DBGLOG(P2P, ERROR, ("Role Port to BSS index fail.\n"));
                break;
            } 
            DBGLOG(P2P, TRACE, ("Role Port Scan.\n"));
        }
#else
        ucBssIdx = P2P_DEV_BSS_INDEX;
        DBGLOG(P2P, TRACE, ("Device Port Scan.\n"));
#endif

        u4MsgSize = sizeof(MSG_P2P_SCAN_REQUEST_T) +
                                    (request->n_channels * sizeof(RF_CHANNEL_INFO_T)) +
                                    (request->n_ssids * sizeof(PARAM_SSID_T)) +
                                    request->ie_len;

        prMsgScanRequest = cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, u4MsgSize);

        if (prMsgScanRequest == NULL) {
            ASSERT(FALSE);
            i4RetRslt = -ENOMEM;
            break;
        }

        DBGLOG(P2P, TRACE, ("Generating scan request message.\n"));

        prMsgScanRequest->rMsgHdr.eMsgId = MID_MNY_P2P_DEVICE_DISCOVERY;
        prMsgScanRequest->eScanType = SCAN_TYPE_ACTIVE_SCAN;
        prMsgScanRequest->ucBssIdx = ucBssIdx;



        DBGLOG(P2P, TRACE, ("Requesting channel number:%d.\n", request->n_channels));

        for (u4Idx = 0; u4Idx < request->n_channels; u4Idx++) {
            /* Translate Freq from MHz to channel number. */
            prRfChannelInfo = &(prMsgScanRequest->arChannelListInfo[u4Idx]);
            prChannel = request->channels[u4Idx];

            prRfChannelInfo->ucChannelNum = nicFreq2ChannelNum(prChannel->center_freq * 1000);
            DBGLOG(P2P, TRACE, ("Scanning Channel:%d,  freq: %d\n",
                                                        prRfChannelInfo->ucChannelNum,
                                                        prChannel->center_freq));
            switch (prChannel->band) {
            case IEEE80211_BAND_2GHZ:
                prRfChannelInfo->eBand = BAND_2G4;
                break;
            case IEEE80211_BAND_5GHZ:
                prRfChannelInfo->eBand = BAND_5G;
                break;
            default:
                DBGLOG(P2P, TRACE, ("UNKNOWN Band info from supplicant\n"));
                prRfChannelInfo->eBand = BAND_NULL;
                break;
            }

            /* Iteration. */
            prRfChannelInfo++;
        }
        prMsgScanRequest->u4NumChannel = request->n_channels;

        DBGLOG(P2P, TRACE, ("Finish channel list.\n"));

        /* SSID */
        prSsid = request->ssids;
        prSsidStruct = (P_P2P_SSID_STRUCT_T)prRfChannelInfo;
        if (request->n_ssids) {
            ASSERT((UINT_32)prSsidStruct == (UINT_32)&(prMsgScanRequest->arChannelListInfo[u4Idx]));
            prMsgScanRequest->prSSID = prSsidStruct;
        }

        for (u4Idx = 0; u4Idx < request->n_ssids; u4Idx++) {
            COPY_SSID(prSsidStruct->aucSsid,
                            prSsidStruct->ucSsidLen,
                            request->ssids->ssid,
                            request->ssids->ssid_len);

            prSsidStruct++;
            prSsid++;
        }

        prMsgScanRequest->i4SsidNum = request->n_ssids;

        DBGLOG(P2P, TRACE, ("Finish SSID list:%d.\n", request->n_ssids));

        /* IE BUFFERS */
        prMsgScanRequest->pucIEBuf = (PUINT_8)prSsidStruct;
        if (request->ie_len) {
            kalMemCopy(prMsgScanRequest->pucIEBuf, request->ie, request->ie_len);
            prMsgScanRequest->u4IELen = request->ie_len;
        }

        DBGLOG(P2P, TRACE, ("Finish IE Buffer.\n"));


        mboxSendMsg(prGlueInfo->prAdapter,
                MBOX_ID_0,
                (P_MSG_HDR_T)prMsgScanRequest,
                MSG_SEND_METHOD_BUF);

        i4RetRslt = 0;
    } while (FALSE);

    return i4RetRslt;
} /* mtk_p2p_cfg80211_scan */

int mtk_p2p_cfg80211_set_wiphy_params(
    struct wiphy *wiphy,
    u32 changed
    )
{
    INT_32 i4Rslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = NULL;


    do {
        if (wiphy == NULL) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_set_wiphy_params\n"));
        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (changed & WIPHY_PARAM_RETRY_SHORT) {
            // TODO:
            DBGLOG(P2P, TRACE, ("The RETRY short param is changed.\n"));
        }

        if (changed & WIPHY_PARAM_RETRY_LONG) {
            // TODO:
            DBGLOG(P2P, TRACE, ("The RETRY long param is changed.\n"));
        }


        if (changed & WIPHY_PARAM_FRAG_THRESHOLD) {
            // TODO:
            DBGLOG(P2P, TRACE, ("The RETRY fragmentation threshold is changed.\n"));
        }

        if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
            // TODO:
            DBGLOG(P2P, TRACE, ("The RETRY RTS threshold is changed.\n"));
        }

        if (changed & WIPHY_PARAM_COVERAGE_CLASS) {
            // TODO:
            DBGLOG(P2P, TRACE, ("The coverage class is changed???\n"));
        }

        i4Rslt = 0;
    } while (FALSE);




    return i4Rslt;
} /* mtk_p2p_cfg80211_set_wiphy_params */



int
mtk_p2p_cfg80211_join_ibss(
    struct wiphy *wiphy,
    struct net_device *dev,
    struct cfg80211_ibss_params *params
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    // not implemented yet

    return -EINVAL;
}

int
mtk_p2p_cfg80211_leave_ibss(
    struct wiphy *wiphy,
    struct net_device *dev
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    // not implemented yet

    return -EINVAL;
}

int
mtk_p2p_cfg80211_set_txpower(
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    struct wireless_dev *wdev,
#endif
    enum nl80211_tx_power_setting type,
    int mbm
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    // not implemented yet

    return -EINVAL;
}

int
mtk_p2p_cfg80211_get_txpower(
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	struct wireless_dev *wdev,
#endif
    int *dbm
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    // not implemented yet

    return -EINVAL;
}

int
mtk_p2p_cfg80211_set_power_mgmt(
    struct wiphy *wiphy,
    struct net_device *dev,
    bool enabled,
    int timeout
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    // not implemented yet
    // TODO: for JB.

    return -EINVAL;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&& Add for ICS Wi-Fi Direct Support. &&&&&&&&&&&&&&&&&&&&&&&
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
int
mtk_p2p_cfg80211_start_ap (
    struct wiphy *wiphy,
    struct net_device *dev,
    struct cfg80211_ap_settings *settings
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;
    P_MSG_P2P_BEACON_UPDATE_T prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T)NULL;
    P_MSG_P2P_START_AP_T prP2pStartAPMsg = (P_MSG_P2P_START_AP_T)NULL;
    PUINT_8 pucBuffer = (PUINT_8)NULL;
    UINT_8 ucRoleIdx = 0;
//    P_IE_SSID_T prSsidIE = (P_IE_SSID_T)NULL;

    do {
        if ((wiphy == NULL) || (settings == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_start_ap.\n"));
        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, 
                                        dev,
                                        &ucRoleIdx) < 0) {
            break;
        }

        prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T)cnmMemAlloc(
                                                                prGlueInfo->prAdapter,
                                                                RAM_TYPE_MSG,
                                                                (sizeof(MSG_P2P_BEACON_UPDATE_T) + settings->beacon.head_len + settings->beacon.tail_len));

        if (prP2pBcnUpdateMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }


        prP2pBcnUpdateMsg->ucRoleIndex = ucRoleIdx;
        prP2pBcnUpdateMsg->rMsgHdr.eMsgId = MID_MNY_P2P_BEACON_UPDATE;
        pucBuffer = prP2pBcnUpdateMsg->aucBuffer;

        if (settings->beacon.head_len != 0) {
            kalMemCopy(pucBuffer, settings->beacon.head, settings->beacon.head_len);

            prP2pBcnUpdateMsg->u4BcnHdrLen = settings->beacon.head_len;

            prP2pBcnUpdateMsg->pucBcnHdr = pucBuffer;

            pucBuffer = (PUINT_8)((UINT_32)pucBuffer + (UINT_32)settings->beacon.head_len);
        }
        else {
            prP2pBcnUpdateMsg->u4BcnHdrLen = 0;

            prP2pBcnUpdateMsg->pucBcnHdr = NULL;
        }

        if (settings->beacon.tail_len != 0) {
            UINT_8 ucLen = settings->beacon.tail_len;

            prP2pBcnUpdateMsg->pucBcnBody = pucBuffer;

            /*Add TIM IE*/
            // IEEE 802.11 2007 - 7.3.2.6
            TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
            TIM_IE(pucBuffer)->ucLength = (3 + MAX_LEN_TIM_PARTIAL_BMP)/*((u4N2 - u4N1) + 4)*/; // NOTE: fixed PVB length (AID is allocated from 8 ~ 15 only)
            TIM_IE(pucBuffer)->ucDTIMCount = 0/*prBssInfo->ucDTIMCount*/; // will be overwrite by FW
            TIM_IE(pucBuffer)->ucDTIMPeriod = 1;
            TIM_IE(pucBuffer)->ucBitmapControl = 0/*ucBitmapControl | (UINT_8)u4N1*/; // will be overwrite by FW
            ucLen += IE_SIZE(pucBuffer);
            pucBuffer += IE_SIZE(pucBuffer);

            kalMemCopy(pucBuffer, settings->beacon.tail, settings->beacon.tail_len);

            prP2pBcnUpdateMsg->u4BcnBodyLen = ucLen;
        }
        else {
            prP2pBcnUpdateMsg->u4BcnBodyLen = 0;

            prP2pBcnUpdateMsg->pucBcnBody = NULL;
        }


        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prP2pBcnUpdateMsg,
                            MSG_SEND_METHOD_BUF);


        prP2pStartAPMsg = (P_MSG_P2P_START_AP_T)cnmMemAlloc(
                                                        prGlueInfo->prAdapter,
                                                        RAM_TYPE_MSG,
                                                        sizeof(MSG_P2P_START_AP_T));

        if (prP2pStartAPMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prP2pStartAPMsg->rMsgHdr.eMsgId = MID_MNY_P2P_START_AP;

        prP2pStartAPMsg->fgIsPrivacy = settings->privacy;

        prP2pStartAPMsg->u4BcnInterval = settings->beacon_interval;

        prP2pStartAPMsg->u4DtimPeriod = settings->dtim_period;

        /* Copy NO SSID. */
        prP2pStartAPMsg->ucHiddenSsidType = settings->hidden_ssid;

        prP2pStartAPMsg->ucRoleIdx = ucRoleIdx;

        COPY_SSID(prP2pStartAPMsg->aucSsid,
                        prP2pStartAPMsg->u2SsidLen,
                        settings->ssid,
                        settings->ssid_len);

        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prP2pStartAPMsg,
                            MSG_SEND_METHOD_BUF);

        i4Rslt = 0;

    } while (FALSE);

    return i4Rslt;


/////////////////////////
    /**
         * struct cfg80211_ap_settings - AP configuration
         *
         * Used to configure an AP interface.
         *
         * @beacon: beacon data
         * @beacon_interval: beacon interval
         * @dtim_period: DTIM period
         * @ssid: SSID to be used in the BSS (note: may be %NULL if not provided from
         *      user space)
         * @ssid_len: length of @ssid
         * @hidden_ssid: whether to hide the SSID in Beacon/Probe Response frames
         * @crypto: crypto settings
         * @privacy: the BSS uses privacy
         * @auth_type: Authentication type (algorithm)
         * @inactivity_timeout: time in seconds to determine station's inactivity.
         */
//        struct cfg80211_ap_settings {
//                struct cfg80211_beacon_data beacon;
//
//                int beacon_interval, dtim_period;
//                const u8 *ssid;
//                size_t ssid_len;
//                enum nl80211_hidden_ssid hidden_ssid;
//                struct cfg80211_crypto_settings crypto;
//                bool privacy;
//                enum nl80211_auth_type auth_type;
//                int inactivity_timeout;
//        };
////////////////////

    return i4Rslt;
} /* mtk_p2p_cfg80211_start_ap */



int
mtk_p2p_cfg80211_change_beacon (
    struct wiphy *wiphy,
    struct net_device *dev,
    struct cfg80211_beacon_data *info
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;
    P_MSG_P2P_BEACON_UPDATE_T prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T)NULL;
    PUINT_8 pucBuffer = (PUINT_8)NULL;
    UINT_8 ucRoleIdx = 0;

    do {
        if ((wiphy == NULL) || (info == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_change_beacon.\n"));
        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, 
                                        dev,
                                        &ucRoleIdx) < 0) {
            break;
        }


        if ((info->head_len != 0) || (info->tail_len != 0)) {
            prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T)cnmMemAlloc(
                                                                prGlueInfo->prAdapter,
                                                                RAM_TYPE_MSG,
                                                                (sizeof(MSG_P2P_BEACON_UPDATE_T) + info->head_len + info->tail_len));


            if (prP2pBcnUpdateMsg == NULL) {
                ASSERT(FALSE);
                i4Rslt = -ENOMEM;
                break;
            }


            prP2pBcnUpdateMsg->ucRoleIndex = ucRoleIdx;
            prP2pBcnUpdateMsg->rMsgHdr.eMsgId = MID_MNY_P2P_BEACON_UPDATE;
            pucBuffer = prP2pBcnUpdateMsg->aucBuffer;

            if (info->head_len != 0) {
                kalMemCopy(pucBuffer, info->head, info->head_len);

                prP2pBcnUpdateMsg->u4BcnHdrLen = info->head_len;

                prP2pBcnUpdateMsg->pucBcnHdr = pucBuffer;

                pucBuffer = (PUINT_8)((UINT_32)pucBuffer + (UINT_32)info->head_len);
            }
            else {
                prP2pBcnUpdateMsg->u4BcnHdrLen = 0;

                prP2pBcnUpdateMsg->pucBcnHdr = NULL;
            }

            if (info->tail_len != 0) {
                UINT_8 ucLen = info->tail_len;

                prP2pBcnUpdateMsg->pucBcnBody = pucBuffer;

                /*Add TIM IE*/
                // IEEE 802.11 2007 - 7.3.2.6
                TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
                TIM_IE(pucBuffer)->ucLength = (3 + MAX_LEN_TIM_PARTIAL_BMP)/*((u4N2 - u4N1) + 4)*/; // NOTE: fixed PVB length (AID is allocated from 8 ~ 15 only)
                TIM_IE(pucBuffer)->ucDTIMCount = 0/*prBssInfo->ucDTIMCount*/; // will be overwrite by FW
                TIM_IE(pucBuffer)->ucDTIMPeriod = 1;
                TIM_IE(pucBuffer)->ucBitmapControl = 0/*ucBitmapControl | (UINT_8)u4N1*/; // will be overwrite by FW
                ucLen += IE_SIZE(pucBuffer);
                pucBuffer += IE_SIZE(pucBuffer);

                kalMemCopy(pucBuffer, info->tail, info->tail_len);

                prP2pBcnUpdateMsg->u4BcnBodyLen = ucLen;
            }
            else {
                prP2pBcnUpdateMsg->u4BcnBodyLen = 0;
                prP2pBcnUpdateMsg->pucBcnBody = NULL;
            }

            mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prP2pBcnUpdateMsg,
                            MSG_SEND_METHOD_BUF);
            
        }



        // TODO: Probe Rsp, Assoc Rsp, Beacon IE update.

////////////////////////////
//struct cfg80211_beacon_data {
//        const u8 *head, *tail;
//        const u8 *beacon_ies;
//        const u8 *proberesp_ies;
//        const u8 *assocresp_ies;
//        const u8 *probe_resp;

//        size_t head_len, tail_len;
//        size_t beacon_ies_len;
//        size_t proberesp_ies_len;
//        size_t assocresp_ies_len;
//        size_t probe_resp_len;
//};

////////////////////////////

    } while (FALSE);

    return i4Rslt;
} /* mtk_p2p_cfg80211_change_beacon */

#else
int
mtk_p2p_cfg80211_add_set_beacon (
    struct wiphy *wiphy,
    struct net_device *dev,
    struct beacon_parameters *info
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;
    P_MSG_P2P_BEACON_UPDATE_T prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T)NULL;
    P_MSG_P2P_START_AP_T prP2pStartAPMsg = (P_MSG_P2P_START_AP_T)NULL;
    PUINT_8 pucBuffer = (PUINT_8)NULL;
    P_IE_SSID_T prSsidIE = (P_IE_SSID_T)NULL;
    UINT_8 ucRoleIdx = 0;

    do {
        if ((wiphy == NULL) || (info == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_add_set_beacon.\n"));
        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2pInfo, 
                                        dev,
                                        &ucRoleIdx) < 0) {
            break;
        }

        prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T)cnmMemAlloc(
                                                                prGlueInfo->prAdapter,
                                                                RAM_TYPE_MSG,
                                                                (sizeof(MSG_P2P_BEACON_UPDATE_T) + info->head_len + info->tail_len));

        if (prP2pBcnUpdateMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prP2pBcnUpdateMsg->ucRoleIndex = ucRoleIdx;
        prP2pBcnUpdateMsg->rMsgHdr.eMsgId = MID_MNY_P2P_BEACON_UPDATE;
        pucBuffer = prP2pBcnUpdateMsg->aucBuffer;

        if (info->head_len != 0) {
            kalMemCopy(pucBuffer, info->head, info->head_len);

            prP2pBcnUpdateMsg->u4BcnHdrLen = info->head_len;

            prP2pBcnUpdateMsg->pucBcnHdr = pucBuffer;

            pucBuffer = (PUINT_8)((UINT_32)pucBuffer + (UINT_32)info->head_len);
        }
        else {
            prP2pBcnUpdateMsg->u4BcnHdrLen = 0;

            prP2pBcnUpdateMsg->pucBcnHdr = NULL;
        }

        if (info->tail_len != 0) {
            UINT_8 ucLen = info->tail_len;

            prP2pBcnUpdateMsg->pucBcnBody = pucBuffer;

            /*Add TIM IE*/
            // IEEE 802.11 2007 - 7.3.2.6
            TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
            TIM_IE(pucBuffer)->ucLength = (3 + MAX_LEN_TIM_PARTIAL_BMP)/*((u4N2 - u4N1) + 4)*/; // NOTE: fixed PVB length (AID is allocated from 8 ~ 15 only)
            TIM_IE(pucBuffer)->ucDTIMCount = 0/*prBssInfo->ucDTIMCount*/; // will be overwrite by FW
            TIM_IE(pucBuffer)->ucDTIMPeriod = 1;
            TIM_IE(pucBuffer)->ucBitmapControl = 0/*ucBitmapControl | (UINT_8)u4N1*/; // will be overwrite by FW
            ucLen += IE_SIZE(pucBuffer);
            pucBuffer += IE_SIZE(pucBuffer);

            kalMemCopy(pucBuffer, info->tail, info->tail_len);

            prP2pBcnUpdateMsg->u4BcnBodyLen = ucLen;
        }
        else {
            prP2pBcnUpdateMsg->u4BcnBodyLen = 0;

            prP2pBcnUpdateMsg->pucBcnBody = NULL;
        }


        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prP2pBcnUpdateMsg,
                            MSG_SEND_METHOD_BUF);


        prP2pStartAPMsg = (P_MSG_P2P_START_AP_T)cnmMemAlloc(
                                                        prGlueInfo->prAdapter,
                                                        RAM_TYPE_MSG,
                                                        sizeof(MSG_P2P_START_AP_T));

        if (prP2pStartAPMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prP2pStartAPMsg->rMsgHdr.eMsgId = MID_MNY_P2P_START_AP;

        prP2pStartAPMsg->fgIsPrivacy = FALSE;

        prP2pStartAPMsg->u4BcnInterval = info->interval;

        prP2pStartAPMsg->u4DtimPeriod = info->dtim_period;

        /* Copy NO SSID. */
        prP2pStartAPMsg->ucHiddenSsidType = ENUM_HIDDEN_SSID_NONE;

#if 0
        if (info->head_len > OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem)) {
            P_WLAN_BEACON_FRAME_T prWlanBcnFrame = info->head;

            prSsidIE = (P_IE_HDR_T)p2pFuncGetSpecIE(prGlueInfo->prAdapter,
                                    (PUINT_8)prWlanBcnFrame->aucInfoElem,
                                    (info->head_len - OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem)),
                                    ELEM_ID_SSID,
                                    NULL);

            kalMemCopy(prP2pStartAPMsg->aucSsid, SSID_IE(prSsidIE)->aucSSID, IE_LEN(prSsidIE));

        }
#endif

        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prP2pStartAPMsg,
                            MSG_SEND_METHOD_BUF);

        i4Rslt = 0;

    } while (FALSE);

    return i4Rslt;
}
/* mtk_p2p_cfg80211_add_set_beacon */
#endif

int
mtk_p2p_cfg80211_stop_ap (
    struct wiphy *wiphy,
    struct net_device *dev
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;
    P_MSG_P2P_SWITCH_OP_MODE_T prP2pSwitchMode = (P_MSG_P2P_SWITCH_OP_MODE_T)NULL;
    UINT_8 ucRoleIdx = 0;

    do {
        if (wiphy == NULL) {
            break;
        }


        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_stop_ap.\n"));
        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, 
                                        dev,
                                        &ucRoleIdx) < 0) {
            break;
        }

        // Switch OP MOde.
        prP2pSwitchMode = cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_SWITCH_OP_MODE_T));

        if (prP2pSwitchMode == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prP2pSwitchMode->rMsgHdr.eMsgId = MID_MNY_P2P_STOP_AP;
        prP2pSwitchMode->ucRoleIdx = ucRoleIdx;

        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prP2pSwitchMode,
                            MSG_SEND_METHOD_BUF);

        i4Rslt = 0;
    } while (FALSE);


    return i4Rslt;
} /* mtk_p2p_cfg80211_stop_ap */

// TODO:
int
mtk_p2p_cfg80211_deauth (
    struct wiphy *wiphy,
    struct net_device *dev,
    struct cfg80211_deauth_request *req
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
    , void *cookie
#endif
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    // not implemented yet
    DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_deauth.\n"));

    return -EINVAL;
} /* mtk_p2p_cfg80211_deauth */


// TODO:
int
mtk_p2p_cfg80211_disassoc (
    struct wiphy *wiphy,
    struct net_device *dev,
    struct cfg80211_disassoc_request *req
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
    , void *cookie
#endif
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_disassoc.\n"));

    // not implemented yet

    return -EINVAL;
} /* mtk_p2p_cfg80211_disassoc */


int
mtk_p2p_cfg80211_remain_on_channel (
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)    
    struct wireless_dev *wdev,
#else
    struct net_device *dev,
#endif
    struct ieee80211_channel *chan,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)    
    enum nl80211_channel_type channel_type,
#endif
    unsigned int duration,
    u64 *cookie
    )
{
    INT_32 i4Rslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    P_GL_P2P_INFO_T prGlueP2pInfo = (P_GL_P2P_INFO_T)NULL;
    P_MSG_P2P_CHNL_REQUEST_T prMsgChnlReq = (P_MSG_P2P_CHNL_REQUEST_T)NULL;


    do {
        if ((wiphy == NULL) ||
                //(dev == NULL) ||
                (chan == NULL) ||
                (cookie == NULL)) {
            break;
        }

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
        prGlueP2pInfo = prGlueInfo->prP2PInfo;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)  
        if (prGlueP2pInfo->prDevHandler != dev) {
            ASSERT(FALSE);
            // TODO: Group Interface also has remain on channel requirement?
            DBGLOG(P2P, WARN, ("Group Interface also has remain on channel requirement\n"));
            break;
        }
#endif

        *cookie = prGlueP2pInfo->u8Cookie++;

        prMsgChnlReq = cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_CHNL_REQUEST_T));

        if (prMsgChnlReq == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_remain_on_channel:%d\n", (INT_32)*cookie));

        prMsgChnlReq->rMsgHdr.eMsgId = MID_MNY_P2P_CHNL_REQ;
        prMsgChnlReq->u8Cookie = *cookie;
        prMsgChnlReq->u4Duration = duration;
        prMsgChnlReq->eChnlReqType = CH_REQ_TYPE_P2P_LISTEN;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)  
        mtk_p2p_cfg80211func_channel_format_switch(chan,
                                                    channel_type,
                                                    &prMsgChnlReq->rChannelInfo,
                                                    &prMsgChnlReq->eChnlSco);
#else
        mtk_p2p_cfg80211func_channel_format_switch(chan,
                                                    NL80211_CHAN_NO_HT,
                                                    &prMsgChnlReq->rChannelInfo,
                                                    &prMsgChnlReq->eChnlSco);
#endif

        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prMsgChnlReq,
                            MSG_SEND_METHOD_BUF);

        i4Rslt = 0;
    } while (FALSE);


    return i4Rslt;
}
/* mtk_p2p_cfg80211_remain_on_channel */


int
mtk_p2p_cfg80211_cancel_remain_on_channel (
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)    
    struct wireless_dev *wdev,
#else
    struct net_device *dev,
#endif
    u64 cookie
    )
{
    INT_32 i4Rslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    P_GL_P2P_INFO_T prGlueP2pInfo = (P_GL_P2P_INFO_T)NULL;
    P_MSG_P2P_CHNL_ABORT_T prMsgChnlAbort = (P_MSG_P2P_CHNL_ABORT_T)NULL;

    do {
        if ((wiphy == NULL) 
            //|| (dev == NULL)
            ) {
            break;
        }


        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
        prGlueP2pInfo = prGlueInfo->prP2PInfo;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0) 
        if (prGlueP2pInfo->prDevHandler != dev) {
            ASSERT(FALSE);
            // TODO: Group Interface also has remain on channel requirement?
            DBGLOG(P2P, WARN, ("Group Interface also has remain on channel requirement\n"));
            break;
        }
#endif        

        prMsgChnlAbort = cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_CHNL_ABORT_T));

        if (prMsgChnlAbort == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_cancel_remain_on_channel%d\n", (INT_32)cookie));

        prMsgChnlAbort->rMsgHdr.eMsgId = MID_MNY_P2P_CHNL_ABORT;
        prMsgChnlAbort->u8Cookie = cookie;


        mboxSendMsg(prGlueInfo->prAdapter,
                                    MBOX_ID_0,
                                    (P_MSG_HDR_T)prMsgChnlAbort,
                                    MSG_SEND_METHOD_BUF);

        i4Rslt = 0;
    } while (FALSE);

    return i4Rslt;
} /* mtk_p2p_cfg80211_cancel_remain_on_channel */

int
mtk_p2p_cfg80211_mgmt_tx (
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)    
    struct wireless_dev *wdev,
#else
    struct net_device *dev,
#endif
    struct ieee80211_channel *chan,
    bool offchan,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)    
    enum nl80211_channel_type channel_type,
    bool channel_type_valid,
#endif
    unsigned int wait,
    const u8 *buf,
    size_t len,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
    bool no_cck,
    bool dont_wait_for_ack,
#endif
    u64 *cookie
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    P_GL_P2P_INFO_T prGlueP2pInfo = (P_GL_P2P_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;
    P_MSG_P2P_MGMT_TX_REQUEST_T prMsgTxReq = (P_MSG_P2P_MGMT_TX_REQUEST_T)NULL;
    P_MSDU_INFO_T prMgmtFrame = (P_MSDU_INFO_T)NULL;
    PUINT_8 pucFrameBuf = (PUINT_8)NULL;
    PUINT_64 pu8GlCookie = (PUINT_64)NULL;
    UINT_8 ucRoleIdx = 0, ucBssIdx = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)    
    struct net_device *dev = NULL;
#endif    

    do {
        if ((wiphy == NULL) ||
                (buf == NULL) ||
                (len == 0) ||
                //(dev == NULL) ||
                (cookie == NULL)) {
            break;
        }

        //DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_mgmt_tx\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
        prGlueP2pInfo = prGlueInfo->prP2PInfo;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0) 
        dev = wdev->netdev;
#endif
        if (mtk_Netdev_To_RoleIdx(prGlueP2pInfo, dev, &ucRoleIdx) < 0) {
            // Device Interface.
            ucBssIdx = P2P_DEV_BSS_INDEX;
        }
        else {
            ASSERT(ucRoleIdx < KAL_P2P_NUM);
            // Role Interface.
            if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter, ucRoleIdx, &ucBssIdx) < 0) {
                /* Can't find BSS index. */
                break;
            }
        }

        *cookie = prGlueP2pInfo->u8Cookie++;

        /* Channel & Channel Type & Wait time are ignored. */
        prMsgTxReq = cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_MGMT_TX_REQUEST_T));

        if (prMsgTxReq == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        if (offchan) {
            DBGLOG(P2P, TRACE, ("   Off channel TRUE\n"));
            prMsgTxReq->fgIsOffChannel = TRUE;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)  
            mtk_p2p_cfg80211func_channel_format_switch(chan,
                                                        channel_type,
                                                        &prMsgTxReq->rChannelInfo,
                                                        &prMsgTxReq->eChnlExt);
#else
            mtk_p2p_cfg80211func_channel_format_switch(chan,
                                                        NL80211_CHAN_NO_HT,
                                                        &prMsgTxReq->rChannelInfo,
                                                        &prMsgTxReq->eChnlExt);
#endif
            
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)  
            if (!channel_type_valid) {
                prMsgTxReq->eChnlExt = CHNL_EXT_RES;
            }
#endif            
        }
        else {
            prMsgTxReq->fgIsOffChannel = FALSE;
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
        if (no_cck) {
            prMsgTxReq->fgNoneCckRate = TRUE;
        }
        else {
            prMsgTxReq->fgNoneCckRate = FALSE;
        }

        if (dont_wait_for_ack) {
            prMsgTxReq->fgIsWaitRsp = FALSE;
        }
        else {
            prMsgTxReq->fgIsWaitRsp = TRUE;
        }
#endif
        prMgmtFrame = cnmMgtPktAlloc(prGlueInfo->prAdapter, (UINT_32)(len + sizeof(UINT_64) + MAC_TX_RESERVED_FIELD));

        if ((prMsgTxReq->prMgmtMsduInfo = prMgmtFrame) == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prMsgTxReq->u8Cookie = *cookie;
        prMsgTxReq->rMsgHdr.eMsgId = MID_MNY_P2P_MGMT_TX;
        prMsgTxReq->ucBssIdx = ucBssIdx;

        pucFrameBuf = (PUINT_8)((UINT_32)prMgmtFrame->prPacket + MAC_TX_RESERVED_FIELD);

        pu8GlCookie = (PUINT_64)((UINT_32)prMgmtFrame->prPacket + (UINT_32)len + MAC_TX_RESERVED_FIELD);

        kalMemCopy(pucFrameBuf, buf, len);

        *pu8GlCookie = *cookie;

        prMgmtFrame->u2FrameLength = len;

        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prMsgTxReq,
                            MSG_SEND_METHOD_BUF);

        i4Rslt = 0;
    } while (FALSE);

    if ((i4Rslt != 0) && (prMsgTxReq != NULL)) {
        if (prMsgTxReq->prMgmtMsduInfo != NULL) {
            cnmMgtPktFree(prGlueInfo->prAdapter, prMsgTxReq->prMgmtMsduInfo);
        }

        cnmMemFree(prGlueInfo->prAdapter, prMsgTxReq);
    }

    return i4Rslt;
} /* mtk_p2p_cfg80211_mgmt_tx */


int
mtk_p2p_cfg80211_mgmt_tx_cancel_wait (
    struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
    struct wireless_dev *wdev,
#else
    struct net_device *dev,
#endif  
    u64 cookie
    )
{
    INT_32 i4Rslt = -EINVAL;

    return i4Rslt;
} /* mtk_p2p_cfg80211_mgmt_tx_cancel_wait */


int
mtk_p2p_cfg80211_change_bss (
    struct wiphy *wiphy,
    struct net_device *dev,
    struct bss_parameters *params
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));


    switch (params->use_cts_prot) {
    case -1:
        DBGLOG(P2P, TRACE, ("CTS protection no change\n"));
        break;
    case 0:
        DBGLOG(P2P, TRACE, ("CTS protection disable.\n"));
        break;
    case 1:
        DBGLOG(P2P, TRACE, ("CTS protection enable\n"));
        break;
    default:
        DBGLOG(P2P, TRACE, ("CTS protection unknown\n"));
        break;
    }



    switch (params->use_short_preamble) {
    case -1:
        DBGLOG(P2P, TRACE, ("Short prreamble no change\n"));
        break;
    case 0:
        DBGLOG(P2P, TRACE, ("Short prreamble disable.\n"));
        break;
    case 1:
        DBGLOG(P2P, TRACE, ("Short prreamble enable\n"));
        break;
    default:
        DBGLOG(P2P, TRACE, ("Short prreamble unknown\n"));
        break;
    }



#if 0
    // not implemented yet
    p2pFuncChangeBssParam(prGlueInfo->prAdapter,
                        prBssInfo->fgIsProtection,
                        prBssInfo->fgIsShortPreambleAllowed,
                        prBssInfo->fgUseShortSlotTime,
                        // Basic rates
                        // basic rates len
                        // ap isolate
                        // ht opmode.
                        );
#else
    i4Rslt = 0;
#endif

    return i4Rslt;
} /* mtk_p2p_cfg80211_change_bss */



int
mtk_p2p_cfg80211_del_station (
    struct wiphy *wiphy,
    struct net_device *dev,
    u8 *mac
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;
    P_MSG_P2P_CONNECTION_ABORT_T prDisconnectMsg = (P_MSG_P2P_CONNECTION_ABORT_T)NULL;
    UINT_8 aucBcMac[] = BC_MAC_ADDR;
    UINT_8 ucRoleIdx = 0;


    do {
        if ((wiphy == NULL) ||
                    (dev == NULL)) {
            break;
        }

        if (mac == NULL) {
            mac = aucBcMac;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_del_station.\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo,
                        dev,
                        &ucRoleIdx) < 0) {
            break;
        }    

        //prDisconnectMsg = (P_MSG_P2P_CONNECTION_ABORT_T)kalMemAlloc(sizeof(MSG_P2P_CONNECTION_ABORT_T), VIR_MEM_TYPE);
        prDisconnectMsg = (P_MSG_P2P_CONNECTION_ABORT_T)cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_CONNECTION_ABORT_T));

        if (prDisconnectMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prDisconnectMsg->rMsgHdr.eMsgId = MID_MNY_P2P_CONNECTION_ABORT;
        prDisconnectMsg->ucRoleIdx = ucRoleIdx;
        COPY_MAC_ADDR(prDisconnectMsg->aucTargetID, mac);
        prDisconnectMsg->u2ReasonCode = REASON_CODE_UNSPECIFIED;

        mboxSendMsg(prGlueInfo->prAdapter,
                        MBOX_ID_0,
                        (P_MSG_HDR_T)prDisconnectMsg,
                        MSG_SEND_METHOD_BUF);

        i4Rslt = 0;
    } while (FALSE);

    return i4Rslt;

} /* mtk_p2p_cfg80211_del_station */


int 
mtk_p2p_cfg80211_connect (
    struct wiphy *wiphy,
    struct net_device *dev,
    struct cfg80211_connect_params *sme
    )
{
    INT_32 i4Rslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = NULL;
    P_MSG_P2P_CONNECTION_REQUEST_T prConnReqMsg = (P_MSG_P2P_CONNECTION_REQUEST_T)NULL;
    UINT_8 ucRoleIdx = 0;


    do {
        if ((wiphy == NULL) ||
                (dev == NULL) ||
                (sme == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_connect.\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo,
                        dev,
                        &ucRoleIdx) < 0) {
            break;
        }        

        prConnReqMsg = (P_MSG_P2P_CONNECTION_REQUEST_T)cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, (sizeof(MSG_P2P_CONNECTION_REQUEST_T) + sme->ie_len));

        if (prConnReqMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prConnReqMsg->rMsgHdr.eMsgId = MID_MNY_P2P_CONNECTION_REQ;
        prConnReqMsg->ucRoleIdx = ucRoleIdx;

        COPY_SSID(prConnReqMsg->rSsid.aucSsid,
                                prConnReqMsg->rSsid.ucSsidLen,
                                sme->ssid,
                                sme->ssid_len);

        COPY_MAC_ADDR(prConnReqMsg->aucBssid, sme->bssid);

        DBGLOG(P2P, TRACE, ("Assoc Req IE Buffer Length:%d\n", sme->ie_len));
        kalMemCopy(prConnReqMsg->aucIEBuf, sme->ie, sme->ie_len);
        prConnReqMsg->u4IELen = sme->ie_len;

        kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_NONE);
        
        if (sme->crypto.n_ciphers_pairwise) {
            switch (sme->crypto.ciphers_pairwise[0]) {
            case WLAN_CIPHER_SUITE_WEP40:
            case WLAN_CIPHER_SUITE_WEP104:
                kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_WEP40);
                break;
            case WLAN_CIPHER_SUITE_TKIP:
                kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_TKIP);
                break;
            case WLAN_CIPHER_SUITE_CCMP:
            case WLAN_CIPHER_SUITE_AES_CMAC:
                kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_CCMP);
                break;
            default:
                DBGLOG(REQ, WARN, ("invalid cipher pairwise (%d)\n",
                       sme->crypto.ciphers_pairwise[0]));
                return -EINVAL;
            }
        }
        
        mtk_p2p_cfg80211func_channel_format_switch(sme->channel,
                                                    NL80211_CHAN_NO_HT,
                                                    &prConnReqMsg->rChannelInfo,
                                                    &prConnReqMsg->eChnlSco);

        mboxSendMsg(prGlueInfo->prAdapter,
                        MBOX_ID_0,
                        (P_MSG_HDR_T)prConnReqMsg,
                        MSG_SEND_METHOD_BUF);


        i4Rslt = 0;
    } while (FALSE);

    return i4Rslt;
} /* mtk_p2p_cfg80211_connect */

int 
mtk_p2p_cfg80211_disconnect (
    struct wiphy *wiphy,
    struct net_device *dev,
    u16 reason_code
    )
{
    INT_32 i4Rslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = NULL;
    P_MSG_P2P_CONNECTION_ABORT_T prDisconnMsg = (P_MSG_P2P_CONNECTION_ABORT_T)NULL;
    UINT_8 aucBCAddr[] = BC_MAC_ADDR;
    UINT_8 ucRoleIdx = 0;

    do {
        if ((wiphy == NULL) ||
                (dev == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_disconnect.\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo,
                        dev,
                        &ucRoleIdx) < 0) {
            break;
        }  

//        prDisconnMsg = (P_MSG_P2P_CONNECTION_ABORT_T)kalMemAlloc(sizeof(P_MSG_P2P_CONNECTION_ABORT_T), VIR_MEM_TYPE);
        prDisconnMsg = (P_MSG_P2P_CONNECTION_ABORT_T)cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_CONNECTION_ABORT_T));

        if (prDisconnMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prDisconnMsg->rMsgHdr.eMsgId = MID_MNY_P2P_CONNECTION_ABORT;
        prDisconnMsg->ucRoleIdx = ucRoleIdx;
        prDisconnMsg->u2ReasonCode = reason_code;
        prDisconnMsg->fgSendDeauth = TRUE;
        COPY_MAC_ADDR(prDisconnMsg->aucTargetID, aucBCAddr);

        mboxSendMsg(prGlueInfo->prAdapter,
                                MBOX_ID_0,
                                (P_MSG_HDR_T)prDisconnMsg,
                                MSG_SEND_METHOD_BUF);

        i4Rslt = 0;
    } while (FALSE);

    return i4Rslt;
} /* mtk_p2p_cfg80211_disconnect */


int
mtk_p2p_cfg80211_change_iface (
    IN struct wiphy *wiphy,
    IN struct net_device *ndev,
    IN enum nl80211_iftype type,
    IN u32 *flags,
    IN struct vif_params *params
    )
{
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    INT_32 i4Rslt = -EINVAL;
    P_MSG_P2P_SWITCH_OP_MODE_T prSwitchModeMsg = (P_MSG_P2P_SWITCH_OP_MODE_T)NULL;
    UINT_8 ucRoleIdx = 0;

    do {
        if ((wiphy == NULL) ||
                (ndev == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_change_iface.\n"));

        if (ndev->ieee80211_ptr) {
            ndev->ieee80211_ptr->iftype = type;
        }

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, ndev, &ucRoleIdx) != 0) {
            DBGLOG(P2P, TRACE, ("Device Interface no need to change interface type.\n"));
            return 0;
        }
        
        // Switch OP MOde.
        prSwitchModeMsg = (P_MSG_P2P_SWITCH_OP_MODE_T)cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_SWITCH_OP_MODE_T));

        if (prSwitchModeMsg == NULL) {
            ASSERT(FALSE);
            i4Rslt = -ENOMEM;
            break;
        }

        prSwitchModeMsg->rMsgHdr.eMsgId = MID_MNY_P2P_FUN_SWITCH;
        prSwitchModeMsg->ucRoleIdx = ucRoleIdx;

        switch (type) {
        case NL80211_IFTYPE_P2P_CLIENT:
            DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_P2P_CLIENT.\n"));
        case NL80211_IFTYPE_STATION:
            if (type == NL80211_IFTYPE_STATION) {
                DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_STATION.\n"));
            }
            prSwitchModeMsg->eOpMode = OP_MODE_INFRASTRUCTURE;
            break;
        case NL80211_IFTYPE_AP:
            DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_AP.\n"));
        case NL80211_IFTYPE_P2P_GO:
            if (type == NL80211_IFTYPE_P2P_GO) {
                DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_P2P_GO not AP.\n"));
            }
            prSwitchModeMsg->eOpMode = OP_MODE_ACCESS_POINT;
            break;
        default:
            DBGLOG(P2P, TRACE, ("Other type :%d .\n", type));
            prSwitchModeMsg->eOpMode = OP_MODE_P2P_DEVICE;
            break;
        }


        mboxSendMsg(prGlueInfo->prAdapter,
                            MBOX_ID_0,
                            (P_MSG_HDR_T)prSwitchModeMsg,
                            MSG_SEND_METHOD_BUF);

        i4Rslt = 0;

    } while (FALSE);

    return i4Rslt;

} /* mtk_p2p_cfg80211_change_iface */


int
mtk_p2p_cfg80211_set_channel (
    IN struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    struct cfg80211_chan_def *chandef
#else
    IN struct net_device *dev,
    IN struct ieee80211_channel *chan,
    IN enum nl80211_channel_type channel_type
#endif
)
{
    INT_32 i4Rslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;
    RF_CHANNEL_INFO_T rRfChnlInfo;
    UINT_8 ucRoleIdx = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    struct net_device *dev = (struct net_device *)wiphy_dev(wiphy);
#endif

    do {
        if ((wiphy == NULL) ||
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)            
            (chandef == NULL)
#else
            (dev == NULL) ||
            (chan == NULL)
#endif
            ) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_set_channel.\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)   
        mtk_p2p_cfg80211func_channel_format_switch(chan,
                                        channel_type,
                                        &rRfChnlInfo,
                                        NULL);

#else
        mtk_p2p_cfg80211func_channel_format_switch(chandef->chan,
                                        chandef->width,
                                        &rRfChnlInfo,
                                        NULL);
#endif
        if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, 
                                        dev,
                                        &ucRoleIdx) < 0) {
            break;
        }

        p2pFuncSetChannel(prGlueInfo->prAdapter, ucRoleIdx, &rRfChnlInfo);

        i4Rslt = 0;
    }
while (FALSE);

    return i4Rslt;

}
/* mtk_p2p_cfg80211_set_channel */

int
mtk_p2p_cfg80211_set_bitrate_mask (
    IN struct wiphy *wiphy,
    IN struct net_device *dev,
    IN const u8 *peer,
    IN const struct cfg80211_bitrate_mask *mask
    )
{
    INT_32 i4Rslt = -EINVAL;
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;

    do {
        if ((wiphy == NULL) ||
                (dev == NULL) ||
                (mask == NULL)) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_set_bitrate_mask\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

        // TODO: Set bitrate mask of the peer?

        i4Rslt = 0;
    }
while (FALSE);

    return i4Rslt;
} /* mtk_p2p_cfg80211_set_bitrate_mask */


void
mtk_p2p_cfg80211_mgmt_frame_register (
    IN struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    struct wireless_dev *wdev,
#else
    IN struct net_device *dev,
#endif  
    IN u16 frame_type,
    IN bool reg
    )
{
#if 0
    P_MSG_P2P_MGMT_FRAME_REGISTER_T prMgmtFrameRegister = (P_MSG_P2P_MGMT_FRAME_REGISTER_T)NULL;
#endif
    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T)NULL;

    do {
        if ((wiphy == NULL)
            //|| (dev == NULL)
            ) {
            break;
        }

        DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_mgmt_frame_register\n"));

        prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));


        switch (frame_type) {
        case MAC_FRAME_PROBE_REQ:
            if (reg) {
                prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter |= PARAM_PACKET_FILTER_PROBE_REQ;
                DBGLOG(P2P, TRACE, ("Open packet filer probe request\n"));
            }
            else {
                prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter &= ~PARAM_PACKET_FILTER_PROBE_REQ;
                DBGLOG(P2P, TRACE, ("Close packet filer probe request\n"));
            }
            break;
        case MAC_FRAME_ACTION:
            if (reg) {
                prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter |= PARAM_PACKET_FILTER_ACTION_FRAME;
                DBGLOG(P2P, TRACE, ("Open packet filer action frame.\n"));
            }
            else {
                prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter &= ~PARAM_PACKET_FILTER_ACTION_FRAME;
                DBGLOG(P2P, TRACE, ("Close packet filer action frame.\n"));
            }
            break;
        default:
                DBGLOG(P2P, ERROR, ("Ask frog to add code for mgmt:%x\n", frame_type));
                break;
        }

        

        prGlueInfo->u4Flag |= GLUE_FLAG_FRAME_FILTER;

        /* wake up main thread */
        wake_up_interruptible(&prGlueInfo->waitq);

        if (in_interrupt()) {
            DBGLOG(P2P, TRACE, ("It is in interrupt level\n"));
        }



#if 0


        prMgmtFrameRegister = (P_MSG_P2P_MGMT_FRAME_REGISTER_T)cnmMemAlloc(prGlueInfo->prAdapter, 
                                                                    RAM_TYPE_MSG, 
                                                                    sizeof(MSG_P2P_MGMT_FRAME_REGISTER_T));

        if (prMgmtFrameRegister == NULL) {
            ASSERT(FALSE);
            break;
        }

        prMgmtFrameRegister->rMsgHdr.eMsgId = MID_MNY_P2P_MGMT_FRAME_REGISTER;

        prMgmtFrameRegister->u2FrameType = frame_type;
        prMgmtFrameRegister->fgIsRegister = reg;

        mboxSendMsg(prGlueInfo->prAdapter,
                                    MBOX_ID_0,
                                    (P_MSG_HDR_T)prMgmtFrameRegister,
                                    MSG_SEND_METHOD_BUF);

#endif

    } while (FALSE);


    return;
} /* mtk_p2p_cfg80211_mgmt_frame_register */


BOOLEAN
mtk_p2p_cfg80211func_channel_format_switch (
    IN struct ieee80211_channel *channel,
    IN enum nl80211_channel_type channel_type,
    IN P_RF_CHANNEL_INFO_T prRfChnlInfo,
    IN P_ENUM_CHNL_EXT_T prChnlSco
    )
{
    BOOLEAN fgIsValid = FALSE;

    do {
        if (channel == NULL) {
            break;
        }

        if (prRfChnlInfo) {
            prRfChnlInfo->ucChannelNum = nicFreq2ChannelNum(channel->center_freq * 1000);

            switch (channel->band) {
            case IEEE80211_BAND_2GHZ:
                prRfChnlInfo->eBand = BAND_2G4;
                break;
            case IEEE80211_BAND_5GHZ:
                prRfChnlInfo->eBand = BAND_5G;
                break;
            default:
                prRfChnlInfo->eBand = BAND_2G4;
                break;
            }
        
        }

        
        if (prChnlSco) {
            
            switch (channel_type) {
            case NL80211_CHAN_NO_HT:
                *prChnlSco = CHNL_EXT_SCN;
                break;
            case NL80211_CHAN_HT20:
                *prChnlSco = CHNL_EXT_SCN;
                break;
            case NL80211_CHAN_HT40MINUS:
                *prChnlSco = CHNL_EXT_SCA;
                break;
            case NL80211_CHAN_HT40PLUS:
                *prChnlSco = CHNL_EXT_SCB;
                break;
            default:
                ASSERT(FALSE);
                *prChnlSco = CHNL_EXT_SCN;
                break;
            }
        }

        fgIsValid = TRUE;
        
    } while (FALSE);

    return fgIsValid;
}
/* mtk_p2p_cfg80211func_channel_format_switch */


#if CONFIG_NL80211_TESTMODE
int mtk_p2p_cfg80211_testmode_cmd(
    IN struct wiphy *wiphy,
    IN void *data,
    IN int len
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    P_NL80211_DRIVER_TEST_PARAMS prParams = (P_NL80211_DRIVER_TEST_PARAMS)NULL;
    BOOLEAN fgIsValid = FALSE;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_testmode_cmd\n"));
    
    if(data && len)
        prParams = (P_NL80211_DRIVER_TEST_PARAMS)data;

    if(prParams->index >> 24 == 0x01) { 
        /* New version */
        prParams->index = prParams->index & ~ BITS(24,31);
    }
    else {  
        /* Old version*/
        mtk_p2p_cfg80211_testmode_p2p_sigma_pre_cmd(wiphy, data, len); 
    }

    /* Clear the version byte */
    prParams->index = prParams->index & ~ BITS(24,31);

    if(prParams){
        switch(prParams->index){
            case 1: /* P2P Simga */
                if(mtk_p2p_cfg80211_testmode_p2p_sigma_cmd(wiphy, data, len))
                    fgIsValid = TRUE;
                break;
            case 2: /* WFD */
                break;
            case 3: /* Hotspot Client Management */
                if(mtk_p2p_cfg80211_testmode_hotspot_block_list_cmd(wiphy, data, len))
                    fgIsValid = TRUE;
                break;
            default:
                fgIsValid = TRUE;
                break;
        }
    }

    return fgIsValid;

}


int mtk_p2p_cfg80211_testmode_p2p_sigma_pre_cmd(
    IN struct wiphy *wiphy,
    IN void *data,
    IN int len
    )
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    NL80211_DRIVER_TEST_PRE_PARAMS rParams;
    P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T)NULL;
    //P_P2P_CONNECTION_SETTINGS_T prP2pConnSettings = (P_P2P_CONNECTION_SETTINGS_T)NULL;
    UINT_32 index_mode;
    UINT_32 index;
    INT_32  value;
    int     status = 0;
    UINT_32 u4Leng;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    kalMemZero(&rParams, sizeof(NL80211_DRIVER_TEST_PRE_PARAMS));

    prP2pSpecificBssInfo = prGlueInfo->prAdapter->rWifiVar.prP2pSpecificBssInfo;
    //prP2pConnSettings = prGlueInfo->prAdapter->rWifiVar.prP2PConnSettings;

    DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_testmode_cmd\n"));

    if(data && len)
        memcpy(&rParams, data, len);

    DBGLOG(P2P, TRACE, ("NL80211_ATTR_TESTDATA,idx_mode=%d idx=%d value=%lu\n",
        (INT_16)rParams.idx_mode, (INT_16)rParams.idx, rParams.value));

    index_mode = rParams.idx_mode;
    index = rParams.idx;
    value = rParams.value;

    switch (index) {
      case 0: /* Listen CH */
          break;
      case 1: /* P2p mode */
          break;
      case 4: /* Noa duration */
          prP2pSpecificBssInfo->rNoaParam.u4NoaDurationMs = value;
          // only to apply setting when setting NOA count
          //status = mtk_p2p_wext_set_noa_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rNoaParam);
          break;
      case 5: /* Noa interval */
          prP2pSpecificBssInfo->rNoaParam.u4NoaIntervalMs = value;
          // only to apply setting when setting NOA count
          //status = mtk_p2p_wext_set_noa_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rNoaParam);
          break;
      case 6: /* Noa count */
          prP2pSpecificBssInfo->rNoaParam.u4NoaCount = value;
          //status = mtk_p2p_wext_set_noa_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rNoaParam);
          break;
      case 100: /* Oper CH */
          // 20110920 - frog: User configurations are placed in ConnSettings.
          // prP2pConnSettings->ucOperatingChnl = value;
          break;
      case 101: /* Local config Method, for P2P SDK */
          //prP2pConnSettings->u2LocalConfigMethod = value;
          break;
      case 102: /* Sigma P2p reset */
          //kalMemZero(prP2pConnSettings->aucTargetDevAddr, MAC_ADDR_LEN);
          //prP2pConnSettings->eConnectionPolicy = ENUM_P2P_CONNECTION_POLICY_AUTO;
          //p2pFsmUninit(prGlueInfo->prAdapter);
          //p2pFsmInit(prGlueInfo->prAdapter);
          break;
      case 103: /* WPS MODE */
          kalP2PSetWscMode(prGlueInfo, value);
          break;
      case 104: /* P2p send persence, duration */
          break;
      case 105: /* P2p send persence, interval */
          break;
      case 106: /* P2P set sleep  */
            value = 1;
            kalIoctl(prGlueInfo,
                wlanoidSetP2pPowerSaveProfile,
                &value,
                sizeof(value),
                FALSE,
                FALSE,
                TRUE,
                &u4Leng);
          break;
      case 107: /* P2P set opps, CTWindowl */
            prP2pSpecificBssInfo->rOppPsParam.u4CTwindowMs = value;
            //status = mtk_p2p_wext_set_oppps_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rOppPsParam);
          break;
      case 108: /* p2p_set_power_save */
            kalIoctl(prGlueInfo,
                wlanoidSetP2pPowerSaveProfile,
                &value,
                sizeof(value),
                FALSE,
                FALSE,
                TRUE,
                &u4Leng);

          break;
      default:
          break;
    }

    return status;

}


int
mtk_p2p_cfg80211_testmode_p2p_sigma_cmd(
    IN struct wiphy *wiphy,
    IN void *data,
    IN int len)
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    P_NL80211_DRIVER_P2P_SIGMA_PARAMS prParams = (P_NL80211_DRIVER_P2P_SIGMA_PARAMS)NULL;
    P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T)NULL;
    //P_P2P_CONNECTION_SETTINGS_T prP2pConnSettings = (P_P2P_CONNECTION_SETTINGS_T)NULL;
    UINT_32 index;
    INT_32  value;
    int     status = 0;
    UINT_32 u4Leng;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    prP2pSpecificBssInfo = prGlueInfo->prAdapter->rWifiVar.prP2pSpecificBssInfo;
    //prP2pConnSettings = prGlueInfo->prAdapter->rWifiVar.prP2PConnSettings;

    DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_testmode_p2p_sigma_cmd\n"));

    if(data && len)
        prParams = (P_NL80211_DRIVER_P2P_SIGMA_PARAMS)data;

    index = (INT_32)prParams->idx;
    value = (INT_32)prParams->value;

    DBGLOG(P2P, TRACE, ("NL80211_ATTR_TESTDATA, idx=%lu value=%lu\n",
        (INT_32)prParams->idx, (INT_32)prParams->value));

    switch (index) {
      case 0: /* Listen CH */
          break;
      case 1: /* P2p mode */
          break;
      case 4: /* Noa duration */
          prP2pSpecificBssInfo->rNoaParam.u4NoaDurationMs = value;
          // only to apply setting when setting NOA count
          //status = mtk_p2p_wext_set_noa_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rNoaParam);
          break;
      case 5: /* Noa interval */
          prP2pSpecificBssInfo->rNoaParam.u4NoaIntervalMs = value;
          // only to apply setting when setting NOA count
          //status = mtk_p2p_wext_set_noa_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rNoaParam);
          break;
      case 6: /* Noa count */
          prP2pSpecificBssInfo->rNoaParam.u4NoaCount = value;
          //status = mtk_p2p_wext_set_noa_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rNoaParam);
          break;
      case 100: /* Oper CH */
          // 20110920 - frog: User configurations are placed in ConnSettings.
          // prP2pConnSettings->ucOperatingChnl = value;
          break;
      case 101: /* Local config Method, for P2P SDK */
          //prP2pConnSettings->u2LocalConfigMethod = value;
          break;
      case 102: /* Sigma P2p reset */
          //kalMemZero(prP2pConnSettings->aucTargetDevAddr, MAC_ADDR_LEN);
          //prP2pConnSettings->eConnectionPolicy = ENUM_P2P_CONNECTION_POLICY_AUTO;
          break;
      case 103: /* WPS MODE */
          kalP2PSetWscMode(prGlueInfo, value);
          break;
      case 104: /* P2p send persence, duration */
          break;
      case 105: /* P2p send persence, interval */
          break;
      case 106: /* P2P set sleep  */
            value = 1;
            kalIoctl(prGlueInfo,
                wlanoidSetP2pPowerSaveProfile,
                &value,
                sizeof(value),
                FALSE,
                FALSE,
                TRUE,
                &u4Leng);
          break;
      case 107: /* P2P set opps, CTWindowl */
            prP2pSpecificBssInfo->rOppPsParam.u4CTwindowMs = value;
            //status = mtk_p2p_wext_set_oppps_param(prDev, info, wrqu, (char *)&prP2pSpecificBssInfo->rOppPsParam);
          break;
      case 108: /* p2p_set_power_save */
            kalIoctl(prGlueInfo,
                wlanoidSetP2pPowerSaveProfile,
                &value,
                sizeof(value),
                FALSE,
                FALSE,
                TRUE,
                &u4Leng);

          break;
      case 109: /* Max Clients*/
          kalP2PSetMaxClients(prGlueInfo, value);
          break;
      default:
          break;
    }

    return status;

}


int
mtk_p2p_cfg80211_testmode_hotspot_block_list_cmd(
    IN struct wiphy *wiphy,
    IN void *data,
    IN int len)
{
    P_GLUE_INFO_T prGlueInfo = NULL;
    P_NL80211_DRIVER_hotspot_block_PARAMS prParams = (P_NL80211_DRIVER_hotspot_block_PARAMS)NULL;
    int fgIsValid = 0;

    ASSERT(wiphy);

    prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

    if(data && len)
        prParams = (P_NL80211_DRIVER_hotspot_block_PARAMS)data;

    DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_testmode_hotspot_block_list_cmd\n"));

    fgIsValid = kalP2PSetBlackList(prGlueInfo, prParams->aucBssid, prParams->ucblocked);
 
    return fgIsValid;

}

#endif


#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)

#endif // CFG_ENABLE_WIFI_DIRECT && CFG_ENABLE_WIFI_DIRECT_CFG_80211
