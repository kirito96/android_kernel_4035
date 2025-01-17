
#ifndef __MT6630_CUST_CFG_H__
#define __MT6630_CUST_CFG_H__

//scan sort algorithm 
enum{
    FM_SCAN_SORT_NON = 0,
    FM_SCAN_SORT_UP,
    FM_SCAN_SORT_DOWN,
    FM_SCAN_SORT_MAX
};
//*****************************************************************************************
//***********************************FM config for customer: start******************************
//*****************************************************************************************
//RX
#define FM_RX_RSSI_TH_LONG_MT6630    -296      //FM radio long antenna RSSI threshold(-4dBuV)
#define FM_RX_RSSI_TH_SHORT_MT6630   -296      //FM radio short antenna RSSI threshold(-4dBuV)
#define FM_RX_DESENSE_RSSI_MT6630   -258      
#define FM_RX_PAMD_TH_MT6630          -12     
#define FM_RX_MR_TH_MT6630           -67      
#define FM_RX_ATDC_TH_MT6630           3496      
#define FM_RX_PRX_TH_MT6630           64      
#define FM_RX_SMG_TH_MT6630          16421      //FM soft-mute gain threshold
#define FM_RX_DEEMPHASIS_MT6630       0           //0-50us, China Mainland; 1-75us China Taiwan
#define FM_RX_OSC_FREQ_MT6630         0           //0-26MHz; 1-19MHz; 2-24MHz; 3-38.4MHz; 4-40MHz; 5-52MHz  
//#define FM_RX_SEEK_SPACE_MT6630      1           //FM radio seek space,1:100KHZ; 2:200KHZ
//#define FM_RX_SCAN_CH_SIZE_MT6630    40          //FM radio scan max channel size
//#define FM_RX_BAND_MT6630            1           //FM radio band, 1:87.5MHz~108.0MHz; 2:76.0MHz~90.0MHz; 3:76.0MHz~108.0MHz; 4:special
//#define FM_RX_SCAN_SORT_SELECT_MT6630 FM_SCAN_SORT_NON
//#define FM_RX_FAKE_CH_NUM_MT6630      1
//#define FM_RX_FAKE_CH_RSSI_MT6630     40
//#define FM_RX_FAKE_CH_1_MT6630        1075
//#define FM_RX_FAKE_CH_2_MT6630        0
//#define FM_RX_FAKE_CH_3_MT6630        0
//#define FM_RX_FAKE_CH_4_MT6630        0
//#define FM_RX_FAKE_CH_5_MT6630        0

//TX
//#define FM_TX_PWR_LEVEL_MAX_MT6630  120  
//#define FM_TX_SCAN_HOLE_LOW_MT6630  923         //92.3MHz~95.4MHz should not show to user
//#define FM_TX_SCAN_HOLE_HIGH_MT6630 954         //92.3MHz~95.4MHz should not show to user


//*****************************************************************************************
//***********************************FM config for customer:end *******************************
//*****************************************************************************************

//#define FM_SEEK_SPACE_MT6630 FM_RX_SEEK_SPACE_MT6630
//max scan chl num
//#define FM_MAX_CHL_SIZ_MT6630E FM_RX_SCAN_CH_SIZE_MT6630
// auto HiLo
#define FM_AUTO_HILO_OFF_MT6630    0
#define FM_AUTO_HILO_ON_MT6630     1


// seek threshold
#define FM_SEEKTH_LEVEL_DEFAULT_MT6630 4

#endif // __MT6630_CUST_CFG_H__
