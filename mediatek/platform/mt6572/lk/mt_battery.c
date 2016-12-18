#include <target/board.h>

//#define CFG_POWER_CHARGING

#ifdef CFG_POWER_CHARGING
#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <platform/mt_pmic.h>
#include <platform/boot_mode.h>
#include <platform/mt_gpt.h>
#include <platform/mt_sleep.h>
#include <platform/mt_rtc.h>
#include <platform/mt_disp_drv.h>
#include <platform/mtk_wdt.h>
#include <platform/mtk_key.h>
#include <platform/mt_logo.h>
#include <platform/mt_leds.h>
#include <printf.h>
#include <sys/types.h>
#include <target/cust_battery.h>

#undef printf
#ifndef NTC_CHECK_SUPPORT
#define NTC_CHECK_SUPPORT	1
#define BAT_TEMP_VOL_P60 (249+5)
#define BAT_TEMP_VOL_N20 (949-5)
#endif


#ifndef BATT_ID_CHECK_SUPPORT
#define BATT_ID_CHECK_SUPPORT       1
#define AUXADC_BATT_ID_CHANNEL 0
#define CHG_MIN_VALID_BATT_ID  400
#define CHG_MAX_VALID_BATT_ID  650
#endif
/*****************************************************************************
 *  Type define
 ****************************************************************************/
#define BATTERY_LOWVOL_THRESOLD             3450


/*****************************************************************************
 *  Global Variable
 ****************************************************************************/
bool g_boot_reason_change = false;


/*****************************************************************************
 *  Externl Variable
 ****************************************************************************/
extern bool g_boot_menu;


void kick_charger_wdt(void)
{
    upmu_set_rg_chrwdt_td(0x0);           // CHRWDT_TD, 4s
    upmu_set_rg_chrwdt_wr(1); 			  // CHRWDT_WR
    upmu_set_rg_chrwdt_int_en(1);         // CHRWDT_INT_EN
    upmu_set_rg_chrwdt_en(1);             // CHRWDT_EN
    upmu_set_rg_chrwdt_flag_wr(1);        // CHRWDT_WR
}


void pchr_turn_on_charging (void)
{
	upmu_set_rg_usbdl_rst(1);		//force leave USBDL mode
	
	kick_charger_wdt();
	
	upmu_set_rg_cs_vth(0xC);    	// CS_VTH, 450mA	
	upmu_set_rg_csdac_en(1);        // CSDAC_EN
    upmu_set_rg_chr_en(1);          // CHR_EN       

#ifdef MTK_FAN5405_SUPPORT
    fan5405_hw_init();
    fan5405_turn_on_charging();
    fan5405_dump_register();
#endif
}

/*add by wangxingxing for FR460121 start*/
#if BATT_ID_CHECK_SUPPORT
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
void check_battery_id(void){
   int i = 0, bat_id_vol = 0, data[4] = {0,0,0,0};
   int res =0;
   int rawdata=0;
 printf("check_battery_id() begin:20130528");

  for(i = 0; i < 3; i++)  //fix FR440426 cut down power consumption     
  {    
   res = IMM_GetOneChannelValue(AUXADC_BATT_ID_CHANNEL,data,&rawdata);
   if(res < 0)
     {
        printf("[adc_driver]: get data error\n");
        break;
     }
   else
     {
        	printf("[adc_driver]: channel4[%d]raw =%d\n",i,rawdata);
     }
   //msleep(5);	
   bat_id_vol += (rawdata * 1500 / 4096);
   printf("adc: channel4[%d]vol =%d\n",i,rawdata * 1500 / 4096);
  }

  if(res < 0)  /*get adc value fail*/
       return; 

  bat_id_vol = bat_id_vol/3;  //fix FR440426 cut down power consumption     
  if((bat_id_vol < CHG_MIN_VALID_BATT_ID) || (bat_id_vol > CHG_MAX_VALID_BATT_ID))
       {
	printf("battery id error %d!\n", bat_id_vol);
       //BMT_status.bat_id_valid = 0;	
	  
      // pchr_turn_off_charging();//turn off charging
      // leds_battery_full_charging();//turn off led
#if 1
        #ifndef NO_POWER_OFF
		printf("mt6575_power_off() called:20130528");
        mt6575_power_off();//power off
        
        #endif
        while(1);	
#endif
       }
  else
  	{
	//BMT_status.bat_id_valid = 1;
	printf("battery id BYD %d!\n", bat_id_vol);
  	}
}
#endif	
/*add by swangxingxing for FR460121 end*/

extern kal_int32 sw_bat_vol = 0;
extern kal_int32 sw_current = 0;
void mt65xx_bat_init(void)
{
    int i = 0;
	kal_int32 bat_vol;
	kal_int32 sw_i_vol;
//check temperature in lk modified by yuan-li@tcl.com @2013.12.30 for PRxxx
	#ifdef NTC_CHECK_SUPPORT
	kal_int32 bat_temp_vol = 0;
	#ifndef TARGET_BUILD_MMITEST
	for(bat_temp_vol = get_tbat_volt(3); bat_temp_vol <= BAT_TEMP_VOL_P60 
		|| bat_temp_vol >= BAT_TEMP_VOL_N20;
		bat_temp_vol = get_tbat_volt(3))
	{
		printf("carl lk check fatal temp lead power off, vol %d \n",bat_temp_vol);
		mt6575_power_off();
	}
	#endif
	#endif

    // Low Battery Safety Booting
    
    bat_vol = get_bat_sense_volt(1);
	
	//add by feichen@tcl.com for PR700143,MTK platform issue,HW OCV is higher when charging power on.
	//LK would get voltage and charging current and kernel meter initial OCV should calibrate the OCV by voltage,charging current and battery R.
	sw_i_vol = get_i_sense_volt(5);
	sw_bat_vol = get_bat_sense_volt(5);
	//sw_current means charging into battery,if negative,means battery current consumption
	//sw_ivol > sw_bat_vol means charging
	if(sw_i_vol > sw_bat_vol && ((sw_i_vol - sw_bat_vol)*5 > 50))
		sw_current = ((sw_i_vol - sw_bat_vol)*5 - 140);
	else
		sw_current = -140;

	
    printf("check VBAT=%d mV with %d mV\n", bat_vol, BATTERY_LOWVOL_THRESOLD);
//20130529 add by wangxingxing for FR460121 start
	#if BATT_ID_CHECK_SUPPORT
        /*susheng.ye add for FR470200 20130614 begin*/
        #ifdef TARGET_BUILD_MMITEST
        if(meta_mode_check()==FALSE)
        {
    	    //check_battery_id();
         }
        #else
        check_battery_id();
        #endif
        /*susheng.ye add for FR470200 20130614 end*/
      #endif
//20130529 add by wangxingxing for FR460121 end	  
    pchr_turn_on_charging();

	if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT && (upmu_get_pwrkey_deb()==0) ) {
		printf("[mt65xx_bat_init] KPOC+PWRKEY => change boot mode\n");
		
		g_boot_reason_change = true;
	}
    rtc_boot_check(false);

	#ifndef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
    if (bat_vol < BATTERY_LOWVOL_THRESOLD)
    {
        if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT && upmu_is_chr_det() == KAL_TRUE)
        {
            printf("[%s] Kernel Low Battery Power Off Charging Mode\n", __func__);
            g_boot_mode = LOW_POWER_OFF_CHARGING_BOOT;
            return;
        }
        else
        {
            printf("[BATTERY] battery voltage(%dmV) <= CLV ! Can not Boot Linux Kernel !! \n\r",bat_vol);
#ifndef NO_POWER_OFF
            mt6575_power_off();
#endif			
            while(1)
            {
                printf("If you see the log, please check with RTC power off API\n\r");
            }
        }
    }
	#endif
    return;
}

#else

#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <printf.h>

void mt65xx_bat_init(void)
{
    printf("[BATTERY] Skip mt65xx_bat_init !!\n\r");
    printf("[BATTERY] If you want to enable power off charging, \n\r");
    printf("[BATTERY] Please #define CFG_POWER_CHARGING!!\n\r");
}

#endif
