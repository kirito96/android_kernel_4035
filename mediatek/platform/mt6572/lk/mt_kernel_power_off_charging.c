

#include <printf.h>
#include <platform/mt_typedefs.h>
#include <platform/mt_rtc.h>
#include <platform/boot_mode.h>
#include <platform/mtk_wdt.h>
#include <platform/mt_pmic.h>

extern BOOL meta_mode_check(void);
extern int mtk_wdt_boot_check(void);
#ifdef RTC_2SEC_REBOOT_ENABLE
extern bool rtc_2sec_boot_check_kpoc(void);
extern bool rtc_2sec_boot_check(void);
#endif

BOOL is_force_boot(void)
{
	if (rtc_boot_check(true))
	{
		printf("[%s] Bypass Kernel Power off charging mode and enter Alarm Boot\n", __func__);
		return TRUE;
	}
	else if (meta_mode_check())
	{
		printf("[%s] Bypass Kernel Power off charging mode and enter Meta Boot\n", __func__);
		return TRUE;	
	}
#ifdef RTC_2SEC_REBOOT_ENABLE
	else if (pmic_detect_powerkey() || mtk_wdt_boot_check()==WDT_BY_PASS_PWK_REBOOT || rtc_2sec_boot_check_kpoc() == RTC_2SEC_BOOT_NORMAL) 
#else
	else if (pmic_detect_powerkey() || mtk_wdt_boot_check()==WDT_BY_PASS_PWK_REBOOT)			
#endif
	{
		printf("[%s] Bypass Kernel Power off charging mode and enter Normal Boot\n", __func__);
		g_boot_mode = NORMAL_BOOT;
		return TRUE;
	}
	
	return FALSE;

}


BOOL kernel_power_off_charging_detection(void)
{
#if 0
	if((upmu_is_chr_det() == KAL_TRUE))
	{
		if(bypass_kernel_power_off_charging()){
			return FALSE;
		}
		else
		{
            
			g_boot_mode = KERNEL_POWER_OFF_CHARGING_BOOT;
			return TRUE;
		}		
	}
	else
	{	
		upmu_set_rg_chrind_on(0);
		printf("[%s] Turn off HW Led\n", __func__);
		return FALSE;
	}
#else
    /* */
    if(is_force_boot()) {
        upmu_set_rg_chrind_on(0);
		printf("[%s] Turn off HW Led\n", __func__);
        return FALSE;
    }

    if((upmu_is_chr_det() == KAL_TRUE)) {
        g_boot_mode = KERNEL_POWER_OFF_CHARGING_BOOT;
		return TRUE;
    }
#ifdef RTC_2SEC_REBOOT_ENABLE
    else if(rtc_2sec_boot_check()){ //customize reboot to KPOC, but not charger in, still boot to normal
        printf("[%s] customize auto boot to KPOC, but charger not in, still boot to normal\n", __func__);
        g_boot_mode = NORMAL_BOOT;
        return TRUE;
    }
#endif
    else {
        /* power off */
        #ifndef NO_POWER_OFF
        printf("[kernel_power_off_charging_detection] power off\n");
        mt6575_power_off();        
        #endif
		return FALSE;	
    }
    /* */
#endif
}





