#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "tpd_custom_ft6306.h"
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/eint.h>
#include <linux/miscdevice.h>
#include "cust_gpio_usage.h"

#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>

#define FTS_CTL_IIC

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#define  GPIO_CTP_EN_PIN   GPIO22
#define  GPIO_CTP_EN_PIN_M_GPIO  GPIO_MODE_00
 
#define TPD_DEVICEFT "FT6306"
#define FTS_UPGRADE_LOOP  10
extern struct tpd_device *tpd;
 
enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE,
	WK_WDT_LOC_TYPE_NOLOCK,
	WK_WDT_EXT_TYPE_NOLOCK,
	
};
extern void mtk_wdt_restart(enum wk_wdt_type type);
 
struct i2c_client *i2c_clientft = NULL;
struct task_struct *threadft = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
 /*
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
*/
 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;

//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif
//register define

struct touch_info {
    int y[3];
    int x[3];
    int p[3];
    int count;
};
 
 static const struct i2c_device_id tpd_id[] = {{TPD_DEVICEFT,0},{}};
 //unsigned short forceft[] = {1,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
 //static const unsigned short * const forces[] = { forceft, NULL };
 //static struct i2c_client_address_data addr_data = { .forces = forces, };
   static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICEFT, (0x70>>1))};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = TPD_DEVICEFT,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
 };
 
 #define UPGRADE_FIRMWARE

 #ifdef UPGRADE_FIRMWARE
/********************************Upgrade Firmware Function star****************************************/

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL               0x0
#define FTS_TRUE               0x01
#define FTS_FALSE              0x0

#define    I2C_CTPM_ADDRESS       0x38
#define    FTS_PACKET_LENGTH        128 //128 +6 =134;
#define    FTS_SETTING_BUF_LEN        128

unsigned char tp_vendor;
unsigned char tp_fw_ver;

static unsigned char CTPM_FW_CAMRY_TRULY[]=
{
	//#include "ft_app_camry_truly.i"
	#include "YARIS_M_ID80_V14_20130701_app.i"
};

static unsigned char CTPM_FW_CAMRY_MUTTO[]=
{
	//#include "ft_app_camry_mutto.i"
	#include "YARIS_M_ID80_V14_20130701_app.i"
};

static unsigned char CTPM_FW_BEETLE_EACH[]=
{
	//#include "Ver0x0B_Beetle_FT6306_0x80_20130222_app.i"
	/*#include "YARIS_ID80_V15_20130708_app.i"*/
	//#include "YarisMVF_6306_ID80_V20_20131029_app.i"
        /*#include "YarisMVF_6306_ID80_V61_20131127_app.i" */
		#include "YarisMVF_6306_ID80_V64_20131212_app.i"
};

static unsigned char CTPM_FW_BEETLE_JUNDA[]=
{
	//#include "Ver0x0B_Beetle_FT6306_0x85_20130222_app.i"
	//#include "YARIS_M_ID85_V11_20130710_app.i"
	//#include  "Soul4_6306_ID85_V0d_20140306_app.i"
	//#include "Soul4_6306_ID85_V0e_20140317_app.i"
        //#include "Soul4_6306_ID85_V0f_20140410_app.i"
          #include "Soul4_6306_ID85_V11_20140605_app.i"
	
};

static unsigned char CTPM_FW_BEETLE_JUNDA_NEW[] =
{
//	#include "Soul4_6306_ID85_V31_20140728_app.i"
	#include "Soul4_6306_ID85_V32_20140807_app.i"
};
void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			udelay(1);
		}
	}
}

static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[2];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret=i2c_master_send(i2c_clientft,buf, sizeof(buf));
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", addr, ret);
		return -1;
	}
	return 0;
}

static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[1];
	i2c_master_send(i2c_clientft, &addr, 1);
	ret=i2c_master_recv(i2c_clientft, buf, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret; 
}

static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(0xA6, &ver);
	return(ver);
}

FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;   
	ret = i2c_master_recv(i2c_clientft, pbt_buf, dw_lenth);
	if(ret <= 0)
	{
		TPD_DMESG("[FTS]i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
	ret = i2c_master_send(i2c_clientft, pbt_buf, dw_lenth);
	if(ret<=0)
	{
		TPD_DMESG("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}
static FTS_BYTE *dma_buffer = NULL;//FTS_BYTE=u8;
static u32 dma_handle = NULL;
static struct mutex dma_model_mutex;

FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{ 
	int ret=-1;
	u16 addr;

	memcpy(dma_buffer,pbt_buf,dw_len);
	if(dw_len <= 8)
	{
		ret= i2c_master_send(i2c_clientft, pbt_buf, dw_len);
	}
	else
	{
		addr=i2c_clientft->addr;
		
		//TPD_DMESG("[FT6306] I2C  DMA transfer----byte_write------llf\n");
		mutex_lock(&dma_model_mutex);
		i2c_clientft->addr = i2c_clientft->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		mutex_unlock(&dma_model_mutex);
		ret = i2c_master_send(i2c_clientft, dma_handle, dw_len);

		i2c_clientft->addr=addr;
	}
	if(ret<=0)
	{
		TPD_DMESG("[FT6306]i2c_write_byte error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}
	return FTS_TRUE;
}

FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
	int ret=-1;
	u16 addr;

	addr=i2c_clientft->addr;
	if(bt_len <= 8)
	{
		i2c_clientft->addr = i2c_clientft->addr & I2C_MASK_FLAG;
		ret=i2c_master_recv(i2c_clientft, pbt_buf, bt_len);
		if(ret<=0)
		{
			i2c_clientft->addr=addr;
			TPD_DMESG("[FT6306]i2c_read_byte error line = %d, ret = %d\n", __LINE__, ret);
			return FTS_FALSE;
		}
	}
	else
	{

		//TPD_DMESG("[FT6306] I2C  DMA transfer-------byte_read-------llf\n");
		mutex_lock(&dma_model_mutex);
		i2c_clientft->addr = i2c_clientft->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		mutex_unlock(&dma_model_mutex);
		ret = i2c_master_recv(i2c_clientft, dma_handle,bt_len);

		if(ret<=0)
		{
			i2c_clientft->addr=addr;
			TPD_DMESG("[FT6306]i2c_read_byte error line = %d, ret = %d\n", __LINE__, ret);
			return FTS_FALSE;
		}
		memcpy(pbt_buf,dma_buffer,bt_len);
	}
	i2c_clientft->addr=addr;
	return FTS_TRUE;
}
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	FTS_BYTE reg_val[2] = {0};
	FTS_DWRD i = 0;

	FTS_DWRD  packet_number;
	FTS_DWRD  j;
	FTS_DWRD  temp;
	FTS_DWRD  lenght;
	FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
	FTS_BYTE  auc_i2c_write_buf[10];
	FTS_BYTE  bt_ecc;
	int       i_ret;
	int reg_addr[] = {0xbc,0xbc};
	int  index=0;
	int delay_time1[] = {100,100};
	int delay_time2[] = {30,30};

	mutex_init(&dma_model_mutex);
	dma_buffer= (u8 *)dma_alloc_coherent(NULL, 134, &dma_handle, GFP_KERNEL);
    	if(!dma_buffer)
	{
    		printk("[FT6306][TSP] dma_alloc_coherent error\n");
	}
	
	i2c_clientft->timing = 400;	   //when in upgrade mode ,the clock shall be changed to 400
	//for(index = 0;index < sizeof(reg_addr)/sizeof(reg_addr[0]);index++)		//add ft6306 bootloader mode test
	for(index = 0; index <= FTS_UPGRADE_LOOP; index++)
	{
		/*write 0xaa to register 0xfc(FT6306) or 0xbc(ft6306)*/
		/*ft5x0x_write_reg(reg_addr[index],0xaa);*/
		/*delay_qt_ms(delay_time1[index]);*/
		/*write 0x55 to register 0xfc(FT6306) or 0xbc(ft6306)*/
		/*ft5x0x_write_reg(reg_addr[index],0x55);*/
		/*TPD_DMESG("[FT6306-upgrade] Step 1: Reset CTPM test\n");*/
		/*delay_qt_ms(delay_time2[index]);*/

		// Use Hardware Reset
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
		//msleep(10);
		delay_qt_ms(10);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
		//msleep(50);
		delay_qt_ms(20+index*2);// in upgrade mode, make sure the attempts
		TPD_DMESG("[FT6306-upgrade] Step 1: Reset CTPM test\n");


		/*********Step 2:Enter upgrade mode *****/
		TPD_DMESG("[FT6306-upgrade] Step 2: Enter upgrade mode\n");
		auc_i2c_write_buf[0] = 0x55;
		auc_i2c_write_buf[1] = 0xaa;
		do
		{
			i ++;
			i_ret = i2c_master_send(i2c_clientft,auc_i2c_write_buf, 2);
			delay_qt_ms(5);
		}while(i_ret <= 0 && i < 5 );
		
		i= 0;
		/*********Step 3:check READ-ID***********************/
		delay_qt_ms(10);
		cmd_write(0x90,0x00,0x00,0x00,4);
		byte_read(reg_val,2);
		TPD_DMESG("[FT6306-upgrade] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
		if (reg_val[0] == 0x79 && reg_val[1] == 0x08)
		{
			TPD_DMESG("[FT6306] check READ-ID Successful ! index = %d------\n", index);
			break;
		}
		else if(index >= FTS_UPGRADE_LOOP)
		{
			TPD_DMESG("[FT6306] check READ-ID faild !\n");
			i2c_clientft->timing = 100;
			return ERR_READID;
		}
	}	
	printk("[ft6306]---index: %d------\n", index);
	cmd_write(0xcd,0x0,0x00,0x00,1);
	byte_read(reg_val,1);
	TPD_DMESG("[FT6306-upgrade] bootloader version = 0x%x\n", reg_val[0]);

	 /*********Step 4:erase app and panel paramenter area ********************/
	cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
	delay_qt_ms(1500); 
	cmd_write(0x63,0x00,0x00,0x00,1);  //erase panel parameter area
	delay_qt_ms(100);
	TPD_DMESG("[FT6306-upgrade] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	TPD_DMESG("[FT6306-upgrade] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j = 0;j < packet_number;j ++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(lenght>>8);
		packet_buf[5] = (FTS_BYTE)lenght;

		for (i = 0;i < FTS_PACKET_LENGTH;i ++)
		{
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}
        
		byte_write(&packet_buf[0],sizeof(packet_buf));
		delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0)
		{
			TPD_DMESG("[FT6306-upgrade] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;

		for (i=0;i<temp;i++)
		{
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);    
		delay_qt_ms(20);
	}

    //send the last six byte
	for (i = 0; i < 6; i++)
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp =1;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0],7);  
		delay_qt_ms(20);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	cmd_write(0xcc,0x00,0x00,0x00,1);
	byte_read(reg_val,1);
	TPD_DMESG("[FT6306-upgrade] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc)
	{
		i2c_clientft->timing = 100;
		return ERR_ECC;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);
	i2c_clientft->timing = 100;
	msleep(300);  //make sure CTP startup normally

	mtk_wdt_restart(WK_WDT_EXT_TYPE);//avoid watchdog timeout
	if(dma_buffer)
	{
		dma_free_coherent(NULL, 134, dma_buffer, dma_handle);
		dma_buffer = NULL;
		dma_handle= 0;
	}
	return ERR_OK;
}

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp;
	unsigned char i;

	TPD_DMESG("[FT6306] start auto CLB.\n");
	msleep(200);
	ft5x0x_write_reg(0, 0x40);  
	delay_qt_ms(100);   //make sure already enter factory mode=Test mode
	ft5x0x_write_reg(2, 0x4);  //write command to start calibration
	delay_qt_ms(300);
	
	for(i=0;i<50;i++) //llf 20130220
	{
		uc_temp=0x0;
		ft5x0x_read_reg(0x0,&uc_temp);
		if ( (uc_temp&0x70) == 0x0)  //return to normal mode, calibration finish
		{
			break;
		}
		delay_qt_ms(200);
		//mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//avoid watchdog timeout
		TPD_DMESG("[FT6306] waiting calibration %d\n",i);
	}
	mtk_wdt_restart(WK_WDT_EXT_TYPE);//avoid watchdog timeout
	msleep(200);
	ft5x0x_write_reg(0, 0x40);  
	delay_qt_ms(100);   //make sure already enter factory mode=Test mode
	ft5x0x_write_reg(2, 0x4);  //write command to start calibration
	delay_qt_ms(300);
	for(i=50;i<100;i++) //llf 20130220
	{
		uc_temp=0x0;
		ft5x0x_read_reg(0x0,&uc_temp);
		if ( (uc_temp&0x70) == 0x0)  //return to normal mode, calibration finish
		{
			break;
		}
		delay_qt_ms(200);
		//mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//avoid watchdog timeout
		TPD_DMESG("[FT6306] waiting calibration %d\n",i);

	}
	TPD_DMESG("[FT6306] calibration OK.\n");

	msleep(300);
	ft5x0x_write_reg(0, 0x40);  //goto factory mode
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5x0x_write_reg(2, 0x5);  //store CLB result
	delay_qt_ms(300);
	ft5x0x_write_reg(0, 0x0); //return to normal mode 
	msleep(300);
	TPD_DMESG("[FT6306] store CLB result OK.\n");

	mtk_wdt_restart(WK_WDT_EXT_TYPE);//avoid watchdog timeout
	//i2c_clientft->timing = 100;   // return to normal clock hz
	return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE*  pbt_buf = FTS_NULL;
	unsigned int ui_sz;
	int i_ret;

	//=ven========FW upgrade========================*/
	if(tp_vendor == 0x5a){
		pbt_buf = CTPM_FW_CAMRY_TRULY;
		ui_sz = sizeof(CTPM_FW_CAMRY_TRULY);
	}
	else if(tp_vendor == 0x53){
		pbt_buf = CTPM_FW_CAMRY_MUTTO;
		ui_sz = sizeof(CTPM_FW_CAMRY_MUTTO);
	}
	else if(tp_vendor == 0x80){
		pbt_buf = CTPM_FW_BEETLE_EACH;
		ui_sz = sizeof(CTPM_FW_BEETLE_EACH);
	}
	else if(tp_vendor == 0x85){
		if(tp_fw_ver >= 0x30){
			pbt_buf = CTPM_FW_BEETLE_JUNDA_NEW;
			ui_sz = sizeof(CTPM_FW_BEETLE_JUNDA_NEW);
		}
		else{
			pbt_buf = CTPM_FW_BEETLE_JUNDA;
			ui_sz = sizeof(CTPM_FW_BEETLE_JUNDA);					
		}
			
	}

	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf,ui_sz);
	if (i_ret != 0)
	{
		TPD_DMESG("[FT6306] upgrade failed i_ret = %d.\n", i_ret);
		//error handling ...
		//TBD
	}
	else
	{
		TPD_DMESG("[FT6306] upgrade successfully.\n");
		fts_ctpm_auto_clb();  //start auto CLB
	}
	return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
	unsigned int ui_sz;

	if(tp_vendor == 0x5a)
		ui_sz = sizeof(CTPM_FW_CAMRY_TRULY);
	else if(tp_vendor == 0x53)
		ui_sz = sizeof(CTPM_FW_CAMRY_MUTTO);
	else if(tp_vendor == 0x80)
		ui_sz = sizeof(CTPM_FW_BEETLE_EACH);
	else if(tp_vendor == 0x85)
	{
		if(tp_fw_ver >= 0x30)
			ui_sz = sizeof(CTPM_FW_BEETLE_JUNDA_NEW);
		else
			ui_sz = sizeof(CTPM_FW_BEETLE_JUNDA);
	}
	if (ui_sz > 2)
	{
		if(tp_vendor == 0x5a)
			return CTPM_FW_CAMRY_TRULY[ui_sz - 2];
		else if(tp_vendor == 0x53)
			return CTPM_FW_CAMRY_MUTTO[ui_sz - 2];
		else if(tp_vendor == 0x80)
			return CTPM_FW_BEETLE_EACH[ui_sz - 2];
		else if(tp_vendor == 0x85)
		{
			if(tp_fw_ver >= 0x30)
				return CTPM_FW_BEETLE_JUNDA_NEW[ui_sz - 2];
			else
				return CTPM_FW_BEETLE_JUNDA[ui_sz - 2];
		}
	}
	else
		//TBD, error handling?
		return 0xff; //default value
}

int fts_ctpm_get_panel_factory_setting(void)    //return factory ID
{
	unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
	unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;    1----1.8v
	unsigned char uc_panel_factory_id;     //TP panel factory ID

	unsigned char buf[FTS_SETTING_BUF_LEN];
	unsigned char reg_val[2] = {0};
	unsigned char  auc_i2c_write_buf[10];
	unsigned char  packet_buf[FTS_SETTING_BUF_LEN + 6];
	int reg_addr[] = {0xbc,0xbc};//{0xfc,0xbc};
	int delay_time1[] = {100,100};//{50,100};
	int delay_time2[] = {30,30};

	int i = 0;
	int j = 0;
	int i_ret;

	uc_i2c_addr = I2C_CTPM_ADDRESS;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;

	TPD_DMESG("[FT6306] Enter fts_ctpm_get_panel_factory_setting to get factory ID\n");

	i2c_clientft->timing = 400;       //when in upgrade mode ,the clock should be changed to 400
	/*********Step 1:Reset  CTPM *****/
	for(j = 0;j < sizeof(reg_addr)/sizeof(reg_addr[0]);j++)		//add ft6306 bootloader mode test
	{
		/*write 0xaa to register 0xfc(FT6306) or 0xbc(ft6306)*/
		ft5x0x_write_reg(reg_addr[j],0xaa);
		delay_qt_ms(delay_time1[j]);
		/*write 0x55 to register 0xfc(FT6306) or 0xbc(ft6306)*/
		ft5x0x_write_reg(reg_addr[j],0x55);
		TPD_DMESG("[FT6306] Step 1: Reset CTPM test\n");

		delay_qt_ms(delay_time2[j]);

		/*********Step 2:Enter upgrade mode *****/
		TPD_DMESG("[FT6306] Step 2: Enter upgrade mode\n");
		auc_i2c_write_buf[0] = 0x55;
		auc_i2c_write_buf[1] = 0xaa;
		do
		{
			i ++;
			i_ret = i2c_master_send(i2c_clientft,auc_i2c_write_buf, 2);
			delay_qt_ms(5);
		}while(i_ret <= 0 && i < 5 );
		
		i= 0;
		/*********Step 3:check READ-ID***********************/
		delay_qt_ms(10);
		cmd_write(0x90,0x00,0x00,0x00,4);
		byte_read(reg_val,2);
		if (reg_val[0] == 0x79)
		{
			if(reg_val[1] == 0x7)
				TPD_DMESG("[FT6306] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
			else if(reg_val[1] == 0x8)
				TPD_DMESG("[FT6306] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
			break;
		}
		else if(j == sizeof(reg_addr)/sizeof(reg_addr[0]) - 1)
		{
			i2c_clientft->timing =100;
			return ERR_READID;
		}
	}


	/* --------- read current project setting  ---------- */
	//set read start address
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;
	//  i2c_master_send(i2c_clientft, &buf[0], 4);
	cmd_write(0x3,0x0,0x78,0x0,4);
	i2c_clientft->addr = i2c_clientft->addr & I2C_MASK_FLAG;
	i2c_master_recv(i2c_clientft, &buf, 8);

	TPD_DMESG("[FT6306] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
	buf[0],  buf[2], buf[4]);
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;
	//  i2c_master_send(i2c_clientft, &buf[0], 4);
	cmd_write(0x3,0x0,0x78,0x04,4);
	i2c_clientft->addr = i2c_clientft->addr & I2C_MASK_FLAG;
	i2c_master_recv(i2c_clientft, &buf, 1);
	TPD_DMESG("[FT6306] The buf[0] is 0x%x\n",buf[0]);	
//set read start address

	buf[0] = 0x3;

	buf[1] = 0x0;

	buf[2] = 0x78;     //buf[2]buf[3]对应0x7840地址

	buf[3] = 0x40;

	//  i2c_master_send(i2c_clientft, &buf[0], 4);

	cmd_write(0x3,0x0,0x78,0x40,4);

	i2c_clientft->addr = i2c_clientft->addr & I2C_MASK_FLAG;

	i2c_master_recv(i2c_clientft, &buf, 1);


	
	TPD_DMESG("[FT6306] old setting: fw id = 0x%x\n",buf[0]);
	tp_fw_ver = buf[0];
	TPD_DMESG("[FT6306] old setting: tp_fw_ver = 0x%x\n",tp_fw_ver);
 

 //         tp_fw_ver = buf[0];                 // get old FW id 

	/********* reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);

	msleep(200);

	i2c_clientft->timing = 100;     // return to normal clock hz
	return buf[4];

}

int fts_ctpm_auto_upg(unsigned char tp_vendor)
{
	unsigned char uc_host_fm_ver;
	unsigned char uc_tp_fm_ver;
	unsigned char version_list_camry_truly[] = {0x0,0xa6,0xa,0xb};  //Keep the Truly tp firmware old version list that allows to be updated.
	unsigned char version_list_camry_mutto[] = {0x0,0xa6};  //Keep the Mutto tp firmware old version list that allows to be updated.
	unsigned char version_list_camry_each[] = {0x0,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x20,0x61};	//Keep the ft6306 tp firmware old version list that allows to be updated.
//	unsigned char version_list_camry_junda[] = {0x0,0x10,0x11,0x12,0x13,0x14,0x15};	//Keep the ft6306 tp firmware old version list that allows to be updated.
	unsigned char version_list_camry_junda[] = {0x0,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x30,0x31,0x32,0x33,0x34,0x35};	//Keep the ft6306 tp firmware old version list that allows to be updated.
	int i,i_ret = -1;

	uc_tp_fm_ver = ft5x0x_read_fw_ver();
	if(tp_vendor == 0x5a){
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		TPD_DMESG("[FT6306] tp vendor is Truly(0x%x),uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",tp_vendor, uc_tp_fm_ver, uc_host_fm_ver);
		for (i = 0; i < sizeof(version_list_camry_truly)/sizeof(version_list_camry_truly[0]); i++)
		{
			if(uc_tp_fm_ver<uc_host_fm_ver)//llf 20130222
			{
				if (uc_tp_fm_ver == version_list_camry_truly[i])
	       	 		{
	       	 			TPD_DMESG("[FT6306]  tp firmware have new version \n");
					i_ret = fts_ctpm_fw_upgrade_with_i_file();
					if (!i_ret)
						TPD_DMESG("[FTS] Truly tp firmware upgrade to new version 0x%x\n", uc_host_fm_ver);
					else
						TPD_DMESG("[FTS] Truly tp firmware upgrade failed ret=%d.\n", i_ret);
					break;
				}
			}
			else
			{
				TPD_DMESG("[FT6306] current tp firmware is the latest version!\n");
				break;
			}
		}
	}
	else if(tp_vendor == 0x53){
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		TPD_DMESG("[FT6306] tp vendor is Mutto(0x%x),uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",tp_vendor, uc_tp_fm_ver, uc_host_fm_ver);
		for (i = 0; i < sizeof(version_list_camry_mutto)/sizeof(version_list_camry_mutto[0]); i++)
		{
			if(uc_tp_fm_ver<uc_host_fm_ver)//llf 20130222
			{
				if (uc_tp_fm_ver == version_list_camry_mutto[i])
	       	 		{
	       	 			TPD_DMESG("[FT6306]  tp firmware have new version \n");
					i_ret = fts_ctpm_fw_upgrade_with_i_file();
					if (!i_ret )
						TPD_DMESG("[FT6306] Mutto tp firmware upgrade to new version 0x%x\n", uc_host_fm_ver);
					else
						TPD_DMESG("[FT6306] Mutto tp firmware upgrade failed ret=%d.\n", i_ret);
					break;
				}
			}
			else
			{
				TPD_DMESG("[FT6306] current tp firmware is the latest version!\n");
				break;
			}
		}		
	}
	else if(tp_vendor == 0x80){
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		TPD_DMESG("[FT6306] tp vendor is unkown(0x%x),uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",tp_vendor, uc_tp_fm_ver, uc_host_fm_ver);
		for(i = 0;i < sizeof(version_list_camry_each)/sizeof(version_list_camry_each[0]); i++)
		{
			if(uc_tp_fm_ver<uc_host_fm_ver)//llf 20130222
			{
		//		if(uc_tp_fm_ver == version_list_camry_each[i])  
				{
					TPD_DMESG("[FT6306] Each tp firmware have new version\n");
					i_ret = fts_ctpm_fw_upgrade_with_i_file();
					if(!i_ret)
						TPD_DMESG("[FT6306] Each tp firmware upgrade to new version 0x%x\n", uc_host_fm_ver);
					else
						TPD_DMESG("[FT6306] Each tp firmware upgrade failed ret=%d.\n", i_ret);
					break;
				}
			}
			else
			{
				TPD_DMESG("[FT6306] current Each tp firmware is the latest version!\n");
				break;
			}
		}
	}
	else if(tp_vendor == 0x85){
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		TPD_DMESG("[FT6306] tp vendor is unkown(0x%x),uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",tp_vendor, uc_tp_fm_ver, uc_host_fm_ver);
		for(i = 0;i < sizeof(version_list_camry_junda)/sizeof(version_list_camry_junda[0]); i++)
		{
			if((uc_tp_fm_ver<uc_host_fm_ver) || uc_tp_fm_ver==0x13)//llf 2013626
			//if(1) //force to up
			{
				if(uc_tp_fm_ver == version_list_camry_junda[i])  
				{
					TPD_DMESG("[FT6306] Junda tp firmware have new version\n");
					i_ret = fts_ctpm_fw_upgrade_with_i_file();
					if(!i_ret)
						TPD_DMESG("[FT6306] Junda tp firmware upgrade to new version 0x%x\n", uc_host_fm_ver);
					else
						TPD_DMESG("[FT6306] Junda tp firmware upgrade failed ret=%d.\n", i_ret);
					break;
				}
			}
			else
			{
				TPD_DMESG("[FT6306] current Junda tp firmware is the latest version!\n");
				break;
			}
		}
	}
	else{
		TPD_DMESG("[FT6306] Unknown tp vendor(0x%x).\n", uc_tp_fm_ver);
		return -1;
	}
    return 0;
}


/********************************Upgrade Firmware Function end****************************************/
#endif

// #define CTP_FACE_PROBE

/**************************************************************************************/
#define ALSPS_DEVICE_NAME		"ctp_alps"
#define FACE_DETECT_NEAR		0xc0 /*face detected*/
#define FACE_DETECT_FAR 		0xe0 /* face moved*/

int fps_state = -1;
//extern int tp_pls_status;
static int ft6306_pls_opened=0;

/*************************************************************************************/
#ifdef CTP_FACE_PROBE
static void  ft6306_ps_mode_enable(bool enable)
{	      
	int err = 0;
	unsigned char rx_data;
	if(enable)  //enter phone open
	{
		err= ft5x0x_write_reg(0xB0,0x01);
		TPD_DMESG("[FT6306_PS]open ps is %d \n",err);
		err=ft5x0x_read_reg(0xB0,&rx_data);
		if(err>=0 && (rx_data&0x01>0))
			TPD_DMESG("[FT6306_PS]open ps OK\n");
		else
			TPD_DMESG("[FT6306_PS]open ps failed\n");
	}
	else //close phone
	{     
		err= ft5x0x_write_reg(0xB0,0x00);
		TPD_DMESG("[FT6306_PS]close ps is %d \n",err);
		err=ft5x0x_read_reg(0xB0,&rx_data);
		if(err>=0 && (rx_data&0x01==0))
			TPD_DMESG("[FT6306_PS]close ps OK\n");
		else
			TPD_DMESG("[FT6306_PS]close ps failed\n");
	}
}

int ft6306_pls_enable(void)
{
	TPD_DMESG("[FT6306_PS]ft6306_pls_enable\n");
	ft6306_ps_mode_enable(true);
	ft6306_pls_opened = 1;
	return 0;
}
 
int ft6306_pls_disable(void)
{
	TPD_DMESG("[FT6306_PS]ft6306_pls_disable\n");
	ft6306_ps_mode_enable(false);
	ft6306_pls_opened = 0;
	return 0;
}	

int  get_ft6306_data(void )
{
	return fps_state;
}

 static int ft6306_pls_open(struct inode *inode, struct file *file)
 {
	 TPD_DMESG("[FT6306_PS] %s\n", __func__);
	 if (ft6306_pls_opened)
		 return -EBUSY;
	 ft6306_pls_opened = 1;
	 return 0;
 }
 static int ft6306_pls_release(struct inode *inode, struct file *file)
 {
	 TPD_DMESG("[FT6306_PS] %s", __func__);
	 ft6306_pls_opened = 0;
	 return 0;//ap3212c_pls_disable(AP3212C_PLS_BOTH);
 }

 static long ft6306_pls_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
 {
	 void __user *argp = (void __user *)arg;
	 TPD_DMESG("[FT6306_PS] %s: cmd %d", __func__, _IOC_NR(cmd));
	 switch (cmd) {
	 case 1:
		 TPD_DMESG("[FT6306_PS] %s: FACE_DETECT_NEAR*************************\n", __func__);
		 ft6306_pls_enable();
		 break;
	 case 2:
		 TPD_DMESG("[FT6306_PS] %s: FACE_DETECT_FAR*************************\n", __func__);
		 ft6306_pls_disable();
		 break;
	 default:
		 TPD_DMESG("[FT6306_PS] %s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		 return -EINVAL;
	 }
	 return 0;
 }
 
 static struct file_operations ft6306_pls_fops = {
	 .owner 			 = THIS_MODULE,
	 .open			 = ft6306_pls_open,
	 .release			 = ft6306_pls_release,
	 .unlocked_ioctl 			 = ft6306_pls_ioctl,
 };
 static struct miscdevice ft6306_pls_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = ALSPS_DEVICE_NAME,
	.fops = &ft6306_pls_fops,
};
#endif

/*****************************************APK UPDRADE**************************************/
#define APK_UPGRADE
static struct mutex g_device_mutex;

#ifdef APK_UPGRADE
static int ft5x0x_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	printk("[FT6306_llf] Get firmware size begain\n");
	sprintf(filepath, "%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	printk("[FT6306_llf] Read firmware begain!\n");
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);

	printk("[FT6306_llf] Get firmware size ok size=%d \n",fwsize);
	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",__func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 32 * 1024) {
		dev_dbg(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}

	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);

	if (ft5x0x_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",__func__);
		kfree(pbt_buf);
		return -EIO;
	}
	printk("[FT6306_llf] Get firmware file ok\n");

	printk("[FT6306_llf] File just is 0x%x 0x%x 0x%x \n",(pbt_buf[fwsize - 8] ^ pbt_buf[fwsize - 6]),
		(pbt_buf[fwsize - 7] ^ pbt_buf[fwsize - 5]),(pbt_buf[fwsize - 3] ^ pbt_buf[fwsize - 4]));
	
	if ((pbt_buf[fwsize - 8] ^ pbt_buf[fwsize - 6]) == 0xFF
		&& (pbt_buf[fwsize - 7] ^ pbt_buf[fwsize - 5]) == 0xFF
		&& (pbt_buf[fwsize - 3] ^ pbt_buf[fwsize - 4]) == 0xFF) {
		/*call the upgrade function */
		printk("[FT6306_llf] Begain to Upgrade firmware \n");
		i_ret = fts_ctpm_fw_upgrade(pbt_buf, fwsize);
		if (i_ret != 0){
			kfree(pbt_buf);
			dev_dbg(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",__func__);
			}
		else {
			kfree(pbt_buf);
			fts_ctpm_auto_clb();	/*start auto CLB*/
		 }
	} else {
		dev_dbg(&client->dev, "%s:FW format error\n", __func__);
		printk("[FT6306_llf] Firmware file format error\n");
		kfree(pbt_buf);
		return -EIO;
	}
	return i_ret;
}

static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	char fwname[128];
	unsigned char fm_ver;

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	printk("[FT6306_llf]Input file name is %s \n",fwname);

	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(300);	
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	fm_ver = ft5x0x_read_fw_ver();
	//i2c_smbus_read_i2c_block_data(i2c_clientft, 0xA6, 1, &fm_ver);
	printk("[FT6306_llf] current firmware version is  0x%x\n", fm_ver);
	
	ft5x0x_read_reg(0xA8, &tp_vendor);
	//i2c_smbus_read_i2c_block_data(i2c_clientft, 0xA8, 1, &tp_vendor);
	//ID detect 
	if((tp_vendor != 0x5a) && (tp_vendor != 0x53) && (tp_vendor != 0x80)&& (tp_vendor != 0x85)){
		//destory #######################################
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		printk("[%s] can't find ft6306,vendor:0x%x\n",__func__,tp_vendor);
		return -1;
	}
	printk("FT6306_llf] ft6306 vendor is 0x%x\n", tp_vendor);

	mutex_lock(&g_device_mutex);
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	fts_ctpm_fw_upgrade_with_app_file(i2c_clientft, fwname);

	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
	mutex_unlock(&g_device_mutex);

	printk("[FT6306_llf] Firmware Upgrade Successful!\n");

	fm_ver = ft5x0x_read_fw_ver();
	
	printk("[FT6306_llf]  After Upgrade firmware version is  0x%x\n", fm_ver);

	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(300);	
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	i2c_clientft->timing = 100;
	return count;
}
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO | S_IWUSR, ft5x0x_fwupgradeapp_show,
			ft5x0x_fwupgradeapp_store);

#endif

static ssize_t firmware_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char tp_version;
	tp_version=ft5x0x_read_fw_ver();
	TPD_DMESG("*** firmware_version_show fw_version = 0x%x***\n", tp_version);
	return sprintf(buf, "fw_version = 0x%x \n", tp_version);
}

static DEVICE_ATTR(version, 0444, firmware_version_show,NULL);

static  void tpd_down(int x, int y, int p) {
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
	 input_report_abs(tpd->dev, ABS_PRESSURE, 128);
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 //TPD_DMESG("D[%4d %4d %4d] ", x, y, p);
	 input_mt_sync(tpd->dev);
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 1);  
	}	 
	 TPD_DOWN_DEBUG_TRACK(x,y);
 }
 
static  int tpd_up(int x, int y,int *count) {
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(tpd->dev, ABS_PRESSURE, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//TPD_DMESG("U[%4d %4d %4d] ", x, y, 0);
	input_mt_sync(tpd->dev);
	TPD_UP_DEBUG_TRACK(x,y);
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 0); 
	}   		 
	//return 1;
	 //} 
	return 0;
 }

//#define SWAP_X_Y   (1)
#ifdef SWAP_X_Y
	int tempx;
	int tempy;
#endif

 static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {
	int i = 0;

	char data[30] = {0};

	u16 high_byte,low_byte;

	p_point_num = point_num;
	
	i2c_smbus_read_i2c_block_data(i2c_clientft, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_clientft, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_clientft, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_clientft, 0xa6, 1, &(data[24]));
	//TPD_DMESG("FW version=%x]\n",data[24]);
	
	//TPD_DMESG("received raw data from touch panel as following:\n");
	//TPD_DMESG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	//TPD_DMESG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DMESG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if(data[0] & 0x70 != 0) return false; 

	/*get the number of the touch points*/
	point_num= data[2] & 0x07;
	
	//TPD_DMESG("point_num =%d\n",point_num);
	
//	if(point_num == 0) return false;

	  //TPD_DMESG("Procss raw data...\n");

		for(i = 0; i < point_num; i++)
		{
			cinfo->p[i] = data[3+6*i] >> 6; //event flag 

	       /*get the X coordinate, 2 bytes*/
			high_byte = data[3+6*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i + 1];
			cinfo->x[i] = high_byte |low_byte;

				//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
		
			/*get the Y coordinate, 2 bytes*/
			
			high_byte = data[3+6*i+2];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i+3];
			cinfo->y[i] = high_byte |low_byte;

			  //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
			//FR 448954 llf 20130506 begain
#ifdef SWAP_X_Y
			if(cinfo->y[i]<=800){

				tempx =480 -cinfo->x[i];
				cinfo->x[i] =tempx;
				tempy =800 -cinfo->y[i];
				cinfo->y[i] =tempy;
				TPD_DMESG("After change x[i] = %d, y[i] = %d, p[i] = %d\n", cinfo->x[i], cinfo->y[i], cinfo->p[i]);
			}
#endif
			//FR 448954 llf 20130506 end
			cinfo->count++;
		}
		//TPD_DMESG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
		//TPD_DMESG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
		//TPD_DMESG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	 return true;

 };

 static int touch_event_handler(void *unused)
 {
  
	struct touch_info cinfo, pinfo;
	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
	 
	 do
	 {
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
	
	set_current_state(TASK_INTERRUPTIBLE); 
	wait_event_interruptible(waiter,tpd_flag!=0);				 
	tpd_flag = 0;		 
	set_current_state(TASK_RUNNING);

	if (tpd_touchinfo(&cinfo, &pinfo)) {
		//TPD_DMESG("point_num = %d\n",point_num);

		if(point_num >0) {
			tpd_down(cinfo.x[0], cinfo.y[0], 1);
			if(point_num>1)
			{
			tpd_down(cinfo.x[1], cinfo.y[1], 1);
			if(point_num >2) tpd_down(cinfo.x[2], cinfo.y[2], 1);
			}
			input_sync(tpd->dev);
			//TPD_DMESG("press --->\n");

		} else  {
			tpd_up(cinfo.x[0], cinfo.y[0], 0);
			//TPD_DMESG("release --->\n"); 
			//input_mt_sync(tpd->dev);
			input_sync(tpd->dev);
			}
		}

	}while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
 	TPD_DMESG("[FT6306]--------------------------------------tpd_detect\n",__func__);
	 strcpy(info->type, TPD_DEVICEFT);	
	 return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 TPD_DMESG("TPD interrupt has been triggered\n");
	 tpd_flag = 1; 
	 wake_up_interruptible(&waiter); 
 }
 
//=============================ADB upgrade device Inode "/sys/tp-information/ftsfwupgradeapp"============================
//static DEVICE_ATTR(ftsrawdatashow, S_IRUGO | S_IWUSR, ft5x0x_rawdata_show,
			//ft5x0x_rawdata_store);

//static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO | S_IWUSR, ft5x0x_fwupgradeapp_show,
			//ft5x0x_fwupgradeapp_store);

/*add your attr in here*/
static struct attribute *ft5x0x_attributes[] = {
	//&dev_attr_ftsrawdatashow.attr,
        &dev_attr_ftsfwupgradeapp.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes
};



//add sysfs
struct kobject *TP_ctrl_kobj;

int TP_sysfs_init(void)
{ 
	TP_ctrl_kobj = kobject_create_and_add("tp-upgradeapp", NULL);
	if (!TP_ctrl_kobj)
		return -ENOMEM;

        mutex_init(&g_device_mutex);

	return sysfs_create_group(TP_ctrl_kobj, &ft5x0x_attribute_group);
}
//remove sysfs
void TP_sysfs_exit(void)
{
	sysfs_remove_group(TP_ctrl_kobj, &ft5x0x_attribute_group);

        mutex_destroy(&g_device_mutex);

	kobject_put(TP_ctrl_kobj);
}
//===========================ADB upgrade END=============================================

//===========================TP info===================================================
static ssize_t tp_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
	unsigned char chip_fw_ver, tp_factory_id_cmd;
	if (ft5x0x_read_reg(0xa6, &chip_fw_ver) < 0) {
		printk("read chip version is error\n");
	}
	
	if((ft5x0x_read_reg(0xa8, &tp_factory_id_cmd))< 0)
	    {
		   TPD_DMESG("I2C read factory ID CMD mode error, line: %d\n", __LINE__);
	    }
	printk("tp_factory_id_cmd = %x, read ID !\n", tp_factory_id_cmd); 
	printk("FW Version: %x\n", chip_fw_ver);
	s += sprintf(s, "FW Factory ID CMD: %x\n", tp_factory_id_cmd);
	s += sprintf(s, "CTP Vendor: %s\n", "FocalTech");
	s += sprintf(s, "FW Version: %x\n", chip_fw_ver);
        s += sprintf(s,"CTP Type: %s\n", "Interrupt trigger");
	return (s - buf);
}

static ssize_t tp_value_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	int save;

	if (sscanf(buf, "%d", &save)==0) {
		printk(KERN_ERR "%s -- invalid save string '%s'...\n", __func__, buf);
		return -EINVAL;
	}

	return n;
}
struct kobject *tpswitch_ctrl_kobj;

#define tpswitch_ctrl_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

static ssize_t name_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", "ft6306");
}
static ssize_t name_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
	return 0;
}
tpswitch_ctrl_attr(name);
tpswitch_ctrl_attr(tp_value);
static struct attribute *g_attr[] = {
	&tp_value_attr.attr,
	&name_attr.attr,	
	NULL,
};

static struct attribute_group tpswitch_attr_group = {
	.attrs = g_attr,
};

static int tpswitch_sysfs_init(void)
{ 
	tpswitch_ctrl_kobj = kobject_create_and_add("tp_information", NULL);
	if (!tpswitch_ctrl_kobj)
		return -ENOMEM;

	return sysfs_create_group(tpswitch_ctrl_kobj, &tpswitch_attr_group);
}

static void tpswitch_sysfs_exit(void)
{
	sysfs_remove_group(tpswitch_ctrl_kobj, &tpswitch_attr_group);

	kobject_put(tpswitch_ctrl_kobj);
}
//===========================TP info END===================================================
#define FTS_APK_DEBUG
#ifdef FTS_APK_DEBUG
#define PROC_UPGRADE						0
#define PROC_READ_REGISTER			1
#define PROC_WRITE_REGISTER	    2
#define PROC_RAWDATA						3
#define PROC_AUTOCLB						4
#define PROC_UPGRADE_INFO 			5
#define PROC_WRITE_DATA        	6
#define PROC_READ_DATA        	7

#define PROC_NAME	"ft5x0x-debug"
#define PROC_NAME1	"boot_status"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
static struct proc_dir_entry *fts_proc_entry1;

static int g_upgrade_successful = 0;
/*interface of write proc*/
int fts_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret,i;
	
#ifndef MTK_I2C_NEED_DMA_TRANSFER//for normal I2c transfer
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
#else// for DMA I2c transfer
	if(writelen!=0)
	{
		//DMA Write
		if(writelen < 8  )
		{
#if (defined (MT6589) || defined (MT8389))
			client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
#endif			
			//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
			ret= i2c_master_send(client, writebuf, writelen);
		}
		else
		{
			for(i = 0 ; i < writelen; i++)
			{
				I2CDMABuf_va[i] = writebuf[i];
			}
#if (defined (MT6589) || defined (MT8389))
			client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
#else//if (defined MT6572)
			client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
#endif			
			if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
				dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
			//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
			//return ret;
		}
	}
	//DMA Read 
	if(readlen!=0)
	{
		if (readlen <8) {
#if (defined (MT6589) || defined (MT8389))
			client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
#else//if (defined MT6572)
			client->addr = client->addr & I2C_MASK_FLAG;
#endif			
			ret = i2c_master_recv(client, (unsigned char *)readbuf, readlen);
		}
		else
		{
#if (defined (MT6589) || defined (MT8389))
			client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
#else//if (defined MT6572)
			client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
#endif			
			
			ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, readlen);

			for(i = 0; i < readlen; i++)
	        {
	            readbuf[i] = I2CDMABuf_va[i];
	        }
		}
	}
#endif
	return ret;
}
/*write data by i2c*/

int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int i = 0;
	//printk("fts_i2c_Write  %d  %x  %x %x %x",writelen,writebuf,I2CDMABuf_va,I2CDMABuf_pa,client);
	
   client->addr = client->addr & I2C_MASK_FLAG;
  // client->ext_flag |= I2C_DIRECTION_FLAG; 
  // client->timing = 100;
#ifndef MTK_I2C_NEED_DMA_TRANSFER//for normal I2c transfer
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
#else
	
	if(writelen < 8)
	{
#if (defined (MT6589) || defined (MT8389))
		client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
#endif			
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		ret = i2c_master_send(client, writebuf, writelen);
	}
	else
	{
		for(i = 0 ; i < writelen; i++)
		{
			I2CDMABuf_va[i] = writebuf[i];
		}
#if (defined (MT6589) || defined (MT8389))
		client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;	
#else//if (defined MT6572)
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
#endif
		if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
			dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
	} 
#endif
	return ret;
}

int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_Write(client, buf, sizeof(buf));
}


int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	unsigned char writebuf[FTS_PACKET_LENGTH];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		printk("%s,%d:PROC_UPGRADE\n",__func__,__LINE__);
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			//DBG("%s\n", upgrade_file_path);
			mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

			g_upgrade_successful = 0;
			ret = fts_ctpm_fw_upgrade_with_app_file(client, upgrade_file_path);

			mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
			else
			{
				g_upgrade_successful = 1;
			}
		}
		break;
	case PROC_READ_REGISTER:
		printk("%s,%d:PROC_READ_REGISTER\n",__func__,__LINE__);
		writelen = 1;
		//DBG("%s:register addr=0x%02x\n", __func__, writebuf[1]);
		ret = fts_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		printk("%s,%d:PROC_WRITE_REGISTER\n",__func__,__LINE__);
		writelen = 2;
		ret = fts_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_RAWDATA:
		printk("%s,%d:PROC_RAWDATA\n",__func__,__LINE__);
		break;
#ifdef AUTO_CLB
	case PROC_AUTOCLB:
		fts_ctpm_auto_clb(client);
		break;
#endif
	case PROC_UPGRADE_INFO:
		printk("%s,%d:PROC_UPGRADE_INFO\n",__func__,__LINE__);
		break;
  case PROC_READ_DATA:                                                    
  case PROC_WRITE_DATA:                                                   
 		printk("%s,%d:PROC_READ_DATA,PROC_WRITE_DATA\n",__func__,__LINE__);      
    writelen = len - 1;                                                
    ret = fts_i2c_Write(client, writebuf + 1, writelen);            
    if (ret < 0) {                                                     
          dev_err(&client->dev, "%s:write iic error\n", __func__);     
          return ret;                                                  
    }                                                                  
    break;                                                             
	default:
		break;
	}
	

	return len;
}
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
	int ret = 0, err = 0;
	u8 tx = 0, rx = 0;
	int i, j;
	unsigned char buf[1024];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		printk("%s,%d:PROC_UPGRADE\n",__func__,__LINE__);    
		/*after calling fts_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = fts_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		printk("%s,%d:PROC_READ_REGISTER\n",__func__,__LINE__);    
		readlen = 1;
		ret = fts_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} else
			//DBG("%s:value=0x%02x\n", __func__, buf[0]);
		num_read_chars = 1;
		break;
	case PROC_RAWDATA:
		printk("%s,%d:PROC_RAWDATA\n",__func__,__LINE__);    
		break;
	case PROC_UPGRADE_INFO:
		printk("%s,%d:PROC_UPGRADE_INFO\n",__func__,__LINE__);    
		if(1 == g_upgrade_successful)
			buf[0] = 1;
		else
			buf[0] = 0;
		//DBG("%s:value=0x%02x\n", __func__, buf[0]);
		num_read_chars = 1;
		break;
  case PROC_READ_DATA:                                                              
		printk("%s,%d:PROC_READ_DATA\n",__func__,__LINE__);                                
    readlen = count;                                                             
    ret = fts_i2c_Read(client, NULL, 0, buf, readlen);                        
    if (ret < 0) {                                                               
          dev_err(&client->dev, "%s:read iic error\n", __func__);                
          return ret;                                                            
    }                                                                            
                                                                                 
    num_read_chars = readlen;                                                    
    break;                                                                       
  case PROC_WRITE_DATA:                                                             
		printk("%s,%d:PROC_WRITE_DATA\n",__func__,__LINE__);                               
    break;                                                                       
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);

	return num_read_chars;

}

int fts_create_apk_debug_channel(struct i2c_client * client)
{
	fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	if (NULL == fts_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
		fts_proc_entry->data = client;
		fts_proc_entry->write_proc = fts_debug_write;
		fts_proc_entry->read_proc = fts_debug_read;
	}
	return 0;
}
void fts_release_apk_debug_channel(void)
{
	if (fts_proc_entry)
		remove_proc_entry(PROC_NAME, NULL);
}
#endif
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 

 	TPD_DMESG("[FT6306]--------------------------------------tpd_probe\n",__func__);
	int retval = TPD_OK;
	int ret;
	char data;

	i2c_clientft = client;

	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
#ifdef MT6575
	//power on, need confirm with SA
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_3300, "TP");
	hwPowerOn(MT65XX_POWER_LDO_VGP1, VOL_1800, "TP");  
#endif 
#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
#endif
	msleep(5);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(200);

	if((i2c_smbus_read_i2c_block_data(i2c_clientft, 0x00, 1, &data))< 0)
	{
		printk("[CTP-I2C-ERROR] transfer error addr=0x%x------llf, line: %d\n",i2c_clientft->addr, __LINE__);
		return -1;
	}
	else
	{
		tpd_load_status = 1;
		printk("[CTP-I2C-Successful] test OK addr =0x%x------llf, line: %d\n",i2c_clientft->addr, __LINE__);

	}
	tpswitch_sysfs_init(); // TP information
	TP_sysfs_init(); //adb upgrade init
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	msleep(100);
#ifdef UPGRADE_FIRMWARE
	tp_fw_ver = ft5x0x_read_fw_ver();
	ft5x0x_read_reg(0xA8, &tp_vendor);
	
	if((tp_vendor != 0x5a) && (tp_vendor != 0x53) && (tp_vendor != 0x80)&& (tp_vendor != 0x85))
	{
		tp_vendor = fts_ctpm_get_panel_factory_setting();
	}
	if(0x0 == tp_fw_ver)
	{
		fts_ctpm_get_panel_factory_setting();
	}
	//ID detect 
	if((tp_vendor != 0x5a) && (tp_vendor != 0x53) && (tp_vendor != 0x80)&& (tp_vendor != 0x85)){
		//destory #######################################
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		TPD_DMESG("[%s] can't find ft6306,vendor:0x%x\n",__func__,tp_vendor);
		return -1;
	}
//	fts_ctpm_get_panel_factory_setting();
	fts_ctpm_auto_upg(tp_vendor);
#endif

	threadft = kthread_run(touch_event_handler, 0, TPD_DEVICEFT);
	 if (IS_ERR(threadft))
		 { 
		  retval = PTR_ERR(threadft);
		  TPD_DMESG(TPD_DEVICEFT " failed to create kernel thread: %d\n", retval);
		}

	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(300);	
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef APK_UPGRADE
	mutex_init(&g_device_mutex);
	if(device_create_file(&i2c_clientft->dev, &dev_attr_ftsfwupgradeapp)>=0)
	{            
		printk("device_create_file_ftsfwupgradeapp \n");        
	}
#endif
  #ifdef FTS_APK_DEBUG
    fts_create_apk_debug_channel(i2c_clientft);
  #endif
#ifdef CTP_FACE_PROBE
	ret = misc_register(&ft6306_pls_device);
	if (ret != 0) {
		printk("cannot register miscdev on ft6306_pls_device\n");
	}
#endif
	if(device_create_file(&i2c_clientft->dev, &dev_attr_version)>=0)
	{            
		printk("device_create_file_version \n");        
	}

	#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",__func__);
	#endif

	return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client) 
 {
  #ifdef FTS_APK_DEBUG
    fts_release_apk_debug_channel();
  #endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	TPD_DMESG("TPD removed\n");
	return 0;
 }
 
 
 static int tpd_local_init(void)
 {
	TPD_DMESG("[FT6306]--------------------------------------tpd_local_init\n");

	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	if(tpd_load_status==0)
	{
		TPD_DMESG("[FT6306] add i2c driver faild.\n");
		//i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
	return 0; 
 }

 static void tpd_resume( struct early_suspend *h )
 {
	int retval = TPD_OK;
	
	TPD_DMESG("TPD wake up******************************************\n");
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(5);
#else
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(300);
	
	//i2c_smbus_write_i2c_block_data(i2c_clientft, 0xA5, 1, &data);  //TP wake from sleep mode
#endif

	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	
	 //return retval;
 }
 
 static int tpd_suspend(struct i2c_client *client)
 {
	int retval = TPD_OK;
	static char data[2] ={0xA5,0x03};

	TPD_DMESG("TPD enter sleep******************************************\n");

#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
	TPD_DMESG("TP enter sleep mode_____________________________________\n");
	retval=i2c_master_send(i2c_clientft, data, 2);
	//i2c_smbus_write_i2c_block_data(i2c_clientft, 0xA5, 1, &data);  //TP enter sleep mode
#endif

	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	
	 return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "FT6306",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
	TPD_DMESG("MediaTek FT6306 touch panel driver init\n");
	i2c_register_board_info(1, &i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT6306 driver failed\n");
	return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek FT6306 touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);
