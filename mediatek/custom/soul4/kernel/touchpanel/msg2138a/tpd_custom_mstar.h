
#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#define SOUL4_KK_SUPPORT

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE         
#define TPD_I2C_NUMBER           1
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_VELOCITY_CUSTOM_X 12
#define TPD_VELOCITY_CUSTOM_Y 16


#define TPD_DELAY                (2*HZ/100)
//#define TPD_RES_X                480
//#define TPD_RES_Y                800
#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_BUTTON
//#define TPD_HAVE_TREMBLE_ELIMINATION
#define TPD_HAVE_BUTTON

#define TPD_BUTTON_HEIGH        (60)
#define TPD_KEY_COUNT           3
#define TPD_KEYS                { KEY_BACK,KEY_HOMEPAGE,KEY_MENU}  // KEY_BACK,KEY_HOME,RECENTAPP,key 229   MENU,key 139   MENU,key 59    MENU
#define TPD_KEYS_DIM            {{60,870,50,TPD_BUTTON_HEIGH},{170,870,50,TPD_BUTTON_HEIGH},{300,870,50,TPD_BUTTON_HEIGH}}
#endif /* TOUCHPANEL_H__ */
