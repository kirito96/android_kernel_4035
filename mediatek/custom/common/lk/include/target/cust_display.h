


#ifndef __CUST_DISPLAY_H__
#define __CUST_DISPLAY_H__

#define VERSION3_CHARGING_ANIMATION

// color
#define BAR_OCCUPIED_COLOR  (0x07E0)    // Green
#define BAR_EMPTY_COLOR     (0xFFFF)    // White
#define BAR_BG_COLOR        (0x0000)    // Black

// LOGO number
#define ANIM_V0_LOGO_NUM   5            // version 0: show 4 recatangle growing animation without battery number
#ifdef VERSION3_CHARGING_ANIMATION
#define ANIM_V1_LOGO_NUM   44
#else
#define ANIM_V1_LOGO_NUM   39           // version 1: show wave animation with  battery number 
#endif
#define ANIM_V2_LOGO_NUM   68           // version 2: show wireless charging animation      

// Common LOGO index
#define BOOT_LOGO_INDEX   0 
#define KERNEL_LOGO_INDEX   43 

#define ANIM_V0_BACKGROUND_INDEX   1 
#define ANIM_V1_BACKGROUND_INDEX   35
 
 
#define LOW_BATTERY_INDEX   2 
#define CHARGER_OV_INDEX   3 
#define FULL_BATTERY_INDEX   37 

//version3 
//the label rect local

#define VERSION3_BATTERY_CAPACITY     38
#define VERSION3_BATTERY_CAPACITY_LOW     39
#define VERSION3_POWER_OFF_CHARGING_BG  40
#define VERSION3_POWER_OFF_CHARGING_BG1   41
#define VERSION3_POWER_OFF_CHARGING_FULL   42 

// version 1: show wave animation with  battery number 

// NUMBER LOGO INDEX
#define NUMBER_PIC_START_0   4 
#define NUMBER_PIC_PERCENT   14 

// DYNAMIC ANIMATION LOGO INDEX
#define BAT_ANIM_START_0   15 

// LOW BATTERY(0~10%) ANIMATION LOGO
#define LOW_BAT_ANIM_START_0    25 

#define ANIM_LINE_INDEX   36 


// version 2: show wireless charging animation logo index

#define V2_NUM_START_0_INDEX  39  
#define V2_NUM_PERCENT_INDEX  49 
 
#define V2_BAT_0_10_START_INDEX     50  
#define V2_BAT_10_40_START_INDEX    54 
#define V2_BAT_40_80_START_INDEX    58 
#define V2_BAT_80_100_START_NDEX   62

#define V2_BAT_0_INDEX   66
#define V2_BAT_100_INDEX   67

#if defined(FHD) || defined(CU_FHD) || defined(CMCC_FHD) || defined(CT_FHD)
	// fhd 1080*1920
	
	// battery capacity rectangle
	#define CAPACITY_LEFT                (387) // battery capacity center
	#define CAPACITY_TOP                 (802)
	#define CAPACITY_RIGHT               (691)
	#define CAPACITY_BOTTOM              (1292)

	// first number rectangle
	#define NUMBER_LEFT                  (351+84) // number
	#define NUMBER_TOP                   (479)
	#define NUMBER_RIGHT                 (435+84)
	#define NUMBER_BOTTOM                (600)

	// %  rectangle
	#define PERCENT_LEFT                 (519+84) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (479)
	#define PERCENT_RIGHT                (627+84)
	#define PERCENT_BOTTOM               (600)

	// top animation part
	#define TOP_ANIMATION_LEFT           (387) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (691)
	#define TOP_ANIMATION_BOTTOM         (152)

	// for old animation
	#define BAR_LEFT            (470)
	#define BAR_TOP             (356)
	#define BAR_RIGHT           (610)
	#define BAR_BOTTOM          (678)

#elif defined(WQHD) || defined(CU_WQHD) || defined(CMCC_WQHD) || defined(CT_WQHD)
	// hd720 720*1280

	// battery capacity rectangle
	#define CAPACITY_LEFT                (393) // battery capacity center
	#define CAPACITY_TOP                 (786)
	#define CAPACITY_RIGHT               (623)
	#define CAPACITY_BOTTOM              (1155)

	// first number rectangle
	#define NUMBER_LEFT                  (410) // number
	#define NUMBER_TOP                   (546)
	#define NUMBER_RIGHT                 (474)
	#define NUMBER_BOTTOM                (636)

	// %  rectangle
	#define PERCENT_LEFT                 (537) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (546)
	#define PERCENT_RIGHT                (618)
	#define PERCENT_BOTTOM               (636)

	// top animation part
	#define TOP_ANIMATION_LEFT           (393) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (624)
	#define TOP_ANIMATION_BOTTOM         (182)

	// for old animation
	#define BAR_LEFT            (443)
	#define BAR_TOP             (337)
	#define BAR_RIGHT           (574)
	#define BAR_BOTTOM          (641)

#elif defined(HD720) || defined(CU_HD720) || defined(CMCC_HD720) || defined(CT_HD720)
	// hd720 720*1280

	// battery capacity rectangle
	#define CAPACITY_LEFT                (278) // battery capacity center
	#define CAPACITY_TOP                 (556)
	#define CAPACITY_RIGHT               (441)
	#define CAPACITY_BOTTOM              (817)

	// first number rectangle
	#define NUMBER_LEFT                  (290) // number
	#define NUMBER_TOP                   (386)
	#define NUMBER_RIGHT                 (335)
	#define NUMBER_BOTTOM                (450)

	// %  rectangle
	#define PERCENT_LEFT                 (380) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (386)
	#define PERCENT_RIGHT                (437)
	#define PERCENT_BOTTOM               (450)

	// top animation part
	#define TOP_ANIMATION_LEFT           (278) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (441)
	#define TOP_ANIMATION_BOTTOM         (129)

	// for old animation
	#define BAR_LEFT            (313)
	#define BAR_TOP             (238)
	#define BAR_RIGHT           (406)
	#define BAR_BOTTOM          (453)

#elif defined(FWVGA) || defined(CU_FWVGA) || defined(CMCC_FWVGA) || defined(CT_FWVGA)
	// fwvga 480*854

	// battery capacity rectangle
	#define CAPACITY_LEFT                (172) // battery capacity center
	#define CAPACITY_TOP                 (357)
	#define CAPACITY_RIGHT               (307)
	#define CAPACITY_BOTTOM              (573)

	// first number rectangle
	#define NUMBER_LEFT                  (172) // number
	#define NUMBER_TOP                   (213)
	#define NUMBER_RIGHT                 (210)
	#define NUMBER_BOTTOM                (267)

	// %  rectangle
	#define PERCENT_LEFT                 (248) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (213)
	#define PERCENT_RIGHT                (296)
	#define PERCENT_BOTTOM               (267)

	// top animation part
	#define TOP_ANIMATION_LEFT           (172) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (307)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (184)
	#define BAR_TOP             (227)
	#define BAR_RIGHT           (294)
	#define BAR_BOTTOM          (437)

#elif defined(QHD) || defined(CU_QHD) || defined(CMCC_QHD) || defined(CT_QHD)
	// qhd 540*960

	// battery capacity rectangle
	#define CAPACITY_LEFT                (202) // battery capacity center
	#define CAPACITY_TOP                 (410)
	#define CAPACITY_RIGHT               (337)
	#define CAPACITY_BOTTOM              (626)

	// first number rectangle
	#define NUMBER_LEFT                  (202) // number
	#define NUMBER_TOP                   (266)
	#define NUMBER_RIGHT                 (240)
	#define NUMBER_BOTTOM                (320)

	// %  rectangle
	#define PERCENT_LEFT                 (278) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (266)
	#define PERCENT_RIGHT                (326)
	#define PERCENT_BOTTOM               (320)

	// top animation part
	#define TOP_ANIMATION_LEFT           (202) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (337)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (235)
	#define BAR_TOP             (179)
	#define BAR_RIGHT           (305)
	#define BAR_BOTTOM          (340)

#elif defined(WVGA) || defined(CU_WVGA) || defined(CMCC_WVGA) || defined(CT_WVGA)
	// default wvga 480*800
 #ifdef VERSION3_CHARGING_ANIMATION
	//flashing coordinate include point a,b,c,d,e,f
	#define POINT_A_X  	(258)
	#define POINT_A_Y	(369)
	#define POINT_B_X	(216)
	#define POINT_B_Y	(411)
	#define POINT_C_X  	(189)
	#define POINT_C_Y	(411)
	#define POINT_D_X	(246)
	#define POINT_D_Y	(407)
	#define POINT_E_X  	(264)
	#define POINT_E_Y	(407)
	#define POINT_F_X	(222)
	#define POINT_F_Y	(449)

// battery capacity rectangle
	#define CAPACITY_LEFT                (184) // battery capacity center
	#define CAPACITY_TOP                 (306)
	#define CAPACITY_RIGHT               (294)
	#define CAPACITY_BOTTOM              (510)
	
  #else
	#define CAPACITY_LEFT                (172) // battery capacity center
	#define CAPACITY_TOP                 (330)
	#define CAPACITY_RIGHT               (307)
	#define CAPACITY_BOTTOM              (546)
#endif
	// first number rectangle
	#define NUMBER_LEFT                  (178) // number
	#define NUMBER_TOP                   (190)
	#define NUMBER_RIGHT                 (216)
	#define NUMBER_BOTTOM                (244)

	// %  rectangle
	#define PERCENT_LEFT                 (254) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (190)
	#define PERCENT_RIGHT                (302)
	#define PERCENT_BOTTOM               (244)

	// top animation part
	#define TOP_ANIMATION_LEFT           (172) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (307)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (209)
	#define BAR_TOP             (149)
	#define BAR_RIGHT           (271)
	#define BAR_BOTTOM          (282)

#elif defined(HVGA) || defined(CU_HVGA) || defined(CMCC_HVGA) || defined(CT_HVGA)

	// hvga 320*480
#ifdef VERSION3_CHARGING_ANIMATION
	//flashing coordinate include point a,b,c,d,e,f
	#define POINT_A_X  	(172)
	#define POINT_A_Y	(193)
	#define POINT_B_X	(145)
	#define POINT_B_Y	(222)
	#define POINT_C_X  	(157)
	#define POINT_C_Y	(222)
	#define POINT_D_X	(165)
	#define POINT_D_Y	(218)
	#define POINT_E_X  	(177)
	#define POINT_E_Y	(218)
	#define POINT_F_X	(150)
	#define POINT_F_Y	(246)
// battery capacity rectangle
	#define CAPACITY_LEFT                (127) // battery capacity center
	#define CAPACITY_TOP                 (162)
	#define CAPACITY_RIGHT               (193)
	#define CAPACITY_BOTTOM              (285)
	
#else
	#define CAPACITY_LEFT                (109) // battery capacity center
	#define CAPACITY_TOP                 (189)
	#define CAPACITY_RIGHT               (211)
	#define CAPACITY_BOTTOM              (350)
#endif

	// first number rectangle
	#define NUMBER_LEFT                  (126) // number
	#define NUMBER_TOP                   (95)
	#define NUMBER_RIGHT                 (153)
	#define NUMBER_BOTTOM                (131)

	// %  rectangle
	#define PERCENT_LEFT                 (180) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (95)
	#define PERCENT_RIGHT                (212)
	#define PERCENT_BOTTOM               (131)

	// top animation part
	#define TOP_ANIMATION_LEFT           (109) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (211)
	#define TOP_ANIMATION_BOTTOM         (118)

	// for old animation
	#define BAR_LEFT            (129)
	#define BAR_TOP             (128)
	#define BAR_RIGHT           (190)
	#define BAR_BOTTOM          (245)
#elif defined(QVGA) || defined(CU_QVGA) || defined(CMCC_QVGA) || defined(CT_QVGA)

	// hvga 320*480

	// battery capacity rectangle
	#define CAPACITY_LEFT                (82) // battery capacity center
	#define CAPACITY_TOP                 (124)
	#define CAPACITY_RIGHT               (158)
	#define CAPACITY_BOTTOM              (241)

	// first number rectangle
	#define NUMBER_LEFT                  (93) // number
	#define NUMBER_TOP                   (50)
	#define NUMBER_RIGHT                 (109)
	#define NUMBER_BOTTOM                (73)

	// %  rectangle
	#define PERCENT_LEFT                 (125) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (50)
	#define PERCENT_RIGHT                (145)
	#define PERCENT_BOTTOM               (73)

	// top animation part
	#define TOP_ANIMATION_LEFT           (82) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (158)
	#define TOP_ANIMATION_BOTTOM         (113)

	// for old animation
	#define BAR_LEFT            (97)
	#define BAR_TOP             (96)
	#define BAR_RIGHT           (140)
	#define BAR_BOTTOM          (184)
	
#elif defined(WSVGA)
	// wsvga 600*1024

	// battery capacity rectangle
	#define CAPACITY_LEFT                (232) // battery capacity center
	#define CAPACITY_TOP                 (442)
	#define CAPACITY_RIGHT               (367)
	#define CAPACITY_BOTTOM              (658)

	// first number rectangle
	#define NUMBER_LEFT                  (250) // number
	#define NUMBER_TOP                   (300)
	#define NUMBER_RIGHT                 (288)
	#define NUMBER_BOTTOM                (354)

	// %  rectangle
	#define PERCENT_LEFT                 (326) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (300)
	#define PERCENT_RIGHT                (374)
	#define PERCENT_BOTTOM               (354)

	// top animation part
	#define TOP_ANIMATION_LEFT           (232) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (367)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (260)
	#define BAR_TOP             (190)
	#define BAR_RIGHT           (338)
	#define BAR_BOTTOM          (360)

#elif defined(WSVGANL)
	// wsvganl 1024*600

	// battery capacity rectangle
	#define CAPACITY_LEFT                (444) // battery capacity center
	#define CAPACITY_TOP                 (230)
	#define CAPACITY_RIGHT               (579) // 444 + 135
	#define CAPACITY_BOTTOM              (446)

	// first number rectangle
	#define NUMBER_LEFT                  (466) // number
	#define NUMBER_TOP                   (90)
	#define NUMBER_RIGHT                 (504) // 466 + 38
	#define NUMBER_BOTTOM                (144) // 90 + 54

	// %  rectangle
	#define PERCENT_LEFT                 (542) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (90)
	#define PERCENT_RIGHT                (590)
	#define PERCENT_BOTTOM               (144)

	// top animation part
	#define TOP_ANIMATION_LEFT           (444) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (579)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (414)
	#define BAR_TOP             (186)
	#define BAR_RIGHT           (608)
	#define BAR_BOTTOM          (477)

#elif defined(WXGANL)
	// wxganl 1280*800

	// battery capacity rectangle
	#define CAPACITY_LEFT                (558) // battery capacity center
	#define CAPACITY_TOP                 (265)
	#define CAPACITY_RIGHT               (721) //558+163
	#define CAPACITY_BOTTOM              (525)

	#define NUMBER_LEFT                  (585) // number
	#define NUMBER_TOP                   (95)
	#define NUMBER_RIGHT                 (630)//585+45
	#define NUMBER_BOTTOM                (159) //95+64

	#define PERCENT_LEFT                 (675) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (95)
	#define PERCENT_RIGHT                (732) //675+57
	#define PERCENT_BOTTOM               (159)

	#define TOP_ANIMATION_LEFT           (558) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (721)
	#define TOP_ANIMATION_BOTTOM         (129)

	// for old animation
	#define BAR_LEFT            (525)
	#define BAR_TOP             (250)
	#define BAR_RIGHT           (755)
	#define BAR_BOTTOM          (640)
	
#elif defined(WXGA)
	// wxga 800*1280

	// battery capacity rectangle
	#define CAPACITY_LEFT                (318) // battery capacity center
	#define CAPACITY_TOP                 (556)
	#define CAPACITY_RIGHT               (481) //318+163
	#define CAPACITY_BOTTOM              (815)

	#define NUMBER_LEFT                  (345) // number
	#define NUMBER_TOP                   (385)
	#define NUMBER_RIGHT                 (390) //345+45
	#define NUMBER_BOTTOM                (449) //385+64

	#define PERCENT_LEFT                 (435) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (385)
	#define PERCENT_RIGHT                (492) //435+57
	#define PERCENT_BOTTOM               (449)

	#define TOP_ANIMATION_LEFT           (318) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (481)
	#define TOP_ANIMATION_BOTTOM         (129)

	// for old animation
	#define BAR_LEFT            (348)
	#define BAR_TOP             (238)
	#define BAR_RIGHT           (453)
	#define BAR_BOTTOM          (452)
	
#elif defined(WUXGANL)
	// wuxganl 1920*1200

	// battery capacity rectangle
	#define CAPACITY_LEFT                (806) // battery capacity center
	#define CAPACITY_TOP                 (443)
	#define CAPACITY_RIGHT               (1110)
	#define CAPACITY_BOTTOM              (929)

	#define NUMBER_LEFT                  (855) // number
	#define NUMBER_TOP                   (124)
	#define NUMBER_RIGHT                 (939) //855+84
	#define NUMBER_BOTTOM                (245)

	#define PERCENT_LEFT                 (1023) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (124)
	#define PERCENT_RIGHT                (1131) //1023+108
	#define PERCENT_BOTTOM               (245)  //124+121

	#define TOP_ANIMATION_LEFT           (806) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (1110) //806+304
	#define TOP_ANIMATION_BOTTOM         (152)

	// for old animation
	#define BAR_LEFT            (890)
	#define BAR_TOP             (357)
	#define BAR_RIGHT           (1030)
	#define BAR_BOTTOM          (678)
	
#elif defined(WUXGA) || defined(CU_WUXGA)
	// wuxga 1200*1920

	// battery capacity rectangle
	#define CAPACITY_LEFT                (447) // battery capacity center
	#define CAPACITY_TOP                 (803)
	#define CAPACITY_RIGHT               (751)
	#define CAPACITY_BOTTOM              (1289)

	#define NUMBER_LEFT                  (494) // number
	#define NUMBER_TOP                   (481)
	#define NUMBER_RIGHT                 (578)//494+84
	#define NUMBER_BOTTOM                (602) //481+121

	#define PERCENT_LEFT                 (662) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (481)
	#define PERCENT_RIGHT                (770) //662+108
	#define PERCENT_BOTTOM               (602) //481+121

	#define TOP_ANIMATION_LEFT           (447) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (751)
	#define TOP_ANIMATION_BOTTOM         (152)

	// for old animation
	#define BAR_LEFT            (529)
	#define BAR_TOP             (357)
	#define BAR_RIGHT           (672)
	#define BAR_BOTTOM          (680)
	
#elif defined(XGA)
	// xga 768*1024

	// battery capacity rectangle
	#define CAPACITY_LEFT                (316) // battery capacity center
	#define CAPACITY_TOP                 (442)
	#define CAPACITY_RIGHT               (451)
	#define CAPACITY_BOTTOM              (658)

	#define NUMBER_LEFT                  (338) // number
	#define NUMBER_TOP                   (300)
	#define NUMBER_RIGHT                 (376)
	#define NUMBER_BOTTOM                (354)

	#define PERCENT_LEFT                 (414) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (300)
	#define PERCENT_RIGHT                (462)
	#define PERCENT_BOTTOM               (354)

	#define TOP_ANIMATION_LEFT           (316) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (451)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (334)
	#define BAR_TOP             (243)
	#define BAR_RIGHT           (434)
	#define BAR_BOTTOM          (463)
	
#elif defined(XGANL)
	// xganl 1024*768

	// battery capacity rectangle
	#define CAPACITY_LEFT                (444) // battery capacity center
	#define CAPACITY_TOP                 (314)	
	#define CAPACITY_RIGHT               (579)
	#define CAPACITY_BOTTOM              (530)

	#define NUMBER_LEFT                  (467) // number
	#define NUMBER_TOP                   (170)
	#define NUMBER_RIGHT                 (505)
	#define NUMBER_BOTTOM                (224)

	#define PERCENT_LEFT                 (543) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (170)
	#define PERCENT_RIGHT                (591)
	#define PERCENT_BOTTOM               (224)

	#define TOP_ANIMATION_LEFT           (444) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (579)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (486)
	#define BAR_TOP             (292)
	#define BAR_RIGHT           (590)
	#define BAR_BOTTOM          (506) 

#elif defined(QXGA)
	// qxga 1536x2048
	
		// battery capacity rectangle
	#define CAPACITY_LEFT                (605) // battery capacity center
	#define CAPACITY_TOP                 (857)	
	#define CAPACITY_RIGHT               (929)
	#define CAPACITY_BOTTOM              (1375)

	#define NUMBER_LEFT                  (662) // number
	#define NUMBER_TOP                   (517)
	#define NUMBER_RIGHT                 (756)
	#define NUMBER_BOTTOM                (647)

	#define PERCENT_LEFT                 (850) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (517)
	#define PERCENT_RIGHT                (968)
	#define PERCENT_BOTTOM               (647)

	#define TOP_ANIMATION_LEFT           (605) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (929)
	#define TOP_ANIMATION_BOTTOM         (155)

	// for old animation
	#define BAR_LEFT            (693)
	#define BAR_TOP             (381)
	#define BAR_RIGHT           (844)
	#define BAR_BOTTOM          (725) 

#else 

	// default wvga 480*800

	// battery capacity rectangle
	#define CAPACITY_LEFT                (172) // battery capacity center
	#define CAPACITY_TOP                 (330)
	#define CAPACITY_RIGHT               (307)
	#define CAPACITY_BOTTOM              (546)

	// first number rectangle
	#define NUMBER_LEFT                  (178) // number
	#define NUMBER_TOP                   (190)
	#define NUMBER_RIGHT                 (216)
	#define NUMBER_BOTTOM                (244)

	// %  rectangle
	#define PERCENT_LEFT                 (254) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (190)
	#define PERCENT_RIGHT                (302)
	#define PERCENT_BOTTOM               (244)

	// top animation part
	#define TOP_ANIMATION_LEFT           (172) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (307)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (209)
	#define BAR_TOP             (149)
	#define BAR_RIGHT           (271)
	#define BAR_BOTTOM          (282)

#endif

/* The option of new charging animation */
#define ANIMATION_NEW

#endif // __CUST_DISPLAY_H__
