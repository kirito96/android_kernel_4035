LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_SRC_FILES := m4u_ut.cpp \
rgb565_390x210.c \

LOCAL_MODULE := m4u_ut

LOCAL_C_INCLUDES:= \
	$(TOP)/$(MTK_PATH_SOURCE)/external/mhal/src/core/drv/inc \
	$(MTK_PATH_PLATFORM)hardware/m4u \
	$(TOP)/$(MTK_PATH_SOURCE)/kernel/drivers/video \
  
LOCAL_SHARED_LIBRARIES := \
  libcutils \
  liblog \
  libmhaldrv \
  
#LOCAL_STATIC_LIBRARIES := 
LOCAL_MODULE_TAGS := tests
LOCAL_PRELINK_MODULE:=false
include $(BUILD_EXECUTABLE) 
