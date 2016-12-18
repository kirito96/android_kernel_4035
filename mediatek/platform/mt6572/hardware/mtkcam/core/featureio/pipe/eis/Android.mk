LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(BUILD_MTK_LDVT),true)
    LOCAL_CFLAGS += -DUSING_MTK_LDVT
endif

#LOCAL_SRC_FILES += \
#    eis_hal.cpp
  
LOCAL_C_INCLUDES:= \
    $(TOP)/bionic \
    $(TOP)/external/stlport/stlport \
    #$(TOP)/$(MTK_PATH_SOURCE)/hardware/mtkcam/inc \
    #$(TOP)/$(MTK_PATH_SOURCE)/hardware/mtkcam/inc/drv \
    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/include/mtkcam/algorithm/libeis \
    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/core/featureio/drv/inc \
    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam \
    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc/common \
    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc/featureio \
    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc/scenario \
    $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc/drv \
    $(TOP)/$(MTK_PATH_CUSTOM)/hal/inc

PLATFORM_VERSION_MAJOR := $(word 1,$(subst .,$(space),$(PLATFORM_VERSION)))
LOCAL_CFLAGS += -DPLATFORM_VERSION_MAJOR=$(PLATFORM_VERSION_MAJOR)

#LOCAL_STATIC_LIBRARIES := \

#LOCAL_WHOLE_STATIC_LIBRARIES := \

LOCAL_MODULE := libfeatureiopipe_eis


#
# Start of common part ------------------------------------
sinclude $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/mtkcam.mk

#-----------------------------------------------------------
LOCAL_CFLAGS += $(MTKCAM_CFLAGS)

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(MTKCAM_C_INCLUDES)

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/include

# End of common part ---------------------------------------
#
include $(BUILD_STATIC_LIBRARY)


