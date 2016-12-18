LOCAL_PATH := $(call my-dir)

################################################################################
#
################################################################################
include $(CLEAR_VARS)

#-----------------------------------------------------------
LOCAL_SRC_FILES += $(call all-c-cpp-files-under, .)

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(TOP)/$(MTK_ROOT)/frameworks-ext/av/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_ROOT)/hardware/mtkcam/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_ROOT)/hardware/mtkcam/inc/common
LOCAL_C_INCLUDES += $(TOP)/$(MTK_ROOT)/hardware/mtkcam/inc/featureio
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc/common
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/inc/featureio
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/v1/hal/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/v1/hal/client/CamClient
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/v1/hal/client/CamClient/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/v1/hal/client/CamClient/PreviewFeature
LOCAL_C_INCLUDES += $(TOP)/bionic
LOCAL_C_INCLUDES += $(TOP)/external/stlport/stlport
LOCAL_C_INCLUDES += $(TOP)/$(MTK_ROOT)/hardware/dpframework/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_CUSTOM)/kernel/imgsensor/inc


PLATFORM_VERSION_MAJOR := $(word 1,$(subst .,$(space),$(PLATFORM_VERSION)))
LOCAL_CFLAGS += -DPLATFORM_VERSION_MAJOR=$(PLATFORM_VERSION_MAJOR)

#-----------------------------------------------------------
LOCAL_MODULE := libcam.client.camclient.featurebase.panorama

#-----------------------------------------------------------

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



