LOCAL_PATH := $(call my-dir)

################################################################################
#
################################################################################
include $(CLEAR_VARS)

#-----------------------------------------------------------
sinclude $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/mtkcam.mk

#-----------------------------------------------------------
LOCAL_SRC_FILES += $(call all-c-cpp-files-under, .)

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(MTKCAM_C_INCLUDES)
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/include
#

#-----------------------------------------------------------
LOCAL_CFLAGS += $(MTKCAM_CFLAGS)

#-----------------------------------------------------------
LOCAL_WHOLE_STATIC_LIBRARIES += libcam.utils.common
LOCAL_WHOLE_STATIC_LIBRARIES += libcam.utils.platform
#
LOCAL_WHOLE_STATIC_LIBRARIES += libcam_mmp
#
LOCAL_STATIC_LIBRARIES += 

#-----------------------------------------------------------
LOCAL_SHARED_LIBRARIES += liblog
LOCAL_SHARED_LIBRARIES += libutils
LOCAL_SHARED_LIBRARIES += libcamera_client
#
LOCAL_SHARED_LIBRARIES += libcam_utils
#
ifeq "yes" "$(strip $(MTK_MMPROFILE_SUPPORT))"
LOCAL_SHARED_LIBRARIES += libmmprofile
endif

#-----------------------------------------------------------
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := libcam.utils

#-----------------------------------------------------------
include $(BUILD_SHARED_LIBRARY)


################################################################################
#
################################################################################
include $(CLEAR_VARS)
include $(call all-makefiles-under,$(LOCAL_PATH))

