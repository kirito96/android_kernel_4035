LOCAL_PATH := $(call my-dir)

################################################################################
#
################################################################################
include $(CLEAR_VARS)

#-----------------------------------------------------------
sinclude $(TOP)/$(MTK_PATH_PLATFORM)/hardware/mtkcam/mtkcam.mk

#-----------------------------------------------------------
MTKCAM_HAVE_CAMFEATURE ?= '1'   # built-in if '1' ; otherwise not built-in
MTKCAM_HAVE_SENSOR_HAL ?= '1'   # built-in if '1' ; otherwise not built-in
MTKCAM_HAVE_3A_HAL     ?= '1'   # built-in if '1' ; otherwise not built-in

#-----------------------------------------------------------
LOCAL_SRC_FILES += $(call all-c-cpp-files-under, .)

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(MTKCAM_C_INCLUDES)
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_PLATFORM)/hardware/include
#
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/hardware/mtkcam/v1/common/paramsmgr
#
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_SOURCE)/frameworks-ext/av/include
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_CUSTOM)/hal/inc
LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_CUSTOM)/cgen/cfgfileinc
#
$(info $(LOCAL_C_INCLUDES))

#-----------------------------------------------------------
LOCAL_CFLAGS += $(MTKCAM_CFLAGS)
#
ifeq "'1'" "$(strip $(MTKCAM_HAVE_CAMFEATURE))"
LOCAL_CFLAGS += -DMTKCAM_HAVE_CAMFEATURE="$(MTKCAM_HAVE_CAMFEATURE)"
endif
#
ifeq "'1'" "$(strip $(MTKCAM_HAVE_SENSOR_HAL))"
LOCAL_CFLAGS += -DMTKCAM_HAVE_SENSOR_HAL="$(MTKCAM_HAVE_SENSOR_HAL)"
endif
#
ifeq "'1'" "$(strip $(MTKCAM_HAVE_3A_HAL))"
LOCAL_CFLAGS += -DMTKCAM_HAVE_3A_HAL="$(MTKCAM_HAVE_3A_HAL)"
endif

PLATFORM_VERSION_MAJOR := $(word 1,$(subst .,$(space),$(PLATFORM_VERSION)))
LOCAL_CFLAGS += -DPLATFORM_VERSION_MAJOR=$(PLATFORM_VERSION_MAJOR)

#-----------------------------------------------------------
LOCAL_WHOLE_STATIC_LIBRARIES += 
#
LOCAL_STATIC_LIBRARIES += 

#-----------------------------------------------------------
LOCAL_MODULE := libcam.paramsmgr.platform

#-----------------------------------------------------------
include $(BUILD_STATIC_LIBRARY)


################################################################################
#
################################################################################
include $(CLEAR_VARS)
include $(call all-makefiles-under,$(LOCAL_PATH))

