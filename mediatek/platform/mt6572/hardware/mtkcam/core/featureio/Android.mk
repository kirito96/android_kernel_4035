LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(HAVE_AEE_FEATURE),yes)
	LOCAL_CFLAGS += -DHAVE_AEE_FEATURE
endif

PLATFORM_VERSION_MAJOR := $(word 1,$(subst .,$(space),$(PLATFORM_VERSION)))
LOCAL_CFLAGS += -DPLATFORM_VERSION_MAJOR=$(PLATFORM_VERSION_MAJOR)

LOCAL_SHARED_LIBRARIES := \
    libstlport \
    libcutils \
    liblog \
    libcamdrv \
    libcamalgo

ifeq ($(HAVE_AEE_FEATURE),yes)
	LOCAL_SHARED_LIBRARIES += libaed
endif

ifneq ($(BUILD_MTK_LDVT),true)    
#    LOCAL_SHARED_LIBRARIES += lib3a
    LOCAL_SHARED_LIBRARIES += libnvram
    LOCAL_SHARED_LIBRARIES += libcam.exif
    LOCAL_SHARED_LIBRARIES += libcameracustom
    LOCAL_SHARED_LIBRARIES += libcam.utils
    LOCAL_SHARED_LIBRARIES += libdpframework
endif
            
LOCAL_STATIC_LIBRARIES := \

LOCAL_WHOLE_STATIC_LIBRARIES := \
    libfeatureiodrv \
    libfeatureiopipe \

ifeq ($(BUILD_MTK_LDVT),true)
LOCAL_WHOLE_STATIC_LIBRARIES += \
    libuvvf
endif

LOCAL_MODULE := libfeatureio

LOCAL_MODULE_TAGS := eng

LOCAL_PRELINK_MODULE := false


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
include $(BUILD_SHARED_LIBRARY) 


include $(call all-makefiles-under,$(LOCAL_PATH))
