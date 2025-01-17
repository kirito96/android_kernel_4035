LOCAL_PATH := $(call my-dir)

################################################################################
#
################################################################################
include $(CLEAR_VARS)

#-----------------------------------------------------------
#$(call config-custom-folder,hal:hal)

#-----------------------------------------------------------
hal_folder := $(call to-root,$(LOCAL_PATH))/$(MTK_PATH_CUSTOM)/hal
LOCAL_SRC_FILES += $(call all-c-cpp-files-under, $(hal_folder)/eeprom)

#-----------------------------------------------------------
#LOCAL_C_INCLUDES += $(TOP)/$(MTK_PATH_CUSTOM)/kernel/eeprom/inc

#-----------------------------------------------------------
LOCAL_STATIC_LIBRARIES += 
#
LOCAL_WHOLE_STATIC_LIBRARIES += 

#-----------------------------------------------------------
LOCAL_MODULE := libcameracustom.eeprom

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

 

################################################################################
#
################################################################################
include $(CLEAR_VARS)
include $(call all-makefiles-under,$(LOCAL_PATH))

