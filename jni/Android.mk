LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := vrtrackplugin
LOCAL_SRC_FILES := vrtrackplugin.cpp
LOCAL_LDLIBS    := -llog

include $(BUILD_SHARED_LIBRARY)
