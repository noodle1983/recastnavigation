LOCAL_PATH := $(call my-dir)


####################################################################################################
#include $(CLEAR_VARS) 

LOCAL_MODULE := DetourRuntime

LOCAL_MODULE_FILENAME := libDetourRuntime

cmd-strip = $(TOOLCHAIN_PREFIX)strip --strip-unneeded -x $1

LOCAL_SRC_FILES := \
../src/interface.cpp \
../src/NavMeshWrapper.cpp \
../src/nd_Detour_DetourAlloc.cpp \
../src/nd_Detour_DetourAssert.cpp \
../src/nd_Detour_DetourCommon.cpp \
../src/nd_Detour_DetourNavMesh.cpp \
../src/nd_Detour_DetourNavMeshBuilder.cpp \
../src/nd_Detour_DetourNavMeshQuery.cpp \
../src/nd_Detour_DetourNode.cpp \
../src/nd_DetourCrowd_DetourCrowd.cpp \
../src/nd_DetourCrowd_DetourLocalBoundary.cpp \
../src/nd_DetourCrowd_DetourObstacleAvoidance.cpp \
../src/nd_DetourCrowd_DetourPathCorridor.cpp \
../src/nd_DetourCrowd_DetourPathQueue.cpp \
../src/nd_DetourCrowd_DetourProximityGrid.cpp \
../src/nd_DetourTileCache_DetourTileCache.cpp \
../src/nd_DetourTileCache_DetourTileCacheBuilder.cpp \


LOCAL_CPPFLAGS := -DTARGET_ARCH_ABI=\"$(TARGET_ARCH_ABI)\"  -std=c++11  -I../src -I../../Detour/Include -I../../DetourCrowd/Include -I../../DetourTileCache/Include -I..

LOCAL_LDLIBS := -llog

LOCAL_C_INCLUDES  += system/core/include/cutils

LOCAL_SHARED_LIBRARIES := 

include $(BUILD_SHARED_LIBRARY)

####################################################################################################

