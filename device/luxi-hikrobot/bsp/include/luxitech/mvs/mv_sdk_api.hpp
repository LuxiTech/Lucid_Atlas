#pragma once

#include "luxitech/mvs/mv_sdk_types.hpp"

extern "C" {

int MV_CC_Initialize();
int MV_CC_Finalize();
unsigned int MV_CC_GetSDKVersion();

unsigned int MV_CC_EnumDevices(unsigned int nTLayerType, luxitech::mvs::MV_CC_DEVICE_INFO_LIST* pstDevList);
unsigned int MV_CC_CreateHandle(void** handle, const luxitech::mvs::MV_CC_DEVICE_INFO* pstDevInfo);
unsigned int MV_CC_DestroyHandle(void* handle);
unsigned int MV_CC_OpenDevice(void* handle, unsigned int nAccessMode, unsigned short nSwitchoverKey);
unsigned int MV_CC_CloseDevice(void* handle);

unsigned int MV_CC_StartGrabbing(void* handle);
unsigned int MV_CC_StopGrabbing(void* handle);
unsigned int MV_CC_GetOneFrameTimeout(
    void* handle,
    unsigned char* pData,
    unsigned int nDataSize,
    luxitech::mvs::MV_FRAME_OUT_INFO_EX* pstFrameInfo,
    unsigned int nMsec);

unsigned int MV_CC_GetOptimalPacketSize(void* handle);

unsigned int MV_CC_SetEnumValueByString(void* handle, const char* strKey, const char* sValue);
unsigned int MV_CC_SetFloatValue(void* handle, const char* strKey, float fValue);
unsigned int MV_CC_SetIntValueEx(void* handle, const char* strKey, std::int64_t nValue);
unsigned int MV_CC_SetCommandValue(void* handle, const char* strKey);

unsigned int MV_CC_GetIntValue(void* handle, const char* strKey, luxitech::mvs::MVCC_INTVALUE* pstIntValue);
unsigned int MV_CC_GetIntValueEx(void* handle, const char* strKey, luxitech::mvs::MVCC_INTVALUE_EX* pstIntValue);
unsigned int MV_CC_GetFloatValue(void* handle, const char* strKey, luxitech::mvs::MVCC_FLOATVALUE* pstFloatValue);
unsigned int MV_CC_GetEnumValue(void* handle, const char* strKey, luxitech::mvs::MVCC_ENUMVALUE* pstEnumValue);
unsigned int MV_CC_GetEnumEntrySymbolic(void* handle, const char* strKey, luxitech::mvs::MVCC_ENUMENTRY* pstEnumEntry);

unsigned int MV_CC_ConvertPixelType(void* handle, luxitech::mvs::MV_CC_PIXEL_CONVERT_PARAM* pstCvtParam);

}  // extern "C"
