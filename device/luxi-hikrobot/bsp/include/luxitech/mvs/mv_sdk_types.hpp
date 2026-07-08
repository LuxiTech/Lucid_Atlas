#pragma once

#include <cstddef>
#include <cstdint>

namespace luxitech::mvs {

constexpr unsigned int MV_OK = 0x00000000;
constexpr unsigned int MV_GIGE_DEVICE = 0x00000001;
constexpr unsigned int MV_USB_DEVICE = 0x00000004;
constexpr unsigned int MV_ACCESS_Exclusive = 1;

constexpr std::size_t INFO_MAX_BUFFER_SIZE = 64;
constexpr std::size_t MV_MAX_DEVICE_NUM = 256;
constexpr std::size_t MV_MAX_XML_SYMBOLIC_NUM = 64;
constexpr std::size_t MV_MAX_SYMBOLIC_LEN = 64;

constexpr std::int64_t PixelType_Gvsp_Mono8 = 17301505;
constexpr std::int64_t PixelType_Gvsp_RGB8_Packed = 35127316;

using MvGvspPixelType = std::int64_t;

struct MV_GIGE_DEVICE_INFO {
    unsigned int nIpCfgOption;
    unsigned int nIpCfgCurrent;
    unsigned int nCurrentIp;
    unsigned int nCurrentSubNetMask;
    unsigned int nDefultGateWay;
    unsigned char chManufacturerName[32];
    unsigned char chModelName[32];
    unsigned char chDeviceVersion[32];
    unsigned char chManufacturerSpecificInfo[48];
    unsigned char chSerialNumber[16];
    unsigned char chUserDefinedName[16];
    unsigned int nNetExport;
    unsigned int nReserved[4];
};

struct MV_USB3_DEVICE_INFO {
    unsigned char CrtlInEndPoint;
    unsigned char CrtlOutEndPoint;
    unsigned char StreamEndPoint;
    unsigned char EventEndPoint;
    unsigned short idVendor;
    unsigned short idProduct;
    unsigned int nDeviceNumber;
    unsigned char chDeviceGUID[INFO_MAX_BUFFER_SIZE];
    unsigned char chVendorName[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chFamilyName[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturerName[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned int nbcdUSB;
    unsigned int nDeviceAddress;
    unsigned int nReserved[2];
};

union MV_SPECIAL_INFO {
    MV_GIGE_DEVICE_INFO stGigEInfo;
    MV_USB3_DEVICE_INFO stUsb3VInfo;
};

struct MV_CC_DEVICE_INFO {
    unsigned short nMajorVer;
    unsigned short nMinorVer;
    unsigned int nMacAddrHigh;
    unsigned int nMacAddrLow;
    unsigned int nTLayerType;
    unsigned int nDevTypeInfo;
    unsigned int nReserved[3];
    MV_SPECIAL_INFO SpecialInfo;
};

struct MV_CC_DEVICE_INFO_LIST {
    unsigned int nDeviceNum;
    MV_CC_DEVICE_INFO* pDeviceInfo[MV_MAX_DEVICE_NUM];
};

struct MVCC_INTVALUE {
    unsigned int nCurValue;
    unsigned int nMax;
    unsigned int nMin;
    unsigned int nInc;
    unsigned int nReserved[4];
};

struct MVCC_INTVALUE_EX {
    std::int64_t nCurValue;
    std::int64_t nMax;
    std::int64_t nMin;
    std::int64_t nInc;
    unsigned int nReserved[16];
};

struct MVCC_FLOATVALUE {
    float fCurValue;
    float fMax;
    float fMin;
    unsigned int nReserved[4];
};

struct MVCC_ENUMVALUE {
    unsigned int nCurValue;
    unsigned int nSupportedNum;
    unsigned int nSupportValue[MV_MAX_XML_SYMBOLIC_NUM];
    unsigned int nReserved[4];
};

struct MVCC_ENUMENTRY {
    unsigned int nValue;
    char chSymbolic[MV_MAX_SYMBOLIC_LEN];
    unsigned int nReserved[4];
};

struct MV_CHUNK_DATA_CONTENT;
struct MV_CC_IMAGE;

union MV_FRAME_OUT_INFO_EX_UNPARSED {
    MV_CHUNK_DATA_CONTENT* pUnparsedChunkContent;
    std::int64_t nAligning;
};

union MV_FRAME_OUT_INFO_EX_SUBIMAGE {
    MV_CC_IMAGE* pstSubImage;
    std::int64_t nAligning;
};

union MV_FRAME_OUT_INFO_EX_USERPTR {
    void* pUser;
    std::int64_t nAligning;
};

struct MV_FRAME_OUT_INFO_EX {
    unsigned short nWidth;
    unsigned short nHeight;
    MvGvspPixelType enPixelType;
    unsigned int nFrameNum;
    unsigned int nDevTimeStampHigh;
    unsigned int nDevTimeStampLow;
    unsigned int nReserved0;
    std::int64_t nHostTimeStamp;
    unsigned int nFrameLen;
    unsigned int nSecondCount;
    unsigned int nCycleCount;
    unsigned int nCycleOffset;
    float fGain;
    float fExposureTime;
    unsigned int nAverageBrightness;
    unsigned int nRed;
    unsigned int nGreen;
    unsigned int nBlue;
    unsigned int nFrameCounter;
    unsigned int nTriggerIndex;
    unsigned int nInput;
    unsigned int nOutput;
    unsigned short nOffsetX;
    unsigned short nOffsetY;
    unsigned short nChunkWidth;
    unsigned short nChunkHeight;
    unsigned int nLostPacket;
    unsigned int nUnparsedChunkNum;
    MV_FRAME_OUT_INFO_EX_UNPARSED UnparsedChunkList;
    unsigned int nExtendWidth;
    unsigned int nExtendHeight;
    std::uint64_t nFrameLenEx;
    unsigned int nReserved1;
    unsigned int nSubImageNum;
    MV_FRAME_OUT_INFO_EX_SUBIMAGE SubImageList;
    MV_FRAME_OUT_INFO_EX_USERPTR UserPtr;
    unsigned int nReserved[26];
};

struct MV_CC_PIXEL_CONVERT_PARAM {
    unsigned short nWidth;
    unsigned short nHeight;
    MvGvspPixelType enSrcPixelType;
    unsigned char* pSrcData;
    unsigned int nSrcDataLen;
    MvGvspPixelType enDstPixelType;
    unsigned char* pDstBuffer;
    unsigned int nDstLen;
    unsigned int nDstBufferSize;
    unsigned int nRes[4];
};

}  // namespace luxitech::mvs
