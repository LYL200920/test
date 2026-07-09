#ifndef includeguard_common_h_includeguard
#define includeguard_common_h_includeguard

#include <string>
#include <cstddef>
#include <cstdint>

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"
#include "Mv3dRgbdImgProc.h"

typedef struct _SDK_VERSION_
{
  uint32_t major, minor, revision;
} SDK_VERSION;

typedef struct _CAMERA_DEVICE_INFO_
{
  size_t index = 0;
  std::uint32_t device_type = 0;
  std::string model_name;
  std::string serial_number;
  std::string ip_address;
}CAMERA_DEVICE_INFO;

typedef struct _POINT_XYZ_
{
  float fX;
  float fY;
  float fZ;
} POINT_XYZ;

typedef struct _POINT_NORMAL_
{
  float fNormalX;
  float fNormalY;
  float fNormalZ;
  float fCurvature;
} POINT_NORMAL;

typedef struct _POINT_XYZBGR_
{
  float fX;
  float fY;
  float fZ;

  union
  {
    struct
    {
      uint8_t nB;
      uint8_t nG;
      uint8_t nR;
      uint8_t nA;
    };
    float fRgb;
  };
} POINT_XYZBGR;

typedef struct _POINT_XYZNORMALS_
{
  POINT_XYZ pPoint;
  POINT_NORMAL pNormals;
} POINT_XYZNORMALS;

typedef struct _POINT_XYZRGBNORMALS_
{
  POINT_XYZBGR pColorPoint;
  POINT_NORMAL pNormals;
} POINT_XYZRGBNORMALS;

typedef struct _RGB_TEXTURE_DATA_
{
  uint8_t nR;
  uint8_t nG;
  uint8_t nB;
} RGB_TEXTURE_DATA;

inline bool convertRGB8P_2_RGB(unsigned char *pSrcData, int nWidth, int nHeight, unsigned char *pDstData)
{
  if (nullptr == pSrcData || nullptr == pDstData)
  {
    return false;
  }

  int nStepDist = nWidth * nHeight;
  int nByteCount = 0;
  for (int nHeightIndex = 0; nHeightIndex < nHeight; ++nHeightIndex)
  {
    for (int nWidthIndex = 0; nWidthIndex < nWidth; ++nWidthIndex)
    {
      int nDataIndex = nHeightIndex * nWidth + nWidthIndex;
      pDstData[nByteCount++] = pSrcData[0 * nStepDist + nDataIndex];
      pDstData[nByteCount++] = pSrcData[1 * nStepDist + nDataIndex];
      pDstData[nByteCount++] = pSrcData[2 * nStepDist + nDataIndex];
    }
    while (nByteCount % 4)
    {
      nByteCount++;
    }
  }

  return true;
}

#ifndef ASSERT
#define ASSERT(x)                                         \
  do                                                      \
  {                                                       \
    if (!(x))                                             \
    {                                                     \
      LOGE("Assert failed at %s:%d", __FILE__, __LINE__); \
      LOGE("Source Code: " #x);                           \
      system("pause");                                    \
      exit(0);                                            \
    }                                                     \
  } while (0)
#endif

#ifndef ASSERT_OK
#define ASSERT_OK(x)                                                      \
  do                                                                      \
  {                                                                       \
    int err = (x);                                                        \
    if (err != MV3D_RGBD_OK)                                              \
    {                                                                     \
      LOGE("Assert failed: error %#x at %s:%d", err, __FILE__, __LINE__); \
      LOGE("Source Code: " #x);                                           \
      system("pause");                                                    \
      exit(0);                                                            \
    }                                                                     \
  } while (0)
#endif

#ifdef _WIN32
#include <windows.h>
#include <time.h>
static inline char *getLocalTime()
{
  static char local[26] = {0};
  SYSTEMTIME wtm;
  struct tm tm;
  GetLocalTime(&wtm);
  tm.tm_year = wtm.wYear - 1900;
  tm.tm_mon = wtm.wMonth - 1;
  tm.tm_mday = wtm.wDay;
  tm.tm_hour = wtm.wHour;
  tm.tm_min = wtm.wMinute;
  tm.tm_sec = wtm.wSecond;
  tm.tm_isdst = -1;

  strftime(local, 26, "%Y-%m-%d %H:%M:%S", &tm);

  return local;
}

#else
#include <sys/time.h>
#include <unistd.h>
static inline char *getLocalTime()
{
  static char local[26] = {0};
  time_t time;

  struct timeval tv;
  gettimeofday(&tv, NULL);

  time = tv.tv_sec;
  struct tm *p_time = localtime(&time);
  strftime(local, 26, "%Y-%m-%d %H:%M:%S", p_time);

  return local;
}
#endif

#define LOG(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#define LOGD(fmt, ...) printf("(%s) " fmt "\n", getLocalTime(), ##__VA_ARGS__)
#define LOGE(fmt, ...) printf("(%s) Error: " fmt "\n", getLocalTime(), ##__VA_ARGS__)

#define MAX_IMAGE_COUNT 10

#endif
