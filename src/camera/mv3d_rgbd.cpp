#include "mv3d_rgbd.h"

#include <algorithm>
#include <iterator>
#include <sstream>

namespace
{
bool Fail(MV3D_RGBD_STATUS code, const char *operation, std::string *error)
{
  if (error)
  {
    std::ostringstream text;
    text << operation << " failed: 0x" << std::hex << std::uppercase
         << static_cast<std::uint32_t>(code);
    *error = text.str();
  }
  return false;
}

template <std::size_t Size>
std::string From_Fixed_String(const char (&value)[Size])
{
  const auto end = std::find(std::begin(value), std::end(value), '\0');
  return std::string(std::begin(value), end);
}
} // namespace

Mv3d_Rgbd_Camera::~Mv3d_Rgbd_Camera()
{
  Shut_Down();
}

bool Mv3d_Rgbd_Camera::Initialize(std::string *error)
{
  if (error)
    error->clear();

  if (m_initialized)
    return true;

  const auto status = MV3D_RGBD_Initialize();
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_Initialize", error);
  // return false;

  m_initialized = true;
  return true;
}

bool Mv3d_Rgbd_Camera::Get_Version(SDK_VERSION &version, std::string *error)
{
  if (error)
    error->clear();

  if (!m_initialized)
    return Fail(MV3D_RGBD_E_CALLORDER, "SDK is not initialized", error);
  // return false;

  MV3D_RGBD_VERSION_INFO sdk_version{};
  const auto status = MV3D_RGBD_GetSDKVersion(&sdk_version);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_GetSDKVersion", error);

  version = {sdk_version.nMajor, sdk_version.nMinor, sdk_version.nRevision};
  return true;
}

bool Mv3d_Rgbd_Camera::Enumerate_Devices(std::vector<CAMERA_DEVICE_INFO> &devices, std::string *error)
{
  std::vector<MV3D_RGBD_DEVICE_INFO> sdk_devices;
  return Enumerate_Devices(devices, sdk_devices, error);
}

bool Mv3d_Rgbd_Camera::Enumerate_Devices(
    std::vector<CAMERA_DEVICE_INFO> &devices,
    std::vector<MV3D_RGBD_DEVICE_INFO> &sdk_devices,
    std::string *error)
{
  if (error)
    error->clear();

  devices.clear();
  sdk_devices.clear();
  if (!m_initialized)
    return Fail(MV3D_RGBD_E_CALLORDER, "SDK is not initialized", error);

  uint32_t count = 0;
  auto status = MV3D_RGBD_GetDeviceNumber(kAllDeviceTypes, &count);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_GetDeviceNumber", error);

  sdk_devices.resize(count);
  if (count == 0)
    return true;

  status = MV3D_RGBD_GetDeviceList(kAllDeviceTypes, sdk_devices.data(), count, &count);
  if (status != MV3D_RGBD_OK)
  {
    sdk_devices.clear();
    return Fail(status, "MV3D_RGBD_GetDeviceList", error);
  }

  const auto filled_count = std::min<std::size_t>(count, sdk_devices.size());
  sdk_devices.resize(filled_count);
  devices.reserve(filled_count);
  for (std::size_t i = 0; i < filled_count; ++i)
  {
    const auto &source = sdk_devices[i];
    CAMERA_DEVICE_INFO device;
    device.index = i;
    device.device_type = static_cast<std::uint32_t>(source.enDeviceType);
    device.model_name = From_Fixed_String(source.chModelName);
    device.serial_number = From_Fixed_String(source.chSerialNumber);
    if (source.enDeviceType == DeviceType_Ethernet ||
        source.enDeviceType == DeviceType_Ethernet_Vir)
      device.ip_address = From_Fixed_String(source.SpecialInfo.stNetInfo.chCurrentIp);
    devices.push_back(std::move(device));
  }
  return true;
}

bool Mv3d_Rgbd_Camera::Open_Device(
    const MV3D_RGBD_DEVICE_INFO &sdk_device,
    std::string *error)
{
  if (error)
    error->clear();

  if (!m_initialized)
    return Fail(MV3D_RGBD_E_CALLORDER, "SDK is not initialized", error);
  if (m_handle != nullptr)
    return true;

  auto device = sdk_device;
  const auto status = MV3D_RGBD_OpenDevice(&m_handle, &device);
  if (status != MV3D_RGBD_OK)
  {
    m_handle = nullptr;
    return Fail(status, "MV3D_RGBD_OpenDevice", error);
  }
  if (m_handle == nullptr)
    return Fail(MV3D_RGBD_E_HANDLE, "MV3D_RGBD_OpenDevice", error);

  return true;
}

bool Mv3d_Rgbd_Camera::Get_Device_Info(
    Camera_Device_Detail &detail,
    std::string *error)
{
  if (error)
    error->clear();
  detail = {};

  if (m_handle == nullptr)
    return Fail(MV3D_RGBD_E_CALLORDER, "Camera device is not open", error);

  MV3D_RGBD_DEVICE_INFO sdk_detail{};
  const auto status = MV3D_RGBD_GetDeviceInfo(m_handle, &sdk_detail);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_GetDeviceInfo", error);

  detail = Build_Camera_Device_Detail(sdk_detail);
  return true;
}

bool Mv3d_Rgbd_Camera::Get_Parameter(
    const std::string &key,
    Camera_Parameter &parameter,
    std::string *error)
{
  if (error)
    error->clear();
  parameter = {};

  if (m_handle == nullptr)
    return Fail(MV3D_RGBD_E_CALLORDER, "Camera device is not open", error);
  if (key.empty())
    return Fail(MV3D_RGBD_E_PARAMETER, "Camera parameter key is empty", error);

  MV3D_RGBD_PARAM sdk_parameter{};
  const auto status = MV3D_RGBD_GetParam(
      m_handle, key.c_str(), &sdk_parameter);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_GetParam", error);

  parameter.key = key;
  return Decode_Camera_Parameter(sdk_parameter, &parameter, error);
}

bool Mv3d_Rgbd_Camera::Set_Parameter(
    const Camera_Parameter_Update &update,
    std::string *error)
{
  if (error)
    error->clear();

  if (m_handle == nullptr)
    return Fail(MV3D_RGBD_E_CALLORDER, "Camera device is not open", error);
  if (update.key.empty())
    return Fail(MV3D_RGBD_E_PARAMETER, "Camera parameter key is empty", error);

  MV3D_RGBD_PARAM sdk_parameter{};
  if (!Encode_Camera_Parameter_Update(update, &sdk_parameter, error))
    return false;

  const auto status = MV3D_RGBD_SetParam(
      m_handle, update.key.c_str(), &sdk_parameter);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_SetParam", error);
  return true;
}

bool Mv3d_Rgbd_Camera::Get_Stream_Configuration(
    std::vector<Camera_Stream_Config> &streams,
    std::string *error)
{
  if (error)
    error->clear();
  streams.clear();

  if (m_handle == nullptr)
    return Fail(MV3D_RGBD_E_CALLORDER, "Camera device is not open", error);

  MV3D_RGBD_STREAM_CFG_LIST sdk_streams{};
  const auto status = MV3D_RGBD_GetStreamCfgList(m_handle, &sdk_streams);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_GetStreamCfgList", error);

  if (!Build_Camera_Stream_Configuration(sdk_streams, &streams))
    return Fail(MV3D_RGBD_E_PARAMETER, "Invalid stream configuration output", error);
  return true;
}

bool Mv3d_Rgbd_Camera::Close_Device(std::string *error)
{
  if (error)
    error->clear();

  if (m_handle == nullptr)
    return true;
  if (m_grabbing && !Stop(error))
    return false;

  const auto status = MV3D_RGBD_CloseDevice(&m_handle);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_CloseDevice", error);

  m_handle = nullptr;
  return true;
}

bool Mv3d_Rgbd_Camera::Start(std::string *error)
{
  if (error)
    error->clear();

  if (m_handle == nullptr)
    return Fail(MV3D_RGBD_E_CALLORDER, "Camera device is not open", error);
  if (m_grabbing)
    return true;

  const auto status = MV3D_RGBD_Start(m_handle);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_Start", error);

  m_grabbing = true;
  return true;
}

bool Mv3d_Rgbd_Camera::Stop(std::string *error)
{
  if (error)
    error->clear();

  if (!m_grabbing)
    return true;
  if (m_handle == nullptr)
    return Fail(MV3D_RGBD_E_HANDLE, "Camera device is not open", error);

  const auto status = MV3D_RGBD_Stop(m_handle);
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_Stop", error);

  m_grabbing = false;
  return true;
}

bool Mv3d_Rgbd_Camera::Fetch_Frame(
    Camera_Frame &frame,
    std::uint32_t timeout_ms,
    MV3D_RGBD_STATUS *status_code,
    std::string *error)
{
  if (error)
    error->clear();
  if (status_code)
    *status_code = MV3D_RGBD_OK;
  frame = {};

  if (!m_grabbing || m_handle == nullptr)
  {
    if (status_code)
      *status_code = MV3D_RGBD_E_CALLORDER;
    return Fail(MV3D_RGBD_E_CALLORDER, "Camera acquisition is not running", error);
  }

  MV3D_RGBD_FRAME_DATA sdk_frame{};
  const auto status = MV3D_RGBD_FetchFrame(m_handle, &sdk_frame, timeout_ms);
  if (status_code)
    *status_code = status;
  if (status != MV3D_RGBD_OK)
    return Fail(status, "MV3D_RGBD_FetchFrame", error);

  frame.valid_info = sdk_frame.nValidInfo;
  frame.trigger_id = sdk_frame.nTriggerId;
  const auto image_count = std::min<std::size_t>(
      sdk_frame.nImageCount, MV3D_RGBD_MAX_IMAGE_COUNT);
  frame.images.reserve(image_count);
  for (std::size_t i = 0; i < image_count; ++i)
  {
    const auto &source = sdk_frame.stImageData[i];
    Camera_Image_Frame image;
    image.image_type = static_cast<std::uint32_t>(source.enImageType);
    image.width = source.nWidth;
    image.height = source.nHeight;
    image.frame_number = source.nFrameNum;
    image.timestamp = source.nTimeStamp;
    image.is_rectified = source.bIsRectified != FALSE;
    image.stream_type = static_cast<std::uint32_t>(source.enStreamType);
    image.coordinate_type = static_cast<std::uint32_t>(source.enCoordinateType);
    if (source.pData != nullptr && source.nDataLen > 0)
      image.data.assign(source.pData, source.pData + source.nDataLen);
    frame.images.push_back(std::move(image));
  }
  return true;
}

void Mv3d_Rgbd_Camera::Shut_Down()
{
  if (m_grabbing && m_handle != nullptr)
    MV3D_RGBD_Stop(m_handle);
  m_grabbing = false;

  if (m_handle != nullptr)
    MV3D_RGBD_CloseDevice(&m_handle);
  m_handle = nullptr;

  if (m_initialized)
  {
    MV3D_RGBD_Release();
    m_initialized = false;
  }
}
