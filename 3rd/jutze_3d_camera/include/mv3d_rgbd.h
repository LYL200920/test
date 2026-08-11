#ifndef includeguard_mv3d_rgbd_h_includeguard
#define includeguard_mv3d_rgbd_h_includeguard

#include "common.hpp"
#include "camera_device_detail.h"
#include "camera_frame.h"
#include "camera_parameter.h"
#include "camera_stream_config.h"

#include <cstdint>
#include <string>
#include <vector>

class Mv3d_Rgbd_Camera
{
public:
    Mv3d_Rgbd_Camera() = default;
    ~Mv3d_Rgbd_Camera();

    Mv3d_Rgbd_Camera(const Mv3d_Rgbd_Camera &) = delete;
    Mv3d_Rgbd_Camera &operator=(const Mv3d_Rgbd_Camera &) = delete;

    bool Initialize(std::string *error = nullptr);
    bool Get_Version(SDK_VERSION &version, std::string *error = nullptr);
    bool Enumerate_Devices(std::vector<CAMERA_DEVICE_INFO> &devices, std::string *error = nullptr);
    bool Enumerate_Devices(std::vector<CAMERA_DEVICE_INFO> &devices,
                           std::vector<MV3D_RGBD_DEVICE_INFO> &sdk_devices,
                           std::string *error = nullptr);
    bool Open_Device(const MV3D_RGBD_DEVICE_INFO &sdk_device,
                     std::string *error = nullptr);
    bool Get_Device_Info(Camera_Device_Detail &detail,
                         std::string *error = nullptr);
    bool Get_Parameter(const std::string &key,
                       Camera_Parameter &parameter,
                       std::string *error = nullptr);
    bool Set_Parameter(const Camera_Parameter_Update &update,
                       std::string *error = nullptr);
    bool Get_Stream_Configuration(std::vector<Camera_Stream_Config> &streams,
                                  std::string *error = nullptr);
    bool Close_Device(std::string *error = nullptr);
    bool Start(std::string *error = nullptr);
    bool Stop(std::string *error = nullptr);
    bool Fetch_Frame(Camera_Frame &frame,
                     std::uint32_t timeout_ms,
                     MV3D_RGBD_STATUS *status_code = nullptr,
                     std::string *error = nullptr);
    void Shut_Down();

    bool Is_Device_Open() const { return m_handle != nullptr; }
    bool Is_Grabbing() const { return m_grabbing; }

private:
    bool m_initialized = false;
    HANDLE m_handle = nullptr;
    bool m_grabbing = false;

    static constexpr std::uint32_t kAllDeviceTypes = DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir;
};

#endif
