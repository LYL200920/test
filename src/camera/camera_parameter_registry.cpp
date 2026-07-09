#include "camera_parameter_registry.h"

#include <algorithm>

namespace
{
Camera_Parameter_Choice Choice (std::int64_t value, const char* label)
{
  return { value, label };
}
} // namespace

const std::vector<Camera_Parameter_Definition>&
First_Stage_Camera_Parameter_Definitions ( )
{
  static const std::vector<Camera_Parameter_Definition> definitions = {
    { MV3D_RGBD_ENUM_WORKINGMODE, "工作模式",
      Camera_Parameter_Group::Basic, {}, true },
    { MV3D_RGBD_FLOAT_EXPOSURETIME, "曝光时间 (us)",
      Camera_Parameter_Group::Basic, {}, false },
    { MV3D_RGBD_FLOAT_GAIN, "增益 (dB)",
      Camera_Parameter_Group::Basic, {}, false },
    { MV3D_RGBD_FLOAT_FRAMERATE, "采集帧率 (fps)",
      Camera_Parameter_Group::Basic, {}, false },
    { MV3D_RGBD_BOOL_LASERENABLE, "投射器使能",
      Camera_Parameter_Group::Basic, {}, false },

    { MV3D_RGBD_ENUM_IMAGEMODE, "图像模式",
      Camera_Parameter_Group::Image_Output, {}, true },
    { MV3D_RGBD_INT_IMAGEALIGN, "图像对齐",
      Camera_Parameter_Group::Image_Output,
      { Choice (0, "不对齐"), Choice (1, "对齐到彩色图"),
        Choice (2, "对齐到深度图") }, true },
    { MV3D_RGBD_ENUM_RESOLUTION, "采集分辨率",
      Camera_Parameter_Group::Image_Output, {}, true },
    { MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, "点云输出",
      Camera_Parameter_Group::Image_Output, {}, true },
    { MV3D_RGBD_INT_DOWNSAMPLE, "降采样",
      Camera_Parameter_Group::Image_Output,
      { Choice (0, "原始尺寸"), Choice (1, "宽高减半") }, true },
    { MV3D_RGBD_INT_OUTPUT_RGBD, "RGBD输出",
      Camera_Parameter_Group::Image_Output,
      { Choice (0, "关闭"), Choice (1, "开启") }, true },

    { MV3D_RGBD_ENUM_TRIGGERSELECTOR, "触发选择器",
      Camera_Parameter_Group::Trigger, {}, false },
    { MV3D_RGBD_ENUM_TRIGGERMODE, "触发模式",
      Camera_Parameter_Group::Trigger, {}, false },
    { MV3D_RGBD_ENUM_TRIGGERSOURCE, "触发源",
      Camera_Parameter_Group::Trigger, {}, false },
    { MV3D_RGBD_FLOAT_TRIGGERDELAY, "触发延迟 (us)",
      Camera_Parameter_Group::Trigger, {}, false }
  };
  return definitions;
}

const Camera_Parameter_Definition* Find_Camera_Parameter_Definition (
  const std::string& key)
{
  const auto& definitions = First_Stage_Camera_Parameter_Definitions ( );
  const auto found = std::find_if (
    definitions.begin ( ), definitions.end ( ),
    [&key] (const Camera_Parameter_Definition& definition)
    {
      return definition.key == key;
    });
  return found == definitions.end ( ) ? nullptr : &*found;
}

std::string Camera_Parameter_Enum_Label (
  const std::string& key,
  std::uint32_t value)
{
  if( key == MV3D_RGBD_ENUM_WORKINGMODE )
  {
    if( value == 0 ) return "传感器标定模式";
    if( value == 4 ) return "深度图模式";
    if( value == 10 ) return "体积测量模式";
  }
  else if( key == MV3D_RGBD_ENUM_IMAGEMODE )
  {
    if( value == 2 ) return "深度图模式";
    if( value == 8 ) return "彩色图模式";
    if( value == 12 ) return "Mono模式";
  }
  else if( key == MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT )
  {
    if( value == 0 ) return "关闭";
    if( value == 1 ) return "点云";
    if( value == 2 ) return "带法向量点云";
    if( value == 3 ) return "纹理点云";
    if( value == 4 ) return "带法向量纹理点云";
  }
  else if( key == MV3D_RGBD_ENUM_TRIGGERMODE )
  {
    if( value == 0 ) return "关闭";
    if( value == 1 ) return "开启";
  }

  return std::to_string (value);
}
