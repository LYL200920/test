#include "camera_parameter.h"

#include <iostream>
#include <stdexcept>

namespace
{
void Require (bool condition, const char* message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}

void Test_Int_Parameter ( )
{
  MV3D_RGBD_PARAM source{};
  source.enParamType = ParamType_Int;
  source.ParamInfo.stIntParam.nCurValue = 4;
  source.ParamInfo.stIntParam.nMin = 0;
  source.ParamInfo.stIntParam.nMax = 10;
  source.ParamInfo.stIntParam.nInc = 2;

  Camera_Parameter parameter;
  parameter.key = "TestInt";
  Require (
    Decode_Camera_Parameter (source, &parameter),
    "Integer parameter decode failed");
  Require (
    std::get<std::int64_t> (parameter.current_value) == 4,
    "Integer current value mismatch");

  Camera_Parameter_Update update;
  update.key = parameter.key;
  update.type = Camera_Parameter_Type::Int;
  update.value = std::int64_t{6};
  Require (
    Validate_Camera_Parameter_Update (parameter, update),
    "Valid integer update was rejected");

  MV3D_RGBD_PARAM encoded{};
  Require (
    Encode_Camera_Parameter_Update (update, &encoded),
    "Integer update encode failed");
  Require (
    encoded.enParamType == ParamType_Int &&
    encoded.ParamInfo.stIntParam.nCurValue == 6,
    "Encoded integer value mismatch");

  update.value = std::int64_t{7};
  Require (
    !Validate_Camera_Parameter_Update (parameter, update),
    "Invalid integer increment was accepted");
}

void Test_Enum_Parameter ( )
{
  MV3D_RGBD_PARAM source{};
  source.enParamType = ParamType_Enum;
  source.ParamInfo.stEnumParam.nCurValue = 3;
  source.ParamInfo.stEnumParam.nSupportedNum = 2;
  source.ParamInfo.stEnumParam.nSupportValue[0] = 1;
  source.ParamInfo.stEnumParam.nSupportValue[1] = 3;

  Camera_Parameter parameter;
  parameter.key = "TestEnum";
  Require (
    Decode_Camera_Parameter (source, &parameter),
    "Enumeration parameter decode failed");
  Require (
    parameter.enum_values.size ( ) == 2,
    "Enumeration options mismatch");

  Camera_Parameter_Update update;
  update.key = parameter.key;
  update.type = Camera_Parameter_Type::Enum;
  update.value = std::uint32_t{1};
  Require (
    Validate_Camera_Parameter_Update (parameter, update),
    "Supported enumeration update was rejected");

  update.value = std::uint32_t{2};
  Require (
    !Validate_Camera_Parameter_Update (parameter, update),
    "Unsupported enumeration update was accepted");
}

void Test_Bool_And_Float_Encoding ( )
{
  Camera_Parameter_Update bool_update;
  bool_update.key = "Bool";
  bool_update.type = Camera_Parameter_Type::Bool;
  bool_update.value = true;
  MV3D_RGBD_PARAM encoded_bool{};
  Require (
    Encode_Camera_Parameter_Update (bool_update, &encoded_bool),
    "Boolean update encode failed");
  Require (
    encoded_bool.enParamType == ParamType_Bool &&
    encoded_bool.ParamInfo.bBoolParam == TRUE,
    "Encoded boolean mismatch");

  Camera_Parameter_Update float_update;
  float_update.key = "Float";
  float_update.type = Camera_Parameter_Type::Float;
  float_update.value = 12.5F;
  MV3D_RGBD_PARAM encoded_float{};
  Require (
    Encode_Camera_Parameter_Update (float_update, &encoded_float),
    "Float update encode failed");
  Require (
    encoded_float.enParamType == ParamType_Float &&
    encoded_float.ParamInfo.stFloatParam.fCurValue == 12.5F,
    "Encoded float mismatch");
}
} // namespace

int main ( )
{
  try
  {
    Test_Int_Parameter ( );
    Test_Enum_Parameter ( );
    Test_Bool_And_Float_Encoding ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }

  std::cout << "Camera parameter tests passed.\n";
  return 0;
}
