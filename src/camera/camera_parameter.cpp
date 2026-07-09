#include "camera_parameter.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>

namespace
{
bool Return_Error (const std::string& message, std::string* error)
{
  if( error )
  {
    *error = message;
  }
  return false;
}

template <std::size_t Size>
std::string From_Fixed_String (const char (&value)[Size])
{
  const auto end = std::find (std::begin (value), std::end (value), '\0');
  return std::string (std::begin (value), end);
}

template <typename Value>
const Value* Get_Value (
  const Camera_Parameter_Value& source,
  const char* expected,
  std::string* error)
{
  const auto* value = std::get_if<Value> (&source);
  if( value == nullptr )
  {
    Return_Error (
      std::string ("Camera parameter value is not ") + expected, error);
  }
  return value;
}
} // namespace

bool Decode_Camera_Parameter (
  const MV3D_RGBD_PARAM& source,
  Camera_Parameter* parameter,
  std::string* error)
{
  if( error )
  {
    error->clear ( );
  }
  if( parameter == nullptr )
  {
    return Return_Error ("Camera parameter destination is null", error);
  }

  parameter->int_minimum.reset ( );
  parameter->int_maximum.reset ( );
  parameter->int_increment.reset ( );
  parameter->float_minimum.reset ( );
  parameter->float_maximum.reset ( );
  parameter->string_max_length.reset ( );
  parameter->enum_values.clear ( );
  parameter->choices.clear ( );

  switch( source.enParamType )
  {
  case ParamType_Bool:
    parameter->type = Camera_Parameter_Type::Bool;
    parameter->current_value = source.ParamInfo.bBoolParam != FALSE;
    return true;

  case ParamType_Int:
    parameter->type = Camera_Parameter_Type::Int;
    parameter->current_value = source.ParamInfo.stIntParam.nCurValue;
    parameter->int_minimum = source.ParamInfo.stIntParam.nMin;
    parameter->int_maximum = source.ParamInfo.stIntParam.nMax;
    parameter->int_increment = source.ParamInfo.stIntParam.nInc;
    return true;

  case ParamType_Float:
    parameter->type = Camera_Parameter_Type::Float;
    parameter->current_value = source.ParamInfo.stFloatParam.fCurValue;
    parameter->float_minimum = source.ParamInfo.stFloatParam.fMin;
    parameter->float_maximum = source.ParamInfo.stFloatParam.fMax;
    return true;

  case ParamType_Enum:
  {
    parameter->type = Camera_Parameter_Type::Enum;
    parameter->current_value = source.ParamInfo.stEnumParam.nCurValue;
    const auto count = std::min<std::size_t> (
      source.ParamInfo.stEnumParam.nSupportedNum,
      MV3D_RGBD_MAX_ENUM_COUNT);
    parameter->enum_values.assign (
      source.ParamInfo.stEnumParam.nSupportValue,
      source.ParamInfo.stEnumParam.nSupportValue + count);
    return true;
  }

  case ParamType_String:
    parameter->type = Camera_Parameter_Type::String;
    parameter->current_value =
      From_Fixed_String (source.ParamInfo.stStringParam.chCurValue);
    parameter->string_max_length = std::min<std::uint32_t> (
      source.ParamInfo.stStringParam.nMaxLength,
      MV3D_RGBD_MAX_STRING_LENGTH - 1);
    return true;

  default:
    return Return_Error ("Unsupported camera parameter type", error);
  }
}

bool Encode_Camera_Parameter_Update (
  const Camera_Parameter_Update& update,
  MV3D_RGBD_PARAM* destination,
  std::string* error)
{
  if( error )
  {
    error->clear ( );
  }
  if( destination == nullptr )
  {
    return Return_Error ("SDK parameter destination is null", error);
  }
  *destination = {};

  switch( update.type )
  {
  case Camera_Parameter_Type::Bool:
  {
    const auto* value = Get_Value<bool> (update.value, "Bool", error);
    if( value == nullptr ) return false;
    destination->enParamType = ParamType_Bool;
    destination->ParamInfo.bBoolParam = *value ? TRUE : FALSE;
    return true;
  }
  case Camera_Parameter_Type::Int:
  {
    const auto* value = Get_Value<std::int64_t> (update.value, "Int", error);
    if( value == nullptr ) return false;
    destination->enParamType = ParamType_Int;
    destination->ParamInfo.stIntParam.nCurValue = *value;
    return true;
  }
  case Camera_Parameter_Type::Float:
  {
    const auto* value = Get_Value<float> (update.value, "Float", error);
    if( value == nullptr ) return false;
    if( !std::isfinite (*value) )
      return Return_Error ("Value must be finite", error);
    destination->enParamType = ParamType_Float;
    destination->ParamInfo.stFloatParam.fCurValue = *value;
    return true;
  }
  case Camera_Parameter_Type::Enum:
  {
    const auto* value = Get_Value<std::uint32_t> (update.value, "Enum", error);
    if( value == nullptr ) return false;
    destination->enParamType = ParamType_Enum;
    destination->ParamInfo.stEnumParam.nCurValue = *value;
    return true;
  }
  case Camera_Parameter_Type::String:
  {
    const auto* value = Get_Value<std::string> (update.value, "String", error);
    if( value == nullptr ) return false;
    destination->enParamType = ParamType_String;
    const auto copy_length = std::min (
      value->size ( ),
      sizeof (destination->ParamInfo.stStringParam.chCurValue) - 1);
    std::memcpy (
      destination->ParamInfo.stStringParam.chCurValue,
      value->data ( ),
      copy_length);
    destination->ParamInfo.stStringParam.chCurValue[copy_length] = '\0';
    return true;
  }
  }
  return Return_Error ("Unsupported camera parameter type", error);
}

bool Validate_Camera_Parameter_Update (
  const Camera_Parameter& current,
  const Camera_Parameter_Update& update,
  std::string* error)
{
  if( error )
  {
    error->clear ( );
  }
  if( current.key != update.key || current.type != update.type )
  {
    return Return_Error ("Camera parameter identity or type mismatch", error);
  }

  switch( update.type )
  {
  case Camera_Parameter_Type::Bool:
    return Get_Value<bool> (update.value, "Bool", error) != nullptr;

  case Camera_Parameter_Type::Int:
  {
    const auto* value = Get_Value<std::int64_t> (update.value, "Int", error);
    if( value == nullptr ) return false;
    if( current.int_minimum && *value < *current.int_minimum )
      return Return_Error ("Value is below the minimum", error);
    if( current.int_maximum && *value > *current.int_maximum )
      return Return_Error ("Value is above the maximum", error);
    if( current.int_increment && *current.int_increment > 0 &&
        current.int_minimum &&
        (*value - *current.int_minimum) % *current.int_increment != 0 )
      return Return_Error ("Value does not match the required increment", error);
    return true;
  }

  case Camera_Parameter_Type::Float:
  {
    const auto* value = Get_Value<float> (update.value, "Float", error);
    if( value == nullptr ) return false;
    if( !std::isfinite (*value) )
      return Return_Error ("Value must be finite", error);
    if( current.float_minimum && *value < *current.float_minimum )
      return Return_Error ("Value is below the minimum", error);
    if( current.float_maximum && *value > *current.float_maximum )
      return Return_Error ("Value is above the maximum", error);
    return true;
  }

  case Camera_Parameter_Type::Enum:
  {
    const auto* value = Get_Value<std::uint32_t> (update.value, "Enum", error);
    if( value == nullptr ) return false;
    if( !current.enum_values.empty ( ) &&
        std::find (
          current.enum_values.begin ( ), current.enum_values.end ( ), *value) ==
          current.enum_values.end ( ) )
      return Return_Error ("Enumeration value is not supported", error);
    return true;
  }

  case Camera_Parameter_Type::String:
  {
    const auto* value = Get_Value<std::string> (update.value, "String", error);
    if( value == nullptr ) return false;
    if( current.string_max_length && value->size ( ) > *current.string_max_length )
      return Return_Error ("String exceeds the maximum length", error);
    return true;
  }
  }
  return Return_Error ("Unsupported camera parameter type", error);
}
