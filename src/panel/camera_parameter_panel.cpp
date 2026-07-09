#include "camera_parameter_panel.h"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
wxString From_Utf8 (const char* text)
{
  return wxString::FromUTF8 (text);
}

wxString From_Utf8 (const std::string& text)
{
  return wxString::FromUTF8 (text.c_str ( ));
}

wxString Group_Label (Camera_Parameter_Group group)
{
  switch( group )
  {
  case Camera_Parameter_Group::Basic:
    return From_Utf8 (u8"基础参数");
  case Camera_Parameter_Group::Image_Output:
    return From_Utf8 (u8"图像与输出");
  case Camera_Parameter_Group::Trigger:
    return From_Utf8 (u8"触发设置");
  }
  return From_Utf8 (u8"其他参数");
}

std::int64_t Numeric_Current_Value (const Camera_Parameter& parameter)
{
  if( parameter.type == Camera_Parameter_Type::Int )
  {
    return std::get<std::int64_t> (parameter.current_value);
  }
  return static_cast<std::int64_t> (
    std::get<std::uint32_t> (parameter.current_value));
}

wxString Choice_Label (const Camera_Parameter_Choice& choice)
{
  const auto numeric = std::to_string (choice.value);
  if( choice.label.empty ( ) || choice.label == numeric )
  {
    return From_Utf8 (numeric);
  }
  return From_Utf8 (choice.label) + wxString::Format (" (%lld)", choice.value);
}

bool Parse_Int64 (
  const wxString& text,
  std::int64_t* value)
{
  try
  {
    const auto source = text.ToStdString ( );
    std::size_t consumed = 0;
    const auto parsed = std::stoll (source, &consumed);
    if( consumed != source.size ( ) ) return false;
    *value = parsed;
    return true;
  }
  catch( const std::exception& )
  {
    return false;
  }
}

bool Parse_Float (const wxString& text, float* value)
{
  try
  {
    const auto source = text.ToStdString ( );
    std::size_t consumed = 0;
    const auto parsed = std::stof (source, &consumed);
    if( consumed != source.size ( ) ) return false;
    *value = parsed;
    return true;
  }
  catch( const std::exception& )
  {
    return false;
  }
}
} // namespace

Camera_Parameter_Panel::Camera_Parameter_Panel (wxWindow* parent)
  : wxScrolledWindow (parent, wxID_ANY)
{
  SetScrollRate (0, 10);

  auto* title = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"相机参数配置"));
  m_hint_text = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"进入页面后自动读取相机参数"));
  m_fields_panel = new wxPanel (this, wxID_ANY);
  m_fields_sizer = new wxBoxSizer (wxVERTICAL);
  m_fields_panel->SetSizer (m_fields_sizer);

  m_refresh_button = new wxButton (
    this, wxID_ANY, From_Utf8 (u8"重新读取"));
  m_apply_button = new wxButton (
    this, wxID_ANY, From_Utf8 (u8"应用更改"));
  m_refresh_button->Bind (
    wxEVT_BUTTON, &Camera_Parameter_Panel::On_Refresh, this);
  m_apply_button->Bind (
    wxEVT_BUTTON, &Camera_Parameter_Panel::On_Apply, this);

  auto* button_sizer = new wxBoxSizer (wxHORIZONTAL);
  button_sizer->Add (m_refresh_button, 1, wxEXPAND | wxRIGHT, 4);
  button_sizer->Add (m_apply_button, 1, wxEXPAND);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (m_hint_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_fields_panel, 0, wxEXPAND | wxLEFT | wxRIGHT, 4);
  sizer->Add (button_sizer, 0, wxEXPAND | wxALL, 8);
  sizer->AddStretchSpacer (1);
  SetSizer (sizer);
}

void Camera_Parameter_Panel::Set_Callbacks (Callbacks callbacks)
{
  m_callbacks = std::move (callbacks);
}

void Camera_Parameter_Panel::Update_View (
  const std::vector<Camera_Parameter>& parameters,
  bool parameters_loaded,
  const std::string& parameter_status,
  bool device_open,
  bool grabbing,
  bool busy)
{
  if( Needs_Rebuild (parameters) )
  {
    Rebuild_Editors (parameters);
  }

  m_refresh_button->Enable (device_open && !busy);
  m_apply_button->Enable (
    device_open && !grabbing && !busy && parameters_loaded &&
    !parameters.empty ( ));
  m_fields_panel->Enable (device_open && !grabbing && !busy);

  if( !device_open )
  {
    m_hint_text->SetLabel (From_Utf8 (u8"请先打开相机"));
  }
  else if( busy )
  {
    m_hint_text->SetLabel (From_Utf8 (u8"相机参数操作进行中……"));
  }
  else if( grabbing )
  {
    m_hint_text->SetLabel (
      From_Utf8 (u8"采集中仅允许查看参数，请停止采集后再修改"));
  }
  else if( !parameter_status.empty ( ) )
  {
    m_hint_text->SetLabel (From_Utf8 (parameter_status));
  }
  else if( parameters_loaded )
  {
    m_hint_text->SetLabel (From_Utf8 (u8"参数已加载"));
  }
  else
  {
    m_hint_text->SetLabel (From_Utf8 (u8"正在等待读取相机参数"));
  }
  m_hint_text->Wrap (360);

  Layout ( );
  FitInside ( );
}

bool Camera_Parameter_Panel::Needs_Rebuild (
  const std::vector<Camera_Parameter>& parameters) const
{
  if( parameters.size ( ) != m_editors.size ( ) )
  {
    return true;
  }
  for( std::size_t index = 0; index < parameters.size ( ); ++index )
  {
    const auto& old_parameter = m_editors[index].parameter;
    const auto& new_parameter = parameters[index];
    if( old_parameter.key != new_parameter.key ||
        old_parameter.type != new_parameter.type ||
        old_parameter.current_value != new_parameter.current_value ||
        old_parameter.choices.size ( ) != new_parameter.choices.size ( ) )
    {
      return true;
    }
    for( std::size_t choice_index = 0;
         choice_index < old_parameter.choices.size ( ); ++choice_index )
    {
      if( old_parameter.choices[choice_index].value !=
            new_parameter.choices[choice_index].value ||
          old_parameter.choices[choice_index].label !=
            new_parameter.choices[choice_index].label )
      {
        return true;
      }
    }
  }
  return false;
}

void Camera_Parameter_Panel::Rebuild_Editors (
  const std::vector<Camera_Parameter>& parameters)
{
  m_fields_sizer->Clear (true);
  m_editors.clear ( );

  const Camera_Parameter_Group groups[] = {
    Camera_Parameter_Group::Basic,
    Camera_Parameter_Group::Image_Output,
    Camera_Parameter_Group::Trigger
  };

  for( const auto group : groups )
  {
    const bool has_group = std::any_of (
      parameters.begin ( ), parameters.end ( ),
      [group] (const Camera_Parameter& parameter)
      {
        return parameter.group == group;
      });
    if( !has_group ) continue;

    auto* group_sizer = new wxStaticBoxSizer (
      wxVERTICAL, m_fields_panel, Group_Label (group));
    for( const auto& parameter : parameters )
    {
      if( parameter.group != group ) continue;

      auto* row_sizer = new wxBoxSizer (wxVERTICAL);
      auto* label = new wxStaticText (
        m_fields_panel, wxID_ANY, From_Utf8 (parameter.display_name));
      auto* control = Create_Editor_Control (parameter);
      auto* range = new wxStaticText (
        m_fields_panel, wxID_ANY, Range_Label (parameter));
      range->SetForegroundColour (wxColour (100, 100, 100));
      row_sizer->Add (label, 0, wxEXPAND | wxBOTTOM, 3);
      row_sizer->Add (control, 0, wxEXPAND | wxBOTTOM, 2);
      row_sizer->Add (range, 0, wxEXPAND);
      group_sizer->Add (row_sizer, 0, wxEXPAND | wxALL, 6);
      m_editors.push_back ({ parameter, control });
    }
    m_fields_sizer->Add (group_sizer, 0, wxEXPAND | wxBOTTOM, 8);
  }

  m_fields_panel->Layout ( );
}

wxWindow* Camera_Parameter_Panel::Create_Editor_Control (
  const Camera_Parameter& parameter)
{
  if( parameter.type == Camera_Parameter_Type::Bool )
  {
    auto* control = new wxCheckBox (
      m_fields_panel, wxID_ANY, From_Utf8 (u8"启用"));
    control->SetValue (std::get<bool> (parameter.current_value));
    return control;
  }

  if( !parameter.choices.empty ( ) )
  {
    auto* control = new wxChoice (m_fields_panel, wxID_ANY);
    const auto current = Numeric_Current_Value (parameter);
    int selection = wxNOT_FOUND;
    for( std::size_t index = 0; index < parameter.choices.size ( ); ++index )
    {
      control->Append (Choice_Label (parameter.choices[index]));
      if( parameter.choices[index].value == current )
      {
        selection = static_cast<int> (index);
      }
    }
    if( selection != wxNOT_FOUND ) control->SetSelection (selection);
    return control;
  }

  wxString value;
  switch( parameter.type )
  {
  case Camera_Parameter_Type::Int:
    value = wxString::Format (
      "%lld", std::get<std::int64_t> (parameter.current_value));
    break;
  case Camera_Parameter_Type::Float:
    value = wxString::Format (
      "%.9g", static_cast<double> (
        std::get<float> (parameter.current_value)));
    break;
  case Camera_Parameter_Type::String:
    value = From_Utf8 (std::get<std::string> (parameter.current_value));
    break;
  case Camera_Parameter_Type::Enum:
    value = wxString::Format (
      "%u", std::get<std::uint32_t> (parameter.current_value));
    break;
  case Camera_Parameter_Type::Bool:
    break;
  }
  return new wxTextCtrl (m_fields_panel, wxID_ANY, value);
}

wxString Camera_Parameter_Panel::Range_Label (
  const Camera_Parameter& parameter) const
{
  if( parameter.type == Camera_Parameter_Type::Int &&
      parameter.int_minimum && parameter.int_maximum )
  {
    wxString text = wxString::Format (
      From_Utf8 (u8"范围：%lld ~ %lld"),
      *parameter.int_minimum, *parameter.int_maximum);
    if( parameter.int_increment && *parameter.int_increment > 0 )
    {
      text += wxString::Format (
        From_Utf8 (u8"，步进：%lld"), *parameter.int_increment);
    }
    return text;
  }
  if( parameter.type == Camera_Parameter_Type::Float &&
      parameter.float_minimum && parameter.float_maximum )
  {
    return wxString::Format (
      From_Utf8 (u8"范围：%.6g ~ %.6g"),
      static_cast<double> (*parameter.float_minimum),
      static_cast<double> (*parameter.float_maximum));
  }
  if( !parameter.choices.empty ( ) )
  {
    return wxString::Format (
      From_Utf8 (u8"支持 %zu 个选项"), parameter.choices.size ( ));
  }
  return From_Utf8 (u8"当前设备支持的参数");
}

bool Camera_Parameter_Panel::Collect_Updates (
  std::vector<Camera_Parameter_Update>* updates,
  wxString* error_message) const
{
  updates->clear ( );
  for( const auto& editor : m_editors )
  {
    Camera_Parameter_Update update;
    update.key = editor.parameter.key;
    update.type = editor.parameter.type;

    if( editor.parameter.type == Camera_Parameter_Type::Bool )
    {
      update.value = static_cast<wxCheckBox*> (editor.control)->GetValue ( );
    }
    else if( !editor.parameter.choices.empty ( ) )
    {
      const int selection = static_cast<wxChoice*> (editor.control)->GetSelection ( );
      if( selection == wxNOT_FOUND )
      {
        *error_message = From_Utf8 (editor.parameter.display_name) +
                         From_Utf8 (u8"：请选择一个值");
        return false;
      }
      const auto numeric = editor.parameter.choices[
        static_cast<std::size_t> (selection)].value;
      update.value = editor.parameter.type == Camera_Parameter_Type::Enum
        ? Camera_Parameter_Value (static_cast<std::uint32_t> (numeric))
        : Camera_Parameter_Value (numeric);
    }
    else
    {
      const auto text = static_cast<wxTextCtrl*> (editor.control)->GetValue ( );
      if( editor.parameter.type == Camera_Parameter_Type::Int )
      {
        std::int64_t value = 0;
        if( !Parse_Int64 (text, &value) )
        {
          *error_message = From_Utf8 (editor.parameter.display_name) +
                           From_Utf8 (u8"：请输入有效整数");
          return false;
        }
        update.value = value;
      }
      else if( editor.parameter.type == Camera_Parameter_Type::Float )
      {
        float value = 0.0F;
        if( !Parse_Float (text, &value) )
        {
          *error_message = From_Utf8 (editor.parameter.display_name) +
                           From_Utf8 (u8"：请输入有效数值");
          return false;
        }
        update.value = value;
      }
      else if( editor.parameter.type == Camera_Parameter_Type::String )
      {
        update.value = text.ToStdString ( );
      }
      else if( editor.parameter.type == Camera_Parameter_Type::Enum )
      {
        std::int64_t value = 0;
        if( !Parse_Int64 (text, &value) || value < 0 ||
            static_cast<std::uint64_t> (value) >
              std::numeric_limits<std::uint32_t>::max ( ) )
        {
          *error_message = From_Utf8 (editor.parameter.display_name) +
                           From_Utf8 (u8"：请输入有效枚举值");
          return false;
        }
        update.value = static_cast<std::uint32_t> (value);
      }
      else
      {
        *error_message = From_Utf8 (editor.parameter.display_name) +
                         From_Utf8 (u8"：不支持的参数类型");
        return false;
      }
    }

    if( update.value == editor.parameter.current_value )
    {
      continue;
    }

    std::string validation_error;
    if( !Validate_Camera_Parameter_Update (
          editor.parameter, update, &validation_error) )
    {
      *error_message = From_Utf8 (editor.parameter.display_name) + ": " +
                       From_Utf8 (validation_error);
      return false;
    }
    updates->push_back (std::move (update));
  }
  return true;
}

void Camera_Parameter_Panel::On_Refresh (wxCommandEvent&)
{
  if( m_callbacks.refresh )
  {
    m_callbacks.refresh ( );
  }
}

void Camera_Parameter_Panel::On_Apply (wxCommandEvent&)
{
  std::vector<Camera_Parameter_Update> updates;
  wxString error_message;
  if( !Collect_Updates (&updates, &error_message) )
  {
    m_hint_text->SetLabel (error_message);
    return;
  }
  if( updates.empty ( ) )
  {
    m_hint_text->SetLabel (From_Utf8 (u8"没有需要应用的更改"));
    return;
  }
  if( m_callbacks.apply )
  {
    m_callbacks.apply (std::move (updates));
  }
}
