#ifndef includeguard_point_cloud_renderer_h_includeguard
#define includeguard_point_cloud_renderer_h_includeguard

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPLYReader.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>

namespace point_cloud
{

class Point_Cloud_Renderer
{
public:
  void Attach_Renderer (vtkRenderer* renderer);
  void Detach_Renderer ( );

  bool Load_Ply (const std::filesystem::path& path,
                 std::string* error_message = nullptr);
  bool Set_Point_Data (
    const std::vector<float>& xyz,
    const std::vector<std::uint8_t>& rgb = {},
    std::string* error_message = nullptr);
  void Clear ( );

  bool Has_Point_Cloud ( ) const { return m_actor != nullptr; }
  std::size_t Point_Count ( ) const { return m_point_count; }

private:
  vtkSmartPointer<vtkPolyData> Build_Display_Data (vtkPolyData* input) const;
  void Build_Actor ( );
  void Add_Actor_To_Renderer ( );

private:
  vtkRenderer* m_renderer = nullptr;
  vtkSmartPointer<vtkPLYReader> m_reader;
  vtkSmartPointer<vtkPolyData> m_display_data;
  vtkSmartPointer<vtkVertexGlyphFilter> m_glyph_filter;
  vtkSmartPointer<vtkPolyDataMapper> m_mapper;
  vtkSmartPointer<vtkActor> m_actor;
  std::size_t m_point_count = 0;
};

} // namespace point_cloud

#endif
