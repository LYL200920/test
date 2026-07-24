#include "vtk_scene.h"

#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLight.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <GL/gl.h>

#include <algorithm>

namespace robot_model
{
  namespace
  {

    void compute_scene_center_and_radius(vtkRenderer *renderer,
                                         double center[3],
                                         double &radius)
    {
      double bounds[6] = {-1000.0, 1000.0, -1000.0, 1000.0, 0.0, 1500.0};
      renderer->ComputeVisiblePropBounds(bounds);

      center[0] = (bounds[0] + bounds[1]) * 0.5;
      center[1] = (bounds[2] + bounds[3]) * 0.5;
      center[2] = (bounds[4] + bounds[5]) * 0.5;

      const double x_size = std::max(bounds[1] - bounds[0], 1.0);
      const double y_size = std::max(bounds[3] - bounds[2], 1.0);
      const double z_size = std::max(bounds[5] - bounds[4], 1.0);
      radius = std::max({x_size, y_size, z_size});
    }

  } // namespace

  Vtk_Scene::Vtk_Scene() = default;

  Vtk_Scene::~Vtk_Scene()
  {
    Shutdown();
  }

  void Vtk_Scene::On_Make_Current(vtkObject *, unsigned long, void *client_data, void *)
  {
    auto *self = static_cast<Vtk_Scene *>(client_data);
    if (self && self->m_canvas && self->m_gl_context)
    {
      self->m_canvas->SetCurrent(*self->m_gl_context);
    }
  }

  void Vtk_Scene::On_Is_Current(vtkObject *, unsigned long, void *client_data, void *call_data)
  {
    auto *is_current = static_cast<bool *>(call_data);
    auto *self = static_cast<Vtk_Scene *>(client_data);
    if (is_current && self && self->m_gl_context)
    {
      *is_current = true;
    }
  }

  void Vtk_Scene::On_Frame(vtkObject *, unsigned long, void *client_data, void *)
  {
    auto *self = static_cast<Vtk_Scene *>(client_data);
    if (self && self->m_canvas)
    {
      self->m_canvas->SwapBuffers();
    }
  }

  void Vtk_Scene::On_Supports_OpenGL(vtkObject *, unsigned long, void *, void *call_data)
  {
    auto *supports_opengl = static_cast<int *>(call_data);
    if (supports_opengl)
    {
      *supports_opengl = 1;
    }
  }

  void Vtk_Scene::Init(wxGLCanvas *canvas, wxGLContext *context)
  {
    if (m_ready)
    {
      return;
    }

    m_canvas = canvas;
    m_gl_context = context;

    m_render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_render_window->SetOwnContext(false);
    m_render_window->SetReadyForRendering(true);
    m_render_window->SetMultiSamples(4);
    m_render_window->SetNumberOfLayers(2);

    auto make_current_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    make_current_cb->SetClientData(this);
    make_current_cb->SetCallback(&Vtk_Scene::On_Make_Current);
    m_render_window->AddObserver(vtkCommand::WindowMakeCurrentEvent, make_current_cb);

    auto is_current_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    is_current_cb->SetClientData(this);
    is_current_cb->SetCallback(&Vtk_Scene::On_Is_Current);
    m_render_window->AddObserver(vtkCommand::WindowIsCurrentEvent, is_current_cb);

    auto frame_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    frame_cb->SetClientData(this);
    frame_cb->SetCallback(&Vtk_Scene::On_Frame);
    m_render_window->AddObserver(vtkCommand::WindowFrameEvent, frame_cb);

    auto supports_opengl_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    supports_opengl_cb->SetClientData(this);
    supports_opengl_cb->SetCallback(&Vtk_Scene::On_Supports_OpenGL);
    m_render_window->AddObserver(vtkCommand::WindowSupportsOpenGLEvent, supports_opengl_cb);

    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderer->SetBackground(0.08, 0.09, 0.11);
    m_renderer->SetLayer(0);

    m_render_window->AddRenderer(m_renderer);

    const auto size = canvas->GetClientSize();
    if (size.x > 0 && size.y > 0)
    {
      m_render_window->SetSize(size.x, size.y);
    }

    m_interactor = vtkSmartPointer<vtkGenericRenderWindowInteractor>::New();
    m_interactor->SetRenderWindow(m_render_window);
    auto style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    m_interactor->SetInteractorStyle(style);
    m_interactor->Initialize();
    m_interactor->Enable();

    Setup_View_Cube();

    m_ready = true;
  }

  void Vtk_Scene::Shutdown()
  {
    m_view_cube.Shutdown();

    if (m_interactor)
    {
      m_interactor->Disable();
      m_interactor->SetRenderWindow(nullptr);
      m_interactor = nullptr;
    }

    if (m_render_window)
    {
      m_render_window->RemoveObservers(vtkCommand::WindowMakeCurrentEvent);
      m_render_window->RemoveObservers(vtkCommand::WindowIsCurrentEvent);
      m_render_window->RemoveObservers(vtkCommand::WindowFrameEvent);
      m_render_window->RemoveObservers(vtkCommand::WindowSupportsOpenGLEvent);
      m_render_window->Finalize();
      m_render_window = nullptr;
    }

    m_renderer = nullptr;
    m_canvas = nullptr;
    m_gl_context = nullptr;
    m_ready = false;
  }

  void Vtk_Scene::Render()
  {
    if (m_ready && m_render_window)
    {
      Update_View_Cube_Camera();
      m_render_window->Render();
    }
  }

  void Vtk_Scene::Resize(int width, int height)
  {
    if (m_render_window && width > 0 && height > 0)
    {
      m_render_window->SetSize(width, height);
    }
    if (m_interactor)
    {
      m_interactor->SetSize(width, height);
    }
  }

  vtkGenericRenderWindowInteractor *Vtk_Scene::Get_Interactor() const
  {
    return m_interactor;
  }

  vtkRenderer *Vtk_Scene::Renderer() const
  {
    return m_renderer;
  }

  void Vtk_Scene::Add_Actor(vtkActor *actor)
  {
    if (m_renderer && actor)
    {
      m_renderer->AddActor(actor);
    }
  }

  void Vtk_Scene::Remove_All_ViewProps()
  {
    if (m_renderer)
    {
      m_renderer->RemoveAllViewProps();
    }
  }

  void Vtk_Scene::Remove_All_Lights()
  {
    if (m_renderer)
    {
      m_renderer->RemoveAllLights();
    }
  }

  void Vtk_Scene::Reset_Camera()
  {
    if (!m_renderer)
    {
      return;
    }

    double center[3] = {0.0, 0.0, 0.0};
    double radius = 1.0;
    compute_scene_center_and_radius(m_renderer, center, radius);
    const double camera_xy_scale = 1.25;
    const double camera_z_scale = 0.9;
    const double view_angle = 26.0;

    auto light = vtkSmartPointer<vtkLight>::New();
    light->SetLightTypeToSceneLight();
    light->SetPosition(center[0] + radius * 0.7,
                       center[1] - radius * 0.9,
                       center[2] + radius * 1.2);
    light->SetFocalPoint(center[0], center[1], center[2]);
    light->SetIntensity(0.95);
    m_renderer->AddLight(light);

    if (auto *camera = m_renderer->GetActiveCamera())
    {
      camera->SetFocalPoint(center[0], center[1], center[2]);
      camera->SetPosition(center[0] + radius * camera_xy_scale,
                          center[1] - radius * camera_xy_scale,
                          center[2] + radius * camera_z_scale);
      camera->SetViewUp(0.0, 0.0, 1.0);
      camera->SetViewAngle(view_angle);
    }
    m_renderer->ResetCameraClippingRange();
  }

  void Vtk_Scene::Setup_View_Cube()
  {
    m_view_cube.Setup(m_render_window, m_renderer);
  }

  void Vtk_Scene::Update_View_Cube_Camera()
  {
    m_view_cube.Update_Camera();
  }

  bool Vtk_Scene::Handle_View_Cube_Click(int x, int y)
  {
    const bool handled = m_view_cube.Handle_Click(x, y);
    if (handled)
    {
      Render();
    }
    return handled;
  }

  void Vtk_Scene::Add_Ground_Grid(double extent)
  {
    if (!m_renderer)
    {
      return;
    }

    const double step = extent > 2500.0 ? 500.0 : 250.0;
    const int line_count = static_cast<int>(extent / step);
    const double minor_color[3] = {0.20, 0.23, 0.26};
    const double major_color[3] = {0.31, 0.35, 0.39};
    const double border_color[3] = {0.45, 0.50, 0.55};

    for (int i = -line_count; i <= line_count; ++i)
    {
      const double p = static_cast<double>(i) * step;
      const bool major = i == 0 || (i % 2) == 0;
      const auto &color = major ? major_color : minor_color;

      auto x_line = vtkSmartPointer<vtkLineSource>::New();
      x_line->SetPoint1(-extent, p, 0.0);
      x_line->SetPoint2(extent, p, 0.0);

      auto x_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      x_mapper->SetInputConnection(x_line->GetOutputPort());

      auto x_actor = vtkSmartPointer<vtkActor>::New();
      x_actor->SetMapper(x_mapper);
      x_actor->GetProperty()->SetColor(color[0], color[1], color[2]);
      x_actor->GetProperty()->SetLineWidth(i == 0 ? 2.0 : 1.0);
      m_renderer->AddActor(x_actor);

      auto y_line = vtkSmartPointer<vtkLineSource>::New();
      y_line->SetPoint1(p, -extent, 0.0);
      y_line->SetPoint2(p, extent, 0.0);

      auto y_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      y_mapper->SetInputConnection(y_line->GetOutputPort());

      auto y_actor = vtkSmartPointer<vtkActor>::New();
      y_actor->SetMapper(y_mapper);
      y_actor->GetProperty()->SetColor(color[0], color[1], color[2]);
      y_actor->GetProperty()->SetLineWidth(i == 0 ? 2.0 : 1.0);
      m_renderer->AddActor(y_actor);
    }

    const double border_points[4][3] =
        {
            {-extent, -extent, 0.0},
            {extent, -extent, 0.0},
            {extent, extent, 0.0},
            {-extent, extent, 0.0},
        };

    for (int i = 0; i < 4; ++i)
    {
      const auto &start = border_points[i];
      const auto &end = border_points[(i + 1) % 4];

      auto border_line = vtkSmartPointer<vtkLineSource>::New();
      border_line->SetPoint1(start);
      border_line->SetPoint2(end);

      auto border_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      border_mapper->SetInputConnection(border_line->GetOutputPort());

      auto border_actor = vtkSmartPointer<vtkActor>::New();
      border_actor->SetMapper(border_mapper);
      border_actor->GetProperty()->SetColor(border_color[0],
                                            border_color[1],
                                            border_color[2]);
      border_actor->GetProperty()->SetLineWidth(2.0);
      m_renderer->AddActor(border_actor);
    }
  }

} // namespace robot_model
