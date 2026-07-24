#include "point_cloud_document.h"

#include <cassert>
#include <cstdint>
#include <vector>

namespace
{

point_cloud::Point_Cloud_Data Make_Cloud()
{
  point_cloud::Point_Cloud_Data cloud;
  cloud.xyz = {
    0.0f, 0.0f, 1.0f,
    1.0f, 0.0f, 1.0f,
    2.0f, 0.0f, 1.0f,
    3.0f, 0.0f, 1.0f};
  cloud.rgb = {
    10, 11, 12,
    20, 21, 22,
    30, 31, 32,
    40, 41, 42};
  return cloud;
}

void Test_Selection_Modes()
{
  point_cloud::Point_Cloud_Document document;
  document.Set_Data(Make_Cloud());
  document.Set_Selection({1, 3});
  document.Set_Selection({2, 3}, point_cloud::Point_Selection_Mode::Add);
  assert((document.Selection() == std::vector<std::size_t>{1, 2, 3}));
  document.Set_Selection({0, 2}, point_cloud::Point_Selection_Mode::Toggle);
  assert((document.Selection() == std::vector<std::size_t>{0, 1, 3}));
}

void Test_Delete_Undo_Redo()
{
  point_cloud::Point_Cloud_Document document;
  document.Set_Data(Make_Cloud());
  document.Set_Selection({1, 3});
  assert(document.Delete_Selected() == 2);
  assert(document.Point_Count() == 2);
  assert((document.Data().xyz == std::vector<float>{
    0.0f, 0.0f, 1.0f,
    2.0f, 0.0f, 1.0f}));
  assert((document.Data().rgb == std::vector<std::uint8_t>{
    10, 11, 12,
    30, 31, 32}));

  assert(document.Undo());
  assert(document.Data().xyz == Make_Cloud().xyz);
  assert(document.Data().rgb == Make_Cloud().rgb);
  assert((document.Selection() == std::vector<std::size_t>{1, 3}));

  assert(document.Redo());
  assert(document.Point_Count() == 2);
  assert(document.Selection().empty());
}

void Test_Selection_Is_Normalized()
{
  point_cloud::Point_Cloud_Document document;
  document.Set_Data(Make_Cloud());
  document.Set_Selection({3, 1, 3, 99});
  assert((document.Selection() == std::vector<std::size_t>{1, 3}));
}

void Test_Delete_All_Can_Be_Undone()
{
  point_cloud::Point_Cloud_Document document;
  document.Set_Data(Make_Cloud());
  document.Set_Selection({0, 1, 2, 3});
  assert(document.Delete_Selected() == 4);
  assert(document.Point_Count() == 0);
  assert(document.Undo());
  assert(document.Data().xyz == Make_Cloud().xyz);
  assert(document.Data().rgb == Make_Cloud().rgb);
}

}

int main()
{
  Test_Selection_Modes();
  Test_Delete_Undo_Redo();
  Test_Selection_Is_Normalized();
  Test_Delete_All_Can_Be_Undone();
  return 0;
}
