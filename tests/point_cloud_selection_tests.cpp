#include "point_cloud_selection.h"

#include <cassert>
#include <vector>

namespace
{

void Test_Convex_Polygon()
{
  const std::vector<point_cloud::Display_Point> polygon = {
    {0.0, 0.0},
    {10.0, 0.0},
    {10.0, 10.0},
    {0.0, 10.0}};
  assert(point_cloud::Is_Point_In_Display_Polygon({5.0, 5.0}, polygon));
  assert(!point_cloud::Is_Point_In_Display_Polygon({12.0, 5.0}, polygon));
  assert(point_cloud::Is_Point_In_Display_Polygon({10.0, 5.0}, polygon));
  assert(point_cloud::Is_Point_In_Display_Polygon({0.0, 0.0}, polygon));
}

void Test_Concave_Polygon()
{
  const std::vector<point_cloud::Display_Point> polygon = {
    {0.0, 0.0},
    {10.0, 0.0},
    {5.0, 5.0},
    {10.0, 10.0},
    {0.0, 10.0}};
  assert(point_cloud::Is_Point_In_Display_Polygon({2.0, 5.0}, polygon));
  assert(!point_cloud::Is_Point_In_Display_Polygon({8.0, 5.0}, polygon));
}

void Test_Clockwise_Polygon()
{
  const std::vector<point_cloud::Display_Point> polygon = {
    {0.0, 0.0},
    {0.0, 10.0},
    {10.0, 10.0},
    {10.0, 0.0}};
  assert(point_cloud::Is_Point_In_Display_Polygon({5.0, 5.0}, polygon));
}

void Test_Degenerate_Polygon()
{
  const std::vector<point_cloud::Display_Point> too_few_vertices = {
    {0.0, 0.0},
    {10.0, 0.0}};
  assert(!point_cloud::Is_Point_In_Display_Polygon(
    {5.0, 0.0}, too_few_vertices));

  const std::vector<point_cloud::Display_Point> collinear = {
    {0.0, 0.0},
    {5.0, 0.0},
    {10.0, 0.0}};
  assert(!point_cloud::Is_Point_In_Display_Polygon(
    {5.0, 0.0}, collinear));
}

}

int main()
{
  Test_Convex_Polygon();
  Test_Concave_Polygon();
  Test_Clockwise_Polygon();
  Test_Degenerate_Polygon();
  return 0;
}
