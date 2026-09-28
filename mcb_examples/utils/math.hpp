
#ifndef includeguard_utils_math_includeguard
#define includeguard_utils_math_includeguard

#include <cstdint>
#include <limits>
#include <type_traits>
#include <optional>
#include <utility>
#include "vec_mat.hpp"

namespace utils
{

template <typename T> constexpr inline T pi (void)
{
  return static_cast<T> (3.14159265358979323846);
}

template <typename T> constexpr inline T deg_to_rad (T val)
{
  return val * T (pi<T> () / 180);
}

template <typename T> constexpr inline T rad_to_deg (T val)
{
  return val * T (180 / pi<T> ());
}

// in C++17 there will be a std::clamp
template <typename T, typename TMin, typename TMax> constexpr inline T
clamp (T val, TMin min_val, TMax max_val)
{
  return std::max (T (min_val), std::min (T (max_val), val));
}

template <typename ClampT, typename T> constexpr inline ClampT
clamp (T val)
{
  constexpr auto max_val = std::numeric_limits<ClampT>::max ();
  constexpr auto min_val = std::numeric_limits<ClampT>::lowest ();

  return val > max_val
	 ? (T)max_val
	 : val < min_val
	   ? (T)min_val
	   : val;
}

template <typename T, typename TT> constexpr inline T
lerp (const T& val0, const T& val1, const TT& weight_0_1)
{
  // val0 * (1 - weight_0_1) + val1 * weight_0_1
  // val0 - val0 * w + val1 * w
  // val0 + val1 * w - val0 * w
  // val0 + w * (val1 - val0)
  if constexpr (std::is_signed_v<T>)
    return val0 + weight_0_1 * (val1 - val0);
  else
    return val0 * (1 - weight_0_1) + val1 * weight_0_1;
}


#if 1
template <typename T>
inline std::optional<vec2<T>>
intersect_line_segments (const vec2<T>& a0, const vec2<T>& a1,
			 const vec2<T>& b0, const vec2<T>& b1)
{
  auto s1 = a1 - a0;
  auto s2 = b1 - b0;

  auto x = 1 / cross (s1, s2);
  auto s = (-s1.y * (a0.x - b0.x) + s1.x * (a0.y - b0.y)) * x;
  auto t = ( s2.x * (a0.y - b0.y) - s2.y * (a0.x - b0.x)) * x;

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    return a0 + t * s1;

  else
    return { };
}
#endif


#if 0
template <typename T>
inline std::optional<vec2<T>>
intersect_line_segments (const vec2<T>& a0, const vec2<T>& a1,
			 const vec2<T>& b0, const vec2<T>& b1)
{
  auto p = a0;
  auto r = a1 - a0;
  auto q = b0;
  auto s = b1 - b0;

  auto num = cross (p - q, r);
  auto den = cross (r, s);

  const T zero = 0.0000001f;

  if (std::abs (den) < zero)
  {
    if (std::abs (num) < zero)
    {
      // line segments a colinear
      auto t0 = dot (q - p, r) / dot (r, r);
      auto t1 = t0 + dot (s, r) / dot (r, r);
      return vec2<T> ( t0, t1 );
    }
    else
    {
      // line segments are parallel non intersecting.
      return { };
    }
  }
  else
  {
    auto t = cross (q - p, s) / cross (r, s);
    auto u = cross (p - q, r) / cross (s, r);

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
      return p + t * r;
    else
      return { };
  }
}
#endif


#if 0
template <typename T>
inline std::optional<std::pair<vec2<T>, vec2<T>>>
clip_line_to_rect (const vec2<T>& a0, const vec2<T>& a1,
		   const vec2<T>& rect_top_left,
		   const vec2<T>& rect_bottom_right)
{
  const vec2<T> rect_top_right = { rect_bottom_right.x, rect_top_left.y };
  const vec2<T> rect_bottom_left = { rect_top_left.x, rect_bottom_right.y };


  const bool a0_inside =
	      a0.x >= rect_top_left.x && a0.x < rect_bottom_right.x
	   && a0.y >= rect_top_left.y && a0.y < rect_bottom_right.y;

  const bool a1_inside =
	      a1.x >= rect_top_left.x && a1.x < rect_bottom_right.x
	   && a1.y >= rect_top_left.y && a1.y < rect_bottom_right.y;

  bool img_line_segment_ok = false;

  if (a0_inside && a1_inside)
  {
    // both points of the line segment are inside the image rectangle
    return std::make_pair (a0, a1);
  }

  std::optional<vec2<T>> ii[] =
  {
    intersect_line_segments (a0, a1, rect_top_left, rect_bottom_left),
    intersect_line_segments (a0, a1, rect_top_left, rect_top_right),
    intersect_line_segments (a0, a1, rect_bottom_left, rect_bottom_right),
    intersect_line_segments (a0, a1, rect_top_right, rect_bottom_right)
  };

  auto i = std::find_if (std::begin (ii), std::end (ii), [] (const auto& x) { return x.has_value (); });

  if (i != std::end (ii))
  {
    if (a0_inside)
      return std::make_pair (a0, i->value ());

    else if (a1_inside)
      return std::make_pair (a1, i->value ());

    auto j = std::find_if (i+1, std::end (ii), [] (const auto& x) { return x.has_value (); });

    if (j != std::end (ii))
      return std::make_pair (i->value (), j->value ());
  }

  return { };
}
#endif

template <typename T>
inline std::optional<std::pair<vec2<T>, vec2<T>>>
clip_line_to_rect (const vec2<T>& a0, const vec2<T>& a1,
		   const vec2<T>& rect_top_left,
		   const vec2<T>& rect_bottom_right)
{
  vec2<T> p[2] = { a0, a1 };

  for (int i = 0; i < 2; i ++)	// i = 0: check the line (p[0], p[1])	i = 1: check the line (p[1], p[0])
  {
    auto dx = p[i ^ 1].x - p[i].x;
    auto dy = p[i ^ 1].y - p[i].y;

    if (p[i].x < rect_top_left.x && p[i ^ 1].x > rect_top_left.x)
    {
      auto d = (rect_top_left.x - p[i].x) / dx;

      p[i].x = rect_top_left.x;
      p[i].y = p[i].y + dy * d;

      dx = p[i ^ 1].x - p[i].x;
      dy = p[i ^ 1].y - p[i].y;
    }

    if (p[i].x < rect_bottom_right.x && p[i ^ 1].x > rect_bottom_right.x)
    {
      auto d = (rect_bottom_right.x - p[i].x) / dx;

      p[i ^ 1].x = rect_bottom_right.x;
      p[i ^ 1].y = p[i].y + dy * d;

      dx = p[i ^ 1].x - p[i].x;
      dy = p[i ^ 1].y - p[i].y;
    }

    if (p[i].y < rect_top_left.y && p[i ^ 1].y > rect_top_left.y)
    {
      auto d = (rect_top_left.y - p[i].y) / dy;

      p[i].x = p[i].x + dx * d;
      p[i].y = rect_top_left.y;

      dx = p[i ^ 1].x - p[i].x;
      dy = p[i ^ 1].y - p[i].y;
    }

    if (p[i].y > rect_bottom_right.y && p[i ^ 1].y < rect_bottom_right.y)
    {
      auto d = (rect_bottom_right.y - p[i].y) / dy;

      p[i].x = p[i].x + dx * d;
      p[i].y = rect_bottom_right.y;

      dx = p[i ^ 1].x - p[i].x;
      dy = p[i ^ 1].y - p[i].y;
    }
  }

  if (   p[0].x >= rect_top_left.x && p[0].x <= rect_bottom_right.x
      && p[0].y >= rect_top_left.y && p[0].y <= rect_bottom_right.y
      && p[1].x >= rect_top_left.x && p[1].x <= rect_bottom_right.x
      && p[1].y >= rect_top_left.y && p[1].y <= rect_bottom_right.y)
    return std::make_pair (p[0], p[1]);

  return { };
}



} // namespace utils
#endif // includeguard_utils_math_includeguard
