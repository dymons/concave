/******************************************************************************
 * \author    Emelyanov Dmitry <dmitriy.emelyanov.de@gmail.com>
 *
 * \brief     Helper functions for implementing geometric algorithms
 ******************************************************************************/

#ifndef CONCAVE_BASIC_OPERATIONS_HPP
#define CONCAVE_BASIC_OPERATIONS_HPP

#include "concave_core/concept/geometry.hpp"

#include <cmath>
#include <functional>

namespace concave {

enum class Orientation : std::uint8_t {
    Counterclockwise,
    Clockwise,
    Colinear,
    Unknown
};

enum class Side : std::uint8_t {
    RightSide,
    LeftSide,
    StraightLine,
    Unknown
};

inline Side operator~(Side& t_side)
{
  if (t_side == Side::RightSide) {
    return Side::LeftSide;
  }

  if (t_side == Side::LeftSide) {
    return Side::RightSide;
  }

  return t_side;
}

/**
  * \brief        Calculate the Euclidean distance between two points
  *
  * \param[in]    t_f - first point
  * \param[in]    t_s - second point
  *
  * \return       The Euclidean distance between two points
  */
template<typename PointT, typename PointU>
[[nodiscard]] constexpr decltype(auto) distance(Geometry<PointT>&& t_f, Geometry<PointU>&& t_s) noexcept
{
  using TypePointT = std::decay_t<PointT>;
  using TypePointU = std::decay_t<PointU>;

  auto xt { std::mem_fn(&TypePointT::x) };
  auto yt { std::mem_fn(&TypePointT::y) };
  auto xu { std::mem_fn(&TypePointU::x) };
  auto yu { std::mem_fn(&TypePointU::y) };

  return std::hypot(xu(t_s) - xt(t_f), yu(t_s) - yt(t_f));
}

/**
  * \brief        Given three points t_f, t_s, and t_t, determine whether they form a counterclockwise angle.
  *
  * \param[in]    t_f - first point
  * \param[in]    t_s - second point
  * \param[in]    t_t - third point
  *
  * \return       Return angle between three points t_f, t_s, and t_t
  */
template<typename PointT, typename PointU, typename PointF>
[[nodiscard]] constexpr decltype(auto) orientetion(Geometry<PointT>&& t_f, Geometry<PointU>&& t_s, Geometry<PointF>&& t_t) noexcept
{
  using TypePointT = std::decay_t<PointT>;
  using TypePointU = std::decay_t<PointU>;
  using TypePointF = std::decay_t<PointF>;

  auto xt { std::mem_fn(&TypePointT::x) }; // For t_f - t
  auto yt { std::mem_fn(&TypePointT::y) };
  auto xu { std::mem_fn(&TypePointU::x) }; // For t_s - u
  auto yu { std::mem_fn(&TypePointU::y) };
  auto xf { std::mem_fn(&TypePointF::x) }; // For t_t - f
  auto yf { std::mem_fn(&TypePointF::y) };

  // see https://algs4.cs.princeton.edu/91primitives/
  auto o { (yu(t_s) - yt(t_f)) * (xf(t_t) - xu(t_s)) - (xu(t_s) - xt(t_f)) * (yf(t_t) - yu(t_s)) };

  if (std::isnan(o) || std::isinf(o)) {
    return Orientation::Unknown;
  }

  if (std::abs(o) < std::numeric_limits<decltype(o)>::epsilon()) {
    return Orientation::Colinear;
  }

  if (o > 0) {
    return Orientation::Clockwise;
  }

  return Orientation::Counterclockwise;
}

/**
  * \brief        Finding the side on which the point lies
  *
  * \param[in]    t_f - first point owned by line
  * \param[in]    t_s - second point owned by line
  * \param[in]    t_t - third point, point lying next to the line
  *
  * \return       Returns the side with which the point lies relative to the line
  */
template<typename PointT, typename PointU, typename PointF>
[[nodiscard]] constexpr decltype(auto) side(Geometry<PointT>&& t_f, Geometry<PointU>&& t_s, Geometry<PointF>&& t_t) noexcept
{
  // see https://www.geeksforgeeks.org/direction-point-line-segment/
  switch (auto o { orientetion(t_s, t_f, t_t) }; o) {
    case Orientation::Unknown :
      return Side::Unknown;
    case Orientation::Clockwise :
      return Side::LeftSide;
    case Orientation::Counterclockwise :
      return Side::RightSide;
    default:
      return Side::StraightLine;
  }
}

/**
  * \brief        Estimate that the point t_f is less than t_s
  *
  * \param[in]    t_f - first point
  * \param[in]    t_s - second point
  *
  * \return       Returns true if the point t_f is less than t_s
  */
template<typename PointT, typename PointU>
[[nodiscard]] bool less(const Geometry<PointT>& t_f, const Geometry<PointU>& t_s) noexcept
{
  auto xt { std::mem_fn(&PointT::x) }; // For t_f - t
  auto yt { std::mem_fn(&PointT::y) };
  auto xu { std::mem_fn(&PointU::x) }; // For t_s - u
  auto yu { std::mem_fn(&PointT::y) };

  const auto epsilon { std::numeric_limits<typename std::common_type_t<decltype(xt(t_f)), decltype(xu(t_s))>>::epsilon() };
  return (yt(t_f) < yu(t_s)) || ((xt(t_f) < xu(t_s)) && ((std::abs(yt(t_f) - yu(t_s)) < epsilon)));
}
} // namespace concave

#endif //CONCAVE_BASIC_OPERATIONS_HPP