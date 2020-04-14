/******************************************************************************
 * \author    Emelyanov Dmitry <dmitriy.emelyanov.de@gmail.com>
 *
 * \brief     Helper functions for implementing geometric algorithms
 *
 ******************************************************************************/

#ifndef CONCAVE_UTILITY_HPP
#define CONCAVE_UTILITY_HPP

#include <cmath>
#include <functional>
#include <type_traits>
#include <utility>

namespace concave::utility {
/**
  * \brief        Has a class function member or field 'x | x()' and 'y | y()'
  */
template<typename T, typename = void>
struct HasCoordinates : std::false_type {
};

template<typename T>
struct HasCoordinates<T, typename std::enable_if_t<std::is_member_pointer_v<decltype(&T::x)>
                                                   && std::is_member_pointer_v<decltype(&T::y)>>> : std::true_type {
};

template<typename T>
inline constexpr bool has_coordinates_v = HasCoordinates<T>::value;

enum class Orientation : std::uint8_t {
    COUNTERCLOCKWISE,
    CLOCKWISE,
    COLINEAR,
    UNKNOWN
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
[[nodiscard]] constexpr decltype(auto) distance(PointT&& t_f, PointU&& t_s) noexcept
{
  using TypePointT = std::decay_t<PointT>;
  using TypePointU = std::decay_t<PointU>;

  static_assert(has_coordinates_v<TypePointT>, "Type must have x and y class functions or fields");
  static_assert(has_coordinates_v<TypePointU>, "Type must have x and y class functions or fields");

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
[[nodiscard]] constexpr decltype(auto) orientetion(PointT&& t_f, PointU&& t_s, PointF&& t_t) noexcept
{
  using TypePointT = std::decay_t<PointT>;
  using TypePointU = std::decay_t<PointU>;
  using TypePointF = std::decay_t<PointF>;

  static_assert(has_coordinates_v<TypePointT>, "Type must have x and y class functions or fields");
  static_assert(has_coordinates_v<TypePointU>, "Type must have x and y class functions or fields");
  static_assert(has_coordinates_v<TypePointF>, "Type must have x and y class functions or fields");

  auto xt { std::mem_fn(&TypePointT::x) }; // For t_f - t
  auto yt { std::mem_fn(&TypePointT::y) };
  auto xu { std::mem_fn(&TypePointU::x) }; // For t_s - u
  auto yu { std::mem_fn(&TypePointU::y) };
  auto xf { std::mem_fn(&TypePointF::x) }; // For t_t - f
  auto yf { std::mem_fn(&TypePointF::y) };

  // see https://algs4.cs.princeton.edu/91primitives/
  auto o { (yu(t_s) - yt(t_f)) * (xf(t_t) - xu(t_s)) - (xu(t_s) - xt(t_f)) * (yf(t_t) - yu(t_s)) };

  if (std::isnan(o) || std::isinf(o)) {
    return Orientation::UNKNOWN;
  }

  if (std::abs(o) < std::numeric_limits<decltype(o)>::epsilon()) {
    return Orientation::COLINEAR;
  }

  if (o > 0) {
    return Orientation::CLOCKWISE;
  }

  return Orientation::COUNTERCLOCKWISE;
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
[[nodiscard]] constexpr decltype(auto) side(PointT&& t_f, PointU&& t_s, PointF&& t_t) noexcept
{
  // see https://www.geeksforgeeks.org/direction-point-line-segment/
  switch (auto o { orientetion(t_s, t_f, t_t) }; o) {
    case Orientation::UNKNOWN :
      return Side::Unknown;
      break;
    case Orientation::CLOCKWISE :
      return Side::LeftSide;
      break;
    case Orientation::COUNTERCLOCKWISE :
      return Side::RightSide;
      break;
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
[[nodiscard]] bool less(const PointT& t_f, const PointU& t_s) noexcept
{
  static_assert(has_coordinates_v<PointT>, "Type must have x and y class functions or fields");
  static_assert(has_coordinates_v<PointU>, "Type must have x and y class functions or fields");

  auto xt { std::mem_fn(&PointT::x) }; // For t_f - t
  auto yt { std::mem_fn(&PointT::y) };
  auto xu { std::mem_fn(&PointU::x) }; // For t_s - u
  auto yu { std::mem_fn(&PointT::y) };

  const auto epsilon { std::numeric_limits<typename std::common_type_t<decltype(xt(t_f)), decltype(xu(t_s))>>::epsilon() };
  return (yt(t_f) < yu(t_s)) || ((xt(t_f) < xu(t_s)) && ((std::abs(yt(t_f) - yu(t_s)) < epsilon)));
}
} // namespace concave::utility

#endif //CONCAVE_UTILITY_HPP