/******************************************************************************
 * \author    Emelyanov Dmitry <dmitriy.emelyanov.de@gmail.com>
 *
 * \brief     Helper functions for implementing geometric algorithms
 ******************************************************************************/

#ifndef CONCAVE_BASIC_OPERATIONS_HPP
#define CONCAVE_BASIC_OPERATIONS_HPP

#include <cmath>
#include <tuple>

namespace concave {

namespace detail {
template <std::size_t I, typename T> struct nth {
  inline static typename std::tuple_element<I, T>::type
  get(const T& t_t) { return std::get<I>(t_t); };
};
} // namespace detail

enum class Orientation : unsigned char {
    Counterclockwise,
    Clockwise,
    Colinear,
    Unknown
};

enum class Side : unsigned char {
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

  const auto x1 = detail::nth<0, TypePointT>::get(t_f);
  const auto y1 = detail::nth<1, TypePointT>::get(t_f);

  const auto x2 = detail::nth<0, TypePointU>::get(t_s);
  const auto y2 = detail::nth<1, TypePointU>::get(t_s);

  return std::hypot(x2 - x1, y2 - y1);
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
[[nodiscard]] constexpr decltype(auto) orientetion(const PointT& t_f, const PointU& t_s, const PointF& t_t) noexcept
{
  using TypePointT = std::decay_t<PointT>;
  using TypePointU = std::decay_t<PointU>;
  using TypePointF = std::decay_t<PointF>;

  const auto xt = detail::nth<0, TypePointT>::get(t_f);
  const auto yt = detail::nth<1, TypePointT>::get(t_f);

  const auto xu = detail::nth<0, TypePointU>::get(t_s);
  const auto yu = detail::nth<1, TypePointU>::get(t_s);

  const auto xf = detail::nth<0, TypePointU>::get(t_t);
  const auto yf = detail::nth<1, TypePointU>::get(t_t);

  // see https://algs4.cs.princeton.edu/91primitives/
  const auto o { (yu - yt) * (xf - xu) - (xu - xt) * (yf - yu) };

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
[[nodiscard]] constexpr decltype(auto) side(const PointT& t_f, const PointU& t_s, const PointF& t_t) noexcept
{
  // see https://www.geeksforgeeks.org/direction-point-line-segment/
  switch (auto o { orientetion(t_s, t_f, t_t) }; o) {
    case Orientation::Unknown :
      return Side::Unknown;
    case Orientation::Clockwise :
      return Side::LeftSide;
    case Orientation::Counterclockwise :
      return Side::RightSide;
  }

  return Side::StraightLine;
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
  const auto xt = detail::nth<0, PointT>::get(t_f);
  const auto yt = detail::nth<1, PointT>::get(t_f);

  const auto xu = detail::nth<0, PointU>::get(t_s);
  const auto yu = detail::nth<1, PointU>::get(t_s);

  const auto epsilon { std::numeric_limits<typename std::common_type_t<decltype(xt), decltype(xu)>>::epsilon() };
  return (yt < yu) || ((xt < xu) && ((std::abs(yt - yu) < epsilon)));
}
} // namespace concave

#endif //CONCAVE_BASIC_OPERATIONS_HPP