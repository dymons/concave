/******************************************************************************
 * \author    Emelyanov Dmitry <dmitriy.emelyanov.de@gmail.com>
 *
 * \brief     Helper functions for implementing geometric algorithms
 *
 ******************************************************************************/

#ifndef CONCAVE_UTILITY_HPP_
#define CONCAVE_UTILITY_HPP_

#include <cmath>
#include <utility>
#include <functional>
#include <type_traits>

namespace concave::utility {
  /**
    * \brief        Has a class function member or field 'x | x()' and 'y | y()'
    */
  template <typename T, typename = void>
    struct has_coordinates : std::false_type {};

  template <typename T>
    struct has_coordinates<T, typename std::enable_if_t<std::is_member_pointer_v<decltype(&T::x)>
                                                     && std::is_member_pointer_v<decltype(&T::y)>>> : std::true_type {};

  template <typename T>
    inline constexpr bool has_coordinates_v = has_coordinates<T>::value;

  enum class Orientation : std::size_t {
    COUNTERCLOCKWISE,
    CLOCKWISE,
    COLINEAR
  };

  enum class Side : std::size_t {
    RightSide,
    LeftSide,
    StraightLine
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
  template <typename PointT, typename PointU>
    [[nodiscard]] constexpr decltype(auto) distance (PointT&& t_f, PointU&& t_s) noexcept
    {
      static_assert(has_coordinates_v<std::decay_t<PointT>>, "Type must have x and y class functions or fields");
      static_assert(has_coordinates_v<std::decay_t<PointU>>, "Type must have x and y class functions or fields");

      auto xt = std::mem_fn(&PointT::x);
      auto yt = std::mem_fn(&PointT::y);
      auto xu = std::mem_fn(&PointU::x);
      auto yu = std::mem_fn(&PointU::y);

      return std::hypot(xu(t_s) - xt(t_f), yu(t_s) - yt(t_f));
    }

  template <typename T>
  [[nodiscard]] constexpr decltype(auto) distance (T&& t_f, T&& t_s, T&& t_t) noexcept
    {
      using TypePoint = std::remove_reference_t<T>;
      if constexpr (std::is_member_function_pointer_v<decltype(&TypePoint::x)> && std::is_member_function_pointer_v<decltype(&TypePoint::y)>) {
        return (t_t.y() - t_f.y()) * (t_s.x() - t_f.x()) - (t_s.y() - t_f.y()) * (t_t.x() - t_f.x());
      } else {
        return (t_t.y - t_f.y) * (t_s.x - t_f.x) - (t_s.y - t_f.y) * (t_t.x - t_f.x);
      }
    }

  template <typename T>
  [[nodiscard]] constexpr decltype(auto) orientetion (T&& t_f, T&& t_s, T&& t_t) noexcept
    {
      using TypePoint = std::remove_reference_t<T>;
      if constexpr (std::is_member_function_pointer_v<decltype(&TypePoint::x)> && std::is_member_function_pointer_v<decltype(&TypePoint::y)>) {
        auto o = ((t_s.y() - t_f.y()) * (t_t.x() - t_s.x()) - (t_s.x() - t_f.x()) * (t_t.y() - t_s.y()));

        if (std::abs(o) < std::numeric_limits<decltype(o)>::epsilon()) {
          return Orientation::COLINEAR;
        }

        if (o > 0) {
          return Orientation::CLOCKWISE;
        }

      } else {
        auto o = ((t_s.y - t_f.y) * (t_t.x - t_s.x) - (t_s.x - t_f.x) * (t_t.y - t_s.y));

        if (std::abs(o) < std::numeric_limits<decltype(o)>::epsilon()) {
          return Orientation::COLINEAR;
        }

        if (o > 0) {
          return Orientation::CLOCKWISE;
        }
      }

      return Orientation::COUNTERCLOCKWISE;
    }

  template <typename T>
    struct less_then_x {
    [[nodiscard]] constexpr decltype(auto) operator() (const T& t_f, const T& t_s) noexcept
      {
        using TypePoint = std::remove_reference_t<T>;
        if constexpr (std::is_member_function_pointer_v<decltype(&TypePoint::x)> && std::is_member_function_pointer_v<decltype(&TypePoint::y)>) {
          return (t_f.x() < t_s.x()) || ((std::abs(t_f.x() - t_s.x()) < std::numeric_limits<typename std::common_type_t<decltype(t_f.x()), decltype(t_s.x())>>::epsilon()) && t_f.y() < t_s.y());
        } else {
          return (t_f.x < t_s.x) || ((std::abs(t_f.x - t_s.x) < std::numeric_limits<typename std::common_type_t<decltype(t_f.x), decltype(t_s.x)>>::epsilon()) && t_f.y < t_s.y);
        }
      }
    };

  template <typename T>
    struct less_then_y {
    [[nodiscard]] constexpr decltype(auto) operator() (const T& t_f, const T& t_s) noexcept
      {
        using TypePoint = std::remove_reference_t<T>;
        if constexpr (std::is_member_function_pointer_v<decltype(&TypePoint::x)> && std::is_member_function_pointer_v<decltype(&TypePoint::y)>) {
          return (t_f.y() < t_s.y()) || ((std::abs(t_f.y() - t_s.y()) < std::numeric_limits<typename std::common_type_t<decltype(t_f.x()), decltype(t_s.x())>>::epsilon()) && t_f.x() < t_s.x());
        } else {
          return (t_f.y < t_s.y) || ((std::abs(t_f.y - t_s.y) < std::numeric_limits<typename std::common_type_t<decltype(t_f.x), decltype(t_s.x)>>::epsilon()) && t_f.x < t_s.x);
        }
      }
    };

  template <typename T>
  [[nodiscard]] constexpr decltype(auto) side (T&& t_f, T&& t_s, T&& t_t) noexcept
    {
      using TypePoint = std::remove_reference_t<T>;
      if constexpr (std::is_member_function_pointer_v<decltype(&TypePoint::x)> && std::is_member_function_pointer_v<decltype(&TypePoint::y)>) {
        auto s = distance(t_f, t_s, t_t);

        if (s > 0) {
          return Side::LeftSide;
        }

        if (s < 0) {
          return Side::RightSide;
        }
      } else {
        auto s = distance(t_f, t_s, t_t);

        if (s > 0) {
          return Side::LeftSide;
        }

        if (s < 0) {
          return Side::RightSide;
        }
      }

      return Side::StraightLine;
    }
} // namespace concave::utility

#endif // CONCAVE_UTILITY_HPP_