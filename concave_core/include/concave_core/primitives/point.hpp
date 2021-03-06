/******************************************************************************
 * \author    Emelyanov Dmitry <dmitriy.emelyanov.de@gmail.com>
 *
 * \brief     Description of the point primitive
 ******************************************************************************/

#ifndef CONCAVE_POINT_HPP
#define CONCAVE_POINT_HPP

#include <cmath>
#include <fstream>
#include <type_traits>

namespace concave::primitives {
template<typename DataPoint = double>
class Point {
  public:
    /**
     * \brief          Custom constructor to initialize a point
     */
    explicit Point(DataPoint t_x = 0, DataPoint t_y = 0) : x { t_x }, y { t_y }
    {
    }

    /**
     * \brief          Check that the current point is equivalent t_other
     *
     * \param[in]      t_other - point with which the current compares
     *
     * \return         Returns true if the current point is equal t_other, otherwise false
     */
    [[nodiscard]] bool operator==(const Point<DataPoint>& t_other) const noexcept;

    /**
     * \brief          Check that the current point isn't equivalent t_other
     *
     * \param[in]      t_other - point with which the current compares
     *
     * \return         Returns true if the current point isn't equal t_other, otherwise false
     */
    [[nodiscard]] bool operator!=(const Point<DataPoint>& t_other) const noexcept;

    /**
     * \brief          Check that the current point is less than t_other by coordinates 'y'
     *
     * \param[in]      t_other - point with which the current compares
     *
     * \return         Returns true if the current point is less than t_other, otherwise false
     */
    [[nodiscard]] bool operator<(const Point<DataPoint>& t_other) const noexcept;

    /**
     * \brief          Check that the current point is greater than t_other by coordinates 'y'
     *
     * \param[in]      t_other - point with which the current compares
     *
     * \return         Returns true if the current point is greater than t_other, otherwise false
     */
    [[nodiscard]] bool operator>(const Point<DataPoint>& t_other) const noexcept;

  public:
    DataPoint x;
    DataPoint y;
};

template<typename DataPoint>
bool Point<DataPoint>::operator==(const Point<DataPoint>& t_other) const noexcept
{
  if constexpr (std::is_floating_point_v<DataPoint>) {
    return (std::abs(this->x - t_other.x) < std::numeric_limits<DataPoint>::epsilon())
    && (std::abs(this->y - t_other.y) < std::numeric_limits<DataPoint>::epsilon());
  } else {
    return ((this->x == t_other.x) && (this->y == t_other.y));
  }
}

template<typename DataPoint>
bool Point<DataPoint>::operator!=(const Point<DataPoint>& t_other) const noexcept
{
  return !(*this == t_other);
}

template<typename DataPoint>
bool Point<DataPoint>::operator<(const Point<DataPoint>& t_other) const noexcept
{
  return (this->y < t_other.y) || ((std::abs(this->y - t_other.y) < std::numeric_limits<DataPoint>::epsilon()) && this->x < t_other.x);
}

template<typename DataPoint>
bool Point<DataPoint>::operator>(const Point<DataPoint>& t_other) const noexcept
{
  return !(*this < t_other);
}

std::istream& operator>>(std::istream& t_istream, primitives::Point<double>& t_point)
{
  t_istream >> t_point.x >> t_point.y;
  return t_istream;
}
} // namespace concave::primitives

#endif //CONCAVE_POINT_HPP