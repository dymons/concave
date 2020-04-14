#ifndef CONCAVE_CONVEXHULL_HPP
#define CONCAVE_CONVEXHULL_HPP

#include "concave_core/utility/Utility.hpp"

#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace concave::extension {
template<typename T, typename U>
void quick_hull(const std::vector<T>& t_points, const typename std::vector<T>::const_iterator& t_leftmost,
                const typename std::vector<T>::const_iterator& t_rightmost, U&& t_convex_hull)
{
  auto x = std::mem_fn(&T::x);
  auto y = std::mem_fn(&T::y);

  // For a part, find the point is_point_far with maximum distance from the line (t_leftmost, t_rightmost).
  auto point_far { t_points.end() };
  double upper_distance { 0.0 }, current_distance { 0.0 }; // TODO: Replace with automatic type detection use type_traits.
  for (auto it { t_points.begin() }; it != t_points.end(); ++it) {
    if (utility::side(*t_leftmost, *t_rightmost, *it) == utility::Side::LeftSide) {
      current_distance = (y(*it) - y(t_leftmost)) * (x(t_rightmost) - x(t_leftmost));
      current_distance -= (y(t_rightmost) - y(t_leftmost)) * (x(*it) - x(t_leftmost));
      current_distance = std::abs(current_distance);
      if (current_distance > upper_distance) {
        upper_distance = current_distance;
        point_far = it;
      }
    }
  }

  // No point left with the line. Add the end points of this point to the convex hull.
  if (point_far == t_points.end()) {
    *t_convex_hull = *t_leftmost;
    ++t_convex_hull;
    return;
  }

  // The above step divides the problem into two sub-problems (solved recursively).
  // Now the line joining the points is_point_far and t_leftmost and the line joining
  // the points is_point_far and t_rightmost are new lines.
  quick_hull(t_points, t_leftmost, point_far, t_convex_hull);
  quick_hull(t_points, point_far, t_rightmost, t_convex_hull);
}
}  // namespace concave::extension

namespace concave {
enum class Pattern : std::size_t {
    BruteForce,
    GiftWrapping,
    GrahamScan,
    JarvisMarch,
    QuickHull,
    DivideAndConquer [[maybe_unused]],
    MonotoneChain,
    Incremental,
    MarriageBeforeConquest
};

template<auto T>
struct AlgorithmHull {
  using Type = decltype(T);
  static constexpr Type value = T;
};

template<typename T, typename U>
struct IsAlgorithm {
  static constexpr bool value { T::value == U::value };
};

template<auto U, typename T>
std::vector<T> convexHull(const std::vector<T>& t_points,
                          typename std::enable_if_t<
                          IsAlgorithm<AlgorithmHull<U>, AlgorithmHull<Pattern::JarvisMarch>>::value>*/*unused*/= 0)
{
  std::vector<T> convex_hull;
  convex_hull.reserve(t_points.size());

  if (t_points.size() < 3) {
    return convex_hull;
  }

  // Find the leftmost point in the point set given to us.
  const auto leftmost { std::min_element(t_points.begin(), t_points.end(), utility::LessThenY<T> {}) };

  auto current_point { leftmost }, second_point { leftmost };

  // Go following while we don’t come back to the first (or leftmost) point.
  do {
    convex_hull.push_back(*current_point);

    if (current_point == t_points.end() || current_point == --t_points.end()) {
      second_point = t_points.begin();
    } else {
      second_point = std::next(current_point);
    }

    // Find last triplet (current_point, middle_point, second_point) is counterclockwise.
    for (auto middle_point = t_points.begin(); middle_point != t_points.end(); ++middle_point) {
      if (utility::orientetion(*current_point, *middle_point, *second_point) == utility::Orientation::COUNTERCLOCKWISE) {
        second_point = middle_point;
      }
    }

    current_point = second_point;
  }
  while (current_point != leftmost);

  convex_hull.shrink_to_fit();
  return convex_hull;
}

template<auto U, typename T>
std::vector<T> convexHull(const std::vector<T>& t_points,
                          typename std::enable_if_t<IsAlgorithm<AlgorithmHull<U>, AlgorithmHull<Pattern::QuickHull>>::value>*/*unused*/= 0)
{
  std::vector<T> convex_hull;
  convex_hull.reserve(t_points.size());

  if (t_points.size() < 3) {
    return convex_hull;
  }

  // Find the point with minimum x-coordinate (leftmost), and similarly the point with maximum x-coordinate (rightmost).
  const auto[leftmost, rightmost] { std::minmax_element(t_points.begin(), t_points.end(), utility::LessThenY<T> {}) };

  // Make a line joining these two points. This line will divide the whole set into two parts.
  extension::quick_hull(t_points, leftmost, rightmost, std::back_inserter(convex_hull)); // For left side
  extension::quick_hull(t_points, rightmost, leftmost, std::back_inserter(convex_hull)); // For right side

  convex_hull.shrink_to_fit();
  return convex_hull;
}

template<auto U, typename T>
std::vector<T> convexHull(const std::vector<T>& t_points,
                          typename std::enable_if_t<
                          IsAlgorithm<AlgorithmHull<U>, AlgorithmHull<Pattern::GrahamScan>>::value>*/*unused*/= 0)
{
  std::vector<T> convex_hull;

  if (t_points.size() < 3) {
    return convex_hull;
  }

  std::vector<T> points_copy(t_points);

  // Find the leftmost point in the point set given to us and swap with begin point.
  auto leftmost { std::min_element(points_copy.begin(), points_copy.end(), utility::LessThenY<T> {}) };
  std::iter_swap(points_copy.begin(), leftmost);

  // Sort their points in the polar angle in the counterclockwise direction.
  std::sort(std::next(points_copy.begin()), points_copy.end(), [&](auto& lhs, auto& rhs) {
    return (utility::orientetion(points_copy.front(), lhs, rhs) == utility::Orientation::COUNTERCLOCKWISE);
  });

  convex_hull.reserve(points_copy.size());
  convex_hull.insert(convex_hull.end(), points_copy.begin(), std::next(points_copy.begin(), 3));

  for (auto current_point_it { std::next(points_copy.begin(), 3) }; current_point_it != points_copy.end(); ++current_point_it) {
    while (utility::orientetion(*std::next(convex_hull.rbegin()), convex_hull.back(), *current_point_it)
    != utility::Orientation::COUNTERCLOCKWISE) {
      convex_hull.pop_back();
    }

    convex_hull.push_back(*current_point_it);
  }

  convex_hull.shrink_to_fit();
  return convex_hull;
}

template<typename T>
std::vector<T> convexHull(const std::vector<T>& t_points)
{
  return convexHull<Pattern::GrahamScan>(t_points);
}
} // namespace concave

#endif //CONCAVE_CONVEXHULL_HPP