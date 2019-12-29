#pragma once

#ifndef CONCAVE_CONVEXHULL_HPP_
#define CONCAVE_CONVEXHULL_HPP_

#include "concave_core/utility/Utility.hpp"

#include <vector>
#include <algorithm>
#include <type_traits>

namespace concave::extension {
  template <typename T, typename U>
    void quickHull (const std::vector<T>& t_points, const typename std::vector<T>::const_iterator& t_leftmost, const typename std::vector<T>::const_iterator& t_rightmost, U&& t_convex_hull)
    {
      // For a part, find the point is_point_far with maximum distance from the line (t_leftmost, t_rightmost).
      auto is_point_far {t_points.end()};
      double distance_upper {0.0}, current_distance {0.0}; // TODO: Replace with automatic type detection use type_traits.
      for (auto it {t_points.begin()}; it != t_points.end(); ++it) {
        if (utility::side(*t_leftmost, *t_rightmost, *it) == utility::Side::LeftSide) {
          current_distance = std::abs(utility::distance(*t_leftmost, *t_rightmost, *it));
          if (current_distance > distance_upper) {
            distance_upper = current_distance;
            is_point_far = it;
          }
        }
      }

      // No point left with the line. Add the end points of this point to the convex hull.
      if (is_point_far == t_points.end()) {
        *t_convex_hull = *t_leftmost;
        ++t_convex_hull;
        return;
      }

      // The above step divides the problem into two sub-problems (solved recursively).
      // Now the line joining the points is_point_far and t_leftmost and the line joining
      // the points is_point_far and t_rightmost are new lines.
      quickHull(t_points, t_leftmost, is_point_far, t_convex_hull);
      quickHull(t_points, is_point_far, t_rightmost, t_convex_hull);
    }
}  // namespace concave::extension

namespace concave {
  enum class Pattern : std::size_t {
    BruteForce,
    GiftWrapping,
    GrahamScan,
    JarvisMarch,
    QuickHull,
    DivideAndConquer,
    MonotoneChain,
    Incremental,
    MarriageBeforeConquest
  };

  template <auto T>
    struct AlgorithmHull {
      using type = decltype(T);
      static constexpr type value = T;
    };

  template <typename T, typename U>
    struct is_algorithm {
      static constexpr bool value { T::value == U::value };
    };

  template <typename U, typename T>
    std::vector<T> convexHull (const std::vector<T>& t_points, typename std::enable_if_t<is_algorithm<U, AlgorithmHull<Pattern::JarvisMarch>>::value>* = 0)
    {
      std::vector<T> convex_hull;
      convex_hull.reserve(t_points.size());

      if (t_points.size() < 3) {
        return convex_hull;
      }

      // TODO: Checking all points of the set are different and that it contains at least three non-collinear points.

      // Find the leftmost point in the point set given to us.
      const auto leftmost {std::min_element(t_points.begin(), t_points.end(), [] (auto& lhs, auto& rhs) {
        return utility::less<const T&>(lhs, rhs);
      })};

      auto current_point {leftmost}, second_point {leftmost};

      // Do following while we don’t come back to the first (or leftmost) point.
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
      } while (current_point != leftmost);

      convex_hull.shrink_to_fit();
      return convex_hull;
    }

  template <typename U, typename T>
    std::vector<T> convexHull (const std::vector<T>& t_points, typename std::enable_if_t<is_algorithm<U, AlgorithmHull<Pattern::QuickHull>>::value>* = 0)
    {
      std::vector<T> convex_hull;
      convex_hull.reserve(t_points.size());

      if (t_points.size() < 3) {
        return convex_hull;
      }

      // TODO: Checking all points of the set are different and that it contains at least three non-collinear points.

      // Find the point with minimum x-coordinate (leftmost), and similarly the point with maximum x-coordinate (rightmost).
      const auto& [leftmost, rightmost] {std::minmax_element(t_points.begin(), t_points.end(), [] (auto& lhs, auto& rhs) {
        return utility::less<const T&>(lhs, rhs);
      })};

      if (leftmost == t_points.end() || rightmost == t_points.end()) {
        return convex_hull;
      }

      // Make a line joining these two points. This line will divide the whole set into two parts.
      extension::quickHull(t_points, leftmost, rightmost, std::back_inserter(convex_hull)); // For left side
      extension::quickHull(t_points, rightmost, leftmost, std::back_inserter(convex_hull)); // For right side

      convex_hull.shrink_to_fit();
      return convex_hull;
    }

  template <typename U, typename T>
    std::vector<T> convexHull (const std::vector<T>& t_points, typename std::enable_if_t<is_algorithm<U, AlgorithmHull<Pattern::GrahamScan>>::value>* = 0)
    {
      std::vector<T> convex_hull;

      if (t_points.size() < 3) {
        return convex_hull;
      }

      // TODO: Checking all points of the set are different and that it contains at least three non-collinear points.
      // TODO: I don't like how the algorithm works. I don't understand him very well. To be studied in detail to optimize this code.

      std::vector<T> points_copy {t_points.begin(), t_points.end()};

      // Find the leftmost point in the point set given to us.
      auto leftmost {std::min_element(points_copy.begin(), points_copy.end(), [] (auto& lhs, auto& rhs) {
        return utility::less<T&>(lhs, rhs);
      })};

      if (leftmost == points_copy.end()) {
        return convex_hull;
      }

      std::iter_swap(points_copy.begin(), leftmost);

      // Sort their points in the polar angle in the counterclockwise direction.
      const auto& first_point {points_copy.begin()};
      std::sort(std::next(points_copy.begin()), points_copy.end(), [&] (auto& lhs, auto& rhs) {
        auto orientetion {utility::orientetion(*first_point, lhs, rhs)};

        // If the polar angle of the two points is the same, then first put the nearest point.
        if (orientetion == utility::Orientation::COLINEAR) {
          if (utility::distance(*first_point, rhs) > utility::distance(*first_point, lhs)) {
            return true;
          }
        }

        return orientetion == utility::Orientation::COUNTERCLOCKWISE;
      });

      auto second_point {std::next(first_point)};
      for (auto current_point_it = std::next(points_copy.begin()); current_point_it != points_copy.end(); ++current_point_it) {
        while (std::next(current_point_it) != points_copy.end() && utility::orientetion(*first_point, *current_point_it, *std::next(current_point_it)) == utility::Orientation::COLINEAR) {
          ++current_point_it;
        }

        *second_point = *current_point_it;
        ++second_point;
      }

      if (std::distance(points_copy.begin(), second_point) < 3) {
        return convex_hull;
      }

      convex_hull.reserve(points_copy.size());
      convex_hull.insert(convex_hull.end(), points_copy.begin(), std::next(points_copy.begin(), 3));

      if (points_copy.size() < 4) {
        return convex_hull;
      }

      for (auto current_point_it {std::next(points_copy.begin(), 4)}; current_point_it != second_point; ++current_point_it) {
        while (utility::orientetion(*std::prev(convex_hull.end(), 2), convex_hull.back(), *current_point_it) != utility::Orientation::COUNTERCLOCKWISE) {
          convex_hull.pop_back();
        }

        convex_hull.push_back(*current_point_it);
      }

      convex_hull.shrink_to_fit();
      return convex_hull;
    }

  template <typename T>
    std::vector<T> convexHull (const std::vector<T>& t_points)
    {
      return convexHull<AlgorithmHull<Pattern::GrahamScan>>(t_points);
    }
} // namespace concave

#endif // CONCAVE_CONVEXHULL_HPP_