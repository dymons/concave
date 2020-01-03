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
      auto point_far {t_points.end()};
      double distance_upper {0.0}, current_distance {0.0}; // TODO: Replace with automatic type detection use type_traits.
      for (auto it {t_points.begin()}; it != t_points.end(); ++it) {
        if (utility::side(*t_leftmost, *t_rightmost, *it) == utility::Side::LeftSide) {
          current_distance = std::abs(utility::distance(*t_leftmost, *t_rightmost, *it));
          if (current_distance > distance_upper) {
            distance_upper = current_distance;
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
      quickHull(t_points, t_leftmost, point_far, t_convex_hull);
      quickHull(t_points, point_far, t_rightmost, t_convex_hull);
    }

  template <typename T>
    std::vector<T> mergeDivideAndConquer (const std::vector<T>&& t_lefthull, const std::vector<T>&& t_righthull)
    {
      std::vector<T> merge_convex_hull;
      merge_convex_hull.reserve(t_lefthull.size() + t_righthull.size());

      if (t_lefthull.empty() || t_righthull.empty()) {
        return merge_convex_hull; // TODO: It isn't correct.
      }

      // Search for extreme points.
      const auto rightmost {std::max_element(t_lefthull.begin(),  t_lefthull.end(),  utility::less_then_x<T>{})}; // For left ConvexHull
      const auto leftmost  {std::min_element(t_righthull.begin(), t_righthull.end(), utility::less_then_x<T>{})}; // For right ConvexHull

      // Find upper and lower tangent.
      const auto [upper_rightmost, upper_leftmost] = utility::tangent(rightmost, t_lefthull, leftmost, t_righthull);
      const auto [lower_leftmost, lower_rightmost] = utility::tangent(leftmost, t_righthull, rightmost, t_lefthull);

      // Add points from left hull, from 'upper_rightmost' to 'lower_rightmost'.
      if (std::distance(t_lefthull.begin(), upper_rightmost) > std::distance(t_lefthull.begin(), lower_rightmost)) {
        merge_convex_hull.insert(merge_convex_hull.end(), std::make_move_iterator(upper_rightmost), std::make_move_iterator(t_lefthull.end()));
        merge_convex_hull.insert(merge_convex_hull.end(), std::make_move_iterator(t_lefthull.begin()), std::make_move_iterator(std::next(lower_rightmost)));
      } else {
        merge_convex_hull.insert(merge_convex_hull.end(), std::make_move_iterator(upper_rightmost), std::make_move_iterator(std::next(lower_rightmost)));
      }

      // Add pints from right hull, from 'lower_leftmost' to 'upper_leftmost'.
      if (std::distance(t_righthull.begin(), lower_leftmost) > std::distance(t_righthull.begin(), upper_leftmost)) {
        merge_convex_hull.insert(merge_convex_hull.end(), std::make_move_iterator(lower_leftmost), std::make_move_iterator(t_righthull.end()));
        merge_convex_hull.insert(merge_convex_hull.end(), std::make_move_iterator(t_righthull.begin()), std::make_move_iterator(std::next(upper_leftmost)));
      } else {
        merge_convex_hull.insert(merge_convex_hull.end(), std::make_move_iterator(lower_leftmost), std::make_move_iterator(std::next(upper_leftmost)));
      }

      merge_convex_hull.shrink_to_fit();
      return merge_convex_hull;
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

      // Find the leftmost point in the point set given to us.
      const auto leftmost {std::min_element(t_points.begin(), t_points.end(), utility::less_then_y<T>{})};

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
    std::vector<T> convexHull (const std::vector<T>& t_points, typename std::enable_if_t<is_algorithm<U, AlgorithmHull<Pattern::DivideAndConquer>>::value>* = 0)
    {
      std::vector<T> convex_hull;

      if (t_points.size() < 6) {
        convex_hull = convexHull<AlgorithmHull<Pattern::JarvisMarch>>(t_points); // TODO: Implement Brute force algorithm to find convex hull.
      } else {
        std::vector<T> points_copy (t_points);
        std::sort(points_copy.begin(), points_copy.end(), utility::less_then_x<T>{});

        const auto middle_point  {std::next(points_copy.begin(), static_cast<std::size_t>((std::distance(points_copy.begin(), points_copy.end()) / 2)))};
        std::vector<T> lefthull  {convexHull<AlgorithmHull<Pattern::DivideAndConquer>>
        (std::vector<T>{std::make_move_iterator(points_copy.begin()), std::make_move_iterator(middle_point)})}; // ?
        std::vector<T> righthull {convexHull<AlgorithmHull<Pattern::DivideAndConquer>>
        (std::vector<T>{std::make_move_iterator(middle_point), std::make_move_iterator(points_copy.end())})};
        convex_hull = extension::mergeDivideAndConquer(std::move(lefthull), std::move(righthull));
      }

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

      // Find the point with minimum x-coordinate (leftmost), and similarly the point with maximum x-coordinate (rightmost).
      const auto [leftmost, rightmost] {std::minmax_element(t_points.begin(), t_points.end(), utility::less_then_y<T>{})};

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

      std::vector<T> points_copy (t_points);

      // Find the leftmost point in the point set given to us and swap with begin point.
      auto leftmost {std::min_element(points_copy.begin(), points_copy.end(), utility::less_then_y<T>{})};
      std::iter_swap(points_copy.begin(), leftmost);

      // Sort their points in the polar angle in the counterclockwise direction.
      std::sort(std::next(points_copy.begin()), points_copy.end(), [&] (auto& lhs, auto& rhs) {
        return (utility::orientetion(points_copy.front(), lhs, rhs) == utility::Orientation::COUNTERCLOCKWISE);
      });

      convex_hull.reserve(points_copy.size());
      convex_hull.insert(convex_hull.end(), points_copy.begin(), std::next(points_copy.begin(), 3));

      for (auto current_point_it {std::next(points_copy.begin(), 3)}; current_point_it != points_copy.end(); ++current_point_it) {
        while (utility::orientetion(*std::next(convex_hull.rbegin()), convex_hull.back(), *current_point_it) != utility::Orientation::COUNTERCLOCKWISE) {
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