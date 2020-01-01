#pragma once

#ifndef CONCAVE_CONVEXHULL_HPP_
#define CONCAVE_CONVEXHULL_HPP_

#include "concave_core/utility/Utility.hpp"

#include <iostream>
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
    std::vector<T> mergeDivideAndConquer (const std::vector<T>& t_lefthull, const std::vector<T>& t_righthull)
    {
      std::vector<T> merge_convex_hull;

      if (t_lefthull.empty() || t_righthull.empty()) {
        return merge_convex_hull; // TODO: It isn't correct.
      }

      // Search for extreme points.
      const auto rightmost {std::max_element(t_lefthull.begin(),  t_lefthull.end(),  utility::less_then_x<T>{})}; // For left ConvexHull
      const auto leftmost  {std::min_element(t_righthull.begin(), t_righthull.end(), utility::less_then_x<T>{})}; // For right ConvexHull

      // For debug.
      auto print      ([]  (auto& s, auto& p)    {std::cout << s << " x: " << p->x << ", y: " << p->y << std::endl;});
      auto print_hull ([=] (auto& s, auto& hull) {for (auto p = hull.begin(); p != hull.end(); ++p) {print(s, p);}});

      print_hull("t_lefthull", t_lefthull);
      print_hull("t_righthull", t_righthull);

      print("rightmost", rightmost);
      print("leftmost", leftmost);

      auto upper_rightmost {rightmost}, upper_leftmost {leftmost};

      std::cout << "start" << std::endl;
      {
        bool is_find_tangents { true };
        while (is_find_tangents) {
          is_find_tangents = false;

          if (upper_rightmost == t_lefthull.end()) {
            upper_rightmost = t_lefthull.begin();
          }

          while (utility::orientetion(*upper_leftmost, *upper_rightmost, *std::next(t_lefthull.begin(), (std::distance(t_lefthull.begin(), upper_rightmost) + 1) % t_lefthull.size()))
              != utility::Orientation::COUNTERCLOCKWISE) {
            upper_rightmost = std::next(t_lefthull.begin(), (std::distance(t_lefthull.begin(), upper_rightmost) + 1) % t_lefthull.size());
          }

          if (upper_leftmost == t_righthull.end()) {
            upper_leftmost = t_righthull.begin();
          }

          // TODO: t_righthull.size() +
          while (utility::orientetion(*upper_rightmost, *upper_leftmost, *std::next(t_righthull.begin(), (t_righthull.size() + std::distance(t_righthull.begin(), upper_leftmost) - 1) % t_righthull.size()))
              != utility::Orientation::CLOCKWISE) {
            upper_leftmost = std::next(t_righthull.begin(), (t_righthull.size() + std::distance(t_righthull.begin(), upper_leftmost) - 1) % t_righthull.size());
            is_find_tangents = true;
          }
        }
      }
      print("upper_rightmost", upper_rightmost);
      print("upper_leftmost", upper_leftmost);
      std::cout << "finish" << std::endl;

      auto lower_rightmost {rightmost}, lower_leftmost {leftmost};

      {
        bool is_find_tangents { true };
        while (is_find_tangents) {
          is_find_tangents = false;

          if (lower_leftmost == t_righthull.end()) {
            lower_leftmost = t_righthull.begin();
          }

          //std::cout << "type: " << int(utility::orientetion(*lower_rightmost, *lower_leftmost, *std::next(t_righthull.begin(), (std::distance(t_righthull.begin(), lower_leftmost) - 1) % t_righthull.size()))) << std::endl;

          while (utility::orientetion(*lower_rightmost, *lower_leftmost, *std::next(t_righthull.begin(), (std::distance(t_righthull.begin(), lower_leftmost) + 1) % t_righthull.size()))
//                 != utility::Orientation::COUNTERCLOCKWISE) {
                 == utility::Orientation::CLOCKWISE) {
            std::cout << "current: " << int(utility::orientetion(*lower_rightmost, *lower_leftmost, *std::next(t_righthull.begin(), (std::distance(t_righthull.begin(), lower_leftmost) + 1) % t_righthull.size())));
            std::cout << " need  " << int(utility::Orientation::COUNTERCLOCKWISE) << std::endl;
            std::cout << "distance: " << std::distance(t_righthull.begin(), lower_leftmost) << " | size = " << t_righthull.size() << std::endl;
            lower_leftmost = std::next(t_righthull.begin(), (std::distance(t_righthull.begin(), lower_leftmost) + 1) % t_righthull.size());
            print("lower_leftmost", lower_leftmost);

          }

//          print("lower_leftmost", lower_leftmost);

          if (lower_rightmost == t_lefthull.end()) {
            lower_rightmost = t_lefthull.begin();
          }

          while (utility::orientetion(*lower_leftmost, *lower_rightmost, *std::next(t_lefthull.begin(), (t_lefthull.size() + std::distance(t_lefthull.begin(), lower_rightmost) - 1) % t_lefthull.size()))
              != utility::Orientation::CLOCKWISE) {
            lower_rightmost = std::next(t_lefthull.begin(), (t_lefthull.size() + std::distance(t_lefthull.begin(), lower_rightmost) - 1) % t_lefthull.size());
            is_find_tangents = true;
            print("lower_rightmost", lower_rightmost);
          }
        }
      }

      print("lower_rightmost", lower_rightmost);
      print("lower_leftmost", lower_leftmost);

      std::cout << "start merge left hull" << std::endl;

//      std::cout << int(lower_rightmost == lower_rightmost) << std::endl;

//      do {
//        if (upper_rightmost == t_lefthull.end()) {
//          upper_rightmost = t_lefthull.begin();
//        }
//
//        std::cout << int(upper_rightmost == lower_rightmost) << std::endl;
//
//        print("upper_rightmost", upper_rightmost);
//
//        merge_convex_hull.push_back(*upper_rightmost);
//        ++upper_rightmost;
//      } while (upper_rightmost != lower_rightmost);

      while (true) {
        if (upper_rightmost == t_lefthull.end()) {
          upper_rightmost = t_lefthull.begin();
        }

        merge_convex_hull.push_back(*upper_rightmost);

        if (upper_rightmost == lower_rightmost) {
          break;
        }

        ++upper_rightmost;
      }

      std::cout << "finish merge left hull" << std::endl;
      std::cout << "start merge right hull" << std::endl;

      while (true) {
        if (lower_leftmost == t_righthull.end()) {
          lower_leftmost = t_righthull.begin();
        }

        merge_convex_hull.push_back(*lower_leftmost);

        if (lower_leftmost == upper_leftmost) {
          break;
        }

        ++lower_leftmost;
      }
//
//      do {
//        if (lower_leftmost == t_righthull.end()) {
//          lower_leftmost = t_righthull.begin();
//        }
//
//        print("lower_leftmost", lower_leftmost);
//
//        merge_convex_hull.push_back(*lower_leftmost);
//        ++lower_leftmost;
//      } while (lower_leftmost != upper_leftmost);

      std::cout << "finish merge right hull" << std::endl;

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
      const auto leftmost {std::min_element(t_points.begin(), t_points.end(), utility::less<T>{})};

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
      if (t_points.size() < 7) {
        return convexHull<AlgorithmHull<Pattern::JarvisMarch>>(t_points); // TODO: Implement Brute force algorithm to find convex hull. NRVO
      }

      std::cout << "Input size: " << t_points.size() << std::endl;

      std::vector<T> points_copy (t_points);
      std::sort(points_copy.begin(), points_copy.end(), utility::less_then_x<T>{});

      std::cout << "points in t_points: \n";
      for (auto& p : points_copy) {
        std::cout << "x: " << p.x << ", y: " << p.y << std::endl;
      }

      const auto middle_point {std::next(points_copy.begin(), static_cast<std::size_t>((std::distance(points_copy.begin(), points_copy.end()) / 2)))};
      std::vector<T> lefthull  {convexHull<AlgorithmHull<Pattern::DivideAndConquer>>(std::vector<T>{points_copy.begin(), middle_point})};
      std::vector<T> righthull {convexHull<AlgorithmHull<Pattern::DivideAndConquer>>(std::vector<T>{middle_point, points_copy.end()})};
      return extension::mergeDivideAndConquer(lefthull, righthull);
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
      const auto [leftmost, rightmost] {std::minmax_element(t_points.begin(), t_points.end(), utility::less<T>{})};

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
      auto leftmost {std::min_element(points_copy.begin(), points_copy.end(), utility::less<T>{})};
      std::iter_swap(points_copy.begin(), leftmost);

      // Sort their points in the polar angle in the counterclockwise direction.
      std::sort(std::next(points_copy.begin()), points_copy.end(), [&] (auto& lhs, auto& rhs) {
        return (utility::orientetion(points_copy.front(), lhs, rhs) == utility::Orientation::COUNTERCLOCKWISE);
      });

      convex_hull.reserve(points_copy.size());
      convex_hull.insert(convex_hull.end(), points_copy.begin(), std::next(points_copy.begin(), 3));

      if (points_copy.size() < 4) {
        return convex_hull;
      }

      for (auto current_point_it {std::next(points_copy.begin(), 4)}; current_point_it != points_copy.end(); ++current_point_it) {
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