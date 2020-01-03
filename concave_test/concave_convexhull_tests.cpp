#include "concave_core/primitives/Point.hpp"
#include "concave_core/ConvexHull.hpp"

// STL
#include <algorithm>
#include <iostream>
#include <vector>

// CGAL
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>

// OpenCV
#include <opencv2/core/types.hpp>

// Boost
#include <boost/filesystem.hpp>

// Google test
#include <gtest/gtest.h>

using point_cgal   = CGAL::Point_2<CGAL::Cartesian<double>>;
using point_opencv = cv::Point_<double>;

template <typename T, typename R, typename P>
  R transform_copy (T t_first_in, T t_last_in, R t_first_out, P t_pred)
  {
    while (t_first_in != t_last_in) {
      *t_first_out = t_pred(*t_first_in);
      ++t_first_out;
      ++t_first_in;
    }

    return t_first_out;
  }

namespace CGAL {
  std::istream& operator>>(std::istream& t_istream, point_cgal& t_point)
  {
    double x { 0.0 }, y { 0.0 };
    t_istream >> x >> y;
    t_point = point_cgal(x, y);
    return t_istream;
  }
}

namespace cv {
  std::istream& operator>>(std::istream& t_istream, point_opencv& t_point)
  {
    t_istream >> t_point.x >> t_point.y;
    return t_istream;
  }
}

class ConvexHullTests : public ::testing::Test { };

// Verifying that all algorithms produce the same result.
TEST_F(ConvexHullTests, algorithm_equivalence_test)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  // Get data from file.
  std::ifstream is {pointset, std::ios::in};
  EXPECT_TRUE(!is.fail());

  std::vector<concave::primitives::Point<double>> custom_points;
  std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(custom_points));
  is.close();

  for (std::size_t i {3}; i < custom_points.size(); ++i) {
    auto jarvis_march {concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>
    (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)})};
    auto quick_hull {concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>
    (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)})};
    auto graham_scan {concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>
    (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)})};
    auto divide_and_conquer {concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>
    (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)})};

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty() && !divide_and_conquer.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()) && (jarvis_march.size() == divide_and_conquer.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), divide_and_conquer.begin()));
  }

  // Save to OpenCV data point.
  std::vector<point_opencv> opencv_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), opencv_points.begin(), [] (auto& point) {
    return point_opencv(point.x, point.y);
  });

  {
    auto jarvis_march       {concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(opencv_points)};
    auto quick_hull         {concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(opencv_points)};
    auto graham_scan        {concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>(opencv_points)};
    auto divide_and_conquer {concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>(opencv_points)};

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty() && !divide_and_conquer.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()) && (jarvis_march.size() == divide_and_conquer.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), divide_and_conquer.begin()));
  }

  // Save to CGAL data point.
  std::vector<point_cgal> cgal_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), cgal_points.begin(), [] (auto& point) {
    return point_cgal(point.x, point.y);
  });

  {
    auto jarvis_march       {concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(cgal_points)};
    auto quick_hull         {concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(cgal_points)};
    auto graham_scan        {concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>(cgal_points)};
    auto divide_and_conquer {concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>(cgal_points)};

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty() && !divide_and_conquer.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()) && (jarvis_march.size() == divide_and_conquer.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), divide_and_conquer.begin()));
  }
}