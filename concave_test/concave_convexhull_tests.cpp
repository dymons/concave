#include "concave_core/primitives/Point.hpp"
#include "concave_core/ConvexHull.hpp"

// STL
#include <algorithm>
#include <iostream>
#include <vector>
#include <chrono>

// CGAL
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>

// OpenCV
#include <opencv2/core/types.hpp>

// Boost
#include <boost/filesystem.hpp>

// Google test
#include <gtest/gtest.h>

using namespace std::chrono;

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

  {
    auto jarvis_march        {concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(custom_points)};
    auto quick_hull          {concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(custom_points)};
    auto graham_scan         {concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>(custom_points)};
    auto divide_and_conquer  {concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>(custom_points)};

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty() && !divide_and_conquer.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()) && (jarvis_march.size() == divide_and_conquer.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), divide_and_conquer.begin()));
  }

  {
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
  }

  // Save to OpenCV data point.
  std::vector<point_opencv> opencv_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), opencv_points.begin(), [] (auto& point) {
    return point_opencv(point.x, point.y);
  });

  {
    auto jarvis_march {concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(opencv_points)};
    auto quick_hull   {concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(opencv_points)};
    auto graham_scan  {concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>(opencv_points)};

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
  }

  // Save to CGAL data point.
  std::vector<point_cgal> cgal_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), cgal_points.begin(), [] (auto& point) {
    return point_cgal(point.x, point.y);
  });

  {
    auto jarvis_march {concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(cgal_points)};
    auto quick_hull   {concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(cgal_points)};
    auto graham_scan  {concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>(cgal_points)};

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
  }
}

TEST_F(ConvexHullTests, jarvis_march)
{
  auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(std::vector<concave::primitives::Point<double>>{});
  EXPECT_TRUE(convex_hull.empty());

  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  // Get data from file.
  std::ifstream is {pointset, std::ios::in};
  EXPECT_TRUE(!is.fail());

  std::vector<concave::primitives::Point<double>> custom_points;
  std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(custom_points));
  is.close();

  // Save to OpenCV data point.
  std::vector<point_opencv> opencv_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), opencv_points.begin(), [] (auto& point) {
    return point_opencv(point.x, point.y);
  });

  // Save to CGAL data point.
  std::vector<point_cgal> cgal_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), cgal_points.begin(), [] (auto& point) {
    return point_cgal(point.x, point.y);
  });

  std::vector<concave::primitives::Point<double>> convex_hull_custom;
  std::vector<point_opencv> convex_hull_opencv;
  std::vector<point_cgal> convex_hull_cgal;

  const std::size_t size_points {custom_points.size()};
  for (std::size_t i {3}; i < size_points; ++i) {
    convex_hull_custom = concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>
    (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)});
    convex_hull_opencv = concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>
    (std::vector<point_opencv>{opencv_points.begin(), std::next(opencv_points.begin(),i)});
    convex_hull_cgal = concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>
    (std::vector<point_cgal>{cgal_points.begin(), std::next(cgal_points.begin(),i)});

    EXPECT_TRUE(!convex_hull_custom.empty() && !convex_hull_opencv.empty() && !convex_hull_cgal.empty());
    EXPECT_TRUE((convex_hull_custom.size() == convex_hull_opencv.size()) && (convex_hull_custom.size() == convex_hull_cgal.size()));
    // TODO: Add data equivalence checks from all algorithms.
  }
}

TEST_F(ConvexHullTests, quick_hull)
{
  auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(std::vector<concave::primitives::Point<double>>{});
  EXPECT_TRUE(convex_hull.empty());

  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  // Get data from file.
  std::ifstream is {pointset, std::ios::in};
  EXPECT_TRUE(!is.fail());

  std::vector<concave::primitives::Point<double>> custom_points;
  std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(custom_points));
  is.close();

  // Save to OpenCV data point.
  std::vector<point_opencv> opencv_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), opencv_points.begin(), [] (auto& point) {
    return point_opencv(point.x, point.y);
  });

  // Save to CGAL data point.
  std::vector<point_cgal> cgal_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), cgal_points.begin(), [] (auto& point) {
    return point_cgal(point.x, point.y);
  });

  std::vector<concave::primitives::Point<double>> convex_hull_custom;
  std::vector<point_opencv> convex_hull_opencv;
  std::vector<point_cgal> convex_hull_cgal;

  const std::size_t size_points {custom_points.size()};
  for (std::size_t i {3}; i < size_points; ++i) {
    convex_hull_custom = concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>
    (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)});
    convex_hull_opencv = concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>
    (std::vector<point_opencv>{opencv_points.begin(), std::next(opencv_points.begin(),i)});
    convex_hull_cgal = concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>
    (std::vector<point_cgal>{cgal_points.begin(), std::next(cgal_points.begin(),i)});

    EXPECT_TRUE(!convex_hull_custom.empty() && !convex_hull_opencv.empty() && !convex_hull_cgal.empty());
    EXPECT_TRUE((convex_hull_custom.size() == convex_hull_opencv.size()) && (convex_hull_custom.size() == convex_hull_cgal.size()));
  }
}

TEST_F(ConvexHullTests, graham_scan)
{
  auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>(std::vector<concave::primitives::Point<double>>{});
  EXPECT_TRUE(convex_hull.empty());

  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  // Get data from file.
  std::ifstream is {pointset, std::ios::in};
  EXPECT_TRUE(!is.fail());

  std::vector<concave::primitives::Point<double>> custom_points;
  std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(custom_points));
  is.close();

  // Save to OpenCV data point.
  std::vector<point_opencv> opencv_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), opencv_points.begin(), [] (auto& point) {
    return point_opencv(point.x, point.y);
  });

  // Save to CGAL data point.
  std::vector<point_cgal> cgal_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), cgal_points.begin(), [] (auto& point) {
    return point_cgal(point.x, point.y);
  });

  std::vector<concave::primitives::Point<double>> convex_hull_custom;
  std::vector<point_opencv> convex_hull_opencv;
  std::vector<point_cgal> convex_hull_cgal;

  const std::size_t size_points {custom_points.size()};
  for (std::size_t i {3}; i < size_points; ++i) {
    convex_hull_custom = concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>
    (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)});
    convex_hull_opencv = concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>
    (std::vector<point_opencv>{opencv_points.begin(), std::next(opencv_points.begin(),i)});
    convex_hull_cgal = concave::convexHull<concave::AlgorithmHull<concave::Pattern::GrahamScan>>
    (std::vector<point_cgal>{cgal_points.begin(), std::next(cgal_points.begin(),i)});

    EXPECT_TRUE(!convex_hull_custom.empty() && !convex_hull_opencv.empty() && !convex_hull_cgal.empty());
    EXPECT_TRUE((convex_hull_custom.size() == convex_hull_opencv.size()) && (convex_hull_custom.size() == convex_hull_cgal.size()));
  }
}

TEST_F(ConvexHullTests, divide_and_conquer)
{
  auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>(std::vector<concave::primitives::Point<double>>{});
  EXPECT_TRUE(convex_hull.empty());

  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  // Get data from file.
  std::ifstream is {pointset, std::ios::in};
  EXPECT_TRUE(!is.fail());

  std::vector<concave::primitives::Point<double>> custom_points;
  std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(custom_points));
  is.close();

  // Save to OpenCV data point.
  std::vector<point_opencv> opencv_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), opencv_points.begin(), [] (auto& point) {
    return point_opencv(point.x, point.y);
  });

  // Save to CGAL data point.
  std::vector<point_cgal> cgal_points (custom_points.size());
  transform_copy(custom_points.begin(), custom_points.end(), cgal_points.begin(), [] (auto& point) {
    return point_cgal(point.x, point.y);
  });

  std::vector<concave::primitives::Point<double>> convex_hull_custom;
  std::vector<point_opencv> convex_hull_opencv;
  std::vector<point_cgal> convex_hull_cgal;

  const std::size_t size_points {custom_points.size()};
  for (std::size_t i {3}; i < size_points; ++i) {
    std::cout << i << std::endl;
    convex_hull_custom = concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>
            (std::vector<concave::primitives::Point<double>>{custom_points.begin(), std::next(custom_points.begin(),i)});
    convex_hull_opencv = concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>
            (std::vector<point_opencv>{opencv_points.begin(), std::next(opencv_points.begin(),i)});
    convex_hull_cgal = concave::convexHull<concave::AlgorithmHull<concave::Pattern::DivideAndConquer>>
            (std::vector<point_cgal>{cgal_points.begin(), std::next(cgal_points.begin(),i)});

    EXPECT_TRUE(!convex_hull_custom.empty() && !convex_hull_opencv.empty() && !convex_hull_cgal.empty());
    EXPECT_TRUE((convex_hull_custom.size() == convex_hull_opencv.size()) && (convex_hull_custom.size() == convex_hull_cgal.size()));
  }
}