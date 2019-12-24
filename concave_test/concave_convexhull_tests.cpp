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

class ConvexHullTests : public ::testing::Test { };

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

TEST_F(ConvexHullTests, convex_hull_graham_scan_custom_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  {
    std::vector<concave::primitives::Point<double>> points;
    std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(points));
    auto convex_hull = concave::convexHull(points);

    EXPECT_TRUE(!convex_hull.empty());
    EXPECT_TRUE(convex_hull.size() == 4);

    is.close();
  }

  // is empty
  {
    std::vector<concave::primitives::Point<double>> points;
    auto convex_hull = concave::convexHull(points);
    EXPECT_TRUE(convex_hull.empty());
  }

  // is 1, 2, 3 points
  {
    std::vector<concave::primitives::Point<double>> points;
    points.emplace_back(0,0);
    points.emplace_back(1,2);
    points.emplace_back(1,0);

    EXPECT_TRUE(concave::convexHull(std::vector<concave::primitives::Point<double>>{points.begin(), std::next(points.begin(), 1)}).empty());
    EXPECT_TRUE(concave::convexHull(std::vector<concave::primitives::Point<double>>{points.begin(), std::next(points.begin(), 2)}).empty());
    EXPECT_TRUE(!concave::convexHull(std::vector<concave::primitives::Point<double>>{points.begin(), std::next(points.begin(), 3)}).empty());
  }
}

TEST_F(ConvexHullTests, convex_hull_graham_scan_cgal_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  std::vector<point_cgal> points;
  std::copy(std::istream_iterator<point_cgal>{is}, {}, std::back_inserter(points));
  auto convex_hull = concave::convexHull(points);

  EXPECT_TRUE(!convex_hull.empty());
  EXPECT_TRUE(convex_hull.size() == 4);

  is.close();
}

TEST_F(ConvexHullTests, convex_hull_graham_scan_opencv_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  std::vector<point_opencv> points;
  std::copy(std::istream_iterator<point_opencv>{is}, {}, std::back_inserter(points));
  auto convex_hull = concave::convexHull(points);

  EXPECT_TRUE(!convex_hull.empty());
  EXPECT_TRUE(convex_hull.size() == 4);

  is.close();
}

TEST_F(ConvexHullTests, convex_hull_quick_hull_custom_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  {
    std::vector<concave::primitives::Point<double>> points;
    std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(points));
    auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(points);

    EXPECT_TRUE(!convex_hull.empty());
    EXPECT_TRUE(convex_hull.size() == 4);

    is.close();
  }
}

TEST_F(ConvexHullTests, convex_hull_quick_hull_cgal_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  {
    std::vector<point_cgal> points;
    std::copy(std::istream_iterator<point_cgal>{is}, {}, std::back_inserter(points));
    auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(points);

    EXPECT_TRUE(!convex_hull.empty());
    EXPECT_TRUE(convex_hull.size() == 4);

    is.close();
  }
}

TEST_F(ConvexHullTests, convex_hull_quick_hull_opencv_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  {
    std::vector<point_opencv> points;
    std::copy(std::istream_iterator<point_opencv>{is}, {}, std::back_inserter(points));
    auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::QuickHull>>(points);

    EXPECT_TRUE(!convex_hull.empty());
    EXPECT_TRUE(convex_hull.size() == 4);

    is.close();
  }
}

TEST_F(ConvexHullTests, convex_hull_jarvis_march_custom_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  {
    std::vector<concave::primitives::Point<double>> points;
    std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(points));
    auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(points);

    EXPECT_TRUE(!convex_hull.empty());
    EXPECT_TRUE(convex_hull.size() == 4);

    is.close();
  }
}

TEST_F(ConvexHullTests, convex_hull_jarvis_march_cgal_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  {
    std::vector<point_cgal> points;
    std::copy(std::istream_iterator<point_cgal>{is}, {}, std::back_inserter(points));
    auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(points);

    EXPECT_TRUE(!convex_hull.empty());
    EXPECT_TRUE(convex_hull.size() == 4);

    is.close();
  }
}

TEST_F(ConvexHullTests, convex_hull_jarvis_march_opencv_point)
{
  std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
  EXPECT_TRUE(boost::filesystem::exists(boost::filesystem::path(pointset)));

  std::ifstream is (pointset, std::ios::in);
  EXPECT_TRUE(!is.fail());

  {
    std::vector<point_opencv> points;
    std::copy(std::istream_iterator<point_opencv>{is}, {}, std::back_inserter(points));
    auto convex_hull = concave::convexHull<concave::AlgorithmHull<concave::Pattern::JarvisMarch>>(points);

    EXPECT_TRUE(!convex_hull.empty());
    EXPECT_TRUE(convex_hull.size() == 4);

    is.close();
  }
}