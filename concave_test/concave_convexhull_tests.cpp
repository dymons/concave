#include "concave_core/algorithm/convex_hull.hpp"
#include "concave_core/primitives/point.hpp"

// STL
#include <algorithm>
#include <iostream>
#include <vector>
#include <filesystem>

// CGAL
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>

// OpenCV
#include <opencv2/core/types.hpp>

// Google test
#include <gtest/gtest.h>

using OwnPoint = concave::primitives::Point<double>;
using PointCgal   = CGAL::Point_2<CGAL::Cartesian<double>>;
using PointOpencv = cv::Point_<double>;

namespace concave::detail {

template <>
struct nth<0, OwnPoint> {
    inline static auto get(const OwnPoint &t_own_point) {
        return t_own_point.x;
    };
};
template <>
struct nth<1, OwnPoint> {
    inline static auto get(const OwnPoint &t_own_point) {
        return t_own_point.y;
    };
};

} // namespace concave::detail

namespace CGAL {
std::istream& operator>>(std::istream& t_istream, PointCgal& t_point)
{
  double x { 0.0 };
  double y { 0.0 };
  t_istream >> x >> y;
  t_point = PointCgal(x, y);
  return t_istream;
}
} // namespace CGAL

namespace cv {
std::istream& operator>>(std::istream& t_istream, PointOpencv& t_point)
{
  t_istream >> t_point.x >> t_point.y;
  return t_istream;
}
} // namespace cv

class ConvexHullTests : public ::testing::Test {
  protected:
    void SetUp() final
    {
      std::string pointset { std::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt" };
      std::ifstream is { pointset, std::ios::in };
      std::copy(std::istream_iterator<concave::primitives::Point<double>> { is }, {}, std::back_inserter(m_points));
      is.close();
    }

    void TearDown() final
    {
    }

    std::vector<OwnPoint> m_points;
};

TEST_F(ConvexHullTests, testJarvisMarch)
{
  ASSERT_FALSE(m_points.empty());

  /// Check empty data
  std::vector<OwnPoint> buffer;
  auto jarvis_march_empty { concave::convexHull<concave::Pattern::JarvisMarch>(buffer) };
  EXPECT_TRUE(jarvis_march_empty.empty());

  /// Check little data
  buffer.insert(buffer.end(), m_points.begin(), std::next(m_points.begin(), 2));
  auto jarvis_march_bit { concave::convexHull<concave::Pattern::JarvisMarch>(buffer) };
  EXPECT_TRUE(jarvis_march_bit.empty());
}

// Verifying that all algorithms produce the same result.
TEST_F(ConvexHullTests, algorithm_equivalence_test)
{
  ASSERT_FALSE(m_points.empty());

  for (std::size_t i { 3 }; i < m_points.size() - 1; ++i) {
    std::vector<OwnPoint> buffer { m_points.begin(), std::next(m_points.begin(), i) };
    auto jarvis_march { concave::convexHull<concave::Pattern::JarvisMarch>(buffer) };
    auto quick_hull { concave::convexHull<concave::Pattern::QuickHull>(buffer) };
    auto graham_scan { concave::convexHull<concave::Pattern::GrahamScan>(buffer) };

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
  }
}

int main(int t_argc, char** t_argv)
{
  ::testing::InitGoogleTest(&t_argc, t_argv);

  return RUN_ALL_TESTS();
}