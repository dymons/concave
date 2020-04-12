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

class ConvexHullTests : public ::testing::Test {
  protected:
    void SetUp() final
    {
        std::string pointset { boost::filesystem::current_path().parent_path().string() + "/../concave_test/dataset/pointset_0.txt"};
        std::ifstream is {pointset, std::ios::in};
        std::copy(std::istream_iterator<concave::primitives::Point<double>>{is}, {}, std::back_inserter(m_points));
        is.close();
    }

    void TearDown() final
    {
    }

    std::vector<concave::primitives::Point<double>> m_points;
};

TEST_F(ConvexHullTests, testJarvisMarch)
{
  ASSERT_FALSE(m_points.empty());

  /// Check empty data
  std::vector<concave::primitives::Point<double>> buffer;
  auto jarvis_march_empty {concave::convexHull<concave::Pattern::JarvisMarch>(buffer)};
  EXPECT_TRUE(jarvis_march_empty.empty());

  /// Check little data
  buffer.insert(buffer.end(), m_points.begin(), std::next(m_points.begin(), 2));
  auto jarvis_march_bit {concave::convexHull<concave::Pattern::JarvisMarch>(buffer)};
  EXPECT_TRUE(jarvis_march_bit.empty());
}

// Verifying that all algorithms produce the same result.
TEST_F(ConvexHullTests, algorithm_equivalence_test)
{
  ASSERT_FALSE(m_points.empty());

  for (std::size_t i {3}; i < m_points.size(); ++i) {
    std::vector<concave::primitives::Point<double>> buffer {m_points.begin(), std::next(m_points.begin(),i)};
    auto jarvis_march {concave::convexHull<concave::Pattern::JarvisMarch>(buffer)};
    auto quick_hull {concave::convexHull<concave::Pattern::QuickHull>(buffer)};
    auto graham_scan {concave::convexHull<concave::Pattern::GrahamScan>(buffer)};

    EXPECT_TRUE(!jarvis_march.empty() && !quick_hull.empty() && !graham_scan.empty());
    EXPECT_TRUE((jarvis_march.size() == quick_hull.size()) && (jarvis_march.size() == graham_scan.size()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), quick_hull.begin()));
    EXPECT_TRUE(std::is_permutation(jarvis_march.begin(), jarvis_march.end(), graham_scan.begin()));
  }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}