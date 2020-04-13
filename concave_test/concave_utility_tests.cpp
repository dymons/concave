#include "concave_core/utility/Utility.hpp"
#include "concave_core/primitives/Point.hpp"

// Google.Tests
#include <gtest/gtest.h>

// STL
#include <limits>
#include <type_traits>

// CGAL
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>

// OpenCV
#include <opencv2/core/types.hpp>

// PCL
#include <pcl/point_types.h>

using point_cgal = CGAL::Point_2<CGAL::Cartesian<double>>;
using point_opencv = cv::Point_<double>;

TEST(UtilityTests, support_types)
{
  struct arbitraryType {};
  ASSERT_FALSE(concave::utility::has_coordinates_v<arbitraryType>);

  struct arbitraryTypeOneField { double x {0.0}; };
  ASSERT_FALSE(concave::utility::has_coordinates_v<arbitraryTypeOneField>);

  struct arbitraryTypeAnotherField { double x {0.0}; double z {0.0}; };
  ASSERT_FALSE(concave::utility::has_coordinates_v<arbitraryTypeAnotherField>);

  struct arbitraryTypeCorrectField { double x {0.0}; double y {0.0}; };
  ASSERT_TRUE(concave::utility::has_coordinates_v<arbitraryTypeCorrectField>);

  ASSERT_TRUE(concave::utility::has_coordinates_v<concave::primitives::Point<>>);
  ASSERT_TRUE(concave::utility::has_coordinates_v<point_cgal>);
  ASSERT_TRUE(concave::utility::has_coordinates_v<point_opencv>);
  ASSERT_TRUE(concave::utility::has_coordinates_v<pcl::PointXYZ>);
  ASSERT_TRUE(concave::utility::has_coordinates_v<pcl::PointXYZI>);
}

TEST(DistanceTests, check_distance_double)
{
    using Point = concave::primitives::Point<double>;

    // Regardless of the quarter, the lengths are the same
    EXPECT_DOUBLE_EQ(concave::utility::distance(Point{0.0, 0.0}, Point{3.0, 4.0}), 5.0);
    EXPECT_DOUBLE_EQ(concave::utility::distance(Point{0.0, 0.0}, Point{3.0, -4.0}), 5.0);
    EXPECT_DOUBLE_EQ(concave::utility::distance(Point{0.0, 0.0}, Point{-3.0, 4.0}), 5.0);
    EXPECT_DOUBLE_EQ(concave::utility::distance(Point{0.0, 0.0}, Point{-3.0, -4.0}), 5.0);
    ASSERT_NEAR(concave::utility::distance(Point{1.0789, 1.123}, Point{4.567, 4.789}),
                concave::utility::distance(Point{1.0789, -1.123}, Point{4.567, -4.789}),
                std::numeric_limits<double>::epsilon());

    // Not valid data
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_DOUBLE_EQ(concave::utility::distance(Point{inf, 0.0}, Point{3.0, 4.0}), inf);
    EXPECT_DOUBLE_EQ(concave::utility::distance(Point{0.0, 0.0}, Point{inf, 4.0}), inf);
    EXPECT_TRUE(std::isnan(concave::utility::distance(Point{inf, 0.0}, Point{inf, 4.0})));
    EXPECT_TRUE(std::isnan(concave::utility::distance(Point{inf, inf}, Point{inf, inf})));
}

int main(int t_argc, char** t_argv)
{
    ::testing::InitGoogleTest(&t_argc, t_argv);

    return RUN_ALL_TESTS();
}