#include "concave_core/utility/Utility.hpp"
#include "concave_core/primitives/Point.hpp"

// STL
#include <iostream>
#include <type_traits>

// CGAL
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>

// OpenCV
#include <opencv2/core/types.hpp>

#include <gtest/gtest.h>

using point_cgal   = CGAL::Point_2<CGAL::Cartesian<double>>;
using point_opencv = cv::Point_<double>;

class UtilityTests : public ::testing::Test { };

TEST_F(UtilityTests, concavehullcpp_utility_function)
{
  EXPECT_TRUE(!std::is_member_function_pointer_v<decltype(&concave::primitives::Point<>::x)> && !std::is_member_function_pointer_v<decltype(&concave::primitives::Point<>::y)>); // x/y is data member
  EXPECT_TRUE(std::is_member_function_pointer_v<decltype(&point_cgal::x)> && std::is_member_function_pointer_v<decltype(&point_cgal::y)>); // x/y is function
  EXPECT_TRUE(!std::is_member_function_pointer_v<decltype(&point_opencv::x)> && !std::is_member_function_pointer_v<decltype(&point_opencv::y)>); // x/y is data member
}

TEST_F(UtilityTests, concavehullcpp_utility_orientetion)
{
  {
    auto o = concave::utility::orientetion(point_cgal(0,0), point_cgal(4,4), point_cgal(1,2));
    EXPECT_TRUE(o == concave::utility::Orientation::COUNTERCLOCKWISE);
  }

  {
    auto o = concave::utility::orientetion(point_opencv(0,0), point_opencv(4,4), point_opencv(1,2));
    EXPECT_TRUE(o == concave::utility::Orientation::COUNTERCLOCKWISE);
  }

  {
    auto o = concave::utility::orientetion(concave::primitives::Point<>(0,0), concave::primitives::Point<>(4,4), concave::primitives::Point<>(1,2));
    EXPECT_TRUE(o == concave::utility::Orientation::COUNTERCLOCKWISE);
  }
}

TEST_F(UtilityTests, concavehullcpp_utility_less)
{
  EXPECT_TRUE(concave::utility::is_less_v<concave::primitives::Point<>>);
  EXPECT_TRUE(concave::utility::is_less_v<point_cgal>);
  EXPECT_TRUE(!concave::utility::is_less_v<point_opencv>);
}

TEST_F(UtilityTests, concavehullcpp_utility_side)
{
  EXPECT_TRUE(concave::utility::side(point_cgal(0,0), point_cgal(2,2), point_cgal(1,1)) == concave::utility::Side::StraightLine);
  EXPECT_TRUE(concave::utility::side(point_cgal(0,0), point_cgal(2,2), point_cgal(1,0)) == concave::utility::Side::RightSide);
  EXPECT_TRUE(concave::utility::side(point_cgal(0,0), point_cgal(2,2), point_cgal(0,1)) == concave::utility::Side::LeftSide);

  EXPECT_TRUE(concave::utility::side(point_cgal(2,2), point_cgal(0,0), point_cgal(1,1)) == concave::utility::Side::StraightLine);
  EXPECT_TRUE(!(concave::utility::side(point_cgal(2,2), point_cgal(0,0), point_cgal(1,0)) == concave::utility::Side::RightSide));
  EXPECT_TRUE(!(concave::utility::side(point_cgal(2,2), point_cgal(0,0), point_cgal(0,1)) == concave::utility::Side::LeftSide));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}