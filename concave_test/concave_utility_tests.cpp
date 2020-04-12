#include "concave_core/utility/Utility.hpp"
#include "concave_core/primitives/Point.hpp"

// Google.Tests
#include <gtest/gtest.h>

// STL
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

/**
  * \brief Has a class function member or field 'x | x()' and 'y | y()'
  */
template<typename T, typename = void>
struct has_coordinates : std::false_type {};

template<typename T>
struct has_coordinates<T, typename std::enable_if_t<std::is_member_pointer_v<decltype(&T::x)>
                                                 && std::is_member_pointer_v<decltype(&T::y)>>> : std::true_type {};

template <typename T>
inline constexpr bool has_coordinates_v = has_coordinates<T>::value;

TEST(UtilityTests, support_types)
{
  struct arbitraryType {};
  ASSERT_FALSE(has_coordinates_v<arbitraryType>);

  struct arbitraryTypeOneField { double x {0.0}; };
  ASSERT_FALSE(has_coordinates_v<arbitraryTypeOneField>);

  struct arbitraryTypeAnotherField { double x {0.0}; double z {0.0}; };
  ASSERT_FALSE(has_coordinates_v<arbitraryTypeAnotherField>);

  struct arbitraryTypeCorrectField { double x {0.0}; double y {0.0}; };
  ASSERT_TRUE(has_coordinates_v<arbitraryTypeCorrectField>);

  ASSERT_TRUE(has_coordinates_v<concave::primitives::Point<>>);
  ASSERT_TRUE(has_coordinates_v<point_cgal>);
  ASSERT_TRUE(has_coordinates_v<point_opencv>);
  ASSERT_TRUE(has_coordinates_v<pcl::PointXYZ>);
  ASSERT_TRUE(has_coordinates_v<pcl::PointXYZI>);
}

int main(int t_argc, char** t_argv)
{
    ::testing::InitGoogleTest(&t_argc, t_argv);

    return RUN_ALL_TESTS();
}