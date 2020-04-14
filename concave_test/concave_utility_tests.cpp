#include "concave_core/primitives/Point.hpp"
#include "concave_core/utility/Utility.hpp"

// Google.Tests
#include <gtest/gtest.h>

// STL
#include <limits>

// CGAL
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>

// OpenCV
#include <opencv2/core/types.hpp>

// PCL
#include <pcl/point_types.h>

TEST(UtilityTests, support_types)
{
	struct ArbitraryType {
	};
	ASSERT_FALSE(concave::utility::has_coordinates_v<ArbitraryType>);

	struct ArbitraryTypeOneField {
		double x { 0.0 };
	};
	ASSERT_FALSE(concave::utility::has_coordinates_v<ArbitraryTypeOneField>);

	struct ArbitraryTypeAnotherField {
		double x { 0.0 };
		double z { 0.0 };
	};
	ASSERT_FALSE(concave::utility::has_coordinates_v<ArbitraryTypeAnotherField>);

	struct ArbitraryTypeCorrectField {
		double x { 0.0 };
		double y { 0.0 };
	};
	ASSERT_TRUE(concave::utility::has_coordinates_v<ArbitraryTypeCorrectField>);

	using PointOpenCv = cv::Point_<double>;
	using PointCgal = CGAL::Point_2<CGAL::Cartesian<double>>;
	ASSERT_TRUE(concave::utility::has_coordinates_v<concave::primitives::Point<>>);
	ASSERT_TRUE(concave::utility::has_coordinates_v<PointCgal>);
	ASSERT_TRUE(concave::utility::has_coordinates_v<PointOpenCv>);
	ASSERT_TRUE(concave::utility::has_coordinates_v<pcl::PointXYZ>);
	ASSERT_TRUE(concave::utility::has_coordinates_v<pcl::PointXYZI>);
}

TEST(DistanceTests, check_distance_double)
{
	using Point = concave::primitives::Point<double>;

	// Regardless of the quarter, the lengths are the same
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, Point { 3.0, 4.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, Point { 3.0, -4.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, Point { -3.0, 4.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, Point { -3.0, -4.0 }), 5.0);
	ASSERT_NEAR(concave::utility::distance(Point { 1.0789, 1.123 }, Point { 4.567, 4.789 }),
			concave::utility::distance(Point { 1.0789, -1.123 }, Point { 4.567, -4.789 }),
			std::numeric_limits<double>::epsilon());

	// Different types
	using PointOpenCv = cv::Point_<double>;
	using PointCgal = CGAL::Point_2<CGAL::Cartesian<double>>;
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, PointOpenCv { 3.0, 4.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, PointCgal { 3.0, -4.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, pcl::PointXYZ { -3.0, 4.0, 0.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(PointOpenCv { 0.0, 0.0 }, Point { 3.0, 4.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(PointCgal { 0.0, 0.0 }, Point { 3.0, -4.0 }), 5.0);
	EXPECT_DOUBLE_EQ(concave::utility::distance(pcl::PointXYZ { 0.0, 0.0, 0.0 }, Point { -3.0, 4.0 }), 5.0);

	// Not valid data
	const double inf = std::numeric_limits<double>::infinity();
	const double nan = std::numeric_limits<double>::quiet_NaN();
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { inf, 0.0 }, Point { 3.0, 4.0 }), inf);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, Point { inf, 4.0 }), inf);
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { inf, 0.0 }, Point { inf, 4.0 })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { inf, inf }, Point { inf, inf })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { nan, 0.0 }, Point { 3.0, 4.0 })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { 0.0, 0.0 }, Point { nan, 4.0 })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { nan, 0.0 }, Point { nan, 4.0 })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { nan, nan }, Point { nan, nan })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { inf, 0.0 }, Point { nan, 4.0 })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { inf, inf }, Point { nan, inf })));

	// Not valid data with different types
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { inf, 0.0 }, PointOpenCv { 3.0, 4.0 }), inf);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, PointOpenCv { inf, 4.0 }), inf);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { inf, 0.0 }, PointCgal { 3.0, 4.0 }), inf);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, PointCgal { inf, 4.0 }), inf);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { inf, 0.0 }, pcl::PointXYZ { 3.0, 4.0, 0.0 }), inf);
	EXPECT_DOUBLE_EQ(concave::utility::distance(Point { 0.0, 0.0 }, pcl::PointXYZ { std::numeric_limits<float>::infinity(), .0, .0 }), inf);
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { inf, 0.0 }, PointOpenCv { inf, 4.0 })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { inf, inf }, PointCgal { inf, inf })));
	EXPECT_TRUE(std::isnan(concave::utility::distance(Point { inf, inf }, pcl::PointXYZ { std::numeric_limits<float>::infinity(),
																						  std::numeric_limits<float>::infinity(),
																						  std::numeric_limits<float>::infinity() })));
}

int main(int t_argc, char** t_argv)
{
	::testing::InitGoogleTest(&t_argc, t_argv);

	return RUN_ALL_TESTS();
}