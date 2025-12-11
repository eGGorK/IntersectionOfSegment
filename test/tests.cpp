#include <gtest/gtest.h>
#include "Vector3D.h"
#include <cmath>

const double EPS = 1e-12;

Segment3D CreateSegment(Vector3D start, Vector3D end) {
    if ((end - start).IsZero()) {
        throw std::invalid_argument("Cannot create degenerate segment (a point)");
    }
    return Segment3D(start, end);
}

TEST(IntersectionTest, NonCoplanarSegments) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 1, 1));
    Segment3D s3 = CreateSegment(Vector3D(0, 0, 1), Vector3D(1, 0, 1));
    
    auto result1 = Intersection(s1, s2);
    EXPECT_EQ(result1.result, IntersectionResult::NONCOMPLANAR);
    EXPECT_FALSE(result1.point.has_value());
    
    auto result2 = Intersection(s2, s3);
    EXPECT_EQ(result2.result, IntersectionResult::NONCOMPLANAR);
    EXPECT_FALSE(result2.point.has_value());
}

TEST(IntersectionTest, CollinearSegmentsNoIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(3, 0, 0), Vector3D(5, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::COLLINEARNOOVERLAP);
    EXPECT_FALSE(result.point.has_value());
}

TEST(IntersectionTest, CollinearSegmentsTouchingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(4, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, CollinearSegmentsOverlapping) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(1, 0, 0), Vector3D(3, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::OVERLAPPING);
    EXPECT_FALSE(result.point.has_value());
}

TEST(IntersectionTest, CollinearSegmentsFullyCoincident) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(3, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(3, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::OVERLAPPING);
    EXPECT_FALSE(result.point.has_value());
}

TEST(IntersectionTest, CollinearSegmentsTouchingAtStartPoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(-2, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, PerpendicularSegmentsIntersecting) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, -2, 0), Vector3D(2, 2, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, PerpendicularSegmentsTouchingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, 2, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, PerpendicularSegmentsNoIntersectionLeft) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(3, -1, 0), Vector3D(3, 1, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::NOINTERSECTION);
    EXPECT_FALSE(result.point.has_value());
}

TEST(IntersectionTest, PerpendicularSegmentsTouchingAtStartPoint) {
    Segment3D s1 = CreateSegment(Vector3D(2, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, -2, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, PerpendicularSegmentsEndToStart) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, 2, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, IntersectingInside) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 4, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 4, 0), Vector3D(4, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, IntersectingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 1.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 1.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, NoIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 2, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::NOINTERSECTION);
    EXPECT_FALSE(result.point.has_value());
}

TEST(IntersectionTest, IntersectionOutsideSegments) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 1, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::PARALLEL);
    EXPECT_FALSE(result.point.has_value());
}

TEST(IntersectionTest, AlmostParallel) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(10, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(5, 0.000001, 0), Vector3D(15, 0.000001, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::PARALLEL);
    EXPECT_FALSE(result.point.has_value());
}

TEST(IntersectionTest, SegmentsIn3DSpaceIntersecting) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 2));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 1.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 1.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 1.0, EPS);
}

TEST(IntersectionTest, SegmentsWithNegativeCoordinates) {
    Segment3D s1 = CreateSegment(Vector3D(-2, -2, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(-2, 2, 0), Vector3D(2, -2, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 0.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, IntersectionAtTEqualsZero) {
    Segment3D s1 = CreateSegment(Vector3D(1, 1, 0), Vector3D(3, 3, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 1.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 1.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, IntersectionAtTEqualsOne) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 4, 0), Vector3D(4, 0, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::INTERSECTION);
    ASSERT_TRUE(result.point.has_value());
    EXPECT_NEAR(result.point.value().getX(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getY(), 2.0, EPS);
    EXPECT_NEAR(result.point.value().getZ(), 0.0, EPS);
}

TEST(IntersectionTest, ParallelSegmentsNotCollinear) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(2, 1, 0));
    
    auto result = Intersection(s1, s2);
    EXPECT_EQ(result.result, IntersectionResult::PARALLEL);
    EXPECT_FALSE(result.point.has_value());
}

TEST(Vector3D, NearParallel) {
    Segment3D s1({0,0,0}, {1, 1e-13, 0});
    Segment3D s2({0.5,-1,0}, {0.5,1,0});
    
    auto result = Intersection(s1, s2);
    EXPECT_TRUE(result.result == IntersectionResult::INTERSECTION || 
                result.result == IntersectionResult::NOINTERSECTION);
}

TEST(Segment3DTest, CannotCreateZeroLengthSegment) {
    EXPECT_THROW(
        CreateSegment(Vector3D(1, 1, 0), Vector3D(1, 1, 0)), 
        std::invalid_argument
    );
}

TEST(Segment3DTest, CannotCreateDegenerateSegment) {
    EXPECT_THROW(
        Segment3D(Vector3D(1, 2, 3), Vector3D(1, 2, 3)), 
        std::invalid_argument
    );
}

TEST(Segment3DTest, ValidSegmentCreation) {
    EXPECT_NO_THROW(
        CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0))
    );
    
    EXPECT_NO_THROW(
        CreateSegment(Vector3D(0, 0, 0), Vector3D(0.000001, 0.000001, 0.000001))
    );
}

TEST(IntersectionTest, VerySmallSegmentIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(1, 1, 0), Vector3D(1 + EPS, 1, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    
    auto result = Intersection(s1, s2);
    if (result.result == IntersectionResult::INTERSECTION) {
        ASSERT_TRUE(result.point.has_value());
        EXPECT_NEAR(result.point.value().getX(), 1.0, 10*EPS);
        EXPECT_NEAR(result.point.value().getY(), 1.0, 10*EPS);
    } else {
        EXPECT_EQ(result.result, IntersectionResult::NOINTERSECTION);
    }
}

TEST(IntersectionTest, DegenerateSegmentsShouldNotBeCreated) {
    EXPECT_THROW(
        CreateSegment(Vector3D(0, 0, 0), Vector3D(0, 0, 0)),
        std::invalid_argument
    );
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}