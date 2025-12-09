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
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
    EXPECT_THROW(Intersection(s1, s3), std::invalid_argument);
    
    try {
        Intersection(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("No Intersection. Non-coplanar segments", e.what());
    }
}

TEST(IntersectionTest, CollinearSegmentsNoIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(3, 0, 0), Vector3D(5, 0, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
    
    try {
        Intersection(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("No Intersection.", e.what());
    }
}

TEST(IntersectionTest, CollinearSegmentsTouchingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(4, 0, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_DOUBLE_EQ(result.getX(), 2.0);
    EXPECT_DOUBLE_EQ(result.getY(), 0.0);
    EXPECT_DOUBLE_EQ(result.getZ(), 0.0);
}

TEST(IntersectionTest, CollinearSegmentsOverlapping) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(1, 0, 0), Vector3D(3, 0, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
    
    try {
        Intersection(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("Infinity Intersection. Segment overlaing.", e.what());
    }
}

TEST(IntersectionTest, CollinearSegmentsFullyCoincident) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(3, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(3, 0, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
    
    try {
        Intersection(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("Infinity Intersection. Segment overlaing.", e.what());
    }
}

TEST(IntersectionTest, CollinearSegmentsTouchingAtStartPoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(-2, 0, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_DOUBLE_EQ(result.getX(), 0.0);
    EXPECT_DOUBLE_EQ(result.getY(), 0.0);
    EXPECT_DOUBLE_EQ(result.getZ(), 0.0);
}

TEST(IntersectionTest, PerpendicularSegmentsIntersecting) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, -2, 0), Vector3D(2, 2, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, PerpendicularSegmentsTouchingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, 2, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, PerpendicularSegmentsNoIntersectionLeft) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(3, -1, 0), Vector3D(3, 1, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
}

TEST(IntersectionTest, PerpendicularSegmentsTouchingAtStartPoint) {
    Segment3D s1 = CreateSegment(Vector3D(2, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, -2, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, PerpendicularSegmentsEndToStart) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, 2, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, GeneralCaseIntersectingInside) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 4, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 4, 0), Vector3D(4, 0, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 2.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, GeneralCaseIntersectingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 1.0, EPS);
    EXPECT_NEAR(result.getY(), 1.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, GeneralCaseNoIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 2, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
}

TEST(IntersectionTest, GeneralCaseIntersectionOutsideSegments) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 1, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
}

TEST(IntersectionTest, GeneralCaseAlmostParallel) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(10, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(5, 0.000001, 0), Vector3D(15, 0.000001, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
}

TEST(IntersectionTest, SegmentsIn3DSpaceIntersecting) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 2));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 1.0, EPS);
    EXPECT_NEAR(result.getY(), 1.0, EPS);
    EXPECT_NEAR(result.getZ(), 1.0, EPS);
}

TEST(IntersectionTest, SegmentsWithNegativeCoordinates) {
    Segment3D s1 = CreateSegment(Vector3D(-2, -2, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(-2, 2, 0), Vector3D(2, -2, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 0.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, BoundaryCaseIntersectionAtTEqualsZero) {
    Segment3D s1 = CreateSegment(Vector3D(1, 1, 0), Vector3D(3, 3, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 1.0, EPS);
    EXPECT_NEAR(result.getY(), 1.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, BoundaryCaseIntersectionAtTEqualsOne) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 4, 0), Vector3D(4, 0, 0));
    
    Vector3D result = Intersection(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 2.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(IntersectionTest, ParallelSegmentsNotCollinear) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(2, 1, 0));
    
    EXPECT_THROW(Intersection(s1, s2), std::invalid_argument);
    
    try {
        Intersection(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("No Intersection. Segments are parallel.", e.what());
    }
}

TEST(Vector3D, NearParallel) {
    Segment3D s1({0,0,0}, {1, 1e-13, 0});
    Segment3D s2({0.5,-1,0}, {0.5,1,0});
    EXPECT_NO_THROW({
        auto p = Intersection(s1,s2);
    });
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

TEST(InterseсtionTest, VerySmallSegmentIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(1, 1, 0), Vector3D(1 + EPS, 1, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    
    try {
        Vector3D result = Intersection(s1, s2);
        EXPECT_NEAR(result.getX(), 1.0, EPS);
        EXPECT_NEAR(result.getY(), 1.0, EPS);
    } catch (const std::invalid_argument& e) {
        SUCCEED();
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}