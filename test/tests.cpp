#include <gtest/gtest.h>
#include "Vector3D.h"
#include <cmath>

const double EPS = 1e-10;

Segment3D CreateSegment(Vector3D start, Vector3D end) {
    if ((end - start).IsZero()) {
        throw std::invalid_argument("Cannot create degenerate segment (a point)");
    }
    return Segment3D(start, end);
}

TEST(InterseptionTest, NonCoplanarSegments) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 1, 1));
    Segment3D s3 = CreateSegment(Vector3D(0, 0, 1), Vector3D(1, 0, 1));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
    EXPECT_THROW(Interseption(s1, s3), std::invalid_argument);
    
    try {
        Interseption(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("No Interseption. Non-coplanar segments", e.what());
    }
}

TEST(InterseptionTest, CollinearSegmentsNoIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(3, 0, 0), Vector3D(5, 0, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
    
    try {
        Interseption(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("No Interseption.", e.what());
    }
}

TEST(InterseptionTest, CollinearSegmentsTouchingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(4, 0, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_DOUBLE_EQ(result.getX(), 2.0);
    EXPECT_DOUBLE_EQ(result.getY(), 0.0);
    EXPECT_DOUBLE_EQ(result.getZ(), 0.0);
}

TEST(InterseptionTest, CollinearSegmentsOverlapping) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(1, 0, 0), Vector3D(3, 0, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
    
    try {
        Interseption(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("Infinity Interseption. Segment overlaing.", e.what());
    }
}

TEST(InterseptionTest, CollinearSegmentsFullyCoincident) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(3, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(3, 0, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
    
    try {
        Interseption(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("Infinity Interseption. Segment overlaing.", e.what());
    }
}

TEST(InterseptionTest, CollinearSegmentsTouchingAtStartPoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(-2, 0, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_DOUBLE_EQ(result.getX(), 0.0);
    EXPECT_DOUBLE_EQ(result.getY(), 0.0);
    EXPECT_DOUBLE_EQ(result.getZ(), 0.0);
}

TEST(InterseptionTest, PerpendicularSegmentsIntersecting) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, -2, 0), Vector3D(2, 2, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, PerpendicularSegmentsTouchingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, 2, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, PerpendicularSegmentsNoIntersectionLeft) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(3, -1, 0), Vector3D(3, 1, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
}

TEST(InterseptionTest, PerpendicularSegmentsTouchingAtStartPoint) {
    Segment3D s1 = CreateSegment(Vector3D(2, 0, 0), Vector3D(4, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, -2, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, PerpendicularSegmentsEndToStart) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(2, 0, 0), Vector3D(2, 2, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, GeneralCaseIntersectingInside) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(4, 4, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 4, 0), Vector3D(4, 0, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 2.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, GeneralCaseIntersectingAtEndpoint) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 1.0, EPS);
    EXPECT_NEAR(result.getY(), 1.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, GeneralCaseNoIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 2, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
}

TEST(InterseptionTest, GeneralCaseIntersectionOutsideSegments) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(1, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(1, 1, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
}

TEST(InterseptionTest, GeneralCaseAlmostParallel) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(10, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(5, 0.000001, 0), Vector3D(15, 0.000001, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
}

TEST(InterseptionTest, SegmentsIn3DSpaceIntersecting) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 2));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 2));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 1.0, EPS);
    EXPECT_NEAR(result.getY(), 1.0, EPS);
    EXPECT_NEAR(result.getZ(), 1.0, EPS);
}

TEST(InterseptionTest, SegmentsWithNegativeCoordinates) {
    Segment3D s1 = CreateSegment(Vector3D(-2, -2, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(-2, 2, 0), Vector3D(2, -2, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 0.0, EPS);
    EXPECT_NEAR(result.getY(), 0.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, BoundaryCaseIntersectionAtTEqualsZero) {
    Segment3D s1 = CreateSegment(Vector3D(1, 1, 0), Vector3D(3, 3, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 2, 0), Vector3D(2, 0, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 1.0, EPS);
    EXPECT_NEAR(result.getY(), 1.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, BoundaryCaseIntersectionAtTEqualsOne) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 4, 0), Vector3D(4, 0, 0));
    
    Vector3D result = Interseption(s1, s2);
    EXPECT_NEAR(result.getX(), 2.0, EPS);
    EXPECT_NEAR(result.getY(), 2.0, EPS);
    EXPECT_NEAR(result.getZ(), 0.0, EPS);
}

TEST(InterseptionTest, ParallelSegmentsNotCollinear) {
    Segment3D s1 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 0, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 1, 0), Vector3D(2, 1, 0));
    
    EXPECT_THROW(Interseption(s1, s2), std::invalid_argument);
    
    try {
        Interseption(s1, s2);
        FAIL() << "Expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_STREQ("No Interseption. Segments are parallel.", e.what());
    }
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

TEST(InterseptionTest, VerySmallSegmentIntersection) {
    Segment3D s1 = CreateSegment(Vector3D(1, 1, 0), Vector3D(1 + EPS/10, 1, 0));
    Segment3D s2 = CreateSegment(Vector3D(0, 0, 0), Vector3D(2, 2, 0));
    
    try {
        Vector3D result = Interseption(s1, s2);
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