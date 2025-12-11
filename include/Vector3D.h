#pragma once

#include <iostream>
#include <optional>
#include <exception>
#include <math.h>

extern const double eps;

class Vector3D {
    double x;
    double y;
    double z;
public:

    Vector3D() = delete;

    Vector3D(double x, double y, double z):x(x), y(y), z(z) {}
    
    bool operator==(const Vector3D& vec) const { 
        if (abs(x - vec.x) < eps && abs(y - vec.y) < eps && abs(z - vec.z) < eps) {
            return true;
        }
        return false;
    }
    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }
    Vector3D operator*(const double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }
    Vector3D operator/(const double scalar) const {
        return Vector3D(x / scalar, y / scalar, z / scalar);
    }
    double operator[](size_t indx) const {
        switch (indx)
        {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        default: throw std::out_of_range("Vector3D index out of range");
        }
    }

    double len() const {
        return sqrt(x*x + y*y + z*z);
    }

    bool IsZero() const {
        return len() < eps;
    }
    double getX() const {
        return x;
    }
    double getY() const {
        return y;
    }
    double getZ() const {
        return z;
    }
};

Vector3D operator*(double scalar, const Vector3D& vec);
double dot(const Vector3D& v1, const Vector3D& v2);
Vector3D cross(const Vector3D& v1, const Vector3D& v2);
bool Collinearity(const Vector3D& v1, const Vector3D& v2);
bool Coplanarity(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3);

class Segment3D {
    Vector3D start;
    Vector3D end;
public:
    Segment3D() = delete;
    Segment3D(Vector3D p_start, Vector3D p_end): start(p_start), end(p_end) {
        if ((p_end - p_start).IsZero()) {
            throw std::invalid_argument("Сan't create a segment");
        }
    }

    Vector3D getStart() const {
        return start;
    }
    
    Vector3D getEnd() const {
        return end;
    }
    
    Vector3D getvec() const {
        return end - start;
    }
};

enum class IntersectionResult {
    INTERSECTION = 0,
    NONCOMPLANAR = 1,
    PARALLEL = 2,
    COLLINEARNOOVERLAP = 3,
    OVERLAPPING = 4,
    NOINTERSECTION = 5   
};

struct IntersectionInfo {
    IntersectionResult result;
    std::optional<Vector3D> point;
};

double getParametr(const Vector3D& vec, const Segment3D& sgm);
IntersectionInfo Intersection(const Segment3D& sgm1, const Segment3D& sgm2);


enum TypeOfCollinear {
    SGMNTSOVERLAP = 4,
    SGMNTSTOUCH = 3,
    SGMNTSONONELINE = 2,
    SGMNTSPARALLEL = 1
};

TypeOfCollinear DefineTypeOfCollnear(const Segment3D& sgm1, const Segment3D& sgm2);