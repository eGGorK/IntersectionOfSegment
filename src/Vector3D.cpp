#include "Vector3D.h"
#include <iostream>

const double eps = 1e-12;

Vector3D operator*(double scalar, const Vector3D& vec) {
    return vec * scalar;
}

double dot(const Vector3D& v1, const Vector3D& v2) {
    return v1.getX()*v2.getX() + v1.getY()*v2.getY() + v1.getZ()*v2.getZ();
}

Vector3D cross(const Vector3D& v1, const Vector3D& v2) {
    return Vector3D(
        v1.getY() * v2.getZ() - v1.getZ() * v2.getY(),
        v1.getZ() * v2.getX() - v1.getX() * v2.getZ(),
        v1.getX() * v2.getY() - v1.getY() * v2.getX()
    );
}

bool Coplanarity(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3) {
    if (abs(dot(cross(v1,v2),v3)) < eps) {
        return true;
    }
    return false;
}

bool Collinearity(const Vector3D& v1, const Vector3D& v2) {
    return cross(v1,v2).IsZero();
}

double getParametr(const Vector3D& vec, const Segment3D& sgm) {
    Vector3D lineDir = sgm.getEnd() - sgm.getStart();
    Vector3D diff = vec - sgm.getStart();
    double dotDir = dot(lineDir, lineDir);

    if (abs(dotDir) < eps) throw std::runtime_error("Zero-directional vector");
    
    return dot(diff, lineDir)/ dotDir;
}


TypeOfCollinear DefineTypeOfCollnear(const Segment3D& sgm1, const Segment3D& sgm2) {
    Vector3D v1 = sgm1.getvec();
    Vector3D v2 = sgm2.getvec();
    Vector3D v3 = sgm1.getStart() - sgm2.getStart();

    Vector3D crosscheck = cross(v1,v3);
    if (crosscheck.IsZero()) {
        double t2_start = getParametr(sgm2.getStart(), sgm1);
        double t2_end = getParametr(sgm2.getEnd(), sgm1);
        
        double t_min = std::min(t2_start, t2_end);
        double t_max = std::max(t2_start, t2_end);
        
        double overlay_start = std::max(0.0, t_min);
        double overlay_end = std::min(1.0, t_max);
        if ( (overlay_end - overlay_start) > eps) {
            return SGMNTSOVERLAP;
        }
        if ( abs(overlay_end - overlay_start) < eps) {
            return SGMNTSTOUCH;
        }
        return SGMNTSONONELINE;
    }
    return SGMNTSPARALLEL;
}

Vector3D Intersection(const Segment3D& sgm1, const Segment3D& sgm2) {
    
    Vector3D v1 = sgm1.getEnd() - sgm1.getStart();
    Vector3D v2 = sgm2.getEnd() - sgm2.getStart();
    Vector3D connection = sgm2.getStart() - sgm1.getStart();

    if (Coplanarity(v1,v2,connection) == false) {
        throw std::invalid_argument("No Intersection. Non-coplanar segments");
    }

    if (Collinearity(v1,v2) == true) { 
        TypeOfCollinear type = DefineTypeOfCollnear(sgm1,sgm2);
        switch (type) {
            case SGMNTSPARALLEL:
                throw std::invalid_argument("No Intersection. Segments are parallel.");
            case SGMNTSONONELINE:
                throw std::invalid_argument("No Intersection.");
            case SGMNTSOVERLAP:
                throw std::invalid_argument("Infinity Intersection. Segment overlaing.");
            case SGMNTSTOUCH:
                if (sgm1.getStart() == sgm2.getStart() || sgm1.getStart() == sgm2.getEnd()) return sgm1.getStart();
                if (sgm1.getEnd() == sgm2.getStart() || sgm1.getEnd() == sgm2.getEnd()) return sgm1.getEnd();
        }
    }
    
    Vector3D cross_v1_v2 = cross(v1,v2);
    Vector3D cross_connection_v2 = cross(connection,v2);
    double t_v1 = dot(cross_connection_v2, cross_v1_v2)/dot(cross_v1_v2,cross_v1_v2);

    Vector3D cross_connection_v1 = cross(connection,v1);
    double t_v2 = dot(cross_connection_v1, cross_v1_v2)/dot(cross_v1_v2,cross_v1_v2);
    
    if (std::abs(t_v1) < eps) {
        t_v1 = 0.0;
    } else if (std::abs(t_v1 - 1.0) < eps) {
        t_v1 = 1.0;
    }
    
    if (std::abs(t_v2) < eps) {
        t_v2 = 0.0;
    } else if (std::abs(t_v2 - 1.0) < eps) {
        t_v2 = 1.0;
    }
    
    if (t_v1 >= 0.0 && t_v1 <= 1.0 && t_v2 >= 0.0 && t_v2 <= 1.0) {
        return sgm1.getStart() + t_v1 * v1;
    }
throw std::invalid_argument("No Intersection.");
}