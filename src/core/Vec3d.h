#pragma once
#include <cmath>
#include <ostream>

namespace Prisma {

struct Vec3d {
    double x, y, z;

    Vec3d() : x(0), y(0), z(0) {}
    Vec3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    Vec3d operator+(const Vec3d& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3d operator-(const Vec3d& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3d operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vec3d operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    Vec3d& operator+=(const Vec3d& other) { x += other.x; y += other.y; z += other.z; return *this; }
    Vec3d& operator-=(const Vec3d& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
    Vec3d operator-() const { return {-x, -y, -z}; }

    double Length() const { return std::sqrt(x*x + y*y + z*z); }
    double LengthSquared() const { return x*x + y*y + z*z; }
    Vec3d Normalized() const { double len = Length(); return len > 0 ? Vec3d(x/len, y/len, z/len) : Vec3d(0,0,0); }

    static double Dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    static Vec3d Cross(const Vec3d& a, const Vec3d& b) {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    friend std::ostream& operator<<(std::ostream& os, const Vec3d& v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
};

inline Vec3d operator*(double s, const Vec3d& v) { return v * s; }

}
