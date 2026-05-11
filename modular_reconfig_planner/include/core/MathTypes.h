#pragma once

#include <cmath>
#include <ostream>

namespace mrr {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 operator*(const Vec3& a, double k) { return {a.x*k, a.y*k, a.z*k}; }
inline Vec3 operator*(double k, const Vec3& a) { return a*k; }
inline Vec3 operator/(const Vec3& a, double k) { return {a.x/k, a.y/k, a.z/k}; }

inline double distance(const Vec3& a, const Vec3& b) { return (a-b).norm(); }
inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) { return a*(1.0-t) + b*t; }

struct Pose3D {
    Vec3 position;
    Vec3 rpy; // roll, pitch, yaw in radians
};

inline Pose3D interpolatePose(const Pose3D& a, const Pose3D& b, double t) {
    return {lerp(a.position, b.position, t), lerp(a.rpy, b.rpy, t)};
}

inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

} // namespace mrr
