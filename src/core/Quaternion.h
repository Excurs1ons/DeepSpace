#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {

using Vec3d = ::Mock::Vec3d;

struct Quaternion {
    double w, x, y, z;

    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion Identity() {
        return {1.0, 0.0, 0.0, 0.0};
    }

    static Quaternion FromAxisAngle(const Vec3d& axis, double angle) {
        double halfAngle = angle * 0.5;
        double s = std::sin(halfAngle);
        double c = std::cos(halfAngle);
        Vec3d n = axis.Normalized();
        return {c, n.x * s, n.y * s, n.z * s};
    }

    static Quaternion FromEuler(double pitch, double yaw, double roll) {
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    Quaternion operator*(const Quaternion& o) const {
        return {
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        };
    }

    Vec3d operator*(const Vec3d& v) const {
        double ix =  w * v.x + y * v.z - z * v.y;
        double iy =  w * v.y + z * v.x - x * v.z;
        double iz =  w * v.z + x * v.y - y * v.x;
        double iw = -x * v.x - y * v.y - z * v.z;

        return {
            ix * w + iw * -x + iy * -z - iz * -y,
            iy * w + iw * -y + iz * -x - ix * -z,
            iz * w + iw * -z + ix * -y - iy * -x
        };
    }

    Quaternion Conjugate() const {
        return {w, -x, -y, -z};
    }

    Quaternion Normalized() const {
        double m = Magnitude();
        if (m > 0) {
            return {w / m, x / m, y / m, z / m};
        }
        return Identity();
    }

    double Magnitude() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Vec3d ToEulerAngles() const {
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (w * y - z * x);
        double pitch;
        if (std::abs(sinp) >= 1.0) {
            pitch = std::copysign(M_PI / 2.0, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return {pitch, yaw, roll};
    }
};

}