#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {

using Vec3d = ::Mock::Vec3d;

struct Mat3x3 {
    double m[9];

    static Mat3x3 Identity() {
        return {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
    }

    static Mat3x3 FromDiagonal(double ix, double iy, double iz) {
        return {ix, 0.0, 0.0,
                0.0, iy, 0.0,
                0.0, 0.0, iz};
    }

    static Mat3x3 Rotation(const Vec3d& axis, double angle) {
        Vec3d n = axis.Normalized();
        double c = std::cos(angle);
        double s = std::sin(angle);
        double t = 1.0 - c;

        return {t * n.x * n.x + c,       t * n.x * n.y - n.z * s,  t * n.x * n.z + n.y * s,
                t * n.x * n.y + n.z * s, t * n.y * n.y + c,       t * n.y * n.z - n.x * s,
                t * n.x * n.z - n.y * s, t * n.y * n.z + n.x * s, t * n.z * n.z + c};
    }

    Mat3x3 operator*(const Mat3x3& o) const {
        return {
            m[0] * o.m[0] + m[1] * o.m[3] + m[2] * o.m[6],
            m[0] * o.m[1] + m[1] * o.m[4] + m[2] * o.m[7],
            m[0] * o.m[2] + m[1] * o.m[5] + m[2] * o.m[8],
            m[3] * o.m[0] + m[4] * o.m[3] + m[5] * o.m[6],
            m[3] * o.m[1] + m[4] * o.m[4] + m[5] * o.m[7],
            m[3] * o.m[2] + m[4] * o.m[5] + m[5] * o.m[8],
            m[6] * o.m[0] + m[7] * o.m[3] + m[8] * o.m[6],
            m[6] * o.m[1] + m[7] * o.m[4] + m[8] * o.m[7],
            m[6] * o.m[2] + m[7] * o.m[5] + m[8] * o.m[8]
        };
    }

    Vec3d operator*(const Vec3d& v) const {
        return {
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z
        };
    }

    Mat3x3 operator*(double s) const {
        return {m[0] * s, m[1] * s, m[2] * s,
                m[3] * s, m[4] * s, m[5] * s,
                m[6] * s, m[7] * s, m[8] * s};
    }

    Mat3x3 Transpose() const {
        return {m[0], m[3], m[6],
                m[1], m[4], m[7],
                m[2], m[5], m[8]};
    }

    double Determinant() const {
        return m[0] * (m[4] * m[8] - m[5] * m[7])
             - m[1] * (m[3] * m[8] - m[5] * m[6])
             + m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    Mat3x3 Inverse() const {
        double det = Determinant();
        if (std::abs(det) < 1e-15) {
            return Identity();
        }
        double invDet = 1.0 / det;

        return {
            (m[4] * m[8] - m[5] * m[7]) * invDet,
            (m[2] * m[7] - m[1] * m[8]) * invDet,
            (m[1] * m[5] - m[2] * m[4]) * invDet,
            (m[5] * m[6] - m[3] * m[8]) * invDet,
            (m[0] * m[8] - m[2] * m[6]) * invDet,
            (m[2] * m[3] - m[0] * m[5]) * invDet,
            (m[3] * m[7] - m[4] * m[6]) * invDet,
            (m[1] * m[6] - m[0] * m[7]) * invDet,
            (m[0] * m[4] - m[1] * m[3]) * invDet
        };
    }
};

}