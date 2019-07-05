/*
MIT License

Copyright (c) 2018 LAK132

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef CQL_QUATERNION_H
#define CQL_QUATERNION_H

#include <cmath>
#include <utility>
#include <type_traits>

#if !defined(CQL_HAS_GLM_HEADERS) && \
    __has_include(<glm/gtc/constants.hpp>) && \
    __has_include(<glm/mat4x4.hpp>) && \
    __has_include(<glm/vec4.hpp>) && \
    __has_include(<glm/vec3.hpp>)
#define CQL_HAS_GLM_HEADERS
#endif

#ifdef CQL_HAS_GLM_HEADERS
#include <glm/gtc/constants.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>
#include <glm/vec3.hpp>
#define CQL_GLM_TEMPLATE template<typename GLMT = float, glm::precision GLMP = glm::defaultp>
#define CQL_MAT4 glm::tmat4x4<GLMT,GLMP>
#define CQL_VEC4 glm::tvec4<GLMT,GLMP>
#define CQL_VEC3 glm::tvec3<GLMT,GLMP>
#endif // CQL_HAS_GLM_HEADERS

namespace cql
{
    template<typename T>
    struct quat
    {
        using value_t = T;
        using quat_t = quat<value_t>;
        value_t x;
        value_t y;
        value_t z;
        value_t w;

        quat(const quat_t &rhs)
        : x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w)
        {}

        template<typename R>
        explicit quat(const quat<R> &rhs)
        : x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w)
        {}

        quat(const value_t &val = value_t(0))
        : x(val), y(val), z(val), w(val)
        {}

        quat(const value_t (&val)[4])
        : x(val[0]), y(val[1]), z(val[2]), w(val[3])
        {}

        quat(const value_t &X, const value_t &Y,
             const value_t &Z, const value_t &W)
        : x(X), y(Y), z(Z), w(W)
        {}

        //
        // Conjugate operator
        //

        quat_t conjugate() const
        {
            return {-x, -y, -z, w};
        }

        inline quat_t operator*() const
        {
            return conjugate();
        }

        //
        // Magnitude operator
        //

        value_t magnitude() const
        {
            return std::sqrt((x * x) + (y * y) + (z * z) + (w * w));
        }

        inline value_t operator~() const
        {
            return magnitude();
        }

        //
        // Normalise operator
        //

        quat_t &normalise()
        {
            return *this *= (value_t(1) / magnitude());
        }

        quat_t normalised() const
        {
            return *this * (value_t(1) / magnitude());
        }

        inline quat_t operator!() const
        {
            return normalised();
        }

        //
        // Assignment operator
        //

        quat_t &operator=(const quat_t &rhs)
        {
            x = rhs.x; y = rhs.y; z = rhs.z; w = rhs.w;
            return *this;
        }

        //
        // Addition operators
        //

        quat_t operator+(const quat_t &rhs) const
        {
            return {x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w};
        }

        quat_t &operator+=(const quat_t &rhs)
        {
            x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w;
            return *this;
        }

        //
        // Subtraction operators
        //

        quat_t operator-(const quat_t &rhs) const
        {
            return {x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w};
        }

        template<typename R>
        quat_t &operator-=(const quat_t &rhs)
        {
            x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w;
            return *this;
        }

        //
        // Multiplcation operators
        //

        // Scalar
        quat_t operator*(const value_t &rhs) const
        {
            return {x * rhs, y * rhs, z * rhs, w * rhs};
        }

        quat_t &operator*=(const value_t &rhs)
        {
            x *= rhs; y *= rhs; z *= rhs; w *= rhs;
            return *this;
        }

        friend quat_t operator*(const value_t &lhs, const quat_t &rhs)
        {
            return {lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w};
        }

        // Quaternion
        quat_t operator*(const quat_t &rhs) const
        {
            const value_t _w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const value_t _x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const value_t _y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const value_t _z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);
            return {_x, _y, _z, _w};
        }

        quat_t &operator*=(const quat_t &rhs)
        {
            const value_t _w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const value_t _x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const value_t _y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const value_t _z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);
            x = _x; y = _y; z = _z; w = _w;
            return *this;
        }

        //
        // Division operators
        //

        // Scalar
        quat_t operator/(const value_t &rhs) const
        {
            return {x / rhs, y / rhs, z / rhs, w / rhs};
        }

        quat_t &operator/=(const value_t &rhs)
        {
            x /= rhs; y /= rhs; z /= rhs; w /= rhs;
            return *this;
        }

        // Quaternion
        quat_t operator/(const quat_t &rhs) const
        {
            return *this * *rhs;
        }

        quat_t &operator/=(const quat_t &rhs)
        {
            return *this * *!rhs;
        }

        //
        // Equality operators
        //

        bool operator==(const quat_t &rhs) const
        {
            return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
        }
        template<typename R>
        bool operator!=(const quat_t &rhs) const
        {
            return x != rhs.x && y != rhs.y && z != rhs.z && w != rhs.w;
        }

        //
        // Cast operators
        //

        template<typename TO>
        explicit operator quat<TO>() const
        {
            return quat<TO>(*this);
        }

        //
        // GLM Functions
        //

        #ifdef CQL_HAS_GLM_HEADERS
        CQL_GLM_TEMPLATE
        explicit quat(const CQL_VEC4 &vec)
        : x(vec.x), y(vec.y), z(vec.z), w(vec.w)
        {}

        CQL_GLM_TEMPLATE
        explicit operator CQL_VEC4() const { return {x, y, z, w}; }

        CQL_GLM_TEMPLATE
        explicit operator CQL_VEC3() const { return {x, y, z}; }

        CQL_GLM_TEMPLATE
        CQL_VEC4 toEulerVector() const
        {
            const value_t htheta = std::atan2(std::sqrt((x * x) + (y * y) + (z * z)),w);
            const value_t denom = std::sin(htheta);
            CQL_VEC4 result = {0, 0, 0, value_t(2) * htheta};
            if (denom != value_t(0))
            {
                result.x = x / denom;
                result.y = y / denom;
                result.z = z / denom;
            }
            return result;
        }

        CQL_GLM_TEMPLATE
        static quat_t FromEulerVector(const CQL_VEC4 &vec)
        {
            quat_t result;
            const value_t w2 = vec.w / value_t(2);
            const value_t sw2 = std::sin(w2);
            result.x = vec.x * sw2;
            result.y = vec.y * sw2;
            result.z = vec.z * sw2;
            result.w = std::cos(w2);
            return result;
        }

        CQL_GLM_TEMPLATE
        CQL_VEC3 toEulerAngles() const
        {
            const GLMT sy = GLMT(2) * ((w * y) - (z * x));
            return {
                std::atan2(
                    GLMT(2) * ((w * x) + (y * z)),
                    GLMT(1) - (GLMT(2) * ((x * x) + (y * y)))
                ),
                std::fabs(sy) >= GLMT(1)
                    ? std::copysign(glm::pi<GLMT>() / GLMT(2), sy)
                    : std::asin(sy),
                std::atan2(
                    GLMT(2) * ((w * z ) + (x * y)),
                    GLMT(1) - (GLMT(2) * ((y * y) + (z * z)))
                )
            };
        }

        CQL_GLM_TEMPLATE
        static quat_t FromEulerAngles(const CQL_VEC3 &vec)
        {
            quat_t result;
            const value_t
                cx = std::cos(vec.x * value_t(0.5)),
                sx = std::sin(vec.x * value_t(0.5)),
                cy = std::cos(vec.y * value_t(0.5)),
                sy = std::sin(vec.y * value_t(0.5)),
                cz = std::cos(vec.z * value_t(0.5)),
                sz = std::sin(vec.z * value_t(0.5));
            result.x = (cx * sy * cz) - (sx * cy * sz);
            result.y = (cx * cy * sz) + (sx * sy * cz);
            result.z = (sx * cy * cz) - (cx * sy * sz);
            result.w = (cx * cy * cz) + (sx * sy * sz);
            return result;
        }

        CQL_GLM_TEMPLATE
        quat_t &rotate(const CQL_VEC4 &vec)
        {
            return *this *= FromEulerVector(vec);
        }

        CQL_GLM_TEMPLATE
        quat_t rotated(const CQL_VEC4 &vec) const
        {
            return *this * FromEulerVector(vec);
        }
        #endif // CQL_HAS_GLM_HEADERS
    };

    #ifdef CQL_HAS_GLM_HEADERS
    CQL_GLM_TEMPLATE
    quat<GLMT> FromVector(const CQL_VEC4 &vec)
    {
        return quat<GLMT>(vec);
    }

    CQL_GLM_TEMPLATE
    quat<GLMT> FromEulerVector(const CQL_VEC4 &vec)
    {
        return quat<GLMT>::FromEulerVector(vec);
    }

    CQL_GLM_TEMPLATE
    quat<GLMT> FromEulerAngles(const CQL_VEC3 &vec)
    {
        return quat<GLMT>::FromEulerAngles(vec);
    }
    #endif

    using quatf_t = quat<float>;
    using quatd_t = quat<double>;

    template<typename T>
    struct dquat
    {
        using value_t = T;
        using quat_t = quat<value_t>;
        using dquat_t = dquat<value_t>;
        quat_t real; // rotation
        quat_t dual; // translation

        dquat()
        : real(0, 0, 0, 1), dual(0, 0, 0, 0)
        {}

        dquat(const dquat_t &other)
        : real(other.real), dual(other.dual)
        {}

        template<typename R>
        explicit dquat(const dquat<R> &other)
        : real(other.real), dual(other.dual)
        {}

        dquat(const quat_t (&val)[2])
        : real(val[0]), dual(val[1])
        {}

        template<typename R>
        explicit dquat(const quat<R> (&val)[2])
        : real(val[0]), dual(val[1])
        {}

        dquat(const quat_t &r, const quat_t &d)
        : real(r), dual(d)
        {}

        template<typename R1, typename R2>
        explicit dquat(const quat<R1> &r, const quat<R2> &d)
        : real(r), dual(d)
        {}

        dquat(const value_t (&val)[8])
        : real(val[0], val[1], val[2], val[3]), dual(val[4], val[5], val[6], val[7])
        {}

        dquat(const value_t &rx, const value_t &ry,
              const value_t &rz, const value_t &rw,
              const value_t &dx, const value_t &dy,
              const value_t &dz, const value_t &dw)
        : real(rx, ry, rz, rw), dual(dx, dy, dz, dw)
        {}

        //
        // Dual quaternion operators
        //

        quat_t toTranslationQuat() const
        {
            quat_t rtn = value_t(2) * (dual * *real);
            rtn.w = value_t(0);
            return rtn;
        }

        static dquat_t FromTranslation(const quat_t &trans)
        {
            const quat_t tra(trans.x, trans.y, trans.z, value_t(0));
            const quat_t rea(0, 0, 0, 1);
            return {rea, value_t(0.5) * (tra * rea)};
        }

        quat_t transform(const quat_t &q) const
        {
            return (real * q * *real) + toTranslationQuat();
        }

        //
        // Conjugate operator
        //

        dquat_t operator*() const
        {
            return {*real, *dual};
        }

        //
        // Magnitude operator
        //

        value_t magnitude() const
        {
            return real.magnitude();
        }

        inline value_t operator~() const
        {
            return magnitude();
        }

        //
        // Normalise operator
        //

        dquat_t &normalise()
        {
            return *this *= (value_t(1) / magnitude());
        }

        dquat_t normalised() const
        {
            return *this * (value_t(1) / magnitude());
        }

        dquat_t operator!() const
        {
            return normalised();
        }

        //
        // Assignment operator
        //

        dquat_t &operator=(const dquat_t &rhs)
        {
            real = rhs.real; dual = rhs.dual;
            return *this;
        }

        //
        // Addition operators
        //

        dquat_t operator+(const dquat_t &rhs) const
        {
            return {real + rhs.real, dual + rhs.dual};
        }

        dquat_t &operator+=(const dquat_t &rhs)
        {
            real += rhs.real; dual += rhs.dual;
            return *this;
        }

        //
        // Subtraction operators
        //

        dquat_t operator-(const dquat_t &rhs) const
        {
            return {real - rhs.real, dual - rhs.dual};
        }

        dquat_t &operator-=(const dquat_t &rhs)
        {
            real -= rhs.real; dual -= rhs.dual;
            return *this;
        }

        //
        // Multiplcation operators
        //

        // Scalar
        dquat_t operator*(const value_t &rhs) const
        {
            return {real * rhs, dual * rhs};
        }

        dquat_t &operator*=(const value_t &rhs)
        {
            real *= rhs; dual *= rhs;
            return *this;
        }

        friend dquat_t operator*(const value_t &lhs, const dquat_t &rhs)
        {
            return {lhs * rhs.real, lhs * rhs.dual};
        }

        // Quaternion
        dquat_t operator*(const dquat_t &rhs) const
        {
            const quat_t d = value_t(0.5) * rhs.transform(toTranslationQuat());
            const quat_t r = real * rhs.real;
            return {r, d * r};
        }

        dquat_t &operator*=(const dquat_t &rhs)
        {
            const quat_t d = value_t(0.5) * rhs.transform(toTranslationQuat());
            real *= rhs.real;
            dual  = d * real;
            return *this;
        }

        //
        // Division operators
        //

        // Scalar
        dquat_t operator/(const value_t &rhs) const
        {
            return {real / rhs, dual / rhs};
        }

        dquat_t operator/=(const value_t &rhs)
        {
            real /= rhs; dual /= rhs;
            return *this;
        }

        // Quaternion
        dquat_t operator/(const dquat_t &rhs) const
        {
            return *this * *!rhs;
        }

        dquat_t &operator/=(const dquat_t &rhs)
        {
            return *this *= *!rhs;
        }

        //
        // Equality operators
        //

        bool operator==(const dquat_t &rhs) const
        {
            return real == rhs.real && dual == rhs.dual;
        }

        bool operator!=(const dquat_t &rhs) const
        {
            return real != rhs.real && dual != rhs.dual;
        }

        //
        // Cast operators
        //

        template<typename TO>
        explicit operator dquat<TO>() const
        {
            return dquat<TO>(*this);
        }

        //
        // GLM operators
        //

        #ifdef CQL_HAS_GLM_HEADERS
        CQL_GLM_TEMPLATE
        CQL_VEC4 toEulerVector() const
        {
            return real.toEulerVector();
        }

        CQL_GLM_TEMPLATE
        static dquat_t FromEulerVector(const CQL_VEC4 &vec)
        {
            return dquat{quat_t::FromEulerVector(vec), quat_t{0, 0, 0, 0}};
        }

        CQL_GLM_TEMPLATE
        CQL_VEC3 toEulerAngles() const
        {
            return real.toEulerAngles();
        }

        CQL_GLM_TEMPLATE
        static dquat_t FromEulerAngles(const CQL_VEC3 &vec)
        {
            return {quat_t::FromEulerAngles(vec), quat_t{0, 0, 0, 0}};
        }

        CQL_GLM_TEMPLATE
        static dquat_t FromTranslation(const CQL_VEC3 &vec)
        {
            const quat_t tra = {vec.x, vec.y, vec.z, value_t(0)};
            const quat_t rea(0, 0, 0, 1);
            return {rea, value_t(0.5) * (tra * rea)};
        }

        CQL_GLM_TEMPLATE
        CQL_VEC3 toTranslationVec() const
        {
            return GLMT(2) * (dual * (*real));
        }

        CQL_GLM_TEMPLATE
        CQL_VEC4 transform(const CQL_VEC4 &vec) const
        {
            return (real * quat_t(vec) * *real) + toTranslationQuat();
        }

        CQL_GLM_TEMPLATE
        CQL_MAT4 toTransform() const
        {
            const CQL_VEC3 trans = toTranslationVec<GLMT,GLMP>();
            const GLMT
                x = real.x,
                y = real.y,
                z = real.z,
                w = real.w,
                x2 = x * x,
                y2 = y * y,
                z2 = z * z,
                w2 = w * w,
                tx = 2 * x,
                ty = 2 * y,
                tz = 2 * z;

            return {
                (x2 + w2) - (y2 + z2),
                (tx * y)  + (tz * w),
                (tx * z)  - (ty * w),
                GLMT(0),

                (tx * y)  - (tz * w),
                (y2 + w2) - (x2 + z2),
                (ty * z)  + (tx * w),
                GLMT(0),

                (tx * z)  + (ty * w),
                (ty * z)  - (tx * w),
                (z2 + w2) - (x2 + y2),
                GLMT(0),

                trans.x,
                trans.y,
                trans.z,
                GLMT(1)
            };
            // This is a Euler-Rodrigues transform
            // https://en.wikipedia.org/wiki/Euler-Rodrigues_formula
        }
        #endif // CQL_HAS_GLM_HEADERS
    };

    #ifdef CQL_HAS_GLM_HEADERS
    CQL_GLM_TEMPLATE
    dquat<GLMT> FromVectors(const CQL_VEC4 &real, const CQL_VEC4 &dual)
    {
        return dquat<GLMT>(real, dual);
    }

    CQL_GLM_TEMPLATE
    dquat<GLMT> FromTransaction(const CQL_VEC3 &vec)
    {
        return dquat<GLMT>::FromTranslation(vec);
    }
    #endif

    using dquatf_t = dquat<float>;
    using dquatd_t = dquat<double>;
}

#endif // CQL_QUATERNION_H
