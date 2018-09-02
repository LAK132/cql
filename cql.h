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

#include <cmath>
#include <utility>
#include <type_traits>

#ifndef CQL_NO_GLM
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>
#include <glm/vec3.hpp>
#define CQL_GLM_TEMPLATE template<typename GLMT = float, glm::precision GLMP = glm::defaultp>
#define CQL_MAT4 glm::tmat4x4<GLMT,GLMP>
#define CQL_VEC4 glm::tvec4<GLMT,GLMP>
#define CQL_VEC3 glm::tvec3<GLMT,GLMP>
#endif // CQL_NO_GLM

#ifndef CQL_QUATERNION_H
#define CQL_QUATERNION_H

namespace cql
{
    using std::enable_if;
    template<bool B, typename T>
    using enable_if_t = typename enable_if<B, T>::type;
    using std::is_same;
    using std::sqrt;
    using std::declval;
    using std::decay;

    template<typename L, typename R>
    using bigger_t = decltype(declval<typename decay<L>::type>() * declval<typename decay<R>::type>());

    template<typename T> struct quat;
    template<typename T> struct dquat;

    template<typename T, typename T2 = void>
    struct _is_quat;
    template<typename T>
    struct _is_quat<quat<T>> { static constexpr bool value = true; };
    template<typename T>
    struct _is_quat<T> { static constexpr bool value = false; };
    template<typename T>
    using is_quat = _is_quat<typename decay<T>::type>;

    template<typename T, typename T2 = void>
    struct _is_dquat;
    template<typename T>
    struct _is_dquat<dquat<T>> { static constexpr bool value = true; };
    template<typename T>
    struct _is_dquat<T> { static constexpr bool value = false; };
    template<typename T>
    using is_dquat = _is_dquat<typename decay<T>::type>;

    template<typename L, typename R, typename T = void>
    struct has_rhs_mult { static constexpr bool value = false; };
    template<typename L, typename R>
    struct has_rhs_mult<L, R, decltype(declval<L>().operator*(declval<R>()))> { static constexpr bool value = true; };

    template<typename T>
    struct quat
    {
        using value_t = T;
        using type_t = quat<value_t>;
        value_t x;
        value_t y;
        value_t z;
        value_t w;

        template<typename R>
        quat(const quat<R> &rhs) : x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) {}
        template<typename R>
        quat(quat<R> &&rhs) : x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) {}
        quat(value_t val = value_t(0)) : x(val), y(val), z(val), w(val) {}
        quat(const value_t (&val)[4]) : x(val[0]), y(val[1]), z(val[2]), w(val[3]) {}
        quat(value_t X, value_t Y, value_t Z, value_t W) : x(X), y(Y), z(Z), w(W) {}

        // 
        // Quaternion operators
        //

        // conjugate operator
        type_t operator*() const
        {
            return {-x, -y, -z, w};
        }
        // magnitude operator
        value_t operator~() const
        {
            return sqrt((x*x)+(y*y)+(z*z)+(w*w));
        }
        value_t magnitude() const
        {
            return sqrt((x*x)+(y*y)+(z*z)+(w*w));
        }
        // norm operator
        type_t operator!() const
        {
            return *this * (value_t(1) / magnitude());
        }
        type_t norm() const
        {
            return *this * (value_t(1) / magnitude());
        }
        type_t& normalize()
        {
            return *this *= value_t(1) / magnitude();
        }

        //
        // Assignment operator
        //

        template<typename R>
        type_t& operator=(const quat<R> &rhs)
        {
            x = rhs.x; y = rhs.y; z = rhs.z; w = rhs.w;
            return *this;
        }
        template<typename R>
        type_t& operator=(quat<R> &&rhs)
        {
            x = rhs.x; y = rhs.y; z = rhs.z; w = rhs.w;
            return *this;
        }

        //
        // Addition operators
        //

        template<typename R>
        quat<bigger_t<value_t, R>> operator+(const quat<R> &rhs) const
        {
            return {x+rhs.x, y+rhs.y, z+rhs.z, w+rhs.w};
        }
        template<typename R>
        quat<bigger_t<value_t, R>> operator+(quat<R> &&rhs) const
        {
            return {x+rhs.x, y+rhs.y, z+rhs.z, w+rhs.w};
        }
        template<typename R>
        type_t& operator+=(const quat<R> &rhs)
        {
            x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w;
            return *this;
        }
        template<typename R>
        type_t& operator+=(quat<R> &&rhs)
        {
            x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w;
            return *this;
        }
        
        //
        // Subtraction operators
        //

        template<typename R>
        quat<bigger_t<value_t, R>> operator-(const quat<R> &rhs) const
        {
            return {x-rhs.x, y-rhs.y, z-rhs.z, w-rhs.w};
        }
        template<typename R>
        quat<bigger_t<value_t, R>> operator-(quat<R> &&rhs) const
        {
            return {x-rhs.x, y-rhs.y, z-rhs.z, w-rhs.w};
        }
        template<typename R>
        type_t& operator-=(const quat<R> &rhs)
        {
            x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w;
            return *this;
        }
        template<typename R>
        type_t& operator-=(quat<R> &&rhs)
        {
            x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w;
            return *this;
        }
        
        //
        // Multiplcation operators
        //

        // Scalar
        template<typename R>
        enable_if_t<!is_quat<R>::value, quat<bigger_t<value_t, R>>> 
        operator*(const R &rhs) const
        {
            return {x*rhs, y*rhs, z*rhs, w*rhs};
        }
        template<typename R>
        enable_if_t<!is_quat<R>::value, quat<bigger_t<value_t, R>>> 
        operator*(R &&rhs) const
        {
            return {x*rhs, y*rhs, z*rhs, w*rhs};
        }
        template<typename R>
        enable_if_t<!is_quat<R>::value, type_t&> 
        operator*=(const R &rhs)
        {
            x *= rhs; y *= rhs; z *= rhs; w *= rhs;
            return *this;
        }
        template<typename R>
        enable_if_t<!is_quat<R>::value, type_t&> 
        operator*=(R &&rhs)
        {
            x *= rhs; y *= rhs; z *= rhs; w *= rhs;
            return *this;
        }
        template<typename L> friend 
        enable_if_t<!has_rhs_mult<L, type_t>::value, quat<bigger_t<L, value_t>>> 
        operator*(L lhs, const type_t &rhs)
        {
            return {lhs*rhs.x, lhs*rhs.y, lhs*rhs.z, lhs*rhs.w};
        }
        template<typename L> friend 
        enable_if_t<!has_rhs_mult<L, type_t>::value, quat<bigger_t<L, value_t>>> 
        operator*(L lhs, type_t &&rhs)
        {
            return {lhs*rhs.x, lhs*rhs.y, lhs*rhs.z, lhs*rhs.w};
        }
        // Quaternion
        template<typename R>
        quat<bigger_t<value_t, R>> operator*(const quat<R> &rhs) const
        {
            using _val_t = bigger_t<value_t, R>;
            const _val_t &_w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const _val_t &_x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const _val_t &_y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const _val_t &_z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);
            return {_x, _y, _z, _w};
        }
        template<typename R>
        quat<bigger_t<value_t, R>> operator*(quat<R> &&rhs) const
        {
            using _val_t = bigger_t<value_t, R>;
            const _val_t &_w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const _val_t &_x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const _val_t &_y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const _val_t &_z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);
            return {_x, _y, _z, _w};
        }
        template<typename R>
        type_t& operator*=(const quat<R> &rhs)
        {
            const value_t &_w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const value_t &_x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const value_t &_y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const value_t &_z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);
            x = _x; y = _y; z = _z; w = _w;
            return *this;
        }
        template<typename R>
        type_t& operator*=(quat<R> &&rhs)
        {
            const value_t &_w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const value_t &_x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const value_t &_y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const value_t &_z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);
            x = _x; y = _y; z = _z; w = _w;
            return *this;
        }

        //
        // Division operators
        //

        // Scalar
        template<typename R>
        enable_if_t<!is_quat<R>::value, quat<bigger_t<value_t, R>>> 
        operator/(const R &rhs) const
        {
            return {x/rhs, y/rhs, z/rhs, w/rhs};
        }
        template<typename R>
        enable_if_t<!is_quat<R>::value, quat<bigger_t<value_t, R>>> 
        operator/(R &&rhs) const
        {
            return {x/rhs, y/rhs, z/rhs, w/rhs};
        }
        template<typename R>
        enable_if_t<!is_quat<R>::value, type_t&> 
        operator/=(const R &rhs)
        {
            x /= rhs; y /= rhs; z /= rhs; w /= rhs;
            return *this;
        }
        template<typename R>
        enable_if_t<!is_quat<R>::value, type_t&> 
        operator/=(R &&rhs)
        {
            x /= rhs; y /= rhs; z /= rhs; w /= rhs;
            return *this;
        }
        // Quaternion
        template<typename R>
        quat<bigger_t<value_t, R>> operator/(const quat<R> &rhs) const
        {
            return *this * *rhs;
        }
        template<typename R>
        quat<bigger_t<value_t, R>> operator/(quat<R> &&rhs) const
        {
            return *this * *rhs;
        }
        template<typename R>
        type_t& operator/=(const quat<R> &rhs)
        {
            return *this * *!rhs;
        }
        template<typename R>
        type_t& operator/=(quat<R> &&rhs)
        {
            return *this * *!rhs;
        }

        //
        // Equality operators
        //

        template<typename R>
        bool operator==(const quat<R> &rhs) const
        {
            return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
        }
        template<typename R>
        bool operator==(quat<R> &&rhs) const
        {
            return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
        }
        template<typename R>
        bool operator!=(const quat<R> &rhs) const
        {
            return x != rhs.x && y != rhs.y && z != rhs.z && w != rhs.w;
        }
        template<typename R>
        bool operator!=(quat<R> &&rhs) const
        {
            return x != rhs.x && y != rhs.y && z != rhs.z && w != rhs.w;
        }

        //
        // GLM Functions
        //

        #ifndef CQL_NO_GLM
        CQL_GLM_TEMPLATE
        quat(const CQL_VEC4 &vec) : x(vec.x), y(vec.y), z(vec.z), w(vec.w) {}
        CQL_GLM_TEMPLATE
        quat(CQL_VEC4 &&vec) : x(vec.x), y(vec.y), z(vec.z), w(vec.w) {}
        CQL_GLM_TEMPLATE
        operator CQL_VEC4() const { return {x, y, z, w}; }
        CQL_GLM_TEMPLATE
        operator CQL_VEC3() const { return {x, y, z}; }
        CQL_GLM_TEMPLATE
        CQL_VEC4 toEulerVector() const 
        { 
            const value_t &htheta = atan2(sqrt((x*x)+(y*y)+(z*z)),w);
            const value_t &denom = sin(htheta);
            CQL_VEC4 rtn = {0, 0, 0, value_t(2) * htheta};
            if (denom != value_t(0))
            {
                rtn.x = x / denom;
                rtn.y = y / denom;
                rtn.z = z / denom;
            }
            return rtn;
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerVector(const CQL_VEC4 &vec) 
        { 
            const value_t &w2 = vec.w / value_t(2);
            const value_t &sw2 = sin(w2);
            x = vec.x * sw2;
            y = vec.y * sw2;
            z = vec.z * sw2;
            w = cos(w2);
            return *this;
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerVector(CQL_VEC4 &&vec) 
        { 
            const value_t &w2 = vec.w / value_t(2);
            const value_t &sw2 = sin(w2);
            x = vec.x * sw2;
            y = vec.y * sw2;
            z = vec.z * sw2;
            w = cos(w2);
            return *this;
        }
        CQL_GLM_TEMPLATE
        CQL_VEC3 toEulerAngles() const
        {
            const GLMT &sy = GLMT(2) * ((w * y) - (z * x));
            return {
                atan2(
                    GLMT(2) * ((w * x) + (y * z)),
                    GLMT(1) - (GLMT(2) * ((x * x) + (y * y)))
                ),
                fabs(sy) >= GLMT(1) 
                    ? copysign(glm::pi<GLMT>() / GLMT(2), sy) 
                    : asin(sy),
                atan2(
                    GLMT(2) * ((w * z ) + (x * y)), 
                    GLMT(1) - (GLMT(2) * ((y * y) + (z * z)))
                )
            };
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerAngles(const CQL_VEC3 &vec)
        {
            const value_t 
                &cx = cos(vec.x * value_t(0.5)),
                &sx = sin(vec.x * value_t(0.5)),
                &cy = cos(vec.y * value_t(0.5)),
                &sy = sin(vec.y * value_t(0.5)),
                &cz = cos(vec.z * value_t(0.5)),
                &sz = sin(vec.z * value_t(0.5));
            x = (cx * sy * cz) - (sx * cy * sz);
            y = (cx * cy * sz) + (sx * sy * cz);
            z = (sx * cy * cz) - (cx * sy * sz);
            w = (cx * cy * cz) + (sx * sy * sz);
            return *this;
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerAngles(CQL_VEC3 &&vec)
        {
            const value_t 
                &cx = cos(vec.x * value_t(0.5)),
                &sx = sin(vec.x * value_t(0.5)),
                &cy = cos(vec.y * value_t(0.5)),
                &sy = sin(vec.y * value_t(0.5)),
                &cz = cos(vec.z * value_t(0.5)),
                &sz = sin(vec.z * value_t(0.5));
            x = (cx * sy * cz) - (sx * cy * sz);
            y = (cx * cy * sz) + (sx * sy * cz);
            z = (sx * cy * cz) - (cx * sy * sz);
            w = (cx * cy * cz) + (sx * sy * sz);
            return *this;
        }
        CQL_GLM_TEMPLATE
        type_t& rotate(const CQL_VEC4 &vec)
        {
            return *this * type_t().fromEulerVector(vec);
        }
        CQL_GLM_TEMPLATE
        type_t& rotate(CQL_VEC4 &&vec)
        {
            return *this * type_t().fromEulerVector(vec);
        }
        #endif // CQL_NO_GLM
    };

    using quatf_t = quat<float>;
    using quatd_t = quat<double>;

    template<typename T>
    struct dquat
    {
        using value_t = T;
        using quat_t = quat<value_t>;
        using type_t = dquat<value_t>;
        quat_t real; // rotation
        quat_t dual; // translation

        dquat() : real(0.0f, 0.0f, 0.0f, 1.0f), dual(0.0f, 0.0f, 0.0f, 0.0f) {}
        template<typename R>
        dquat(const dquat<R> &other) : real(other.real), dual(other.dual) {}
        template<typename R>
        dquat(dquat<R> &&other) : real(other.real), dual(other.dual) {}
        template<typename R>
        dquat(const quat<R> (&val)[2]) : real(val[0]), dual(val[1]) {}
        template<typename R1, typename R2>
        dquat(const quat<R1> &r, const quat<R2> &d) : real(r), dual(d) {}
        template<typename R1, typename R2>
        dquat(quat<R1> &&r, const quat<R2> &d) : real(r), dual(d) {}
        template<typename R1, typename R2>
        dquat(const quat<R1> &r, quat<R2> &&d) : real(r), dual(d) {}
        template<typename R1, typename R2>
        dquat(quat<R1> &&r, quat<R2> &&d) : real(r), dual(d) {}
        dquat(const value_t (&val)[8]) : real(val[0], val[1], val[2], val[3]), dual(val[4], val[5], val[6], val[7]) {}
        dquat(value_t rx, value_t ry, value_t rz, value_t rw, value_t dx, value_t dy, value_t dz, value_t dw) : real(rx, ry, rz, rw), dual(dx, dy, dz, dw) {}

        //
        // Dual quaternion operators
        //

        quat_t toTranslationQuat() const 
        { 
            quat_t rtn = value_t(2) * (dual * *real);
            rtn.w = value_t(0);
            return rtn; 
        }
        template<typename R>
        type_t fromTranslation(const quat<R> &trans)
        {
            const quat_t &tra = {trans.x, trans.y, trans.z, value_t(0)};
            dual = value_t(0.5) * (tra * real);
            return *this;
        }
        template<typename R>
        type_t fromTranslation(quat<R> &&trans)
        {
            trans.z = R(0);
            dual = value_t(0.5) * (trans * real);
            return *this;
        }
        template<typename R>
        quat<R> transform(const quat<R> &q) const
        {
            return (real * q * *real) + toTranslationQuat();
        }
        template<typename R>
        quat<R> transform(quat<R> &&q) const
        {
            return (real * q * *real) + toTranslationQuat();
        }

        // conjugate operator
        type_t operator*() const
        {
            return {*real, *dual};
        }
        // magnitude operator
        value_t operator~() const
        {
            return ~real;
        }
        value_t magnitude() const
        {
            return real.magnitude();
        }
        // norm operator
        type_t operator!() const
        {
            return *this * (value_t(1) / real.magnitude());
        }
        type_t norm() const
        {
            return *this * (value_t(1) / real.magnitude());
        }
        type_t& normalize()
        {
            *this *= (value_t(1) / real.magnitude());
            return *this;
        }

        //
        // Assignment operator
        //

        template<typename R>
        type_t& operator=(const dquat<R> &rhs)
        {
            real = rhs.real; dual = rhs.dual;
            return *this;
        }
        template<typename R>
        type_t& operator=(dquat<R> &&rhs)
        {
            real = rhs.real; dual = rhs.dual;
            return *this;
        }

        //
        // Addition operators
        //

        template<typename R>
        dquat<bigger_t<value_t, R>> operator+(const dquat<R> &rhs) const
        {
            return {real + rhs.real, dual + rhs.dual};
        }
        template<typename R>
        dquat<bigger_t<value_t, R>> operator+(dquat<R> &&rhs) const
        {
            return {real + rhs.real, dual + rhs.dual};
        }
        template<typename R>
        type_t& operator+=(const dquat<R> &rhs)
        {
            real += rhs.real; dual += rhs.dual;
            return *this;
        }
        template<typename R>
        type_t& operator+=(dquat<R> &&rhs)
        {
            real += rhs.real; dual += rhs.dual;
            return *this;
        }
        
        //
        // Subtraction operators
        //

        template<typename R>
        dquat<bigger_t<value_t, R>> operator-(const dquat<R> &rhs) const
        {
            return {real - rhs.real, dual - rhs.dual};
        }
        template<typename R>
        dquat<bigger_t<value_t, R>> operator-(dquat<R> &&rhs) const
        {
            return {real - rhs.real, dual - rhs.dual};
        }
        template<typename R>
        type_t& operator-=(const dquat<R> &rhs)
        {
            real -= rhs.real; dual -= rhs.dual;
            return *this;
        }
        template<typename R>
        type_t& operator-=(dquat<R> &&rhs)
        {
            real -= rhs.real; dual -= rhs.dual;
            return *this;
        }
        
        //
        // Multiplcation operators
        //

        // Scalar
        template<typename R>
        enable_if_t<!is_dquat<R>::value, dquat<bigger_t<value_t, R>>>
        operator*(const R &rhs) const
        {
            return {real*rhs, dual*rhs};
        }
        template<typename R>
        enable_if_t<!is_dquat<R>::value, dquat<bigger_t<value_t, R>>>
        operator*(R &&rhs) const
        {
            return {real*rhs, dual*rhs};
        }
        template<typename R>
        enable_if_t<!is_dquat<R>::value, type_t&>
        operator*=(const R &rhs)
        {
            real *= rhs; dual *= rhs;
            return *this;
        }
        template<typename R>
        enable_if_t<!is_dquat<R>::value, type_t&>
        operator*=(R &&rhs)
        {
            real *= rhs; dual *= rhs;
            return *this;
        }
        template<typename L> friend 
        enable_if_t<!has_rhs_mult<L, type_t>::value, dquat<bigger_t<L, value_t>>> 
        operator*(L lhs, const type_t &rhs)
        {
            return {lhs*rhs.real, lhs*rhs.dual};
        }
        template<typename L> friend 
        enable_if_t<!has_rhs_mult<L, type_t>::value, dquat<bigger_t<L, value_t>>> 
        operator*(L lhs, type_t &&rhs)
        {
            return {lhs*rhs.real, lhs*rhs.dual};
        }
        // Quaternion
        template<typename R>
        dquat<bigger_t<value_t, R>> operator*(const dquat<R> &rhs) const
        {
            using _val_t = bigger_t<value_t, R>;
            using _rtn_t = quat<_val_t>;
            const _rtn_t &d = _val_t(0.5) * rhs.transform(toTranslationQuat());
            const _rtn_t &r = real * rhs.real;
            return {r, d * r};
        }
        template<typename R>
        dquat<bigger_t<value_t, R>> operator*(dquat<R> &&rhs) const
        {
            using _val_t = bigger_t<value_t, R>;
            using _rtn_t = quat<_val_t>;
            const _rtn_t &d = _val_t(0.5) * rhs.transform(toTranslationQuat());
            const _rtn_t &r = real * rhs.real;
            return {r, d * r};
        }
        template<typename R>
        type_t& operator*=(const dquat<R> &rhs)
        {
            const quat_t &d = rhs.transform(toTranslationQuat()) * value_t(0.5);
            real *= rhs.real;
            dual  = d * real;
            return *this;
        }
        template<typename R>
        type_t& operator*=(dquat<R> &&rhs)
        {
            const quat_t &d = rhs.transform(toTranslationQuat()) * value_t(0.5);
            real *= rhs.real;
            dual  = d * real;
            return *this;
        }

        //
        // Division operators
        //

        // Scalar
        template<typename R>
        enable_if_t<!is_dquat<R>::value, dquat<bigger_t<value_t, R>>> 
        operator/(const R &rhs) const
        {
            return {real/rhs, dual/rhs};
        }
        template<typename R>
        enable_if_t<!is_dquat<R>::value, dquat<bigger_t<value_t, R>>> 
        operator/(R &&rhs) const
        {
            return {real/rhs, dual/rhs};
        }
        template<typename R>
        enable_if_t<!is_dquat<R>::value, type_t&> 
        operator/=(const R &rhs)
        {
            real /= rhs; dual /= rhs;
            return *this;
        }
        template<typename R>
        enable_if_t<!is_dquat<R>::value, type_t&> 
        operator/=(R &&rhs)
        {
            real /= rhs; dual /= rhs;
            return *this;
        }
        // Quaternion
        template<typename R>
        dquat<bigger_t<value_t, R>> operator/(const dquat<R> &rhs) const
        {
            return *this * *!rhs;
        }
        template<typename R>
        dquat<bigger_t<value_t, R>> operator/(dquat<R> &&rhs) const
        {
            return *this * *!rhs;
        }
        template<typename R>
        type_t& operator/=(const dquat<R> &rhs)
        {
            return *this *= *!rhs;
        }
        template<typename R>
        type_t& operator/=(dquat<R> &&rhs)
        {
            return *this *= *!rhs;
        }

        //
        // Equality operators
        //

        template<typename R>
        bool operator==(const dquat<R> &rhs) const
        {
            return real == rhs.real && dual == rhs.dual;
        }
        template<typename R>
        bool operator==(dquat<R> &&rhs) const
        {
            return real == rhs.real && dual == rhs.dual;
        }
        template<typename R>
        bool operator!=(const dquat<R> &rhs) const
        {
            return real != rhs.real && dual != rhs.dual;
        }
        template<typename R>
        bool operator!=(dquat<R> &&rhs) const
        {
            return real != rhs.real && dual != rhs.dual;
        }
        
        //
        // GLM operators
        //

        #ifndef CQL_NO_GLM
        CQL_GLM_TEMPLATE
        CQL_VEC4 toEulerVector() const 
        { 
            return real.toEulerVector();
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerVector(const CQL_VEC4 &vec) 
        { 
            real.fromEulerVector(vec);
            return *this;
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerVector(CQL_VEC4 &&vec) 
        { 
            real.fromEulerVector(vec);
            return *this;
        }
        CQL_GLM_TEMPLATE
        CQL_VEC3 toEulerAngles() const
        {
            return real.toEulerAngles();
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerAngles(const CQL_VEC3 &vec)
        {
            real.fromEulerAngles(vec);
            return *this;
        }
        CQL_GLM_TEMPLATE
        type_t& fromEulerAngles(CQL_VEC3 &&vec)
        {
            real.fromEulerAngles(vec);
            return *this;
        }
        CQL_GLM_TEMPLATE
        type_t& fromTranslation(const CQL_VEC3 &vec) 
        { 
            const quat_t &trans = {vec.x, vec.y, vec.z, value_t(0)};
            dual = value_t(0.5) * (trans * real);
            return *this;
        }
        CQL_GLM_TEMPLATE
        type_t& fromTranslation(CQL_VEC3 &&vec) 
        { 
            const quat_t &trans = {vec.x, vec.y, vec.z, value_t(0)};
            dual = value_t(0.5) * (trans * real);
            return *this;
        }
        CQL_GLM_TEMPLATE
        CQL_VEC3 toTranslationVec() const 
        { 
            return GLMT(2) * (dual * (*real)); 
        }
        CQL_GLM_TEMPLATE
        CQL_VEC4 transform(const CQL_VEC4 &vec) const
        {
            return (real * ((quat_t)vec) * *real) + toTranslationQuat();
        }
        CQL_GLM_TEMPLATE
        CQL_VEC4 transform(CQL_VEC4 &&vec) const
        {
            return (real * ((quat_t)vec) * *real) + toTranslationQuat();
        }
        CQL_GLM_TEMPLATE
        CQL_MAT4 toTransform() const 
        {
            const CQL_VEC3 &trans = toTranslationVec<GLMT,GLMP>();
            const GLMT 
                &x = real.x, 
                &y = real.y, 
                &z = real.z, 
                &w = real.w,
                &x2 = x*x,
                &y2 = y*y,
                &z2 = z*z,
                &w2 = w*w,
                &tx = 2*x,
                &ty = 2*y,
                &tz = 2*z;
            return {
                (x2 + w2) - (y2 + z2),
                (tx*y) + (tz*w),
                (tx*z) - (ty*w),
                GLMT(0),
                (tx*y) - (tz*w),
                (y2 + w2) - (x2 + z2),
                (ty*z) + (tx*w),
                GLMT(0),
                (tx*z) + (ty*w),
                (ty*z) - (tx*w),
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
        #endif // CQL_NO_GLM
    };

    using dquatf_t = dquat<float>;
    using dquatd_t = dquat<double>;
}

#endif // CQL_QUATERNION_H