/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef EASY3D_CORE_VECTOR_H
#define EASY3D_CORE_VECTOR_H

#include <vector>
#include <iostream>

namespace easy3d {

    class Vector2D;
    class Vector3D;
    class Vector4D;


    // -----------------------------------------------------------------------------------------------------------


    /// A class for N dimensional vectors.
    class Vector {
    public:
        /// Default constructor for an n-dimensional vector. All elements will be initialized to zero.
        Vector(size_t n);

        /// Constructs an n-dimensional vector from a scalar number \p s. All elements will be initialized to this value.
        Vector(size_t n, const double &s);

        /// Constructs an n-dimensional vector from another vector of the same dimension/size.
        Vector(const Vector &rhs);

        /// Constructs an n-dimensional vector (here n=2) from a 2D vector.
        Vector(const Vector2D &rhs);

        /// Constructs an n-dimensional vector (here n=3) from a 3D vector.
        Vector(const Vector3D &rhs);

        /// Constructs an n-dimensional vector (here n=4) from a 4D vector.
        Vector(const Vector4D &rhs);

        /// Constructs a vector from an array of values.
        /// \param rhs The array
        /// \param n The size of the array
        template<typename FT>
        Vector(size_t n, const FT *rhs);

        /// Constructs an n-dimensional vector from an std::vector
        template<typename FT>
        Vector(const std::vector<FT> &rhs);

        /// Assignment operator. It assigns the value of this vector from another vector.
        Vector &operator=(const Vector &rhs);

        /// Returns the dimension/size of this vector.
        size_t dimension() const;

        /// Returns the dimension/size of this vector.
        size_t size() const;

        /// Changes the size of the vector
        /// \attention If the size is made larger, the new values remain uninitialized.
        void resize(size_t n);

        /// Returns the memory address of the vector.
        double *data();

        /// Returns the constant memory address of the vector.
        const double *data() const;

        /// Returns the i_th element of this vector.
        double &operator[](size_t i);

        /// Returns the i_th element of this vector.
        const double &operator[](size_t i) const;

        /// Returns the squared length of this vector.
        double length2() const;

        /// Returns the length of this vector.
        double length() const;

        /// Returns the norm (i.e., length/magnitude0 of this vector.
        double norm() const;

        /// Returns the squared Euclidean distance to another vector.
        double distance2(const Vector &rhs) const;

        /// Normalizes this vector.
        Vector &normalize();

        /// Compound addition with another vector.
        Vector &operator+=(const Vector &v);

        /// Compound subtraction with another vector.
        Vector &operator-=(const Vector &v);

        /// Compound component-wise multiplication with another vector.
        Vector &operator*=(const Vector &v);

        /// Compound component-wise division with another vector.
        Vector &operator/=(const Vector &v);

        /// Compound vector-scalar multiplication.
        template<class FT>
        Vector &operator*=(FT s);

        /// Compound vector-scalar division.
        template<class T2>
        Vector &operator/=(T2 s);

        /// Vector-scalar multiplication.
        Vector operator*(double s) const;

        /// Vector-scalar division.
        template<class T2>
        Vector operator/(T2 s) const;

    protected:
        std::vector<double> data_;
    };


    // -----------------------------------------------------------------------------------------------------------


    /// A class for 2D vectors and points
    class Vector2D : public Vector {
    public:
        /// Construct a 2-dimensional vector.
        Vector2D(double x = 0, double y = 0);

        /// Construct a 2D vector from a general vector.
        /// \attention the size of v must >= 2
        Vector2D(const Vector &v);

        /// Get the x coordinate
        double &x();
        /// Get the x coordinate
        const double &x() const;

        /// Get the y coordinate
        double &y();
        /// Get the y coordinate
        const double &y() const;

        /// Get its Homogeneous coordinates
        Vector3D homogeneous() const;
    };


    // -----------------------------------------------------------------------------------------------------------


    /// A class for 3D vectors and points
    class Vector3D : public Vector {
    public:
        /// Construct a 3-dimensional vector.
        Vector3D(double x = 0, double y = 0, double z = 0);

        /// Construct a 3D vector from a general vector.
        /// \attention the size of v must >= 3
        Vector3D(const Vector &v);

        /// Get the x coordinate
        double &x();
        /// Get the x coordinate
        const double &x() const;

        /// Get the y coordinate
        double &y();
        /// Get the y coordinate
        const double &y() const;

        /// Get the z coordinate
        double &z();
        /// Get the z coordinate
        const double &z() const;

        /// Get its Homogeneous coordinates
        Vector4D homogeneous() const;

        /// Get its Cartesian coordinates (treating this vector as Homogeneous coordinates)
        Vector2D cartesian() const;
    };


    // -----------------------------------------------------------------------------------------------------------


    /// A class for 4D vectors and points
    class Vector4D : public Vector {
    public:
        /// Construct a 4-dimensional vector.
        Vector4D(double x = 0, double y = 0, double z = 0, double w = 0);

        /// Construct a 4D vector from a general vector.
        /// \attention the size of v must >= 4
        Vector4D(const Vector &v);

        /// Get the x coordinate
        double &x();
        /// Get the x coordinate
        const double &x() const;

        /// Get the y coordinate
        double &y();
        /// Get the y coordinate
        const double &y() const;

        /// Get the z coordinate
        double &z();
        /// Get the z coordinate
        const double &z() const;

        /// Get the w coordinate
        double &w();
        /// Get the w coordinate
        const double &w() const;

        /// Get its Cartesian coordinates (treating this vector as Homogeneous coordinates)
        Vector3D cartesian() const;
    };


    // ----------------------- Global functions for vectors --------------------

    /// Computes the dot product of two vectors
    double dot(const Vector &v1, const Vector &v2);

    /// Computes the 'negative' vector
    Vector operator-(const Vector &v1);

    /// Computes the scalar-vector product
    Vector operator*(double s, const Vector &v);

    /// Computes the addition of two vectors
    Vector operator+(const Vector &v1, const Vector &v2);

    /// Computes the subtraction of two vectors
    Vector operator-(const Vector &v1, const Vector &v2);

    /// Computes the length/magnitude of a vector
    double length(const Vector &v);

    /// Computes the length/magnitude of a vector
    double norm(const Vector &v);

    /// Computes the squared length/magnitude of a vector
    double length2(const Vector &v);

    /// Computes the distance between two vectors/points
    double distance(const Vector &v1, const Vector &v2);

    /// Computes the squared distance between two vectors/points
    double distance2(const Vector &v1, const Vector &v2);

    /// Computes and returns the normalized vector (Note: the input vector is not modified).
    Vector normalize(const Vector &v);

    /// linear interpolation between between two vectors (x and y).
    /// The return value is computed as (1 − w) * v1 + w * v2.
    Vector mix(const Vector &v1, const Vector &v2, double w);

    /// overload for std::stream output
    std::ostream &operator<<(std::ostream &out, const Vector &v);

    /// overload for std::stream input
    std::istream &operator>>(std::istream &in, Vector &v);
}


#include <cassert>
#include <cmath>
#include <cfloat>
#include <limits>


namespace easy3d {

    // ----------------------- Implementation of n-dimensional vectors --------------------

    inline Vector::Vector(size_t n) {
        data_.resize(n, 0);
    }

    inline Vector::Vector(size_t n, const double &s) {
        data_.resize(n, s);
    }

    inline Vector::Vector(const Vector &rhs) {
        data_ = rhs.data_;
    }

    inline Vector::Vector(const Vector2D &rhs) {
        data_ = rhs.data_;
    }

    inline Vector::Vector(const Vector3D &rhs) {
        data_ = rhs.data_;
    }

    inline Vector::Vector(const Vector4D &rhs) {
        data_ = rhs.data_;
    }

    template<typename FT>
    inline Vector::Vector(size_t n, const FT *rhs) {
        data_ = std::vector<double>(n);
        for (std::size_t i = 0; i < n; ++i)
            data_[i] = rhs[i];
    }

    template<typename FT>
    inline Vector::Vector(const std::vector<FT> &rhs) {
        data_ = std::vector<double>(rhs.size());
        for (std::size_t i = 0; i < rhs.size(); ++i)
            data_[i] = rhs[i];
    }

    inline Vector &Vector::operator=(const Vector &rhs) {
        data_ = rhs.data_;
        return *this;
    }

    inline size_t Vector::dimension() const { return data_.size(); }

    inline size_t Vector::size() const { return dimension(); }

    inline void Vector::resize(size_t n) {
        data_.resize(n);
    }

    inline double *Vector::data() { return data_.data(); }

    inline const double *Vector::data() const { return data_.data(); }

    inline double &Vector::operator[](size_t i) { return data_[i]; }

    inline const double &Vector::operator[](size_t i) const { return data_[i]; }

    inline double Vector::length2() const {
        double result(0);
        for (size_t i = 0; i < size(); i++) {
            result += data_[i] * data_[i];
        }
        return result;
    }

    inline double Vector::length() const {
        return sqrt(length2());
    }

    inline double Vector::norm() const {
        return length();
    }

    inline double Vector::distance2(const Vector &rhs) const {
        double result(0);
        for (size_t i = 0; i < size(); i++) {
            double val = rhs.data_[i] - data_[i];
            result += val * val;
        }
        return result;
    }

    inline Vector &Vector::normalize() {
        double s = length();
        s = (s > std::numeric_limits<double>::min()) ? 1.0 / s : 0.0;
        *this *= s;
        return *this;
    }

    inline Vector &Vector::operator+=(const Vector &v) {
        for (size_t i = 0; i < size(); i++) {
            data_[i] += v.data_[i];
        }
        return *this;
    }

    inline Vector &Vector::operator-=(const Vector &v) {
        for (size_t i = 0; i < size(); i++) {
            data_[i] -= v.data_[i];
        }
        return *this;
    }

    inline Vector &Vector::operator*=(const Vector &v) {
        for (size_t i = 0; i < size(); i++) {
            data_[i] *= v.data_[i];
        }
        return *this;
    }

    inline Vector &Vector::operator/=(const Vector &v) {
        for (size_t i = 0; i < size(); i++) {
            data_[i] /= v.data_[i];
        }
        return *this;
    }

    template<class FT>
    inline Vector &Vector::operator*=(FT s) {
        for (size_t i = 0; i < size(); i++) {
            data_[i] *= double(s);
        }
        return *this;
    }

    template<class FT>
    inline Vector &Vector::operator/=(FT s) {
        for (size_t i = 0; i < size(); i++) {
            data_[i] /= double(s);
        }
        return *this;
    }

    inline Vector Vector::operator*(double s) const {
        Vector result(*this);
        for (size_t i = 0; i < size(); i++) {
            result.data_[i] *= double(s);
        }
        return result;
    }

    template<class FT>
    inline Vector Vector::operator/(FT s) const {
        Vector result(*this);
        for (size_t i = 0; i < size(); i++) {
            result.data_[i] /= double(s);
        }
        return result;
    }


    // ----------------------- Implementation of global functions for n-dimensional vectors --------------------

    inline double dot(const Vector &v1, const Vector &v2) {
        double result = 0;
        for (size_t i = 0; i < v1.size(); i++) {
            result += v1[i] * v2[i];
        }
        return result;
    }

    inline Vector operator-(const Vector &v1) {
        Vector result(v1.size());
        for (size_t i = 0; i < v1.size(); i++) {
            result[i] = -v1[i];
        }
        return result;
    }

    inline Vector operator*(double s, const Vector &v) {
        Vector result(v.size());
        for (size_t i = 0; i < v.size(); i++) {
            result[i] = double(s) * v[i];
        }
        return result;
    }

    inline Vector operator+(const Vector &v1, const Vector &v2) {
        assert(v1.size() == v2.size());
        Vector result(v1.size());
        for (size_t i = 0; i < v1.size(); i++) {
            result[i] = v1[i] + v2[i];
        }
        return result;
    }

    inline Vector operator-(const Vector &v1, const Vector &v2) {
        assert(v1.size() == v2.size());
        Vector result(v1.size());
        for (size_t i = 0; i < v1.size(); i++) {
            result[i] = v1[i] - v2[i];
        }
        return result;
    }

    inline double length(const Vector &v) { return v.length(); }

    inline double norm(const Vector &v) { return v.length(); }

    inline double length2(const Vector &v) { return v.length2(); }

    inline double distance(const Vector &v1, const Vector &v2) { return length(v2 - v1); }

    inline double distance2(const Vector &v1, const Vector &v2) { return v2.distance2(v1); }

    inline Vector normalize(const Vector &v) {
        double s = v.length();
        s = (s > std::numeric_limits<double>::min()) ? double(1.0) / s : double(0.0);
        return v * s;
    }

    inline Vector mix(const Vector &v1, const Vector &v2, double w) {
        assert(v1.size() == v2.size());
        return (1.0 - w) * v1 + w * v2;
    }

    inline std::ostream &operator<<(std::ostream &out, const Vector &v) {
        for (size_t i = 0; i < v.size(); i++) {
            out << v[i] << ((i == v.size() - 1) ? "" : " ");
        }
        return out;
    }

    inline std::istream &operator>>(std::istream &in, Vector &v) {
        for (size_t i = 0; i < v.size(); i++) {
            in >> v[i];
        }
        return in;
    }


    // ----------------------- Implementation of 2-dimensional vectors --------------------

    inline Vector2D::Vector2D(double x, double y) : Vector(2) {
        data_[0] = x;
        data_[1] = y;
    }

    inline Vector2D::Vector2D(const Vector &v) : Vector(2, v.data()) {}

    inline double &Vector2D::x() { return data_[0]; }

    inline const double &Vector2D::x() const { return data_[0]; }

    inline double &Vector2D::y() { return data_[1]; }

    inline const double &Vector2D::y() const { return data_[1]; }

    inline Vector3D Vector2D::homogeneous() const {
        return Vector3D(x(), y(), 1.0);
    }

    // ----------------------- Implementation of 3-dimensional vectors --------------------

    inline Vector3D::Vector3D(double x, double y, double z) : Vector(3) {
        data_[0] = x;
        data_[1] = y;
        data_[2] = z;
    }

    inline Vector3D::Vector3D(const Vector &v) : Vector(3, v.data()) {}

    inline double &Vector3D::x() { return data_[0]; }

    inline const double &Vector3D::x() const { return data_[0]; }

    inline double &Vector3D::y() { return data_[1]; }

    inline const double &Vector3D::y() const { return data_[1]; }

    inline double &Vector3D::z() { return data_[2]; }

    inline const double &Vector3D::z() const { return data_[2]; }

    inline Vector4D Vector3D::homogeneous() const {
        return Vector4D(x(), y(), z(), 1.0);
    }

    inline Vector2D Vector3D::cartesian() const {
        return Vector2D((*this) / z());
    }

    inline Vector3D cross(const Vector3D &v1, const Vector3D &v2) {
        return Vector3D(
                v1.y() * v2.z() - v1.z() * v2.y(),
                v1.z() * v2.x() - v1.x() * v2.z(),
                v1.x() * v2.y() - v1.y() * v2.x()
        );
    }

    // ----------------------- Implementation of 4-dimensional vectors --------------------

    inline Vector4D::Vector4D(double x, double y, double z, double w) : Vector(4) {
        data_[0] = x;
        data_[1] = y;
        data_[2] = z;
        data_[3] = w;
    }

    inline Vector4D::Vector4D(const Vector &v) : Vector(4, v.data()) {}

    inline double &Vector4D::x() { return data_[0]; }

    inline const double &Vector4D::x() const { return data_[0]; }

    inline double &Vector4D::y() { return data_[1]; }

    inline const double &Vector4D::y() const { return data_[1]; }

    inline double &Vector4D::z() { return data_[2]; }

    inline const double &Vector4D::z() const { return data_[2]; }

    inline double &Vector4D::w() { return data_[3]; }

    inline const double &Vector4D::w() const { return data_[3]; }

    inline Vector3D Vector4D::cartesian() const {
        return Vector3D((*this) / w());
    }
}


#endif  // EASY3D_CORE_VECTOR_H

