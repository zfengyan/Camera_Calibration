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


#ifndef EASY3D_CORE_MATRIX_H
#define EASY3D_CORE_MATRIX_H

#include <vector>

#include "./vector.h"


namespace easy3d {
    
    
    class Matrix33;
    class Matrix44;
    class Matrix34;


    // -----------------------------------------------------------------------------------------------------------


    /// A general m by n matrices
    class Matrix {
    public:
        /**
         * Default constructor (that constructs an empty matrix).
         */
        Matrix();

        /**
         * Copy constructor (i.e., construct a matrix from another matrix). The new matrix will be the same as the input
         * matrix.
         * @param A Input matrix.
         */
        Matrix(const Matrix &A);

        /**
         * Construct a matrix by specifying its dimension (i.e., number of rows and number of columns) and initialize
         * all entries to zero.
         * @param rows Number of rows
         * @param cols Number of columns.
         * @param x Default value of each element of the matrix.
         */
        Matrix(int rows, int cols, const double &x = 0.0);

        /**
         * Construct a matrix by specifying its dimension (i.e., number of rows and number of columns) and initialize
         * the matrix from an array. The first row of the matrix will be initialized by the first cols elements of
         * the array. Then the second row will be initialized by the next cols elements of the array, and so on.
         * @param rows Number of rows
         * @param cols Number of columns.
         * @param array An array containing at least (rows * cols) elements.
         * @Attention: Matrices are stored internally as row-major.
         */
        Matrix(int rows, int cols, const std::vector<double> &array);

        /**
         * This is an override of the above constructor.
         * Construct a matrix by specifying its dimension (i.e., number of rows and number of columns) and initialize
         * the matrix from an array. The first row of the matrix will be initialized by the first cols elements of
         * the array. Then the second row will be initialized by the next cols elements of the array, and so on.
         * @param rows Number of rows
         * @param cols Number of columns.
         * @param array The pointer to the first element of an array used to initialize the matrix. It must have at
         *      least (rows * cols) elements.
         * @Attention: Matrices are stored internally as row-major.
         */
        Matrix(int rows, int cols, const double *array);

        /// Constructs a matrix from a 3 by 3 matrix.
        Matrix(const Matrix33 &rhs);

        /// Constructs a matrix from a 4 by 4 matrix.
        Matrix(const Matrix44 &rhs);

        /// Constructs a matrix from a 3 by 4 matrix.
        Matrix(const Matrix34 &rhs);

        /// Create an identity matrix (i.e., all elements on the diagonal have a value of 1).
        /// @note This function also allow to set the elements on the diagonal to have values other than 1.
        static Matrix identity(int rows, int cols, double v = 1.0);

        ~Matrix();

        /// Assign A to this matrix
        Matrix &operator=(const Matrix &A);    // overload evaluate operator = from matrix to matrix
        /// Assign x to every entry of this matrix
        Matrix &operator=(const double &x);            // overload evaluate operator = from scalar to matrix

        /// Return the row_th row as a 1D array.
        double *operator[](int row);

        /// Return the row_th row as a 1D array (const version).
        const double *operator[](int row) const;
        
        /// Return the data pointer
        /// \attention The matrix has a row-major storage
        double* data() { return data_; }

        /// Return the data pointer
        /// \attention The matrix has a row-major storage
        const double* data() const { return data_; }

        /// Return the element at (row, col).
        double &operator()(int row, int col);

        /// Return the element at (row, col) (const version).
        const double &operator()(int row, int col) const;

        /// Set the value of the element at (row, col).
        void set(int row, int col, double v);

        /// Get the value of the element at (row, col).
        const double &get(int row, int col) const;

        /// Return the number of rows.
        int rows() const;

        /// Return the number of columns.
        int cols() const;

        /// Change the size/dimension of the matrix.
        Matrix &resize(int rows, int cols);

        /// Get the matrix's row vector as a 1D array.
        Vector get_row(int row) const;

        /// Get the matrix's column vector as a 1D array.
        Vector get_column(int col) const;

        /// Set the matrix's row vector
        void set_row(int row, const Vector &v);

        /// Set the matrix's row vector
        void set_row(int row, const std::vector<double> &v);

        /// Set the matrix's column vector
        void set_column(int col, const Vector &v);

        /// Set the matrix's column vector
        void set_column(int col, const std::vector<double> &v);

        /// Set all elements to zero.
        void load_zero();

        /// Make this matrix identity (i.e., all elements on the diagonal have a value of 1).
        /// @note This function also allow to set the elements on the diagonal to have values other than 1.
        void load_identity(double v = 1.0);

        /// Return the transposed matrix.
        Matrix transpose() const;

        /// Return the trace of this matrix, i.e. the sum of the coefficients on the main diagonal.
        /// @note: the matrix can be of any dimension, not necessarily square.
        double trace() const;

        /// computed assignment. Add each element by v
        Matrix &operator+=(const double &v);

        /// computed assignment. Subtract each element by v
        Matrix &operator-=(const double &v);

        /// computed assignment. Multiply each element by v
        Matrix &operator*=(const double &v);

        /// computed assignment. Divide each element by v
        Matrix &operator/=(const double &v);

        /// computed assignment. Add A to this matrix.
        Matrix &operator+=(const Matrix &A);

        /// computed assignment. Subtract A from this matrix.
        Matrix &operator-=(const Matrix &);

    private:

        // 0-based data pointer
        double *data_;

        // 0-based row pointer's pointer
        double **prow_;

        // row number, column number and total number
        int nRow_;
        int nColumn_;
        long nTotal_;

    protected:
        void init(int rows, int cols);

        void copy_from_array(const double *v); // copy matrix from normal array
        void set_by_scalar(const double &x); // set matrix by a scalar
        void destroy();    // destroy the matrix

    };



    // -----------------------------------------------------------------------------------------------------------


    /// A class for 3 by 3 matrices
    class Matrix33 : public Matrix {
    public:
        /// Constructor a 3 by 3 matrix.
        Matrix33(double s00 = 0.0, double s01 = 0.0, double s02 = 0.0,
                 double s10 = 0.0, double s11 = 0.0, double s12 = 0.0,
                 double s20 = 0.0, double s21 = 0.0, double s22 = 0.0
        );

        /// Construct a 3 by 3 matrix from the top-left sub-matrix of general matrix.
        /// \attention the input matrix must have at least 3 rows and 3 columns.
        Matrix33(const Matrix &m);

        /// Create a 3 by 4 identity matrix (i.e., all elements on the diagonal have a value of 1).
        /// @note This function also allow to set the elements on the diagonal to have values other than 1.
        static Matrix33 identity(double v = 1.0);
    };


    // -----------------------------------------------------------------------------------------------------------


    /// A class for 4 by 4 matrices
    class Matrix44 : public Matrix {
    public:
        /// Constructor a 4 by 4 matrix.
        Matrix44(double s00 = 0.0, double s01 = 0.0, double s02 = 0.0, double s03 = 0.0,
                 double s10 = 0.0, double s11 = 0.0, double s12 = 0.0, double s13 = 0.0,
                 double s20 = 0.0, double s21 = 0.0, double s22 = 0.0, double s23 = 0.0,
                 double s30 = 0.0, double s31 = 0.0, double s32 = 0.0, double s33 = 0.0
        );

        /// Construct a 4 by 4 matrix from the top-left sub-matrix of general matrix.
        /// \attention the input matrix must have at least 4 rows and 4 columns.
        Matrix44(const Matrix &m);

        /// Create a 4 by 4 identity matrix (i.e., all elements on the diagonal have a value of 1).
        /// @note This function also allow to set the elements on the diagonal to have values other than 1.
        static Matrix44 identity(double v = 1.0);
    };
    
    
    // -----------------------------------------------------------------------------------------------------------


    /// A class for 3 by 4 matrices
    class Matrix34 : public Matrix {
    public:
        /// Constructor a 3 by 4 matrix.
        Matrix34(double s00 = 0.0, double s01 = 0.0, double s02 = 0.0, double s03 = 0.0,
                 double s10 = 0.0, double s11 = 0.0, double s12 = 0.0, double s13 = 0.0,
                 double s20 = 0.0, double s21 = 0.0, double s22 = 0.0, double s23 = 0.0
        );

        /// Construct a 3 by 4 matrix from the top-left sub-matrix of general matrix.
        /// \attention the input matrix must have at least 3 rows and 4 columns.
        Matrix34(const Matrix &m);

        /// Create a 3 by 4 identity matrix (i.e., all elements on the diagonal have a value of 1).
        /// @note This function also allow to set the elements on the diagonal to have values other than 1.
        static Matrix34 identity(double v = 1.0);
    };
    

    //------------------------------------------------------------------------------------------------------------------

    /// Overload of the output stream.
    std::ostream &operator<<(std::ostream &, const Matrix &);

    /// Overload of the input stream.
    std::istream &operator>>(std::istream &, Matrix &);

    //------------------------------------------------------------------------------------------------------------------

    /// get negative matrix
    Matrix operator-(const Matrix &);

    /// matrix-scalar addition
    Matrix operator+(const Matrix &, const double &);

    /// scalar-matrix addition
    Matrix operator+(const double &, const Matrix &);

    /// matrix-matrix addition
    Matrix operator+(const Matrix &, const Matrix &);

    /// matrix-scalar subtraction
    Matrix operator-(const Matrix &, const double &);

    /// scalar-matrix subtraction
    Matrix operator-(const double &, const Matrix &);

    /// matrix-matrix subtraction
    Matrix operator-(const Matrix &, const Matrix &);

    /// matrix-matrix multiplication
    Matrix operator*(const Matrix &, const Matrix &);

    /// matrix-scalar multiplication
    Matrix operator*(const Matrix &, const double &);

    /// scalar-matrix multiplication
    Matrix operator*(const double &, const Matrix &);

    /// matrix-scalar division
    Matrix operator/(const Matrix &, const double &);

    /// scalar-matrix division
    Matrix operator/(const double &, const Matrix &);

    /// matrix-vector multiplication
    Vector operator*(const Matrix &A, const Vector &b);

    //------------------------------------------------------------------------------------------------------------------

    /// matrix-matrix multiplication (result has already been allocated)
    void mult(const Matrix &, const Matrix &, Matrix &);

    /// matrix-vector multiplication (result has already been allocated)
    void mult(const Matrix &, const Vector &, Vector &);

    /// matrix-matrix multiplication
    Matrix mult(const Matrix &, const Matrix &);

    /// matrix-vector multiplication
    Vector mult(const Matrix &, const Vector &);

    //------------------------------------------------------------------------------------------------------------------

    /// transpose
    Matrix transpose(const Matrix &);

    //------------------------------------------------------------------------------------------------------------------

    /// Generate an identity matrix.
    Matrix identity(int, const double &);

    /// Generate a diagonal matrix by given its diagonal elements.
    Matrix diagonal(const std::vector<double> &);

    /// Get the diagonal entries of matrix.
    std::vector<double> diagonal(const Matrix &);

    //------------------------------------------------------------------------------------------------------------------

    /// Compute the trace of a matrix, i.e. the sum of the coefficients on the main diagonal.
    /// Note: the matrix can be any matrix, not necessarily square.
    double trace(const Matrix &);

    //------------------------------------------------------------------------------------------------------------------

    /// Compute Frobenius norm of matrix.
    double norm(const Matrix &);

    /// Swap two matrices.
    void swap(Matrix &, Matrix &);
}





#include <cassert>
#include <vector>
#include <complex>
#include <algorithm> // for std::min, std::max
#include <cmath>     // for std::sqrt
#include <iostream>


namespace easy3d {


    inline void Matrix::init(int rows, int cols) {
        nRow_ = rows;
        nColumn_ = cols;
        nTotal_ = nRow_ * nColumn_;

        data_ = new double[nTotal_];
        prow_ = new double *[nRow_];
        assert(data_);
        assert(prow_);

        double *p = data_;
        for (int i = 0; i < nRow_; ++i) {
            prow_[i] = p;
            p += nColumn_;
        }
    }


    /**
    * copy matrix from normal array
    */
    inline void Matrix::copy_from_array(const double *v) {
        for (long i = 0; i < nTotal_; ++i)
            data_[i] = v[i];
    }


    /**
    * set matrix by a scalar
    */
    inline void Matrix::set_by_scalar(const double &x) {
        for (long i = 0; i < nTotal_; ++i)
            data_[i] = x;
    }


    /**
    * destroy the matrix
    */
    inline void Matrix::destroy() {
        if (data_ == NULL)
            return;
        else
            delete[] data_;

        if (prow_ != NULL)
            delete[] prow_;
    }


    /**
    * constructors and destructor
    */
    inline Matrix::Matrix()
            : data_(0), prow_(0), nRow_(0), nColumn_(0), nTotal_(0) {
    }

    inline Matrix::Matrix(const Matrix &A) {
        init(A.nRow_, A.nColumn_);
        copy_from_array(A.data_);
    }
    
    inline Matrix::Matrix(const Matrix33 &rhs) {
        init(3, 3);
        copy_from_array(rhs.data());
    }
    
    inline Matrix::Matrix(const Matrix44 &rhs) {
        init(4, 4);
        copy_from_array(rhs.data());
    }

    inline Matrix::Matrix(const Matrix34 &rhs) {
        init(3, 4);
        copy_from_array(rhs.data());
    }

    inline Matrix::Matrix(int rows, int cols, const double &x) {
        init(rows, cols);
        set_by_scalar(x);
    }

    inline Matrix::Matrix(int rows, int cols, const std::vector<double> &array) {

        init(rows, cols);
        copy_from_array(array.data());
    }

    inline Matrix::Matrix(int rows, int cols, const double *array) {
        init(rows, cols);
        copy_from_array(array);
    }

    inline Matrix Matrix::identity(int rows, int cols, double v) {
        Matrix m(rows, cols, 0.0);
        int num = std::min(rows, cols);
        for (int i = 0; i < num; ++i)
            m(i, i) = v;
        return m;
    }

    inline Matrix::~Matrix() {
        destroy();
    }


    /**
    * overload evaluate operator = from matrix to matrix
    */
    inline Matrix &Matrix::operator=(const Matrix &A) {
        if (data_ == A.data_)
            return *this;

        if (nRow_ == A.nRow_ && nColumn_ == A.nColumn_)
            copy_from_array(A.data_);
        else {
            destroy();
            init(A.nRow_, A.nColumn_);
            copy_from_array(A.data_);
        }

        return *this;
    }


    /**
    * overload evaluate operator = from scalar to matrix
    */
    inline Matrix &Matrix::operator=(const double &x) {
        set_by_scalar(x);
        return *this;
    }


    /**
    * overload operator [] for 0-offset access
    */
    inline double *Matrix::operator[](int row) {
        assert(0 <= row);
        assert(row < nRow_);
        return prow_[row];
    }

    inline const double *Matrix::operator[](int row) const {
        assert(0 <= row);
        assert(row < nRow_);
        return prow_[row];
    }

    inline double &Matrix::operator()(int row, int col) {
        assert(0 <= row);
        assert(row < nRow_);
        assert(0 <= col);
        assert(col < nColumn_);
        return prow_[row][col];
    }

    inline const double &Matrix::operator()(int row, int col) const {
        assert(0 <= row);
        assert(row < nRow_);
        assert(0 <= col);
        assert(col < nColumn_);
        return prow_[row][col];
    }

    inline void Matrix::set(int row, int col, double v) {
        assert(0 <= row);
        assert(row < nRow_);
        assert(0 <= col);
        assert(col < nColumn_);
        prow_[row][col] = v;
    }

    inline const double &Matrix::get(int row, int col) const {
        assert(0 <= row);
        assert(row < nRow_);
        assert(0 <= col);
        assert(col < nColumn_);
        return prow_[row][col];
    }


    /**
    * Clears the matrix
    * This resets all values to 0 (zero)
    */
    inline void Matrix::load_zero() {
        for (int i = 0; i < nRow_; i++) {
            for (int j = 0; j < nColumn_; j++) {
                prow_[i][j] = 0.0;
            }
        }
    }

    /**
    * Sets the matrix to identity
    * This sets all coefficients of this matrix to be equal to
    * nROW x nCOL identity matrix.
    */
    inline void Matrix::load_identity(double v /* = double(1.0)*/) {
        for (int i = 0; i < nRow_; i++) {
            for (int j = 0; j < nColumn_; j++) {
                prow_[i][j] = (i == j) ? v : 0.0;
            }
        }
    }


    inline int Matrix::rows() const {
        return nRow_;
    }

    inline int Matrix::cols() const {
        return nColumn_;
    }


    /**
    * reallocate matrix's size
    */
    inline Matrix &Matrix::resize(int rows, int cols) {
        if (rows == nRow_ && cols == nColumn_)
            return *this;

        destroy();
        init(rows, cols);

        return *this;
    }


    /**
    * get the matrix's row vector
    */
    inline Vector Matrix::get_row(int row) const {
        assert(0 <= row);
        assert(row < nRow_);
        Vector tmp(nColumn_);
        for (int j = 0; j < nColumn_; ++j)
            tmp[j] = prow_[row][j];

        return tmp;
    }


    /**
    * get the matrix's column vector
    */
    inline Vector Matrix::get_column(int col) const {
        assert(0 <= col);
        assert(col < nColumn_);
        Vector tmp(nRow_);
        for (int i = 0; i < nRow_; ++i)
            tmp[i] = prow_[i][col];

        return tmp;
    }


    /**
    * set the matrix's row vector
    */
    inline void Matrix::set_row(int row, const Vector &v) {
        assert(0 <= row);
        assert(row < nRow_);
        assert(v.size() == nColumn_);
        for (int j = 0; j < nColumn_; ++j)
            prow_[row][j] = v[j];
    }

    inline void Matrix::set_row(int row, const std::vector<double> &v) {
        assert(0 <= row);
        assert(row < nRow_);
        assert(v.size() == nColumn_);
        for (int j = 0; j < nColumn_; ++j)
            prow_[row][j] = v[j];
    }


    /**
    * set the matrix's column vector
    */
    inline void Matrix::set_column(int col, const Vector &v) {
        assert(0 <= col);
        assert(col < nColumn_);
        assert(v.size() == nRow_);
        for (int i = 0; i < nRow_; ++i)
            prow_[i][col] = v[i];
    }

    inline void Matrix::set_column(int col, const std::vector<double> &v) {
        assert(0 <= col);
        assert(col < nColumn_);
        assert(v.size() == nRow_);
        for (int i = 0; i < nRow_; ++i)
            prow_[i][col] = v[i];
    }


    inline Matrix Matrix::transpose() const {
        Matrix t(nColumn_, nRow_);
        for (int r = 0; r < nRow_; r++) {
            for (int c = 0; c < nColumn_; c++) {
                t(c, r) = (*this)(r, c);
            }
        }
        return t;
    }


    inline double Matrix::trace() const {
        int range = std::min(nRow_, nColumn_);
        double trac = 0.0;
        for (int i = 0; i < range; i++) {
            trac += (*this)[i][i];
        }
        return trac;
    }

    /**
    * compound assignment operators +=
    */
    inline Matrix &Matrix::operator+=(const double &x) {
        double **rowPtr = prow_;
        double *colPtr = 0;

        for (int i = 0; i < nRow_; ++i) {
            colPtr = *rowPtr++;
            for (int j = 0; j < nColumn_; ++j)
                *colPtr++ += x;
        }

        return *this;
    }

    inline Matrix &Matrix::operator+=(const Matrix &rhs) {
        assert(nRow_ == rhs.rows());
        assert(nColumn_ == rhs.cols());

        double **rowPtrL = prow_;
        double *colPtrL = 0;
        double **rowPtrR = rhs.prow_;
        const double *colPtrR = 0;

        for (int i = 0; i < nRow_; ++i) {
            colPtrL = *rowPtrL++;
            colPtrR = *rowPtrR++;
            for (int j = 0; j < nColumn_; ++j)
                *colPtrL++ += *colPtrR++;
        }

        return *this;
    }


    /**
    * compound assignment operators -=
    */
    inline Matrix &Matrix::operator-=(const double &x) {
        double **rowPtr = prow_;
        double *colPtr = 0;

        for (int i = 0; i < nRow_; ++i) {
            colPtr = *rowPtr++;
            for (int j = 0; j < nColumn_; ++j)
                *colPtr++ -= x;
        }

        return *this;
    }

    inline Matrix &Matrix::operator-=(const Matrix &rhs) {
        assert(nRow_ == rhs.rows());
        assert(nColumn_ == rhs.cols());

        double **rowPtrL = prow_;
        double *colPtrL = 0;
        double **rowPtrR = rhs.prow_;
        const double *colPtrR = 0;

        for (int i = 0; i < nRow_; ++i) {
            colPtrL = *rowPtrL++;
            colPtrR = *rowPtrR++;
            for (int j = 0; j < nColumn_; ++j)
                *colPtrL++ -= *colPtrR++;
        }

        return *this;
    }


    /**
    * compound assignment operators *=
    */
    inline Matrix &Matrix::operator*=(const double &x) {
        double **rowPtr = prow_;
        double *colPtr = 0;

        for (int i = 0; i < nRow_; ++i) {
            colPtr = *rowPtr++;
            for (int j = 0; j < nColumn_; ++j)
                *colPtr++ *= x;
        }

        return *this;
    }


    /**
    * compound assignment operators /=
    */
    inline Matrix &Matrix::operator/=(const double &x) {
        double **rowPtr = prow_;
        double *colPtr = 0;

        for (int i = 0; i < nRow_; ++i) {
            colPtr = *rowPtr++;
            for (int j = 0; j < nColumn_; ++j)
                *colPtr++ /= x;
        }

        return *this;
    }

    /**
    * Overload the output stream function.
    */
    inline std::ostream &operator<<(std::ostream &out, const Matrix &A) {
        int rows = A.rows();
        int cols = A.cols();

        out << "size: " << rows << " by " << cols << "\n";
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (std::abs(A[i][j]) < 1e-6)
                    out << 0 << "\t";
                else
                    out << A[i][j] << "\t";
            }
            out << "\n";
        }

        return out;
    }


    /**
    * Overload the input stream function.
    */
    inline std::istream &operator>>(std::istream &in, Matrix &A) {
        int rows, cols;
        in >> rows >> cols;

        if (!(rows == A.rows() && cols == A.cols()))
            A.resize(rows, cols);

        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                in >> A[i][j];

        return in;
    }


    /**
    * get negative matrix
    */
    inline Matrix operator-(const Matrix &A) {
        int rows = A.rows();
        int cols = A.cols();

        Matrix tmp(rows, cols);
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                tmp[i][j] = -A[i][j];

        return tmp;
    }


    /**
    * matrix-scalar addition
    */
    inline Matrix operator+(const Matrix &A, const double &x) {
        Matrix tmp(A);
        return tmp += x;
    }

    inline Matrix operator+(const double &x, const Matrix &A) {
        return A + x;
    }


    /**
    * matrix-matrix addition
    */
    inline Matrix operator+(const Matrix &A1, const Matrix &A2) {
        Matrix tmp(A1);
        return tmp += A2;
    }


    /**
    * matrix-scalar subtraction
    */
    inline Matrix operator-(const Matrix &A, const double &x) {
        Matrix tmp(A);
        return tmp -= x;
    }

    inline Matrix operator-(const double &x, const Matrix &A) {
        Matrix tmp(A);
        return -tmp += x;
    }


    /**
    * matrix-matrix subtraction
    */
    inline Matrix operator-(const Matrix &A1, const Matrix &A2) {
        Matrix tmp(A1);
        return tmp -= A2;
    }

    /**
    * matrix-matrix multiplication
    */
    inline Matrix operator*(const Matrix &A1, const Matrix &A2) {
        assert(A1.cols() == A2.rows());

        int rows = A1.rows();
        int cols = A2.cols();

        Matrix tmp(rows, cols);
        mult(A1, A2, tmp);

        return tmp;
    }


    /**
    * matrix-vector multiplication
    */
    inline Vector operator*(const Matrix &A, const Vector &b) {
        assert(A.cols() == b.size());

        int rows = A.rows();
        //	int cols = A.cols();

        Vector tmp(rows);
        mult(A, b, tmp);

        return tmp;
    }

    /// matrix-scalar multiplication
    inline Matrix operator*(const Matrix &A, const double &s) {
        Matrix tmp(A);
        return tmp *= s;
    }

    // scalar-matrix multiplication
    inline Matrix operator*(const double &s, const Matrix &A) {
        return A * s;
    }

    // matrix-scalar division
    inline Matrix operator/(const Matrix &A, const double &s) {
        Matrix tmp(A);
        return tmp /= s;
    }

    // scalar-matrix division
    inline Matrix operator/(const double &s, const Matrix &A) {
        int rows = A.rows();
        int clumns = A.cols();

        Matrix tmp(rows, clumns);
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < clumns; ++j)
                tmp[i][j] = s / A[i][j];

        return tmp;
    }

    /**
    * This is an optimized version of matrix multiplication,
    * where the destination matrix has already been allocated.
    */
    inline void mult(const Matrix &A, const Matrix &B, Matrix &C) {
        int M = A.rows();
        int N = B.cols();
        int K = A.cols();

        assert(B.rows() == K);

        C.resize(M, N);
        double sum = 0;
        const double *pRow, *pCol;

        for (int i = 0; i < M; i++)
            for (int j = 0; j < N; ++j) {
                pRow = &A[i][0];
                pCol = &B[0][j];
                sum = 0;

                for (int k = 0; k < K; ++k) {
                    sum += (*pRow) * (*pCol);
                    pRow++;
                    pCol += N;
                }
                C[i][j] = sum;
            }
    }


    /**
    * This is an optimized version of matrix and vector multiplication,
    * where the destination vector has already been allocated.
    */
    inline void mult(const Matrix &A, const Vector &b, Vector &c) {
        int M = A.rows();
        int N = A.cols();

        assert(b.size() == N);

        c.resize(M);
        double sum = 0;
        const double *pRow, *pCol;

        for (int i = 0; i < M; i++) {
            pRow = &A[i][0];
            pCol = &b[0];
            sum = 0;

            for (int j = 0; j < N; ++j) {
                sum += (*pRow) * (*pCol);
                pRow++;
                pCol++;
            }
            c[i] = sum;
        }
    }


    /**
    * This is an optimized version of matrix multiplication,
    * where the destination matrix has already been allocated.
    */
    inline Matrix mult(const Matrix &A, const Matrix &B) {
        int M = A.rows();
        int N = B.cols();
        int K = A.cols();

        assert(B.rows() == K);

        Matrix C(M, N);
        double sum = 0;
        const double *pRow, *pCol;

        for (int i = 0; i < M; i++)
            for (int j = 0; j < N; ++j) {
                pRow = &A[i][0];
                pCol = &B[0][j];
                sum = 0;

                for (int k = 0; k < K; ++k) {
                    sum += (*pRow) * (*pCol);
                    pRow++;
                    pCol += N;
                }
                C[i][j] = sum;
            }
        return C;
    }


    /**
    * This is an optimized version of matrix and vector multiplication,
    * where the destination vector has already been allocated.
    */
    inline Vector mult(const Matrix &A, const Vector &b) {
        int M = A.rows();
        int N = A.cols();

        assert(b.size() == N);

        Vector c(M);
        double sum = 0;
        const double *pRow, *pCol;

        for (int i = 0; i < M; i++) {
            pRow = &A[i][0];
            pCol = &b[0];
            sum = 0;

            for (int j = 0; j < N; ++j) {
                sum += (*pRow) * (*pCol);
                pRow++;
                pCol++;
            }
            c[i] = sum;
        }
        return c;
    }


    /**
    * matrix transpose
    */
    inline Matrix transpose(const Matrix &A) {
        int rows = A.cols();
        int clumns = A.rows();

        Matrix tmp(rows, clumns);
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < clumns; ++j)
                tmp[i][j] = A[j][i];

        return tmp;
    }


    /**
    * Generate the identity matrix.
    */
    inline Matrix identity(int N, const double &x) {
        Matrix tmp(N, N);
        for (int i = 0; i < N; ++i)
            tmp[i][i] = x;

        return tmp;
    }


    /**
    * Get the diagonal entries of matrix.
    */
    inline std::vector<double> diagonal(const Matrix &A) {
        int nColumn_ = A.rows();
        if (nColumn_ > A.cols())
            nColumn_ = A.cols();

        std::vector<double> tmp(nColumn_);
        for (int i = 0; i < nColumn_; ++i)
            tmp[i] = A[i][i];

        return tmp;
    }


    /**
    * Generate the diagonal of matrix by given its diagonal elements.
    */
    inline Matrix diagonal(const std::vector<double> &d) {
        int N = static_cast<int>(d.size());

        Matrix tmp(N, N);
        for (int i = 0; i < N; ++i)
            tmp[i][i] = d[i];

        return tmp;
    }


    /**
    * the trace of this matrix, i.e. the sum of the coefficients on the main diagonal.
    * NOTE: the matrix can be any matrix, not necessarily square.
    */
    inline double trace(const Matrix &A) {
        int range = std::min(A.rows(), A.cols());
        double trac = 0.0;
        for (int i = 0; i < range; i++) {
            trac += A[i][i];
        }
        return trac;
    }


    /**
    * Compute Frobenius norm of matrix.
    */
    inline double norm(const Matrix &A) {
        int m = A.rows();
        int n = A.cols();

        double sum = 0;
        for (int i = 0; i < m; ++i)
            for (int j = 0; j < n; ++j)
                sum += A[i][j] * A[i][j];

        return std::sqrt(sum);
    }


    /**
    * Swap two matrices.
    */
    inline void swap(Matrix &lhs, Matrix &rhs) {
        int m = lhs.rows();
        int n = lhs.cols();

        assert(m == rhs.rows());
        assert(n == rhs.cols());

        for (int i = 0; i < m; ++i)
            for (int j = 0; j < n; ++j)
                std::swap(lhs(i, j), rhs(i, j));
    }



    // -----------------------------------------------------------------------------------------------------------


    inline Matrix33::Matrix33(double s00, double s01, double s02,
                              double s10, double s11, double s12,
                              double s20, double s21, double s22) : Matrix(3, 3) {
        (*this)(0, 0) = s00;
        (*this)(0, 1) = s01;
        (*this)(0, 2) = s02;
        (*this)(1, 0) = s10;
        (*this)(1, 1) = s11;
        (*this)(1, 2) = s12;
        (*this)(2, 0) = s20;
        (*this)(2, 1) = s21;
        (*this)(2, 2) = s22;
    }

    
    inline Matrix33::Matrix33(const Matrix &m) : Matrix(3, 3) {
        assert(m.rows() >= 3);
        assert(m.cols() >= 3);
        for (int i=0; i<3; ++i) {
            for (int j = 0; j < 3; ++j)
                (*this)(i, j) = m(i, j);
        }
    }


    inline Matrix33 Matrix33::identity(double v) {
        return Matrix::identity(3, 3, v);
    }
    
    // -----------------------------------------------------------------------------------------------------------

    
    inline Matrix44::Matrix44(
            double s00, double s01, double s02, double s03,
            double s10, double s11, double s12, double s13,
            double s20, double s21, double s22, double s23,
            double s30, double s31, double s32, double s33
    ) : Matrix(4, 4) {
        (*this)(0, 0) = s00; (*this)(0, 1) = s01; (*this)(0, 2) = s02; (*this)(0, 3) = s03;
        (*this)(1, 0) = s10; (*this)(1, 1) = s11; (*this)(1, 2) = s12; (*this)(1, 3) = s13;
        (*this)(2, 0) = s20; (*this)(2, 1) = s21; (*this)(2, 2) = s22; (*this)(2, 3) = s23;
        (*this)(3, 0) = s30; (*this)(3, 1) = s31; (*this)(3, 2) = s32; (*this)(3, 3) = s33;
    }


    inline Matrix44::Matrix44(const Matrix &m) : Matrix(4, 4) {
        assert(m.rows() >= 4);
        assert(m.cols() >= 4);
        for (int i=0; i<4; ++i) {
            for (int j = 0; j < 4; ++j)
                (*this)(i, j) = m(i, j);
        }
    }


    inline Matrix44 Matrix44::identity(double v) {
        return Matrix::identity(4, 4, v);
    }


    // -----------------------------------------------------------------------------------------------------------


    inline Matrix34::Matrix34(
            double s00, double s01, double s02, double s03,
            double s10, double s11, double s12, double s13,
            double s20, double s21, double s22, double s23
    ) : Matrix(3, 4) {
        (*this)(0, 0) = s00; (*this)(0, 1) = s01; (*this)(0, 2) = s02; (*this)(0, 3) = s03;
        (*this)(1, 0) = s10; (*this)(1, 1) = s11; (*this)(1, 2) = s12; (*this)(1, 3) = s13;
        (*this)(2, 0) = s20; (*this)(2, 1) = s21; (*this)(2, 2) = s22; (*this)(2, 3) = s23;
    }


    inline Matrix34::Matrix34(const Matrix &m) : Matrix(3, 4) {
        assert(m.rows() >= 3);
        assert(m.cols() >= 4);
        for (int i=0; i<3; ++i) {
            for (int j = 0; j < 4; ++j)
                (*this)(i, j) = m(i, j);
        }
    }


    inline Matrix34 Matrix34::identity(double v) {
        return Matrix::identity(3, 4, v);
    }


}


#endif // EASY3D_CORE_MATRIX_H
