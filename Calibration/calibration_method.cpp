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

#include "calibration.h"
#include "matrix_algo.h"

#include <chrono>  // for Timer

using namespace easy3d;


// Uncomment this to show the matrix checking results
// #define _MATRIX_CHECK_


/*
* helpful functions for calibration
* use namespace to avoid (possible)confusion
* ---------------------------------------------------------------------------------------------------------------*/
namespace GEO1016_A1 {

    // check whether ALL points are coplanar
    bool check_points_coplanar(const std::vector<Vector3D>& points_3d);

    // check if inputs are valid
    bool if_input_valid(
        const std::vector<Vector3D>& points_3d,
        const std::vector<Vector2D>& points_2d);

    // construct P matrix
    bool construct_matrix_p(
        Matrix& P,
        const std::vector<Vector3D>& points_3d,
        const std::vector<Vector2D>& points_2d);

    // print out a matrix
    void print_matrix(const Matrix& P);

    // solve P for M using SVD
    bool solve_svd(const Matrix& P, Matrix& U, Matrix& S, Matrix& V);

    // verify if P * m = 0, m is the last column of V, default tolerance: 0.01(based on tests)
    bool verify_p_m(const Matrix& P, const Matrix& V, double tolerance = 0.01);

    // check M matrix
    double check_matrix(
        const Matrix34& M,
        const std::vector<Vector3D>& points_3d,
        const std::vector<Vector2D>& points_2d);

    // extract parameters: intrinsic and extrinsic
    void extract_params(
        const Matrix34& M, Matrix33& R, Vector3D& t,  // extrinsic parameters
        double& fx, double& fy,  // intrinsic parameters
        double& cx, double& cy,
        double& skew);


    /*
    * Timer struct
    * to evaluate the execution time
    * e.g.
    * \code
    *   {
    *       Timer time;
    *       //execution process
    *       //...
    *   }
    * \endcode
    */
    struct Timer
    {
        std::chrono::time_point<std::chrono::steady_clock>start, end;
        std::chrono::duration<float>duration;

        Timer() //set default value
        {
            start = end = std::chrono::high_resolution_clock::now();
            duration = end - start;
        }

        ~Timer()
        {
            end = std::chrono::high_resolution_clock::now();
            duration = end - start;

            std::cout << "Execution Time: " << duration.count() * 1000.0 << "ms\n";
        }
    };

}



/**
 * TODO: Finish this function for calibrating a camera from the corresponding 3D-2D point pairs.
 *       You may define a few functions for some sub-tasks.
 * @return True on success, otherwise false. On success, the camera parameters are returned by
 * ---------------------------------------------------------------------------------------------------------------*/
bool Calibration::calibration(
    const std::vector<Vector3D>& points_3d, /// input: An array of 3D points.
    const std::vector<Vector2D>& points_2d, /// input: An array of 2D image points.
    double& fx, double& fy,    /// output: the focal length (in our slides, we use 'alpha' and 'beta/sin_theta'),
    double& cx, double& cy,    /// output: the principal point (in our slides, we use 'u0' and 'v0'),
    double& skew,              /// output: the skew factor ('-alpha * cot_theta')
    Matrix33& R,               /// output: the 3x3 rotation matrix encoding camera orientation.
    Vector3D& t)               /// output：a 3D vector encoding camera translation.
{

    // TODO: the above code just demonstrates some useful data structures and APIs. Please remove all above code in your
    //       final submission.

    //--------------------------------------------------------------------------------------------------------------
    // implementation starts ...

    std::cout << "\n[Liangliang]:\n"
        "\tThe input parameters of this function are:\n"
        "\t\t- points_3d: An array of 3D points (input to this function)\n"
        "\t\t- points_2d: An array of 2D image points (input to this function)\n"
        "\tThis function must return either 'true' on success or 'false' otherwise. On success, the camera\n"
        "\tparameters are returned by the following variables:\n"
        "\t\t- fx and fy: the focal lengths (in our slides, we use 'alpha' and 'beta')\n"
        "\t\t- cx and cy: the principal point (in our slides, we use 'u0' and 'v0')\n"
        "\t\t- skew:      the skew factor ('-alpha * cot_theta')\n"
        "\t\t- R:         the 3x3 rotation matrix encoding camera orientation\n"
        "\t\t- t:         a 3D vector encoding camera location.\n"
        "\tIMPORTANT: don't forget to write your recovered parameters to the above variables." << std::endl;



    /*
    * Timer
    * the time object will be destructed after this function gets executed
    * and the consuming time will be printed out to the console.
    */
    GEO1016_A1::Timer time;

    /*
    * TODO - 1:
    * check if input is valid
    * (1) check (e.g., number of correspondences >= 6, sizes of 2D/3D points must match).
    * (2) check if points are coplanar
    * ---------------------------------------------------------------------------------------------------------------*/
    bool if_valid = GEO1016_A1::if_input_valid(points_3d, points_2d);
    if (!if_valid)return false;


    /*
    * TODO - 2: construct the P matrix to use SVD to solve the m(so P * m = 0).
    * P is a 2n * 12 matrix(2n >= 12)
    * ---------------------------------------------------------------------------------------------------------------*/
    const int nrows = (int)(2 * points_3d.size());
    const int ncols = 12;  // number of cols is constant
    Matrix P(nrows, ncols);
    bool is_constructed = GEO1016_A1::construct_matrix_p(P, points_3d, points_2d);
    if (!is_constructed) {
        std::cout << "construct P matrix failed, please check(maybe the subscript is out of bounds)" << '\n';
        return false;
    }

#ifdef _MATRIX_CHECK_
    std::cout << "print P matrix: " << '\n';
    GEO1016_A1::print_matrix(P);
#endif 


    /*
    * TODO - 3: solve for M (3 * 4 matrix, the whole projection matrix, i.e., M = K * [R, t]) using SVD decomposition.
    * Optional:
    * you can check if your M is correct by applying M on the 3D points. If correct, the projected point
    * should be very close to your input images points.
    * ---------------------------------------------------------------------------------------------------------------*/
    const auto& m = P.rows();
    const auto& n = P.cols();

    Matrix U(m, m);
    Matrix S(m, n);
    Matrix V(n, n);

    // Compute the SVD decomposition of P
    bool is_svd_solved = GEO1016_A1::solve_svd(P, U, S, V);
    if (!is_svd_solved) {
        std::cout << "SVD failed, please check " << '\n';
        return false;
    }

#ifdef _MATRIX_CHECK_
    // verify if P * m = 0
    std::cout << "verify if P * m = 0" << '\n';
    auto p_m = GEO1016_A1::verify_p_m(P, V, 1e-8);  // default tolerance is 0.01
    if (p_m)std::cout << "check: P * m = 0" << '\n';
    else {
        std::cout << "check: P * m != 0\n";
        std::cout << "please check P or the result of SVD, or you may want to select a bigger tolerance\n";
    }
#endif      

    // reformat M matrix, use the last column of matrix V
    const int& v_cols = V.cols();
    const int& last_col = v_cols - 1;  // avoid std::size_t - 1 directly
    Matrix34 M
    (
        V(0, last_col), V(1, last_col), V(2, last_col), V(3, last_col),
        V(4, last_col), V(5, last_col), V(6, last_col), V(7, last_col),
        V(8, last_col), V(9, last_col), V(10, last_col), V(11, last_col)
    );


    // Intermediate option: check whether M matrix is correct -----------------------------
#ifdef _MATRIX_CHECK_
    std::cout << "M matrix: " << '\n';
    GEO1016_A1::print_matrix(M);
#endif
    std::cout << '\n';
    std::cout << "result check: " << '\n';
    auto diff = GEO1016_A1::check_matrix(M, points_3d, points_2d);
    std::cout << "variance: " << (diff / points_3d.size()) << '\n';
    // Intermediate option: check whether M matrix is correct -----------------------------


    /*
    * TODO - 4: extract intrinsic parameters from M.
    * NB:   for choosing the sign of rho, according to the text book
    *       if the origin of the world coordinate system is in front of the camera
    *       then the positive sign(+) should be chosen, otherwise the negative sign(-)
    *       should be chosen(in a general case).
    *
    *       In this assignment, the origin of the world coordinate system(the corner of the chessboard)
    *       is in FRONT of the camera, therefore the positive sign(+) is used.
    *
    * ---------------------------------------------------------------------------------------------------------------*/
    // see the function: extract_params(M, R, t, fx, fy, cx, cy, skew);

    /*
    * TODO - 5: extract extrinsic parameters from M.
    * ---------------------------------------------------------------------------------------------------------------*/
    // see the function: extract_params(M, R, t, fx, fy, cx, cy, skew);

    /*
    * extract parameters for TODO - 4 and TODO - 5
    * ---------------------------------------------------------------------------------------------------------------*/
    GEO1016_A1::extract_params(M, R, t, fx, fy, cx, cy, skew);

    std::cout << "calibration succeed! " << '\n';
    return true;
}


/* --------------------------------------------------------------------------------------------------------------
* ------------------------------------------- IMPLEMENTATION ----------------------------------------------------
* ---------------------------------------------------------------------------------------------------------------*/


namespace GEO1016_A1 {
    // check if ALL points are coplanar
    bool check_points_coplanar(const std::vector<Vector3D>& points_3d) {
        // condition check
        if (points_3d.size() < 6) {
            std::cout << "insufficient number of 3d points, minimum required: 6" << '\n';
            return false;
        }

        // get the first 3 points to calculate the equation of the plane
        const Vector3D& p1 = points_3d[0];
        const Vector3D& p2 = points_3d[1];
        const Vector3D& p3 = points_3d[2];

        const auto& a1 = p2.x() - p1.x();
        const auto& b1 = p2.y() - p1.y();
        const auto& c1 = p2.z() - p1.z();

        const auto& a2 = p3.x() - p1.x();
        const auto& b2 = p3.y() - p1.y();
        const auto& c2 = p3.z() - p1.z();

        const auto& a = b1 * c2 - b2 * c1;
        const auto& b = a2 * c1 - a1 * c2;
        const auto& c = a1 * b2 - b1 * a2;
        const auto& d = (-a * p1.x() - b * p1.y() - c * p1.z());

        // now we have the equation of the plane:
        // a* x + b * y + c * z + d == 0
        std::size_t count_coplanar = 0;  // count the number of coplanar points
        for (const auto& p : points_3d) {
            const auto& x = p.x();
            const auto& y = p.y();
            const auto& z = p.z();
            if (abs(a * x + b * y + c * z + d)<1e-8)++count_coplanar;
        }

        // check if ALL points are coplanar
        if (count_coplanar == points_3d.size())return true;

        return false;
    }


    // check if inputs are valid
    bool if_input_valid(
        const std::vector<Vector3D>& points_3d,
        const std::vector<Vector2D>& points_2d)
    {
        if (points_2d.size() < 6) {
            std::cout << "insufficient number of 2d points, minimum required: 6" << '\n';
            return false;
        }
        if (points_3d.size() < 6) {
            std::cout << "insufficient number of 3d points, minimum required: 6" << '\n';
            return false;
        }
        if (points_2d.size() != points_3d.size()) {
            std::cout << "sizes of 2D/3D points DONT match, please check the input files" << '\n';
            return false;
        }

        // check whether points are coplanar
        if (check_points_coplanar(points_3d)) {
            std::cout << "input 3D points are ALL coplanar, please input not ALL coplanar 3D points " << '\n';
            return false;
        }
        return true;
    }


    // construct the P matrix to use SVD to solve the m(so P * m = 0).
    bool construct_matrix_p(
        Matrix& P,
        const std::vector<Vector3D>& points_3d,
        const std::vector<Vector2D>& points_2d)
    {
#ifdef _MATRIX_CHECK_
        std::cout << "construct P matrix" << '\n';
        std::cout << "P matrix rows: " << P.rows() << '\n';
        std::cout << "P matrix cols: " << P.cols() << '\n';
#endif
        // initialize the P matrix according to eq.(4) in notes: 02-camera_calibration.pdf
        // avoid branching inside for loop body whenever possible
        int k = 0;  // index in points_2d

        for (int odd = 1, even = 0; odd != P.rows() && even != P.rows(); odd += 2, even += 2) {
            const auto& x = points_3d[k].x();  // points_3D, using INDEX k
            const auto& y = points_3d[k].y();
            const auto& z = points_3d[k].z();

            const auto& u = points_2d[k].x(); // poins_2D - x, using INDEX k
            const auto& v = points_2d[k].y();  // poins_2D - y, using INDEX k

            P.set_row(odd,  // set the odd row
                {
                    0, 0, 0, 0,
                    x, y, z, 1.0,
                    -v * x, -v * y, -v * z, -v
                });

            P.set_row(even,  // set the even row
                {
                    x, y, z, 1.0,
                    0, 0, 0, 0,
                    -u * x, -u * y, -u * z, -u
                });

            ++k;
        }

        return true;  // if every step goes fine, return true
    }


    // print a matrix to check its correctness
    void print_matrix(const Matrix& P) {
        for (int i = 0; i != P.rows(); ++i) {
            for (int j = 0; j != P.cols(); ++j) {
                std::cout << P(i, j) << ", ";
            }
            std::cout << '\n';
        }
    }


    // solve for M using SVD
    // the result V is NOT V^T(V.transpose()) in the eq. P = USV^T
    bool solve_svd(const Matrix& P, Matrix& U, Matrix& S, Matrix& V) {
        svd_decompose(P, U, S, V);

        // get the last column of matrix V (the last row of matrix VT) - use lambda to access the code block
        const auto& v_cols = V.cols();
        auto print_last_col_of_v = [&](const Matrix& V) {
            int j = v_cols - 1;
            for (int i = 0; i != V.rows(); ++i) {
                std::cout << V(i, j) << " ";
            }
            std::cout << '\n';
        };

        // get the last row of matrix VT - use lambda to access the code block
        const Matrix& VT = V.transpose();
        const auto& vt_nrows = VT.rows();
        auto print_last_row_of_vt = [&](const Matrix& VT) {
            int i = vt_nrows - 1;
            for (int j = 0; j != VT.cols(); ++j) {
                std::cout << VT(i, j) << " ";
            }
            std::cout << '\n';
        };

#ifdef _MATRIX_CHECK_
        std::cout << "the last column of matrix V is: " << '\n';
        print_last_col_of_v(V);
        std::cout << "the last row of matrix VT is: " << '\n';
        print_last_row_of_vt(VT);
#endif

        return true;
    }


    // verify if P * m = 0, m is the last column of V
    bool verify_p_m(const Matrix& P, const Matrix& V, double tolerance) {
        bool res_flag(true);
        const int& v_cols = V.cols();
        const int& last_col = v_cols - 1;  // avoid std::size_t - 1 directly

        // loop through each row of P, calculate the product of vector m(12 elements)
        for (int i = 0; i != P.rows(); ++i) {
            double res{};
            for (int j = 0; j != 12; ++j) {  // loop through each column(12 in total)
                res += P(i, j) * V(j, last_col);
            }
            std::cout << "P * m - row " << i + 1 << " : " << res << '\n';
            if (abs(res) > tolerance)res_flag = false;
        }
        return res_flag;
    }


    // check M matrix
    // @return: total variance
    double check_matrix(
        const Matrix34& M,
        const std::vector<Vector3D>& points_3d,
        const std::vector<Vector2D>& points_2d)
    {
        double diff{};
        for (int i = 0; i != points_3d.size(); ++i) {
            const Vector4D& point = points_3d[i].homogeneous();
            const Vector3D& res = M * point;  // M is 3 by 4, point is 4 by 1
            const Vector2D& pixel = res.cartesian();

            // calculate the difference
            const Vector2D& image_pt = points_2d[i];
            const auto& diff_u = pixel[0] - image_pt[0];  // x
            const auto& diff_v = pixel[1] - image_pt[1];  // y

            // accumulate the squared difference
            diff += diff_u * diff_u + diff_v * diff_v;

            std::cout << "obtained pixel: " << " " << pixel << '\n';
            std::cout << "original pixel: " << " " << image_pt << '\n';
            std::cout << "difference: " << '\n';
            std::cout << "u: " << diff_u << " " << "v: " << diff_v << '\n';
            std::cout << '\n';
        }
        return diff;
    }


    // extract parameters
    void extract_params(const Matrix34& M, Matrix33& R, Vector3D& t, double& fx, double& fy, double& cx, double& cy, double& skew)
    {
        const double epsilon = 1e-8;  // tolerance

        /*
        * vectors for calculating parameters
        * ------------------------------------------------------------------------------------ */
        const Vector3D a1(M(0, 0), M(0, 1), M(0, 2));  // a1, a2, a3
        const Vector3D a2(M(1, 0), M(1, 1), M(1, 2));
        const Vector3D a3(M(2, 0), M(2, 1), M(2, 2));

        const Vector3D& cross_a1_a3 = cross(a1, a3);
        const Vector3D& cross_a2_a3 = cross(a2, a3);
        const auto& len_a1_a3 = cross_a1_a3.length();
        const auto& len_a2_a3 = cross_a2_a3.length();

        /*
        * @param: rho
        * ------------------------------------------------------------------------------------ */
        if (abs(a3.length()) < epsilon) {  // check the length of a3
            std::cout << "the length of vector a3 is 0, can not calculate intrinsic parameters, please check " << '\n';
            return;
        }
        const auto& rho = 1 / a3.length();  // how to decide the sign of rho?
        const auto& rho_2 = rho * rho;  // pre-calculate the squared of rho, for direct use

        /*
        * @param: sin_theta, cos_theta
        * ------------------------------------------------------------------------------------ */
        if (abs(len_a1_a3) < epsilon || abs(len_a2_a3) < epsilon) {
            std::cout << "length of vector cross_a1_a3 is: " << len_a1_a3 << '\n';
            std::cout << "length of vector cross_a2_a3 is: " << len_a2_a3 << '\n';
            std::cout << "at least one of them is equal to 0, can not calculate intrinsic parameters, please check " << '\n';
            return;
        }
        const auto& cos_theta = -dot(cross_a1_a3, cross_a2_a3) / (len_a1_a3 * len_a2_a3);
        const auto& sin_theta = sqrt(1 - cos_theta * cos_theta);  // theta should <= 180 ?


        /*
        * intrinsic parameters
        */


        /*
        * @param: cx, cy
        * principle point, in slides, use 'u0' and 'v0'
        * ------------------------------------------------------------------------------------ */
        cx = rho_2 * dot(a1, a3);
        cy = rho_2 * dot(a2, a3);

        /*
        * @param: fx, fy
        * the focal length (in our slides, we use 'alpha' and 'beta'),
        * ------------------------------------------------------------------------------------ */
        fx = rho_2 * len_a1_a3 * sin_theta;
        fy = rho_2 * len_a2_a3;  // beta(in the notes) = rho_2 * len_a2_a3 * sin_theta, fy = beta / sin_theta

        /*
        * @param: skew
        * the skew factor ('-alpha * cot_theta')
        * ------------------------------------------------------------------------------------ */
        if (abs(sin_theta) < epsilon) {
            std::cout << "sin theta is 0, please check " << '\n';
            return;
        }
        const auto& cot_theta = cos_theta / sin_theta;
        skew = -fx * cot_theta;


        /*
        * extrinsic parameters
        */


        /*
        * Rotation matrix (3 * 3)
        * ------------------------------------------------------------------------------------ */
        if (abs(len_a2_a3) < epsilon) {
            std::cout << "length of cross_a2_a3 is equal to 0, can not calculate intrinsic parameters, please check " << '\n';
            return;
        }
        const auto& scale_a2_a3 = 1 / len_a2_a3;
        const Vector3D& r1 = cross_a2_a3 * scale_a2_a3;
        const Vector3D& r3 = a3 * rho;
        const Vector3D& r2 = cross(r3, r1);
        R.set_row(0, r1);
        R.set_row(1, r2);
        R.set_row(2, r3);

        /*
        * translation vector
        * ------------------------------------------------------------------------------------ */
        Matrix K(3, 3);
        K(0, 0) = fx; K(0, 1) = skew;  K(0, 2) = cx;
        K(1, 0) = 0;  K(1, 1) = rho_2 * len_a2_a3;  K(1, 2) = cy;
        K(2, 0) = 0; K(2, 1) = 0;  K(2, 2) = 1;

        Matrix invK(3, 3);
        inverse(K, invK);

        const auto& b1 = M(0, 3);
        const auto& b2 = M(1, 3);
        const auto& b3 = M(2, 3);
        const Vector3D b(b1, b2, b3);

        const Vector3D& prev_t = invK * b;
        t = prev_t * rho;

    }

}