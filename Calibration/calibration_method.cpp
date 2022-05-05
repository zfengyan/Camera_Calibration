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


using namespace easy3d;


/*
* helpful functions for calibration
* ---------------------------------------------------------------------------------------------------------------*/

// check whether ALL points are coplanar
bool check_points_coplanar(const std::vector<Vector3D>& points_3d);

// construct P matrix
bool construct_matrix_p(
    Matrix& P,
    const std::vector<Vector3D>& points_3d,
    const std::vector<Vector2D>& points_2d);

// print out a matrix
void print_matrix(const Matrix& P);

// solve for M using SVD
bool solve_svd(const Matrix& P, Matrix& U, Matrix& S, Matrix& V);


/**
 * TODO: Finish this function for calibrating a camera from the corresponding 3D-2D point pairs.
 *       You may define a few functions for some sub-tasks.
 * @return True on success, otherwise false. On success, the camera parameters are returned by
 * ---------------------------------------------------------------------------------------------------------------*/
bool Calibration::calibration(
        const std::vector<Vector3D>& points_3d, /// input: An array of 3D points.
        const std::vector<Vector2D>& points_2d, /// input: An array of 2D image points.
        double& fx, double& fy,    /// output: the focal length (in our slides, we use 'alpha' and 'beta'),
        double& cx, double& cy,    /// output: the principal point (in our slides, we use 'u0' and 'v0'),
        double& skew,              /// output: the skew factor ('-alpha * cot_theta')
        Matrix33& R,               /// output: the 3x3 rotation matrix encoding camera orientation.
        Vector3D& t)               /// output：a 3D vector encoding camera translation.
{
    std::cout << "\nTODO: I am going to implement the calibration() function in the following file:\n"
                 "\t    - calibration_method.cpp\n\n";

    std::cout << "[Liangliang]:\n"
                 "\tCamera calibration requires computing the SVD and inverse of matrices.\n"
                 "\tIn this assignment, I provide you with a 'Matrix' and a 'Vector' data structures for storing and\n"
                 "\tmanipulating matrices and vectors of arbitrary sizes. I also wrote some code to show you how to:\n"
                 "\t    - compute the SVD of a matrix;\n"
                 "\t    - compute the inverse of a matrix;\n"
                 "\t    - compute the transpose of a matrix.\n\n"
                 "\tFeel free to use any of the provided data structures and functions. The commonly used linear algebra\n"
                 "\tfunctions are provided in the following files:\n"
                 "\t    - Calibration/matrix.h  Matrices of arbitrary dimensions and related functions.\n"
                 "\t    - Calibration/vector.h  Vectors of arbitrary dimensions and related functions.\n"
                 "\t    - Calibration/matrix_algo.h  Determinant, inverse, SVD, linear least-squares...\n"
                 "\tPlease refer to the above files for a complete list of useful functions and their usage.\n\n"
                 "\tIn your final submission, please\n"
                 "\t    - delete ALL unrelated test or debug code and avoid unnecessary output.\n"
                 "\t    - include all the source code (and please do NOT modify the structure of the directories).\n"
                 "\t    - do NOT include the 'build' directory (which contains the intermediate files in a build step).\n"
                 "\t    - make sure your code compiles and can reproduce your results without ANY modification.\n\n" << std::flush;

    /// Below are a few examples showing some useful data structures and functions.

    // This is a 1D array of 'double' values. Alternatively, you can use 'double mat[25]' but you cannot change it
    // length. With 'std::vector', you can append/delete/insert elements, and much more. The 'std::vector' can store
    // not only 'double', but also any other types of objects. In case you may want to learn more about 'std::vector'
    // check here: https://en.cppreference.com/w/cpp/container/vector
    std::vector<double> array = {1, 3, 3, 4, 7, 6, 2, 8, 2, 8, 3, 2, 4, 9, 1, 7, 3, 23, 2, 3, 5, 2, 1, 5, 8, 9, 22};
    array.push_back(5); // append 5 to the array (so the size will increase by 1).
    array.insert(array.end(), 10, 3);  // append ten 3 (so the size will grow by 10).

    /// To access the value of an element.
    double a = array[2];

    /// define a 2D vector/point
    Vector2D b(1.1, 2.2);

    /// define a 3D vector/point
    Vector3D c(1.1, 2.2, 3.3);

    /// get the Cartesian coordinates of a (a is treated as Homogeneous coordinates)
    Vector2D p = c.cartesian();

    /// get the Homogeneous coordinates of p
    Vector3D q = p.homogeneous();

    /// the length of a vector
    double len = p.length();
    /// the squared length of a vector
    double sqr_len = p.length2();

    /// the dot product of two vectors
    double dot_prod = dot(p, q);

    /// the cross product of two vectors
    Vector cross_prod = cross(c, q);

    /// normalize this vector
    cross_prod.normalize();

    // Define an m-by-n double valued matrix.
    // Here I use the above array to initialize it. You can also use A(i, j) to initialize/modify/access its elements.
    //const int m = 6, n = 5;
    //Matrix A(m, n, array.data());    // 'array.data()' returns a pointer to the array.
//    std::cout << "M: \n" << A << std::endl;

    /// define a 3 by 4 matrix (and all elements initialized to 0.0)
    //Matrix M(3, 4, 0.0);

    /// set first row by a vector
    //M.set_row(0, Vector4D(1.1, 2.2, 3.3, 4.4));

    /// set second column by a vector
    //M.set_column(1, Vector3D(5.5, 5.5, 5.5));

    /// define a 3 by 3 matrix (and all elements initialized to 0.0)
    Matrix33 B;

    /// define and initialize a 3 by 3 matrix
    Matrix33 T(1.1, 2.2, 3.3,
               0, 2.2, 3.3,
               0, 0, 1);

    /// define and initialize a 3 by 4 matrix
    /*Matrix34 P(1.1, 2.2, 3.3, 0,
               0, 2.2, 3.3, 1,
               0, 0, 1, 1);*/

    /// define a 15 by 9 matrix (and all elements initialized to 0.0)
    Matrix W(15, 9, 0.0);
    /// set the first row by a 9-dimensional vector
    W.set_row(0, {0, 1, 2, 3, 4, 5, 6, 7, 8}); // {....} is equivalent to a std::vector<double>

    /// get the number of rows.
    int num_rows = W.rows();

    /// get the number of columns.
    int num_cols = W.cols();

    /// get the the element at row 1 and column 2
    double value = W(1, 2);

    /// get the last column of a matrix
    Vector last_column = W.get_column(W.cols() - 1);

    /// define a 3 by 3 identity matrix
    Matrix33 I = Matrix::identity(3, 3, 1.0);

    /// matrix-vector product
    // Vector3D v = M * Vector4D(1, 2, 3, 4); // M is 3 by 4

    //Matrix U(m, m, 0.0);   // initialized with 0s
    //Matrix S(m, n, 0.0);   // initialized with 0s
    //Matrix V(n, n, 0.0);   // initialized with 0s

    // Compute the SVD decomposition of A
    //svd_decompose(A, U, S, V);

    // Now let's check if the SVD result is correct

    // Check 1: U is orthogonal, so U * U^T must be identity
//    std::cout << "U*U^T: \n" << U * transpose(U) << std::endl;

    // Check 2: V is orthogonal, so V * V^T must be identity
//    std::cout << "V*V^T: \n" << V * transpose(V) << std::endl;

    // Check 3: S must be a diagonal matrix
//    std::cout << "S: \n" << S << std::endl;

    // Check 4: according to the definition, A = U * S * V^T
//    std::cout << "M - U * S * V^T: \n" << A - U * S * transpose(V) << std::endl;

    // Compute the inverse of a matrix
    Matrix invT;
    inverse(T, invT);
    // Let's check if the inverse is correct
//    std::cout << "B * invB: \n" << B * invB << std::endl;

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
    * TODO - 1: 
    * (1) check if input is valid (e.g., number of correspondences >= 6, sizes of 2D/3D points must match).
    * (2) check if points are coplanar
    * ---------------------------------------------------------------------------------------------------------------*/
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


    /*
    * TODO - 2: construct the P matrix to use SVD to solve the m(so P * m = 0).
    * P is a 2n * 12 matrix(2n >= 12)
    * ---------------------------------------------------------------------------------------------------------------*/
    int nrows = (int)(2 * points_3d.size());
    const int ncols = 12;  // number of cols is constant
    Matrix P(nrows, ncols);
    bool is_constructed = construct_matrix_p(P, points_3d, points_2d);
    if (is_constructed) {
        std::cout << "print P matrix: " << '\n';
        print_matrix(P);
    }
    else {
        std::cout << "construct P matrix failed, please check(maybe the subscript is out of bounds)" << '\n';
    }


    /*
    * TODO - 3: solve for M (3 * 4 matrix, the whole projection matrix, i.e., M = K * [R, t]) using SVD decomposition.
    * Optional: 
    * you can check if your M is correct by applying M on the 3D points. If correct, the projected point
    * should be very close to your input images points.
    * ---------------------------------------------------------------------------------------------------------------*/
    std::cout << "test SVD" << '\n';
    const int& m = P.rows();
    const int& n = P.cols();

	Matrix U(m, m);  
	Matrix S(m, n);  
	Matrix V(n, n); 

	// Compute the SVD decomposition of P
    bool is_svd_solved = solve_svd(P, U, S, V);
    if (!is_svd_solved) {
        std::cout << "SVD failed, please check " << '\n';
        return false;
    }

    // reformat M matrix, use the last column of matrix V
    const int v_cols = V.cols();
    const int last_col = v_cols - 1;  // avoid V.cols() - 1
    Matrix34 M
    (
        V(0, last_col), V(1, last_col), V(2, last_col), V(3, last_col),
        V(4, last_col), V(5, last_col), V(6, last_col), V(7, last_col),
        V(8, last_col), V(9, last_col), V(10,last_col), V(11, last_col)
    );
    std::cout << "M matrix: " << '\n';
    print_matrix(M);

    // Intermediate option: check whether M matrix is correct -----------------------------
    auto check_M = [&](
        const Matrix34& M, 
        const std::vector<Vector3D>& points_3d,
        const std::vector<Vector2D>& points_2d)->void 
    {
        std::cout << "check M matrix: " << '\n';
        for (int i = 0; i != points_3d.size(); ++i) {
            const Vector4D& point = points_3d[i].homogeneous();
            const Vector3D& res = M * point;  // M is 3 by 4, point is 4 by 1
            const Vector2D& pixel = res.cartesian();

            // calculate the difference
            const Vector2D& image_pt = points_2d[i];  
            double diff_u = pixel[0] - image_pt[0];  // x
            double diff_v = pixel[1] - image_pt[1];  // y
            
            std::cout << "obtained pixel position: " << " " << pixel << '\n';
            std::cout << "original pixel position: " << " " << image_pt << '\n';
            std::cout << "difference: " << '\n';
            std::cout << diff_u << "(u)" << " " << diff_v << "(v)" << '\n';
            std::cout << '\n';
        }
    };
    check_M(M, points_3d, points_2d);
    // Intermediate option: check whether M matrix is correct -----------------------------


    // TODO: extract intrinsic parameters from M.

    // TODO: extract extrinsic parameters from M.

    std::cout << "\n\tTODO: After you implement this function, please return 'true' - this will trigger the viewer to\n"
                 "\t\tupdate the rendering using your recovered camera parameters. This can help you to visually check\n"
                 "\t\tif your calibration is successful or not.\n\n" << std::flush;
    return false;
}



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
        if (a * x + b * y + c * z + d == 0)++count_coplanar;
    }

    // check if ALL points are coplanar
    if (count_coplanar == points_3d.size())return true;
    
    return false;
}


// construct the P matrix to use SVD to solve the m(so P * m = 0).
bool construct_matrix_p(
    Matrix& P,
    const std::vector<Vector3D>& points_3d,
    const std::vector<Vector2D>& points_2d) 
{
    std::cout << "construct P matrix" << '\n';
    std::cout << "P matrix rows: " << P.rows() << '\n';
    std::cout << "P matrix cols: " << P.cols() << '\n';

    // initialize the P matrix according to eq.(4) in notes: 02-camera_calibration.pdf
    int j = 0;  // index in points_2d
    for (int i = 0; i != P.rows(); ++i) {
        if (i & 1) {  // the position of a row is an odd number, e.g.: 1, 3, 5, 7, ...
            const auto& x = points_3d[j].x();  // NB: points_3D, using INDEX j
            const auto& y = points_3d[j].y();
            const auto& z = points_3d[j].z();
            const auto& v = points_2d[j].y();  // NB: poins_2D - y, using INDEX j
            
			P.set_row(i,
			    {
					0, 0, 0, 0,
					x, y, z, 1.0,
					-v * x, -v * y, -v * z, -v
				});
            ++j;  // in the odd row, ++j, because two rows correspond to one point in points_2d
        }
        else {  // the position of a col is an even numbe, e.g.: 0, 2, 4, 6, ...
            const auto& x = points_3d[j].x();  // NB: points_3D, using INDEX j
            const auto& y = points_3d[j].y();
            const auto& z = points_3d[j].z();
            const auto& u = points_2d[j].x(); // NB: poins_2D - x, using INDEX j

            P.set_row(i,
                {
                    x, y, z, 1.0,
                    0, 0, 0, 0,
                    -u * x, -u * y, -u * z, -u
                });
        }

    }

    std::cout << "P matrix constructed! " << '\n';
    return true;  // if every step goes fine, return true
}


// if P matrix constructed, print P to check
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

    // get the last column of matrix V (the last row of matrix VT)
    const int v_cols = V.cols();
    auto print_last_col_of_v = [&](const Matrix& V) {
        int j = v_cols - 1;
        for (int i = 0; i != V.rows(); ++i) {
            std::cout << V(i, j) << " ";
        }
        std::cout << '\n';
    };
    std::cout << "the last column of matrix V is: " << '\n';
    print_last_col_of_v(V);

    // get the last row of matrix VT
    const Matrix& VT = V.transpose();
    const int vt_nrows = VT.rows();
    auto print_last_row_of_vt = [&](const Matrix& VT) {
        int i = vt_nrows - 1;
        for (int j = 0; j != VT.cols(); ++j) {
            std::cout << VT(i, j) << " ";
        }
        std::cout << '\n';
    };
    std::cout << "the last row of matrix VT is: " << '\n';
    print_last_row_of_vt(VT);

    return true;
}






















