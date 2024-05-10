#include <iostream>
#include <Eigen/Dense>
#include <vector> 
using namespace std;

Eigen::Matrix4d dosomething(Eigen::Matrix4d T)
{

    return T;
}

// Customised type definitions of matrices
typedef Eigen::Matrix<double, 3, 4> Matrix34d;

int main()
{

    // Matrix definitions
    Eigen::Matrix2d mat;
    mat << 1, 2,
        3, 4;

    // Vector definitions
    Eigen::Vector2d u(-1, 1), v(2, 0);
    std::cout << "Here is mat*mat:\n" << mat * mat << std::endl;
    std::cout << "Here is mat*u:\n" << mat * u << std::endl;
    std::cout << "Here is u^T*mat:\n" << u.transpose() * mat << std::endl;
    std::cout << "Here is u^T*v:\n" << u.transpose() * v << std::endl;
    std::cout << "Here is u*v^T:\n" << u * v.transpose() << std::endl;
    std::cout << "Let's multiply mat by itself" << std::endl;
    mat = mat * mat;
    std::cout << "Now mat is mat:\n" << mat << std::endl;
    std::cout << mat(0) << mat(1) << mat(2) << mat(3) << std::endl; // Eigen uses column-wise indexing, counter-intuitive to what you're used to.

    Eigen::Matrix3d R;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    std::cout << "Rotation matrix R is:\n" << R << std::endl;

    Eigen::Matrix4d T;
    T << 1, 0, 0, 0.123,
        -0.000012354235, 2, 0, 0.345,
        0, 0, 1, 0.456,
        0, 0, 0, 1;
    std::cout << "Transformation matrix T is:\n" << T << std::endl;

    float a;
    a = T(1);

    Eigen::Matrix4d T2 = dosomething(T);
    std::cout << "Functions work:\n" << T2 << std::endl;

    std::cout << "Converted to float is:\n" << a << std::endl;
    

    Matrix34d Pi;
    Pi << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;

    Eigen::Vector4d e_i_tilde(0.123, 0.234, 0.345, 1);
    Eigen::Vector3d answer;

    answer = Pi * e_i_tilde;

    std::cout << "3x4 Matrix Pi is:\n" << Pi << std::endl;

    std::cout << "3x1 vector Pi*e_i_tilde is:\n" << answer << std::endl;


    Eigen::Vector3d e_i(1,2,3);
    std::cout << "Array of 3D vectors looks like:\n" << e_i << std::endl;


    // Vector of vectors
    vector<int> g1;

    for (int i = 1; i <= 5; i++)
        g1.push_back(i);
    cout << "Output of begin and end: ";
    for (auto i = g1.begin(); i != g1.end(); ++i)
        cout << *i << " ";
    std::cout << "\n";
    
    vector<Eigen::Vector3d> g2;
    //for (int i = 1; i <= 5; i++)
    g2.push_back(Eigen::Vector3d(0,0,0));
    g2.push_back(Eigen::Vector3d(1,2,3));
    g2.push_back(Eigen::Vector3d(4,5,6));

    std::cout << "First vector is : \n" << g2[0] << "\nSecond vector is : \n" << g2[1] << "\nThird vector is : \n" << g2[2] << std::endl;

    // composing matrices from sub-matrices and vectors
    //R.rows();
    //VectorXd T_top(R.size() + p.size());
    Eigen::Vector3d p(1, 2, 3);

    Eigen::MatrixXd T3(R.rows(), R.cols() + p.cols());
    T3 << R, p;
    std::cout << T3 << std::endl;

    Eigen::RowVector4d padding(0, 0, 0, 1);
    Eigen::MatrixXd T4(T3.rows() + padding.rows(), T3.cols()); // <-- D(A.rows() + B.rows(), ...)
    T4 << T3, padding; // <-- syntax is the same for vertical and horizontal concatenation
    std::cout << T4 << std::endl;
}