#include <Eigen/Dense>
Eigen::RowVector4d padding(0, 0, 0, 1);

Eigen::Matrix4d compose(Eigen::Vector3d p, Eigen::Matrix3d R) {
    Eigen::Matrix4d T;

    // Horizontal concatenation
    Eigen::MatrixXd T_interim(R.rows(), R.cols() + p.cols());
    T_interim << R, p;

    // Vertical concatenation
    T << T_interim, padding; // <-- syntax is the same for vertical and horizontal concatenation
    return T;
}
