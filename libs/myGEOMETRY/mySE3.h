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

Eigen::Matrix4d compose_bt(btVector3 p_in, Eigen::Matrix3d R) {
    Eigen::Vector3d p(p_in[0],p_in[1],p_in[2]);

    Eigen::Matrix4d T;

    // Horizontal concatenation
    Eigen::MatrixXd T_interim(R.rows(), R.cols() + p.cols());
    T_interim << R, p;

    // Vertical concatenation
    T << T_interim, padding; // <-- syntax is the same for vertical and horizontal concatenation
    return T;
}
