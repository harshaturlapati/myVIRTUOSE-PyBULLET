#include <Eigen/Dense>

Eigen::Matrix3d R_frm_quat(float X[4]) {
    Eigen::Matrix3d R_H;
    float x, y, z, w, a11, a12, a13, a21, a22, a23, a31, a32, a33;
    //std::cout << "quaternion - x = " << quat_actor.getX() << ", y = " << quat_actor.getY() << ", z = " << quat_actor.getZ() << ", w = " << quat_actor.getW() << std::endl;
    x = X[0];
    y = X[1];
    z = X[2];
    w = X[3];


    float norm_quat = x * x + y * y + z * z + w * w;
    if (norm_quat == 0)
    {
        R_H << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    }
    else
    {
        a11 = (w * w + x * x + y * y - z * z) / norm_quat;
        a12 = 2 * (x * y - w * z) / norm_quat;
        a13 = 2 * (x * z + w * y) / norm_quat;
        a21 = 2 * (x * y + w * z) / norm_quat;
        a22 = (w * w - x * x + y * y - z * z) / norm_quat;
        a23 = 2 * (y * z - w * x) / norm_quat;
        a31 = 2 * (x * z - w * y) / norm_quat;
        a32 = 2 * (y * z + w * x) / norm_quat;
        a33 = (w * w - x * x - y * y + z * z) / norm_quat;

        R_H << a11, a12, a13,
            a21, a22, a23,
            a31, a32, a33;
    }
    return R_H;
}
