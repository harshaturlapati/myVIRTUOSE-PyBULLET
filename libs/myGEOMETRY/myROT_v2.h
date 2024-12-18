#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"

#include <Eigen/Dense>

Eigen::Vector3d my_cross(Eigen::Vector3d a, Eigen::Vector3d b) {
    Eigen::Vector3d c;
    c(0) = a(1) * b(2) - a(2) * b(1);
    c(1) = -(a(0) * b(2) - a(2) * b(0));
    c(2) = a(0) * b(1) - a(1) * b(0);
    //std::cout << c << std::endl;
    return c;
}

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
        // using the transpose of https://personal.utdallas.edu/~sxb027100/dock/quaternion.html
        a11 = (w * w + x * x - y * y - z * z) / norm_quat;
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

Eigen::Matrix3d R_frm_quat_v2(Eigen::Vector4d X) {
    Eigen::Matrix3d R_H;
    float x, y, z, w, a11, a12, a13, a21, a22, a23, a31, a32, a33;
    //std::cout << "quaternion - x = " << quat_actor.getX() << ", y = " << quat_actor.getY() << ", z = " << quat_actor.getZ() << ", w = " << quat_actor.getW() << std::endl;
    x = X(0);
    y = X(1);
    z = X(2);
    w = X(3);


    float norm_quat = x * x + y * y + z * z + w * w;
    if (norm_quat == 0)
    {
        R_H << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    }
    else
    {
        // using the transpose of https://personal.utdallas.edu/~sxb027100/dock/quaternion.html
        a11 = (w * w + x * x - y * y - z * z) / norm_quat;
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


Eigen::Matrix3d R_frm_quat_v3(btQuaternion Q_in) {
    Eigen::Matrix3d R_H;
    float x, y, z, w, a11, a12, a13, a21, a22, a23, a31, a32, a33;
    //std::cout << "quaternion - x = " << quat_actor.getX() << ", y = " << quat_actor.getY() << ", z = " << quat_actor.getZ() << ", w = " << quat_actor.getW() << std::endl;
    x = Q_in[0];
    y = Q_in[1];
    z = Q_in[2];
    w = Q_in[3];


    float norm_quat = x * x + y * y + z * z + w * w;
    if (norm_quat == 0)
    {
        R_H << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    }
    else
    {
        // using the transpose of https://personal.utdallas.edu/~sxb027100/dock/quaternion.html
        a11 = (w * w + x * x - y * y - z * z) / norm_quat;
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
