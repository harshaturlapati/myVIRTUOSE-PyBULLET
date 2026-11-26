// Standalone check of the quaternion -> rotation matrix helpers in libs/myGEOMETRY/myROT.h
// (R_frm_quat, R_frm_quat_v2, R_frm_quat_v3). Needs no Virtuose and no PyBullet server.
//
// For each test quaternion q = (x, y, z, w) it builds R and checks, to 1e-6:
//   - R^T R = I      (orthonormal)
//   - det R = +1     (proper rotation)
//   - R equals Eigen::Quaterniond(w, x, y, z).toRotationMatrix()
// It prints PASS/FAIL per function and case, and returns 1 if any check fails.

#include <iostream>
#include <iomanip>
#include <random>
#include <string>
#include <cmath>
#include <myGEOMETRY/myROT.h>

static const double TOL = 1e-6;
static int n_checks = 0;
static int n_fail = 0;

static void check(const std::string& name, double x, double y, double z, double w)
{
    float X[4] = { (float)x, (float)y, (float)z, (float)w };
    Eigen::Vector4d Xv(x, y, z, w);
    btQuaternion Qb(x, y, z, w);
    Eigen::Matrix3d R_ref = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();

    const char* fn[3] = { "R_frm_quat", "R_frm_quat_v2", "R_frm_quat_v3" };
    Eigen::Matrix3d R[3] = { R_frm_quat(X), R_frm_quat_v2(Xv), R_frm_quat_v3(Qb) };

    for (int k = 0; k < 3; k++) {
        double orth = (R[k].transpose() * R[k] - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
        double det = R[k].determinant();
        double ref = (R[k] - R_ref).cwiseAbs().maxCoeff();
        bool ok = orth < TOL && std::abs(det - 1.0) < TOL && ref < TOL;

        n_checks++;
        if (!ok) n_fail++;
        std::cout << (ok ? "PASS  " : "FAIL  ") << std::left << std::setw(15) << fn[k] << std::setw(14) << name
                  << std::right << std::scientific << std::setprecision(1)
                  << "max|R^T R - I| = " << orth
                  << "   det R = " << std::fixed << std::setprecision(6) << det
                  << "   max|R - R_eigen| = " << std::scientific << std::setprecision(1) << ref
                  << std::defaultfloat << "\n";
    }
}

int main()
{
    const double s = std::sqrt(0.5);   // sin(45 deg) = cos(45 deg)

    check("identity", 0, 0, 0, 1);
    check("90deg about x", s, 0, 0, s);
    check("90deg about y", 0, s, 0, s);
    check("90deg about z", 0, 0, s, s);

    std::mt19937 gen(42);   // fixed seed so the output is reproducible
    std::normal_distribution<double> N(0.0, 1.0);
    for (int i = 0; i < 3; i++) {
        double q[4] = { N(gen), N(gen), N(gen), N(gen) };
        double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        check("random #" + std::to_string(i + 1), q[0] / n, q[1] / n, q[2] / n, q[3] / n);
    }

    std::cout << "\n" << (n_fail == 0 ? "PASS" : "FAIL") << ": " << (n_checks - n_fail) << "/" << n_checks << " checks passed\n";
    return n_fail == 0 ? 0 : 1;
}
