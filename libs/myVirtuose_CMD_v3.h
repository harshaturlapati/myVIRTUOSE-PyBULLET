#include <Eigen/Dense>
#include <vector> 
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
using namespace std;

class CMD {

private:


public:
    float k;

    float W[6] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
    float W_safe[6] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
    float tau2_limit;

    vector<Eigen::Vector3d> e_i, f_i_plus;
    vector<Eigen::Vector4d> e_i_tilde;
    Matrix34d Pi;

    void setF(float f_input[3]) {
        for (int i = 0; i < 3; i++)
        {
            W[i] = f_input[i];
        }
    }

    void setTau(float tau_input[3]) {
        for (int i = 3; i < 6; i++)
        {
            W_safe[i] = tau_input[i-3];
        }
    }

    void resetF() {
        for (int i = 0; i < 3; i++)
        {
            W[i] = 0;
        }
    }

    void resetTau() {
        for (int i = 3; i < 6; i++)
        {
            W_safe[i] = 0;
        }
    }

    void resetW() {
        for (int i = 0; i < 6; i++)
        {
            W[i] = 0;
        }
    }

    void setTau_safe(float tau_input[3]) {
        if (tau_input[0] * tau_input[0] + tau_input[1] * tau_input[1] + tau_input[2] * tau_input[2] < tau2_limit) {
            std::cout << "safe" << std::endl;
            setTau(tau_input);
        }
        else
        {
            resetTau();
            std::cout << "unsafe" << std::endl;
        }
    }

    void setTau2_limit(float tau2_limit_in) {
        tau2_limit = tau2_limit_in;
    }

    void set_e_i_default() {
        Pi << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0; // mask

        e_i.push_back(Eigen::Vector3d(0, 0, 0)); // Seems this is VERY important
        e_i.push_back(Eigen::Vector3d(1, 0, 0));
        e_i.push_back(Eigen::Vector3d(0, 1, 0));
        e_i.push_back(Eigen::Vector3d(0, 0, 1)); // can add more e_i here if you want...

        gen_e_i_tilde(); // depending on how you set e_i, e_i_tilde will be generated and kept
    }

    void gen_e_i_tilde() {
        Eigen::Vector4d dummy;
        for (int i = 0; i < e_i.size(); i++) {
            //std::cout << i << std::endl;
            //std::cout << "e-i looks like:\n" << e_i[i] << std::endl;
            dummy << e_i[i], 1;
            //std::cout << "e-i tilde looks like:\n" << dummy << std::endl;
            e_i_tilde.push_back(dummy);
            f_i_plus.push_back(0 * e_i[i]); // initialise f_i_plus to 0's
        }
        //std::cout << "e-i tilde looks like:\n" << e_i_tilde[0] << std::endl;
    }

    void check_safety(float W_in[7]) {
        float tau_check[3];
        for (int i = 0; i < 6; i++) {
            W[i] = W_in[i];
            W_safe[i] = W_in[i];
            if (i >= 3)
            {
                tau_check[i - 3] = W_in[i];
            }
        }

        setTau_safe(tau_check);

    }


    CMD() { // default constructor
        tau2_limit = 4;
        set_e_i_default();
        for (int i = 0; i < 6; i++)
        {
            W[i] = 0;
        }
    }

    CMD(float k_input) {
        tau2_limit = 4;
        k = k_input;
        set_e_i_default();
        for (int i = 0; i < 6; i++)
        {
            W[i] = 0;
        }
    }
};
