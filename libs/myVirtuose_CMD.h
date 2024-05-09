class CMD {

private:


public:
    float k, b;
    float K[7], B[7];

    float X_d[7];
    float X[7], Xdot[7];
    float F_e[6] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };

    float dX[7];

    float W[6] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
    float tau2_limit;

    float m_i_n[3][3];

    void set_F_d(float F_e_input[6]) {
        for (int i = 0; i < 6; i++) {
            F_e[i] = F_e_input[i];
        }
    }

    void set_X_d(float X_d_input[7]) {
        for (int i = 0; i < 7; i++) {
            X_d[i] = X_d_input[i];
        }
    }

    void set_X(float X_input[7]) {
        for (int i = 0; i < 7; i++) {
            X[i] = X_input[i];
        }
    }
    void set_Xdot(float Xdot_input[7]) {
        for (int i = 0; i < 7; i++) {
            Xdot[i] = Xdot_input[i];
        }
    }
    void set_ext_F(float ext_F_input[6]) {
        for (int i = 0; i < 6; i++) {
            F_e[i] = ext_F_input[i];
        }
    }

    float* P_trn(float X_d_input[7], float X_input[7])
    {
        set_X_d(X_d_input);
        set_X(X_input);
        for (int i = 0; i < 3; i++)
        {
            dX[i] = X_d[i] - X[i];
            W[i] = K[i] * dX[i] + F_e[i];
        }
        return W;
    }

    float* PD_trn(float X_d_input[7], float X_input[7])
    {
        set_X_d(X_d_input);
        set_X(X_input);
        for (int i = 0; i < 3; i++)
        {
            dX[i] = X_d[i] - X[i];
            W[i] = K[i] * dX[i] - B[i] * Xdot[i] + F_e[i]; // While pushing the human towards the X_d, also give an opposite force along the current velocity
        }
        return W;
    }

    void set_kb(float k_input, float b_input) {
        k = k_input;
        b = b_input;
        for (int i = 0; i < 7; i++)
        {
            K[i] = k;
            B[i] = b;
        }
    }

    void set_KB(float K_input[7], float B_input[7]) {
        for (int i = 0; i < 7; i++)
        {
            K[i] = K_input[i];
            B[i] = B_input[i];
        }
    }

    void setF(float f_input[3]) {
        for (int i = 0; i < 3; i++)
        {
            W[i] = f_input[i];
        }
    }

    void setTau(float tau_input[3]) {
        for (int i = 3; i < 6; i++)
        {
            W[i] = tau_input[i-3];
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
            W[i] = 0;
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

    void set_m_i_n_default() {
        m_i_n[0][0] = 1; m_i_n[0][1] = 0; m_i_n[0][2] = 0;
        m_i_n[1][0] = 0; m_i_n[1][1] = 1; m_i_n[1][2] = 0;
        m_i_n[2][0] = 0; m_i_n[2][1] = 0; m_i_n[2][2] = 1;
    }

    CMD() { // default constructor
        tau2_limit = 4;
        set_m_i_n_default();
        for (int i = 0; i < 6; i++)
        {
            W[i] = 0;
        }
    }

    CMD(float k_input, float b_input) {
        k = k_input;
        b = b_input;
        tau2_limit = 4;
        set_m_i_n_default();
        for (int i = 0; i < 7; i++)
        {
            K[i] = k;
            B[i] = b;
        }
    }

    CMD(float K_input[7], float B_input[7]) {
        tau2_limit = 4;
        set_m_i_n_default();
        for (int i = 0; i < 7; i++)
        {
            K[i] = K_input[i];
            B[i] = B_input[i];
        }
    }
};
