#include <myINCLUDES.h>                 // baked in
#include "VirtuoseAPI.h"                // comes from Haption
#include <myVIRTUOSE_v4.h>              // including logger inside it
#include <myVIRTUOSE_UDP.h>             // baked in
#include <myBULLET_vv1.h>

int main()
{
    vector<Eigen::Vector3d> e_i, f_i_plus;

    
    f_i_plus.push_back({ 0.0f,0.0f,0.0f });
    e_i.push_back(Eigen::Vector3d(0, 0, 0)); // Seems this is VERY important
    f_i_plus.push_back({ 0.0f,0.0f,0.0f });
    e_i.push_back(Eigen::Vector3d(0.1, 0, 0));
    f_i_plus.push_back({ 0.0f,0.0f,0.0f });
    e_i.push_back(Eigen::Vector3d(-0.1, 0, 0));
    f_i_plus.push_back({ 0.0f,0.0f,0.0f });
    e_i.push_back(Eigen::Vector3d(0, 0.1, 0));
    f_i_plus.push_back({ 0.0f,0.0f,0.0f });
    e_i.push_back(Eigen::Vector3d(0, -0.1, 0));
    f_i_plus.push_back({ 0.0f,0.0f,0.0f });
    e_i.push_back(Eigen::Vector3d(0, 0, 0.1));
    f_i_plus.push_back({ 0.0f,0.0f,0.0f });
    e_i.push_back(Eigen::Vector3d(0, 0, -0.1)); // can add more e_i here if you want...

    float delta_t = 0.01;
    // Impedance control parameters

    float b_r = 0.5;
    float b = 2;
    myBULLET SIM(delta_t, b, b_r);

    int data_count = 0;

    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000)) {

        // Compute object frame O
        SIM.getSIM_state();

        SIM.apply_control_forces(f_i_plus, e_i);

        // Simulation step
        SIM.api.stepSimulation();

        data_count++;
        Sleep(delta_t);
    }

    SIM.close();

    return 0;
}