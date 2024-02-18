#include <myINCLUDES.h>         // baked in
#include "VirtuoseAPI.h"
#include <myVIRTUOSE_v2.h>         // baked in
#include <myVIRTUOSE_UDP.h>     // baked in
#include <myVIRTUOSE_LOGGING.h> // baked in
#include <myBULLET.h>


int main()
{
    // UDP class object
    myVIRTUOSE_UDP myUDP(27017, 27018, "127.0.0.1", "127.0.0.1");
    float input_pos[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f };
    myUDP.setup_UDP();

    // Virtuose object attributes
    const char* myPORT = "127.0.0.1#53210";
    float myFORCEFACTOR = 1.0f, mySPEEDFACTOR = 1.0f, myDT = 0.01f;

    // Virtuose object definition
    ARM RightARM("127.0.0.1#53210", myFORCEFACTOR, mySPEEDFACTOR, myDT);
    RightARM.name = "RightARM";
    RightARM.quick_start();
    RightARM.debug_getPOS();

    // Impedance control parameters
    float k = 0.1f, b = 0.1f;

    // Command structure object
    CMD cmd_R(k, b);

    for (int i = 0; i < 7; i++) {
        cmd_R.X_d[i] = RightARM.getPOS()[i];
    }

    int duration = 1000;
    myVIRTUOSE_LOG RightARM_LOG(duration); // duration input

    myWRITE_VIRT_LOG Right_LOG_writer(duration);

    int data_count = 0;

    /*float myBULLET_k = 8;
    float myBULLET_b = 0.5;*/

    //float myBULLET_k = 50;
    //float myBULLET_b = 3;

    float myBULLET_k = 100;
    float myBULLET_b = 4;
    myBULLET SIM(myDT, myBULLET_k, myBULLET_b);
    btVector3 pos;
    btQuaternion quat;

    float f_HAPTION_CMD[6];
        f_HAPTION_CMD[0] = 0;
        f_HAPTION_CMD[1] = 0;
        f_HAPTION_CMD[2] = 0;
        f_HAPTION_CMD[3] = 0;
        f_HAPTION_CMD[4] = 0;
        f_HAPTION_CMD[5] = 0;

        float m_i_n[3][3];
        m_i_n[0][0] = 1; m_i_n[0][1] = 0; m_i_n[0][2] = 0;
        m_i_n[1][0] = 0; m_i_n[1][1] = 1; m_i_n[1][2] = 0;
        m_i_n[2][0] = 0; m_i_n[2][1] = 0; m_i_n[2][2] = 1;

        float* vec;
        float M_I_N[3];
        float A[3][3], x[3];
        float CMD_virtfix_p[3], virtfix_p[3], virtfix_f[3];
        float kr_CMD = 10;
        float br_CMD = 0.5;

        float arm_quat[4],armR[3][3];

    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000)) {

        // Haption stuff
        myUDP.UDP_send_recv_v3(RightARM.getPOS());

        //cmd_R.P_trn(cmd_R.X_d, RightARM.X); // needed if you wanted to do impedance control with Haption's initial state


        RightARM_LOG.write2LOG(data_count, cmd_R.X, cmd_R.f, myUDP.UDP_f);
        data_count = data_count + 1;

        // PyBullet stuff

        SIM.p_cmd[0] = RightARM.X[0];
        SIM.p_cmd[1] = RightARM.X[1];
        SIM.p_cmd[2] = RightARM.X[2];


        arm_quat[0] = RightARM.X[3];
        arm_quat[1] = RightARM.X[4];
        arm_quat[2] = RightARM.X[5];
        arm_quat[3] = RightARM.X[6];

        RightARM.getR(arm_quat[0], arm_quat[1], arm_quat[2], arm_quat[3]);


        SIM.evalCON();

        //printf("works\n");

        SIM.f_cmd[0] = SIM.k * (SIM.p_cmd[0] - SIM.pos_actor[0]) - SIM.b * (SIM.pdot_actor[0]);
        SIM.f_cmd[1] = SIM.k * (SIM.p_cmd[1] - SIM.pos_actor[1]) - SIM.b * (SIM.pdot_actor[1]);
        SIM.f_cmd[2] = SIM.k * (SIM.p_cmd[2] - SIM.pos_actor[2]) - SIM.b * (SIM.pdot_actor[2]);

        SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(SIM.f_cmd[0]), btScalar(SIM.f_cmd[1]), btScalar(SIM.f_cmd[2])), btVector3(btScalar(SIM.pos_actor[0]), btScalar(SIM.pos_actor[1]), btScalar(SIM.pos_actor[2])), 0);
        //SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(SIM.f_cmd[0]), btScalar(SIM.f_cmd[1]), btScalar(SIM.f_cmd[2])), btVector3(btScalar(0), btScalar(0), btScalar(0)), 0);

        //SIM.R_actor[0][0]

        float A[3][3], x[3];

        for (int hook = 0; hook < 3; hook++) {
            x[0] = m_i_n[hook][0];
            x[1] = m_i_n[hook][1];
            x[2] = m_i_n[hook][2];

            float Ax[3];
            Ax[0] = SIM.R_actor[0][0] * x[0] + SIM.R_actor[0][1] * x[1] + SIM.R_actor[0][2] * x[2];
            Ax[1] = SIM.R_actor[1][0] * x[0] + SIM.R_actor[1][1] * x[1] + SIM.R_actor[1][2] * x[2];
            Ax[2] = SIM.R_actor[2][0] * x[0] + SIM.R_actor[2][1] * x[1] + SIM.R_actor[2][2] * x[2];

            virtfix_p[0] = SIM.pos_actor[0] + Ax[0];
            virtfix_p[1] = SIM.pos_actor[1] + Ax[1];
            virtfix_p[2] = SIM.pos_actor[2] + Ax[2];

            float Bx[3];

            Bx[0] = RightARM.ARM_R[0][0] * x[0] + RightARM.ARM_R[0][1] * x[1] + RightARM.ARM_R[0][2] * x[2];
            Bx[1] = RightARM.ARM_R[1][0] * x[0] + RightARM.ARM_R[1][1] * x[1] + RightARM.ARM_R[1][2] * x[2];
            Bx[2] = RightARM.ARM_R[2][0] * x[0] + RightARM.ARM_R[2][1] * x[1] + RightARM.ARM_R[2][2] * x[2];


            CMD_virtfix_p[0] = SIM.p_cmd[0] + Bx[0];
            CMD_virtfix_p[1] = SIM.p_cmd[1] + Bx[1];
            CMD_virtfix_p[2] = SIM.p_cmd[2] + Bx[2];

            virtfix_f[0] = kr_CMD * (CMD_virtfix_p[0] - virtfix_p[0]);
            virtfix_f[1] = kr_CMD * (CMD_virtfix_p[1] - virtfix_p[1]);
            virtfix_f[2] = kr_CMD * (CMD_virtfix_p[2] - virtfix_p[2]);

            SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(virtfix_f[0]), btScalar(virtfix_f[1]), btScalar(virtfix_f[2])), btVector3(btScalar(virtfix_p[0]), btScalar(virtfix_p[1]), btScalar(virtfix_p[2])), 0);

        }

        SIM.api.applyExternalTorque(SIM.actor, -1, btVector3(btScalar(-br_CMD * (SIM.omega_actor[0])), btScalar(-br_CMD * (SIM.omega_actor[1])), btScalar(-br_CMD * (SIM.omega_actor[2]))), 0);
        //(SIM.actor, -1, btVector3(btScalar(-br_CMD * (SIM.omega_actor[0])), btScalar(-br_CMD * (SIM.omega_actor[1])), btScalar(-br_CMD * (SIM.omega_actor[2]))), btVector3(btScalar(SIM.pos_actor[0]), btScalar(SIM.pos_actor[1]), btScalar(SIM.pos_actor[2])), 0);











        f_HAPTION_CMD[0] = SIM.force[0];
        f_HAPTION_CMD[1] = SIM.force[1];
        f_HAPTION_CMD[2] = SIM.force[2];
        if (SIM.t_prime2[0] * SIM.t_prime2[0] + SIM.t_prime2[1] * SIM.t_prime2[1] + SIM.t_prime2[2] * SIM.t_prime2[2] < 4) {
            std::cout << "safe" << std::endl;
            f_HAPTION_CMD[3] = SIM.t_prime2[0];
            f_HAPTION_CMD[4] = SIM.t_prime2[1];
            f_HAPTION_CMD[5] = SIM.t_prime2[2];
        }
        else
        {
            f_HAPTION_CMD[3] = 0;
            f_HAPTION_CMD[4] = 0;
            f_HAPTION_CMD[5] = 0;
            std::cout << "unsafe" << std::endl;
        }
            
        
        //std::cout << "Haption command force is : x = " << f_HAPTION_CMD[0] << ", y = " << f_HAPTION_CMD[1] << ", z = " << f_HAPTION_CMD[2] << std::endl;
        RightARM.sendCMD_f(f_HAPTION_CMD); // issues the commanded force and resets the force variable also

        SIM.api.stepSimulation();
        Sleep(myDT);
    }

    // Haption clean up

    Right_LOG_writer.write2FILE(RightARM_LOG);
    
    myUDP.cleanup();
    RightARM.quick_stop();

    return 0;
}