#include <myINCLUDES.h>                 // baked in
#include "VirtuoseAPI.h"                // comes from Haption
#include <myVIRTUOSE_v2.h>              // baked in
#include <myVIRTUOSE_UDP.h>             // baked in
#include <myVIRTUOSE_LOGGING.h>         // baked in
#include <myBULLET_v2.h>
#include <myVirtuose_CMD_v2.h>

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
    float k_RIGHT = 100;
    ARM RightARM("127.0.0.1#53210", myFORCEFACTOR, mySPEEDFACTOR, myDT, k_RIGHT);
    RightARM.name = "RightARM";
    RightARM.quick_start(); // always needs to be done
    RightARM.debug_getPOS(); // always needs to be done

    // Impedance control parameters
    float myBULLET_k = 100;
    float myBULLET_b = 4;
    myBULLET SIM(myDT, myBULLET_k, myBULLET_b);

    // to be sent to Haption at the end - may consider making this part of the CMD structure?
    CMD cmd_R;

    float CMD_virtfix_p[3], virtfix_p[3], virtfix_f[3];
    float kr_CMD = 10;
    float br_CMD = 0.5;


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

    float arm_quat[4], armR[3][3];


    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000)) {

        // Query Haption state and output through UDP
        myUDP.UDP_send_recv_v3(RightARM.getPOS()); // getPOS() computes frame H
        
        // Compute object frame O
        SIM.getSIM_state();

        // Compute f_i_plus




        SIM.p_cmd[0] = RightARM.X[0];
        SIM.p_cmd[1] = RightARM.X[1];
        SIM.p_cmd[2] = RightARM.X[2];

        RightARM.updateR(); // updates Arm Rotation matrix from quaternion feedback - ensure its called only after queryPOS is done

        SIM.evalCON();

        //printf("works\n");

        // // // Command force at the center of the virtual object
        // Notice that these are all linear terms - springs and dampers to be connected between com object and com handle	
        SIM.f_cmd[0] = SIM.k * (SIM.p_cmd[0] - SIM.pos_actor[0]) - SIM.b * (SIM.pdot_actor[0]);
        SIM.f_cmd[1] = SIM.k * (SIM.p_cmd[1] - SIM.pos_actor[1]) - SIM.b * (SIM.pdot_actor[1]);
        SIM.f_cmd[2] = SIM.k * (SIM.p_cmd[2] - SIM.pos_actor[2]) - SIM.b * (SIM.pdot_actor[2]);

        SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(SIM.f_cmd[0]), btScalar(SIM.f_cmd[1]), btScalar(SIM.f_cmd[2])), btVector3(btScalar(SIM.pos_actor[0]), btScalar(SIM.pos_actor[1]), btScalar(SIM.pos_actor[2])), 0);
        //SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(SIM.f_cmd[0]), btScalar(SIM.f_cmd[1]), btScalar(SIM.f_cmd[2])), btVector3(btScalar(0), btScalar(0), btScalar(0)), 0);

        //SIM.R_actor[0][0]

        // // // Command forces at different points of the hook
        float A[3][3], x[3];

        for (int hook = 0; hook < 3; hook++) {
            x[0] = cmd_R.m_i_n[hook][0];
            x[1] = cmd_R.m_i_n[hook][1];
            x[2] = cmd_R.m_i_n[hook][2];

        	// Determining virtfix_p - object hook location in space frame
            float Ax[3];
            Ax[0] = SIM.R_actor[0][0] * x[0] + SIM.R_actor[0][1] * x[1] + SIM.R_actor[0][2] * x[2];
            Ax[1] = SIM.R_actor[1][0] * x[0] + SIM.R_actor[1][1] * x[1] + SIM.R_actor[1][2] * x[2];
            Ax[2] = SIM.R_actor[2][0] * x[0] + SIM.R_actor[2][1] * x[1] + SIM.R_actor[2][2] * x[2];

            virtfix_p[0] = SIM.pos_actor[0] + Ax[0];
            virtfix_p[1] = SIM.pos_actor[1] + Ax[1];
            virtfix_p[2] = SIM.pos_actor[2] + Ax[2];

        	// Determining CMD_virtfix_p - handle hook location in space frame
            float Bx[3];

            Bx[0] = RightARM.ARM_R[0][0] * x[0] + RightARM.ARM_R[0][1] * x[1] + RightARM.ARM_R[0][2] * x[2];
            Bx[1] = RightARM.ARM_R[1][0] * x[0] + RightARM.ARM_R[1][1] * x[1] + RightARM.ARM_R[1][2] * x[2];
            Bx[2] = RightARM.ARM_R[2][0] * x[0] + RightARM.ARM_R[2][1] * x[1] + RightARM.ARM_R[2][2] * x[2];


            CMD_virtfix_p[0] = SIM.p_cmd[0] + Bx[0];
            CMD_virtfix_p[1] = SIM.p_cmd[1] + Bx[1];
            CMD_virtfix_p[2] = SIM.p_cmd[2] + Bx[2];

        	// virtfix_f is the force acting at the hook in space frame
            virtfix_f[0] = kr_CMD * (CMD_virtfix_p[0] - virtfix_p[0]);
            virtfix_f[1] = kr_CMD * (CMD_virtfix_p[1] - virtfix_p[1]);
            virtfix_f[2] = kr_CMD * (CMD_virtfix_p[2] - virtfix_p[2]);

        	// apply virtfix_f at virtfix_p
            SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(virtfix_f[0]), btScalar(virtfix_f[1]), btScalar(virtfix_f[2])), btVector3(btScalar(virtfix_p[0]), btScalar(virtfix_p[1]), btScalar(virtfix_p[2])), 0);

        }

        // Apply a damping rotation torque to the object
        SIM.api.applyExternalTorque(SIM.actor, -1, btVector3(btScalar(-br_CMD * (SIM.omega_actor[0])), btScalar(-br_CMD * (SIM.omega_actor[1])), btScalar(-br_CMD * (SIM.omega_actor[2]))), 0);
        //(SIM.actor, -1, btVector3(btScalar(-br_CMD * (SIM.omega_actor[0])), btScalar(-br_CMD * (SIM.omega_actor[1])), btScalar(-br_CMD * (SIM.omega_actor[2]))), btVector3(btScalar(SIM.pos_actor[0]), btScalar(SIM.pos_actor[1]), btScalar(SIM.pos_actor[2])), 0);


        cmd_R.setF(SIM.force);
        cmd_R.setTau_safe(SIM.t_prime2);

        std::cout << "Haption command force is : x = " << f_HAPTION_CMD[0] << ", y = " << f_HAPTION_CMD[1] << ", z = " << f_HAPTION_CMD[2] << std::endl;
        RightARM.sendCMD_f(cmd_R.W); // issues the commanded force and resets the force variable also


     
        SIM.api.stepSimulation();
        Sleep(myDT);
    }

    // Haption clean up

    myUDP.cleanup();
    RightARM.quick_stop();

    return 0;
}