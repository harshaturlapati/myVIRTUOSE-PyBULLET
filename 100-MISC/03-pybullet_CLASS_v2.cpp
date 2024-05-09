#include <myINCLUDES.h>                 // baked in
#include "VirtuoseAPI.h"                // comes from Haption
#include <myVIRTUOSE_v2.h>              // baked in
#include <myVIRTUOSE_UDP.h>             // baked in
#include <myVIRTUOSE_LOGGING.h>         // baked in
#include <myVirtuose_CMD.h>
#include <myBULLET_v2.h>

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


    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000)) {

        // Haption stuff
        myUDP.UDP_send_recv_v3(RightARM.getPOS()); // crucial, because getPOS queries Virtuose pose and updates state variables while also with UDP_send_recv_v3() relaying pose information to PORT 27017
        
        // commented out for writing logs - // RightARM_LOG.write2LOG(data_count, cmd_R.X, cmd_R.f, myUDP.UDP_f);
        // commented out for writing logs - // data_count = data_count + 1;

        // PyBullet stuff

        SIM.p_cmd[0] = RightARM.X[0];
        SIM.p_cmd[1] = RightARM.X[1];
        SIM.p_cmd[2] = RightARM.X[2];

        RightARM.updateR(); // updates Arm Rotation matrix from quaternion feedback - ensure its called only after queryPOS is done

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
            x[0] = cmd_R.m_i_n[hook][0];
            x[1] = cmd_R.m_i_n[hook][1];
            x[2] = cmd_R.m_i_n[hook][2];

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


        cmd_R.setF(SIM.force);
        cmd_R.setTau_safe(SIM.t_prime2);

        //std::cout << "Haption command force is : x = " << f_HAPTION_CMD[0] << ", y = " << f_HAPTION_CMD[1] << ", z = " << f_HAPTION_CMD[2] << std::endl;
        RightARM.sendCMD_f(cmd_R.W); // issues the commanded force and resets the force variable also

        SIM.api.stepSimulation();
        Sleep(myDT);
    }

    // Haption clean up

    // commented out for writing logs - // Right_LOG_writer.write2FILE(RightARM_LOG);

    myUDP.cleanup();
    RightARM.quick_stop();

    return 0;
}