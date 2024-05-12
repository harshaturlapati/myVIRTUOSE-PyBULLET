#include <myINCLUDES.h>                 // baked in
#include "VirtuoseAPI.h"                // comes from Haption
#include <myVIRTUOSE_v3.h>              // baked in
#include <myVIRTUOSE_UDP.h>             // baked in
#include <myVIRTUOSE_LOGGING.h>         // baked in
#include <myBULLET_v3.h>

int main()
{
    // UDP class object
    myVIRTUOSE_UDP myUDP(27017, 27018, "127.0.0.1", "127.0.0.1");
    float input_pos[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f };
    myUDP.setup_UDP();

    // Virtuose object attributes
    const char* myPORT = "127.0.0.1#53210";
    float myFORCEFACTOR = 1.0f, mySPEEDFACTOR = 1.0f, delta_t = 0.01f;

    // Virtuose object definition
    float k = 10;
    ARM RightARM("127.0.0.1#53210", myFORCEFACTOR, mySPEEDFACTOR, delta_t, k);
    RightARM.name = "RightARM";
    RightARM.quick_start(); // always needs to be done
    RightARM.debug_getPOS(); // always needs to be done
    

    // Impedance control parameters
    
    float b_r = 0.5;
    float b = 4;
    myBULLET SIM(delta_t, b, b_r);

    // to be sent to Haption at the end - may consider making this part of the CMD structure?
    
    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000)) {

        // Query Haption state and output through UDP
        myUDP.UDP_send_recv_v3(RightARM.getPOS()); // getPOS() computes frame H
        
        // Compute object frame O
        SIM.getSIM_state();

        // Computer f_i_plus
        RightARM.compute_f_cmd(SIM.O);

        // Apply forces in simulation
        SIM.apply_control_forces(RightARM.cmd.f_i_plus,RightARM.cmd.e_i);

        // Apply forces on handle
        RightARM.render_W_cmd();
     
        // Simulation step
        SIM.api.stepSimulation();
        Sleep(delta_t);
    }

    // Haption clean up

    myUDP.cleanup();
    RightARM.quick_stop();

    return 0;
}