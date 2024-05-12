#include <myINCLUDES.h>                 // baked in
#include "VirtuoseAPI.h"                // comes from Haption
#include <myVIRTUOSE_v3.h>              // baked in
#include <myVIRTUOSE_UDP.h>             // baked in
#include <myVIRTUOSE_LOGGING.h>         // baked in
#include <myBULLET_v4.h>


int main()
{

    float myDT = 0.01f;

    // Right Arm desired configuration
    // 192.168.101.53
    // Port to API : 53211
        
    // Left Arm desired configuration
    // 192.168.100.53
    // Port to API : 53210

    // // // LEFT ARM
    // UDP class object
    myVIRTUOSE_UDP myUDP_L(27015, 27016, "127.0.0.1", "127.0.0.1");
    myUDP_L.setup_UDP();

    // Virtuose object attributes
    const char* myPORT_L = "127.0.0.1#53210";
    float myFORCEFACTOR_L = 1.0f, mySPEEDFACTOR_L = 1.0f;

    // Virtuose object definition
    float k_L = 10;
    ARM ARM_L("127.0.0.1#53210", myFORCEFACTOR_L, mySPEEDFACTOR_L, myDT, k_L);
    ARM_L.name = "ARM_L";
    ARM_L.quick_start(); // always needs to be done
    ARM_L.debug_getPOS(); // always needs to be done


    // // // RIGHT ARM
    // UDP class object
    myVIRTUOSE_UDP myUDP_R(27017, 27018, "127.0.0.1", "127.0.0.1");
    myUDP_R.setup_UDP();

    // Virtuose object attributes
    const char* myPORT_R = "127.0.0.1#53210"; // change to 53211 tomorrow
    float myFORCEFACTOR_R = 1.0f, mySPEEDFACTOR_R = 1.0f;

    // Virtuose object definition
    float k_R = 10;
    ARM ARM_R("127.0.0.1#53210", myFORCEFACTOR_R, mySPEEDFACTOR_R, myDT, k_R);
    ARM_R.name = "ARM_R";
    ARM_R.quick_start(); // always needs to be done
    ARM_R.debug_getPOS(); // always needs to be done



    

    // Impedance control parameters
    
    float b_r = 0.5;
    float b = 4;
    myBULLET SIM(myDT, b, b_r);

    // to be sent to Haption at the end - may consider making this part of the CMD structure?
    
    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000)) {

        // Query Haption state and output through UDP
        myUDP_R.UDP_send_recv_v3(ARM_R.getPOS()); // getPOS() computes frame H
        myUDP_L.UDP_send_recv_v3(ARM_L.getPOS()); // getPOS() computes frame H
        
        // Compute object frame O
        SIM.getSIM_state();

        // Computer f_i_plus
        ARM_L.compute_f_cmd(SIM.O[0]); // need to make this a loop
        ARM_R.compute_f_cmd(SIM.O[1]); // need to make this a loop

        // Apply forces in simulation
        SIM.apply_control_forces_DUAL(ARM_L.cmd.f_i_plus, ARM_R.cmd.f_i_plus, ARM_L.cmd.e_i, ARM_R.cmd.e_i);

        // Apply forces on handle
        ARM_L.render_W_cmd();
        ARM_R.render_W_cmd();
     
        // Simulation step
        SIM.api.stepSimulation();
        Sleep(myDT);
    }

    // Haption clean up

    myUDP_R.cleanup();
    ARM_R.quick_stop();

    return 0;
}