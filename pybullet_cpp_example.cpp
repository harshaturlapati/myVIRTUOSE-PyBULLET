#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include <vector>

#include <myINCLUDES.h>         // baked in
#include "VirtuoseAPI.h"
#include <myVIRTUOSE.h>         // baked in
#include <myVIRTUOSE_UDP.h>     // baked in
#include <myVIRTUOSE_LOGGING.h> // baked in

#include <Windows.h>

int main()
{
    // UDP class object
    myVIRTUOSE_UDP myUDP(27017, 27018, "127.0.0.1", "127.0.0.1");
    float input_pos[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f };
    myUDP.setup_UDP();

    // Virtuose object attributes
    const char* myPORT = "127.0.0.1#53210";
    float myFORCEFACTOR = 1.0f, mySPEEDFACTOR = 1.0f, myDT = 0.001f;

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
    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000))
    {
        myUDP.UDP_send_recv_v3(RightARM.getPOS());

        cmd_R.P_trn(cmd_R.X_d, RightARM.X);

        RightARM_LOG.write2LOG(data_count, cmd_R.X, cmd_R.f, myUDP.UDP_f);

        // RightARM.sendCMD_f(cmd_R.f); // issues the commanded force and resets the force variable also

        data_count = data_count + 1;
    }

    Right_LOG_writer.write2FILE(RightARM_LOG);

    myUDP.cleanup();
    RightARM.quick_stop();

    b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
    if (!b3CanSubmitCommand(client))
    {
        printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
        exit(0);
    }
    b3RobotSimulatorClientAPI_InternalData data;
    data.m_physicsClientHandle = client;
    data.m_guiHelper = 0;
    b3RobotSimulatorClientAPI_NoDirect api;
    api.setInternalData(&data);
    api.setTimeStep(0.001f);
    api.resetSimulation();

    //b3RobotSimulatorLoadUrdfFileArgs args = b3RobotSimulatorLoadUrdfFileArgs();
    //int plane = api.loadURDF("plane.urdf");


    btVector3 m_startPosition(0, 0, 1);
    btQuaternion m_startOrientation(0, 0, 0, 0);

    //args.m_startPosition = m_startPosition;
    //args.m_startOrientation = m_startOrientation;

   // int ball = api.loadURDF("plane.urdf", b3RobotSimulatorLoadUrdfFileArgs(m_startPosition, m_startOrientation));
    int plane = api.loadURDF("plane.urdf");
    int cube = api.loadURDF("cube.urdf");
    btVector3 position, force;
    force[0] = 0;
    force[1] = 0;
    force[2] = 0.001;
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    int i = 1;
    
    while (i < 100){
        api.applyExternalForce(cube, -1, force, position, 0);
        api.stepSimulation();
        Sleep(1);
        i = i + 1;
}
}
