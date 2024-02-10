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
#include <cmath>

int runScripts(char str[])
{
    PROCESS_INFORMATION pi;
    STARTUPINFO si;

    ZeroMemory(&si, sizeof(STARTUPINFO));
    si.cb = sizeof(STARTUPINFO);

    /*
    si.cbReserved2 = 0;
    si.dwFillAttribute = 0;
    si.dwFlags = 0;
    si.dwX = 0;
    si.dwXCountChars = 0;
    si.dwXSize = 0;
    si.dwY = 0;
    si.dwYCountChars = 0;
    si.dwYSize = 0;
    si.dwY = 0;
    si.dwYCountChars = 0;
    si.hStdError = NULL;
    si.hStdInput = NULL;
    si.hStdOutput = NULL;
    si.lpDesktop = NULL;
    si.lpReserved = NULL;
    si.lpReserved2 = NULL;
    si.lpTitle = NULL;
    si.wShowWindow = 0;
    */

    if (CreateProcess("C:\\Windows\\System32\\cmd.exe", (LPSTR)str, NULL, NULL, 0, 0, NULL, NULL, &si, &pi))
    {
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        return 1; //success
    }
    else
    {
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        return 0; //failure
    }
}

class myPyBULLET {
public:
    float time_step;
    b3PhysicsClientHandle client;
    b3RobotSimulatorClientAPI_InternalData data;
    b3RobotSimulatorClientAPI_NoDirect api;

    void init() {
        b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
        if (!b3CanSubmitCommand(client))
        {
            printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
            exit(0);
        }
        data.m_physicsClientHandle = client;
        data.m_guiHelper = 0;
        api.setInternalData(&data);
        api.setTimeStep(time_step);
        api.resetSimulation();
    }

    //int loadURDF(char* str_input) {
    //    return api.loadURDF(str_input);
    //}

    //bool applyExternalForce(int cube, int linkIndex, btVector3 force_in, btVector3 position_in, int flag_in){
    //    return api.applyExternalForce(cube, linkIndex, force_in, position_in, flag_in);
    //}

    ///*bool getCON(int cube, int linkIndex, btVector3 force_in, btVector3 position_in, int flag_in) {
    //    
    //    return api.applyExternalForce(cube, linkIndex, force_in, position_in, flag_in);
    //}*/

    //void stepSimulation(){
    //    api.stepSimulation();
    //}

    myPyBULLET(float time_step_in) {
        time_step = time_step_in;
        init();
    }
};

int main()
{

    //int ret = runScripts("/K python -m pybullet_utils.runServer");
    //printf("%d\n", ret);
    //system("python -m pybullet_utils.runServer");
    //WinExec("python -m pybullet_utils.runServer", 1);

    b3RobotSimulatorGetContactPointsArgs args;
    args.m_bodyUniqueIdA = 1;
    args.m_bodyUniqueIdB = 2;
    args.m_linkIndexA = -1;
    args.m_linkIndexB = -1;

    b3ContactInformation info;

    float ms = 1 / 1000;
    float step_time = 10*ms;
        myPyBULLET SIM(step_time);
    
        int plane = SIM.api.loadURDF("plane.urdf");
        int cube = SIM.api.loadURDF("cube.urdf");

        btVector3 position, force;
        //btQuaternion(const btScalar & _x, const btScalar & _y, const btScalar & _z, const btScalar & _w)
        btQuaternion orientation(0,0,0,1);

        force[0] = 0;
        force[1] = 0;
        force[2] = 0;
        
        position[0] = 0;
        position[1] = 0;
        position[2] = 0;
        float force1;
        
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        SIM.api.setRealTimeSimulation(0);
    int data_count = 0;
    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000))
    {
        SIM.api.resetBasePositionAndOrientation(1, position, orientation);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 
        SIM.api.getContactPoints(args, &info);
        position[1] = sin(std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());
        //td::cout << position[2] << std::endl;
        //std::cout << &info.m_contactPointData->m_bodyUniqueIdA << std::endl;

        //force1 = ;
        std::cout << &info.m_contactPointData->m_normalForce << std::endl;

       //SIM.api.applyExternalForce(cube, -1, force, position, 0);
        SIM.api.stepSimulation();
        Sleep(step_time);
        data_count = data_count + 1;
    }

    SIM.api.disconnect();
    return 0;
}
