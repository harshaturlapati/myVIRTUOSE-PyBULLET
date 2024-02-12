//#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
//#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
//#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
//#include <vector>
//
//#include <myINCLUDES.h>         // baked in
//#include "VirtuoseAPI.h"
//#include <myVIRTUOSE.h>         // baked in
//#include <myVIRTUOSE_UDP.h>     // baked in
//#include <myVIRTUOSE_LOGGING.h> // baked in
//
//#include <Windows.h>
//#include <cmath>
//
//class myPyBULLET {
//public:
//    float time_step;
//    b3PhysicsClientHandle client;
//    b3RobotSimulatorClientAPI_InternalData data;
//    b3RobotSimulatorClientAPI_NoDirect api;
//
//    void init() {
//        b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
//        if (!b3CanSubmitCommand(client))
//        {
//            printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
//            exit(0);
//        }
//        data.m_physicsClientHandle = client;
//        data.m_guiHelper = 0;
//        api.setInternalData(&data);
//        api.setTimeStep(time_step);
//        api.resetSimulation();
//    }
//
//    //int loadURDF(char* str_input) {
//    //    return api.loadURDF(str_input);
//    //}
//
//    //bool applyExternalForce(int cube, int linkIndex, btVector3 force_in, btVector3 position_in, int flag_in){
//    //    return api.applyExternalForce(cube, linkIndex, force_in, position_in, flag_in);
//    //}
//
//    ///*bool getCON(int cube, int linkIndex, btVector3 force_in, btVector3 position_in, int flag_in) {
//    //    
//    //    return api.applyExternalForce(cube, linkIndex, force_in, position_in, flag_in);
//    //}*/
//
//    //void stepSimulation(){
//    //    api.stepSimulation();
//    //}
//
//    myPyBULLET(float time_step_in) {
//        time_step = time_step_in;
//        init();
//    }
//};
//
//int main()
//{
//
//
//    float ms = 1 / 1000;
//    float step_time = 10*ms;
//        
//    myPyBULLET SIM(step_time);
//    
//        int world = SIM.api.loadURDF("world.urdf");
//        int actor = SIM.api.loadURDF("sphere2.urdf");
//        SIM.api.setGravity(btVector3(btScalar(0), btScalar(0), btScalar(-9.8)));
//        SIM.api.resetBasePositionAndOrientation(actor, btVector3(btScalar(0), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));
//
//        b3RobotSimulatorChangeDynamicsArgs dyn_args;
//        dyn_args.m_mass = 10;
//
//        SIM.api.changeDynamics(actor, -1, dyn_args);
//        b3RobotSimulatorGetContactPointsArgs myC;
//        myC.m_bodyUniqueIdA = world;
//        myC.m_bodyUniqueIdB = actor;
//        myC.m_linkIndexA = -1;
//        myC.m_linkIndexB = -1;
//        b3ContactInformation contactInfo;
//
//        double force;
//        double positiononA[3], positiononB[3];
//
//    int data_count = 0;
//    printf("Press Q to exit loop\n");
//    while (!(GetKeyState('Q') & 0x8000))
//    {
//        //api.resetBasePositionAndOrientation(actor, btVector3(btScalar(0), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));
//
//        SIM.api.applyExternalForce(actor, -1, btVector3(btScalar(1), btScalar(1), btScalar(1)), btVector3(btScalar(0), btScalar(0), btScalar(0)), 0);    
//        SIM.api.getContactPoints(myC, &contactInfo);
//        int j = contactInfo.m_numContactPoints;
//        if (j > 0)
//        {
//            //std::cout << contactInfo.m_contactPointData[j].m_bodyUniqueIdA << std::endl;
//            //std::cout << contactInfo.m_contactPointData[j].m_contactDistance << std::endl;
//            for (int k = 0; k < j; k++)
//            {
//                force = contactInfo.m_contactPointData[k].m_normalForce;
//                for (int l = 0; l < 3; l++) {
//                    positiononA[l] = contactInfo.m_contactPointData[k].m_positionOnAInWS[l];
//                    positiononB[l] = contactInfo.m_contactPointData[k].m_positionOnBInWS[l];
//                }
//                std::cout << "contact location on A in WS is : x =" << positiononA[0] << ": y = " << positiononA[1] << ": z = " << positiononA[2] << std::endl;
//                std::cout << "contact location on B in WS is : x =" << positiononB[0] << ": y = " << positiononB[1] << ": z = " << positiononB[2] << std::endl;
//                std::cout << "contact number " << k << " is : " << force << std::endl;
//            }
//        }
//        std::cout << j << std::endl;
//        
//        SIM.api.stepSimulation();
//        Sleep(step_time);
//        data_count = data_count + 1;
//    }
//
//    //SIM.api.disconnect();
//    return 0;
//}

#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"

#include <vector>
#include <Windows.h>
#include <iostream>


class myBULLET {
public:
    float time_step;
    b3RobotSimulatorClientAPI_NoDirect api;
    b3PhysicsClientHandle client;
    b3RobotSimulatorClientAPI_InternalData data;
    b3RobotSimulatorChangeDynamicsArgs dyn_args;
    b3RobotSimulatorGetContactPointsArgs myC;
    b3ContactInformation contactInfo;
    int world, actor;
    double force[3], torque[3];
    double positiononA[3], positiononB[3];
    int N_con;

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
        api.setGravity(btVector3(btScalar(0), btScalar(0), btScalar(-9.8)));
        world = api.loadURDF("plane.urdf");
        actor = api.loadURDF("sphere2.urdf");

        //btQuaternion

        api.resetBasePositionAndOrientation(actor, btVector3(btScalar(0), btScalar(5), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));
        dyn_args.m_mass = 10;

        api.changeDynamics(actor, -1, dyn_args);

        myC.m_bodyUniqueIdA = world;
        myC.m_bodyUniqueIdB = actor;
        myC.m_linkIndexA = -1;
        myC.m_linkIndexB = -1;
    }


    myBULLET(float time_step_in) {
        time_step = time_step_in;
        init();
    }
};


int main()
{
    myBULLET SIM(0.01f);
    
    
    double f;

    printf("Press Q to exit loop\n");
    while (!(GetKeyState('Q') & 0x8000)){

        SIM.api.resetBasePositionAndOrientation(SIM.actor, btVector3(btScalar(0), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));

        //api.applyExternalForce(actor, -1, btVector3(btScalar(1), btScalar(1), btScalar(1)), btVector3(btScalar(0), btScalar(0), btScalar(0)), 0);
        SIM.api.getContactPoints(SIM.myC, &SIM.contactInfo);
        SIM.N_con = SIM.contactInfo.m_numContactPoints;
        std::cout << "number of contacts are : " << SIM.N_con << std::endl;

        if (SIM.N_con > 0)
        {
            //std::cout << contactInfo.m_contactPointData[j].m_bodyUniqueIdA << std::endl;
            //std::cout << contactInfo.m_contactPointData[j].m_contactDistance << std::endl;
            for (int k = 0; k < SIM.N_con; k++)
            {
                f = SIM.contactInfo.m_contactPointData[k].m_normalForce;
                for (int l = 0; l < 3; l++) {
                    SIM.positiononA[l] = SIM.contactInfo.m_contactPointData[k].m_positionOnAInWS[l];
                    SIM.positiononB[l] = SIM.contactInfo.m_contactPointData[k].m_positionOnBInWS[l];
                }
                std::cout << "contact location on A in WS is : x =" << SIM.positiononA[0] << ": y = " << SIM.positiononA[1] << ": z = " << SIM.positiononA[2] << std::endl;
                std::cout << "contact location on B in WS is : x =" << SIM.positiononB[0] << ": y = " << SIM.positiononB[1] << ": z = " << SIM.positiononB[2] << std::endl;
                std::cout << "contact number " << k << " is : " << f << std::endl;
            }
        }
        
        SIM.api.stepSimulation();
        Sleep(1);
    }

}

