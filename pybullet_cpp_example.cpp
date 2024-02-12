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
    float force[3], torque[3];
    float v1[3], v2[3];
    float cross_v1v2[3];
    float positiononA[3], positiononB[3];
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

        api.resetBasePositionAndOrientation(actor, btVector3(btScalar(0), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));
        dyn_args.m_mass = 0.01;

        api.changeDynamics(actor, -1, dyn_args);

        myC.m_bodyUniqueIdA = actor; // VERY IMPORTANT to have the actor as bodyA.
        myC.m_bodyUniqueIdB = world;
        myC.m_linkIndexA = -1;
        myC.m_linkIndexB = -1;
    }

    void myCROSS(float v1_in[3], float v2_in[3]) {
        for (int n = 0; n < 3; n++) {
            v1[n] = v1_in[n];
            v2[n] = v2_in[n];
        }
        
        cross_v1v2[0] = v1[1] * v2[2] - v2[1] * v1[2];
        cross_v1v2[1] = v2[0] * v1[2] - v1[0] * v2[2];
        cross_v1v2[2] = v1[0] * v2[1] - v2[0] * v1[1];
        std::cout << cross_v1v2[0] << cross_v1v2[1] << cross_v1v2[2] << std::endl;
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

        //SIM.api.resetBasePositionAndOrientation(SIM.actor, btVector3(btScalar(0), btScalar(0), btScalar(0.1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));

        SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(0), btScalar(0), btScalar(-0.1)), btVector3(btScalar(0), btScalar(0), btScalar(0)), 0);
        SIM.api.getContactPoints(SIM.myC, &SIM.contactInfo);
        SIM.N_con = SIM.contactInfo.m_numContactPoints;
        std::cout << "number of contacts are : " << SIM.N_con << std::endl;

        if (SIM.N_con > 0)
        {
            //std::cout << contactInfo.m_contactPointData[j].m_bodyUniqueIdA << std::endl;
            //std::cout << contactInfo.m_contactPointData[j].m_contactDistance << std::endl;
            SIM.force[0] = 0;
            SIM.force[1] = 0;
            SIM.force[2] = 0;


            SIM.torque[0] = 0;
            SIM.torque[1] = 0;
            SIM.torque[2] = 0;

            for (int k = 0; k < SIM.N_con; k++)
            {
                
                float curr_v1[3], curr_f1;
                float curr_v2[3], curr_f2;
                float curr_v3[3], curr_f3;
                float f_i[3];
                f_i[0] = 0;f_i[1] = 0;f_i[2] = 0;
                for (int dim = 0; dim < 3; dim++) {
                    curr_f1 = SIM.contactInfo.m_contactPointData[k].m_linearFrictionForce1;
                    curr_f2 = SIM.contactInfo.m_contactPointData[k].m_linearFrictionForce2;
                    curr_f3 = SIM.contactInfo.m_contactPointData[k].m_normalForce;
                    curr_v1[dim] = SIM.contactInfo.m_contactPointData[k].m_linearFrictionDirection1[dim];
                    curr_v2[dim] = SIM.contactInfo.m_contactPointData[k].m_linearFrictionDirection2[dim];
                    curr_v3[dim] = SIM.contactInfo.m_contactPointData[k].m_contactNormalOnBInWS[dim];
                    
                    if (dim == 0) {
                        f_i[dim] = curr_f1* curr_v1[dim];
                        SIM.force[0] = SIM.force[0] + f_i[dim];
                    }
                    if (dim == 1) {
                        f_i[dim] = curr_f2 * curr_v2[dim];
                        SIM.force[dim] = SIM.force[dim] + f_i[dim];
                    }
                    if (dim == 2) {
                        f_i[dim] = curr_f3 * curr_v3[dim];
                        SIM.force[dim] = SIM.force[dim] + f_i[dim];
                    }
                    
                }

                SIM.positiononB[0] = SIM.contactInfo.m_contactPointData[k].m_positionOnBInWS[0];
                SIM.positiononB[1] = SIM.contactInfo.m_contactPointData[k].m_positionOnBInWS[1];
                SIM.positiononB[2] = SIM.contactInfo.m_contactPointData[k].m_positionOnBInWS[2];
                
                std::cout << "contact location is: x = " << SIM.positiononB[0] << ", y = " << SIM.positiononB[1] << ", z = " << SIM.positiononB[2] << std::endl;
                //cross(SIM.positiononB,f_i)
                SIM.myCROSS(SIM.positiononB, f_i);

                SIM.torque[0] = SIM.cross_v1v2[0];
                SIM.torque[1] = SIM.cross_v1v2[1];
                SIM.torque[2] = SIM.cross_v1v2[2];
                
                //SIM.positiononB[0] - f_i[0]

                //std::cout << curr_f3 << std::endl;
                

                    // for all contacts
                    //torque_in_WS = ++cross(SIM.contactInfo.m_contactPointData[k].m_positionOnBInWS, SIM.force)

                //// position \in R3
                //SIM.contactInfo.m_contactPointData[k].m_positionOnBInWS;

                //// v1 \in R3
                //SIM.contactInfo.m_contactPointData[k].m_linearFrictionDirection1;
                //// v2 \in R3
                //SIM.contactInfo.m_contactPointData[k].m_linearFrictionDirection2;
                //// v3 \in R3
                //SIM.contactInfo.m_contactPointData[k].m_contactNormalOnBInWS;

                //// f1 \in R1
                //SIM.contactInfo.m_contactPointData[k].m_linearFrictionForce1;
                //// f2 \in R1
                //SIM.contactInfo.m_contactPointData[k].m_linearFrictionForce2;
                //// f3 \in R1
                //SIM.contactInfo.m_contactPointData[k].m_normalForce;


                /*f = SIM.contactInfo.m_contactPointData[k].m_normalForce;
                for (int l = 0; l < 3; l++) {
                    SIM.positiononA[l] = SIM.contactInfo.m_contactPointData[k].m_positionOnAInWS[l];
                    SIM.positiononB[l] = SIM.contactInfo.m_contactPointData[k].m_positionOnBInWS[l];
                }
                std::cout << "contact location on A in WS is : x =" << SIM.positiononA[0] << ": y = " << SIM.positiononA[1] << ": z = " << SIM.positiononA[2] << std::endl;
                std::cout << "contact location on B in WS is : x =" << SIM.positiononB[0] << ": y = " << SIM.positiononB[1] << ": z = " << SIM.positiononB[2] << std::endl;
                std::cout << "contact number " << k << " is : " << f << std::endl;*/
            }
            std::cout << "Total force is : x = " << SIM.force[0] << ", y = " << SIM.force[1] << ", z = " << SIM.force[2] << std::endl;
            std::cout << "Total torque is : x = " << SIM.torque[0] << ", y = " << SIM.torque[1] << ", z = " << SIM.torque[2] << std::endl;
        }
        
        SIM.api.stepSimulation();
        //Sleep(1);
    }

}

