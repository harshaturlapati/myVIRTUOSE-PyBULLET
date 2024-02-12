
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

    void evalCON() {
        api.getContactPoints(myC, &contactInfo);
        N_con = contactInfo.m_numContactPoints;
        std::cout << "number of contacts are : " << N_con << std::endl;

        if (N_con > 0)
        {
            //std::cout << contactInfo.m_contactPointData[j].m_bodyUniqueIdA << std::endl;
            //std::cout << contactInfo.m_contactPointData[j].m_contactDistance << std::endl;
            force[0] = 0;
            force[1] = 0;
            force[2] = 0;


            torque[0] = 0;
            torque[1] = 0;
            torque[2] = 0;

            for (int k = 0; k < N_con; k++)
            {

                float curr_v1[3], curr_f1;
                float curr_v2[3], curr_f2;
                float curr_v3[3], curr_f3;
                float f_i[3];
                f_i[0] = 0; f_i[1] = 0; f_i[2] = 0;
                for (int dim = 0; dim < 3; dim++) {
                    curr_f1 = contactInfo.m_contactPointData[k].m_linearFrictionForce1;
                    curr_f2 = contactInfo.m_contactPointData[k].m_linearFrictionForce2;
                    curr_f3 = contactInfo.m_contactPointData[k].m_normalForce;
                    curr_v1[dim] = contactInfo.m_contactPointData[k].m_linearFrictionDirection1[dim];
                    curr_v2[dim] = contactInfo.m_contactPointData[k].m_linearFrictionDirection2[dim];
                    curr_v3[dim] = contactInfo.m_contactPointData[k].m_contactNormalOnBInWS[dim];

                    if (dim == 0) {
                        f_i[dim] = curr_f1 * curr_v1[dim];
                        force[0] = force[0] + f_i[dim];
                    }
                    if (dim == 1) {
                        f_i[dim] = curr_f2 * curr_v2[dim];
                        force[dim] = force[dim] + f_i[dim];
                    }
                    if (dim == 2) {
                        f_i[dim] = curr_f3 * curr_v3[dim];
                        force[dim] = force[dim] + f_i[dim];
                    }

                }

                positiononB[0] = contactInfo.m_contactPointData[k].m_positionOnBInWS[0];
                positiononB[1] = contactInfo.m_contactPointData[k].m_positionOnBInWS[1];
                positiononB[2] = contactInfo.m_contactPointData[k].m_positionOnBInWS[2];

                std::cout << "contact location is: x = " << positiononB[0] << ", y = " << positiononB[1] << ", z = " << positiononB[2] << std::endl;
                //cross(positiononB,f_i)
                myCROSS(positiononB, f_i);

                torque[0] = cross_v1v2[0];
                torque[1] = cross_v1v2[1];
                torque[2] = cross_v1v2[2];

                //positiononB[0] - f_i[0]

                //std::cout << curr_f3 << std::endl;


                    // for all contacts
                    //torque_in_WS = ++cross(contactInfo.m_contactPointData[k].m_positionOnBInWS, force)

                //// position \in R3
                //contactInfo.m_contactPointData[k].m_positionOnBInWS;

                //// v1 \in R3
                //contactInfo.m_contactPointData[k].m_linearFrictionDirection1;
                //// v2 \in R3
                //contactInfo.m_contactPointData[k].m_linearFrictionDirection2;
                //// v3 \in R3
                //contactInfo.m_contactPointData[k].m_contactNormalOnBInWS;

                //// f1 \in R1
                //contactInfo.m_contactPointData[k].m_linearFrictionForce1;
                //// f2 \in R1
                //contactInfo.m_contactPointData[k].m_linearFrictionForce2;
                //// f3 \in R1
                //contactInfo.m_contactPointData[k].m_normalForce;


                /*f = contactInfo.m_contactPointData[k].m_normalForce;
                for (int l = 0; l < 3; l++) {
                    positiononA[l] = contactInfo.m_contactPointData[k].m_positionOnAInWS[l];
                    positiononB[l] = contactInfo.m_contactPointData[k].m_positionOnBInWS[l];
                }
                std::cout << "contact location on A in WS is : x =" << positiononA[0] << ": y = " << positiononA[1] << ": z = " << positiononA[2] << std::endl;
                std::cout << "contact location on B in WS is : x =" << positiononB[0] << ": y = " << positiononB[1] << ": z = " << positiononB[2] << std::endl;
                std::cout << "contact number " << k << " is : " << f << std::endl;*/
            }
            std::cout << "Total force is : x = " << force[0] << ", y = " << force[1] << ", z = " << force[2] << std::endl;
            std::cout << "Total torque is : x = " << torque[0] << ", y = " << torque[1] << ", z = " << torque[2] << std::endl;
        }

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
        SIM.evalCON();
        //SIM.api.resetBasePositionAndOrientation(SIM.actor, btVector3(btScalar(0), btScalar(0), btScalar(0.1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));

        //SIM.api.applyExternalForce(SIM.actor, -1, btVector3(btScalar(0), btScalar(0), btScalar(-0.1)), btVector3(btScalar(0), btScalar(0), btScalar(0)), 0);
        
        
        SIM.api.stepSimulation();
        //Sleep(1);
    }

}

