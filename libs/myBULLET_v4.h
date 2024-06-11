#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include <Eigen/Dense>
#include <vector> 

class myBULLET {
public:
    float time_step;
    b3RobotSimulatorClientAPI_NoDirect api;
    b3PhysicsClientHandle client;
    b3RobotSimulatorClientAPI_InternalData data;
    b3RobotSimulatorChangeDynamicsArgs dyn_args;
    b3RobotSimulatorGetContactPointsArgs myC;
    b3ContactInformation contactInfo;
    int world;
    vector<int> actor;
    vector<btVector3> p_O, pdot_O, omega_O;
    vector<btQuaternion> quat_O;

    float b, b_r;

    vector<Eigen::Matrix3d> R_O;
    vector<Eigen::Matrix4d> O;

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
        //api.setGravity(btVector3(btScalar(0), btScalar(0), btScalar(-9.8)));
        api.setGravity(btVector3(btScalar(0), btScalar(0), btScalar(0)));
        world = api.loadURDF("plane.urdf");

        // Later I'd like this to be a for loop which takes in input from main() to initiate different objects as different actors - const char* inputs like "cube.urdf"
        actor.push_back(api.loadURDF("sphere2.urdf")); // sphere2
        actor.push_back(api.loadURDF("sphere2.urdf")); // sphere2


        /* Later I'd like these to be also initiated from main() and taken in as arguments to api.resetBasePositionAndOrientation()
        vector <btVector3> p_O_init;
        vector < btQuaternion > quat_init;*/

        api.resetBasePositionAndOrientation(actor[0], btVector3(btScalar(-1), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));
        api.resetBasePositionAndOrientation(actor[1], btVector3(btScalar(1), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));

        
        dyn_args.m_mass = 0.1;

        for (int i = 0; i < actor.size(); i++){
            api.changeDynamics(actor[i], -1, dyn_args);
            btVector3 p, pdot, omega;
            btQuaternion quat;
            Eigen::Matrix3d R;
            Eigen::Matrix4d T;

            quat_O.push_back(quat);
            p_O.push_back(p);
            pdot_O.push_back(pdot);
            omega_O.push_back(omega);
            R_O.push_back(R);
            O.push_back(T);
        }
        
        //myC.m_bodyUniqueIdA = actor; // VERY IMPORTANT to have the actor as bodyA.
        //myC.m_bodyUniqueIdB = world;
        //myC.m_linkIndexA = -1;
        //myC.m_linkIndexB = -1;

        
    }

    Eigen::Vector3d compose_p(btVector3 pos_actor_in) {
        Eigen::Vector3d p(pos_actor_in[0], pos_actor_in[1], pos_actor_in[2]);
        return p;
    }

    void getSIM_state() {
        for (int i = 0; i < actor.size(); i++) {
            
            api.getBasePositionAndOrientation(actor[i], p_O[i], quat_O[i]);
            api.getBaseVelocity(actor[i], pdot_O[i], omega_O[i]);

            R_O[i] = R_frm_quat_v3(quat_O[i]);
            O[i] = compose_bt(p_O[i], R_O[i]);
        }
    }

    void apply_control_forces_DUAL(CMD cmd_L, CMD cmd_R){
        vector<CMD> cmd_all;
        cmd_all.push_back(cmd_L);
        cmd_all.push_back(cmd_R);
        for (int j = 0; j < cmd_all.size(); j++) { // j = 0 left arm, j = 1 right arm
            for (int i = 0; i < cmd_all[j].e_i.size(); i++) {
                Eigen::Vector3d virt_hook = compose_p(p_O[j]) + R_O[j] * cmd_all[j].e_i[i];
                api.applyExternalForce(actor[j], -1, btVector3(btScalar(cmd_all[j].f_i_plus[i](0)), btScalar(cmd_all[j].f_i_plus[i](1)), btScalar(cmd_all[j].f_i_plus[i](2))), btVector3(btScalar(virt_hook(0)), btScalar(virt_hook(1)), btScalar(virt_hook(2))), 0);
            }

            // Linear Damping force to help stabilise
            api.applyExternalForce(actor[j], -1, -b * pdot_O[j], p_O[j], 0);

            // Rotational Damping torque to stabilise the object
            api.applyExternalTorque(actor[j], -1, -b_r * omega_O[j], 0);
        }

    }

    myBULLET(float time_step_in, float b_in, float b_r_in) {
        time_step = time_step_in;
        b = b_in;
        b_r = b_r_in;
        init();
    }
};