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
    vector < int > actor;
    vector < btVector3 > p_O, pdot_O, omega_O;
    vector < btQuaternion > quat_O;

    float b, b_r;

    vector < Eigen::Matrix3d > R_O;
    vector < Eigen::Matrix4d > O;

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
        actor.push_back(api.loadURDF("cube.urdf")); // sphere2
        actor.push_back(api.loadURDF("cube.urdf")); // sphere2

        //btQuaternion

        /* Later I'd like these to be also initiated from main() and taken in as arguments to api.resetBasePositionAndOrientation()
        vector <btVector3> p_O_init;
        vector < btQuaternion > quat_init;*/

        api.resetBasePositionAndOrientation(actor[0], btVector3(btScalar(-1), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));
        api.resetBasePositionAndOrientation(actor[1], btVector3(btScalar(1), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));

        
        dyn_args.m_mass = 0.1;

        for (int i = 0; i < actor.size(); i++){
            api.changeDynamics(actor[i], -1, dyn_args);
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

    Eigen::Vector4d compose_quat(btQuaternion quat_actor_in) {
        Eigen::Vector4d quat;
        quat(0) = quat_actor_in.getX();
        quat(1) = quat_actor_in.getY();
        quat(2) = quat_actor_in.getZ();
        quat(3) = quat_actor_in.getW();
        return quat;
    }

    void getSIM_state() {
        for (int i = 0; i < actor.size(); i++) {
            api.getBasePositionAndOrientation(actor[i], p_O[i], quat_O[i]);
            api.getBaseVelocity(actor[i], pdot_O[i], omega_O[i]);

            R_O[i] = R_frm_quat_v3(quat_O[i]);
            O[i] = compose_bt(p_O[i], R_O[i]);
        }
    }

    void apply_control_forces_DUAL(vector<Eigen::Vector3d> f_i_plus_L, vector<Eigen::Vector3d> f_i_plus_R, vector<Eigen::Vector3d> e_i_L, vector<Eigen::Vector3d> e_i_R) {

        // Left ARM control
        for (int i = 0; i < e_i_L.size(); i++) {
            Eigen::Vector3d virt_hook = compose_p(p_O[0]) + R_O[0] * e_i_L[i];
            api.applyExternalForce(actor[0], -1, btVector3(btScalar(f_i_plus_L[i](0)), btScalar(f_i_plus_L[i](1)), btScalar(f_i_plus_L[i](2))), btVector3(btScalar(virt_hook(0)), btScalar(virt_hook(1)), btScalar(virt_hook(2))), 0);
        }

        // Linear Damping force to help stabilise
        api.applyExternalForce(actor[0], -1, -b * pdot_O[0], p_O[0], 0);

        // Rotational Damping torque to stabilise the object
        api.applyExternalTorque(actor[0], -1, -b_r * omega_O[0], 0);

        // Right ARM control
        for (int i = 0; i < e_i_R.size(); i++) {
            Eigen::Vector3d virt_hook = compose_p(p_O[1]) + R_O[1] * e_i_R[i];
            api.applyExternalForce(actor[1], -1, btVector3(btScalar(f_i_plus_R[i](0)), btScalar(f_i_plus_R[i](1)), btScalar(f_i_plus_R[i](2))), btVector3(btScalar(virt_hook(0)), btScalar(virt_hook(1)), btScalar(virt_hook(2))), 0);
        }

        // Linear Damping force to help stabilise
        api.applyExternalForce(actor[1], -1, -b * pdot_O[1], p_O[1], 0);

        // Rotational Damping torque to stabilise the object
        api.applyExternalTorque(actor[1], -1, -b_r * omega_O[1], 0);
    }

    myBULLET(float time_step_in, float b_in, float b_r_in) {
        time_step = time_step_in;
        b = b_in;
        b_r = b_r_in;
        init();
    }
};