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
    vector < btVector3 > pos_actor, pdot_actor, omega_actor;
    vector < btQuaternion > quat_actor;

    float b, br;

    // Consider Eigen for lightweight rigid body dynamics computation
    vector < Eigen::Vector3d > p_O;
    vector < Eigen::Matrix3d > R_O;
    vector < Eigen::Matrix4d > O;
    vector < Eigen::Vector4d > quat_O;

    //float quat_O[4];

    // RBDL vs Boost RL

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
            api.getBasePositionAndOrientation(actor[i], pos_actor[i], quat_actor[i]);
            api.getBaseVelocity(actor[i], pdot_actor[i], omega_actor[i]);

            quat_O[i] = compose_quat(quat_actor[i]);
            R_O[i] = R_frm_quat_v2(quat_O[i]);
            p_O[i] = compose_p(pos_actor[i]);
            O[i] = compose(p_O[i], R_O[i]);
        }
    }

    //void apply_control_forces(vector<Eigen::Vector3d> f_i_plus, vector<Eigen::Vector3d> e_i) {

    //    for (int i = 0; i < e_i.size(); i++) {
    //        Eigen::Vector3d virt_hook = p_O + R_O * e_i[i];
    //        api.applyExternalForce(actor, -1, btVector3(btScalar(f_i_plus[i](0)), btScalar(f_i_plus[i](1)), btScalar(f_i_plus[i](2))), btVector3(btScalar(virt_hook(0)), btScalar(virt_hook(1)), btScalar(virt_hook(2))), 0);
    //    }

    //    // Linear Damping force to help stabilise
    //    api.applyExternalForce(actor, -1, btVector3(btScalar(-b * pdot_actor[0]), btScalar(-b * pdot_actor[1]), btScalar(-b * pdot_actor[2])), btVector3(btScalar(p_O(0)), btScalar(p_O(1)), btScalar(p_O(2))), 0);

    //    // Rotational Damping torque to stabilise the object
    //    api.applyExternalTorque(actor, -1, btVector3(btScalar(-br * (omega_actor[0])), btScalar(-br* (omega_actor[1])), btScalar(-br* (omega_actor[2]))), 0);
    //}

    void apply_control_forces_DUAL(vector<Eigen::Vector3d> f_i_plus_L, vector<Eigen::Vector3d> f_i_plus_R, vector<Eigen::Vector3d> e_i_L, vector<Eigen::Vector3d> e_i_R) {

        // Left ARM control
        for (int i = 0; i < e_i_L.size(); i++) {
            Eigen::Vector3d virt_hook = p_O[0] + R_O[0] * e_i_L[i];
            api.applyExternalForce(actor[0], -1, btVector3(btScalar(f_i_plus_L[i](0)), btScalar(f_i_plus_L[i](1)), btScalar(f_i_plus_L[i](2))), btVector3(btScalar(virt_hook(0)), btScalar(virt_hook(1)), btScalar(virt_hook(2))), 0);
        }

        // Linear Damping force to help stabilise
        api.applyExternalForce(actor[0], -1, btVector3(btScalar(-b * pdot_actor[0][0]), btScalar(-b * pdot_actor[0][1]), btScalar(-b * pdot_actor[0][2])), btVector3(btScalar(p_O[0](0)), btScalar(p_O[0](1)), btScalar(p_O[0](2))), 0);

        // Rotational Damping torque to stabilise the object
        api.applyExternalTorque(actor[0], -1, btVector3(btScalar(-br * (omega_actor[0][0])), btScalar(-br* (omega_actor[0][1])), btScalar(-br* (omega_actor[0][2]))), 0);

        // Right ARM control
        for (int i = 0; i < e_i_R.size(); i++) {
            Eigen::Vector3d virt_hook = p_O[1] + R_O[1] * e_i_R[i];
            api.applyExternalForce(actor[1], -1, btVector3(btScalar(f_i_plus_R[i](0)), btScalar(f_i_plus_R[i](1)), btScalar(f_i_plus_R[i](2))), btVector3(btScalar(virt_hook(0)), btScalar(virt_hook(1)), btScalar(virt_hook(2))), 0);
        }

        // Linear Damping force to help stabilise
        api.applyExternalForce(actor[1], -1, btVector3(btScalar(-b * pdot_actor[1][0]), btScalar(-b * pdot_actor[1][1]), btScalar(-b * pdot_actor[1][2])), btVector3(btScalar(p_O[1](0)), btScalar(p_O[1](1)), btScalar(p_O[1](2))), 0);

        // Rotational Damping torque to stabilise the object
        api.applyExternalTorque(actor[1], -1, btVector3(btScalar(-br * (omega_actor[1][0])), btScalar(-br * (omega_actor[1][1])), btScalar(-br * (omega_actor[1][2]))), 0);

    }

    myBULLET(float time_step_in, float b_in, float br_in) {
        time_step = time_step_in;
        b = b_in;
        br = br_in;
        init();
    }
};