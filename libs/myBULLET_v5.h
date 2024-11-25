#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"


class myBULLET {
public:
    float delta_t;
    b3RobotSimulatorClientAPI_NoDirect api;
    b3PhysicsClientHandle client;
    b3RobotSimulatorClientAPI_InternalData data;
    b3RobotSimulatorChangeDynamicsArgs dyn_args;
    b3RobotSimulatorGetContactPointsArgs myC;
    b3ContactInformation contactInfo;
    int world, actor;

    btVector3 p_O, pdot_O, omega_O;
    btQuaternion quat_O;

    float b, b_r;

    // Consider Eigen for lightweight rigid body dynamics computation
    Eigen::Matrix3d R_O;
    Eigen::Matrix4d O;

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
        api.setTimeStep(delta_t);
        api.resetSimulation();
        //api.setGravity(btVector3(btScalar(0), btScalar(0), btScalar(-9.8)));
        api.setGravity(btVector3(btScalar(0), btScalar(0), btScalar(0)));
        world = api.loadURDF("plane.urdf");
        actor = api.loadURDF("cube.urdf"); // sphere2

        //btQuaternion

        api.resetBasePositionAndOrientation(actor, btVector3(btScalar(0), btScalar(0), btScalar(1)), btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1)));
        dyn_args.m_mass = 0.1;

        api.changeDynamics(actor, -1, dyn_args);

        myC.m_bodyUniqueIdA = actor; // VERY IMPORTANT to have the actor as bodyA.
        myC.m_bodyUniqueIdB = world;
        myC.m_linkIndexA = -1;
        myC.m_linkIndexB = -1;

    }

    Eigen::Vector3d compose_p(btVector3 pos_actor_in) {
        Eigen::Vector3d p(pos_actor_in[0], pos_actor_in[1], pos_actor_in[2]);
        return p;
    }

    void getSIM_state() {
        api.getBasePositionAndOrientation(actor, p_O, quat_O);
        api.getBaseVelocity(actor, pdot_O, omega_O);

        R_O = R_frm_quat_v3(quat_O);
        O = compose_bt(p_O, R_O);
    }

    void apply_control_forces(vector<Eigen::Vector3d> f_s_plus, vector<Eigen::Vector3d> c_s) {

        for (int s = 0; s < c_s.size(); s++) {
            Eigen::Vector3d virt_hook = compose_p(p_O) + R_O * c_s[s];
            api.applyExternalForce(actor, -1, btVector3(btScalar(f_s_plus[s](0)), btScalar(f_s_plus[s](1)), btScalar(f_s_plus[s](2))), btVector3(btScalar(virt_hook(0)), btScalar(virt_hook(1)), btScalar(virt_hook(2))), 0);
        }

        // Linear Damping force to help stabilise
        api.applyExternalForce(actor, -1, -b * pdot_O, p_O, 0);

        // Rotational Damping torque to stabilise the object
        api.applyExternalTorque(actor, -1, -b_r * omega_O, 0);
    }

    myBULLET(float delta_t_in, float b_in, float b_r_in) {
        delta_t = delta_t_in;
        b = b_in;
        b_r = b_r_in;
        init();
    }
};