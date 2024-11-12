#include <myVIRTUOSE_LOGGING.h>
#include <Eigen/Dense>
#include "myGEOMETRY/myROT.h"
#include "myGEOMETRY/mySE3.h"
#include <vector> 
#include <myVirtuose_CMD_v3.h>

using namespace std;

class MyVIRTUOSE {                          // The class
private:

public:                                 // Access specifier
    VirtContext VC;
    float X[7];
    float quat[4];
    Eigen::Vector3d p_H, f_cmd, tau_cmd;
    Eigen::Matrix3d R_H;
    Eigen::Matrix4d H;

    float W_cmd[6] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
    const char* PORT;                                       // myAttributes
    float forcefactor, speedfactor, dt;                     // force factor, speed factor, and sampling rate - VERY IMPORTANT
    float identity[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f };
    float worldisbelow[7] = { 0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f };

    float k;

    CMD cmd;

    // active
        CMD set_CMD(float k_in) { // needed to initialise empty class cmd in the public context of MyVIRTUOSE
            std::cout << "I came into the CMD constructor" << std::endl;
            CMD my_cmd(k_in);
            return my_cmd;
        }

        void compute_f_cmd(Eigen::Matrix4d O) {
          f_cmd << 0, 0, 0;
          tau_cmd << 0, 0, 0;
          //std::cout << cmd.k << std::endl;
            for (int i = 0; i < cmd.e_i.size(); i++) {
                cmd.f_i_plus[i] = cmd.k * cmd.Pi * (H * cmd.e_i_tilde[i] - O * cmd.e_i_tilde[i]);

                f_cmd = f_cmd - cmd.f_i_plus[i];

                tau_cmd = tau_cmd + my_cross(R_H * cmd.e_i[i], -cmd.f_i_plus[i]);
            }

            W_cmd[0] = f_cmd(0);
            W_cmd[1] = f_cmd(1);
            W_cmd[2] = f_cmd(2);

            W_cmd[3] = tau_cmd(0);
            W_cmd[4] = tau_cmd(1);
            W_cmd[5] = tau_cmd(2);
        }

        void getquat() {
        quat[0] = X[3];
        quat[1] = X[4];
        quat[2] = X[5];
        quat[3] = X[6];
        }
    
        void set_dt(float dt_input) {
            dt = dt_input;
        }

        void set_forcefactor(float forcefactor_input) {
            forcefactor = forcefactor_input;
        }

        void set_speedfactor(float speedfactor_input) {
            speedfactor = speedfactor_input;
        }

        void update_H() {
            for (int i = 0; i < 3; i++) {
                p_H(i) = X[i];
            }
            getquat(); // keep this though
            R_H = R_frm_quat(quat);
            H = compose(p_H, R_H);
            //std::cout << "Handle frame is: \n" << H << std::endl;
        }

        void queryPOS() {
            virtGetPosition(VC, X);
            update_H();
            //std::cout << "Haption position is: x = " << X[0] << ", y = " << X[1] << ", z = " << X[2] << std::endl;
        }

        float* getPOS() {
            queryPOS();
            return X;
        }

        void set_f(float f_input[6]) {
            for (int i = 0; i < 6; i++) {
                W_cmd[i] = f_input[i];
            }
        }

        void sendCMD_f(float f_input[6]) {
            set_f(f_input);
            virtSetForce(VC, W_cmd);
            //std::cout << f[0] << f[1] << f[2] << f[3] << f[4] << f[5] << std::endl;
            // reset force to 0 - after every issued command - VERY VERY IMPORTANT if UDP drops out... - discuss with dc.
            for (int i = 0; i < 6; i++) {
                W_cmd[i] = 0;
            }
        }

        void render_W_cmd() {
            
            cmd.check_safety(W_cmd); // Check force and torque limits before rendering

            virtSetForce(VC, cmd.W_safe); // Uncomment to render
            // 
            //std::cout << f[0] << f[1] << f[2] << f[3] << f[4] << f[5] << std::endl;
            // reset force to 0 - after every issued command - VERY VERY IMPORTANT if UDP drops out... - discuss with dc.
            for (int i = 0; i < 6; i++) {
                W_cmd[i] = 0;
            }
        }

        void debug_getPOS() {
            printf("Virtuose API debugging initiated\n");
            for (int i = 0; i < 10; i++) {
                queryPOS();
                std::cout << "X1 = " << X[0] << "|X2 = " << X[1] << "|X3 = " << X[2] << "|X4 = " << X[3] << "|X5 = " << X[4] << "|X6 = " << X[5] << "|X7 = " << X[6] << std::endl;
            }
            printf("Debugging done, hopefully you're seeing valid data - don't proceed if you see 0's or 10e8 values.\n");
        }


        void setDEFAULTPARAMS() {
            virtSetIndexingMode(VC, INDEXING_ALL);
        	    virtSetForceFactor(VC, forcefactor);
        	    virtSetSpeedFactor(VC, speedfactor);
        	    virtSetTimeStep(VC, dt);
        	    virtSetBaseFrame(VC, worldisbelow);
        	    virtSetObservationFrame(VC, identity);
        	    virtSetCommandType(VC, COMMAND_TYPE_IMPEDANCE);
        }

        void engage() {
            virtSetPowerOn(VC, 1);
        }

        void disengage() {
            virtSetPowerOn(VC, 0);
        }

        void connect() {  // Method/function defined inside the class
            std::cout << "Hello World!";
            VC = virtOpen(PORT);
        	    if (VC == NULL)
        	    {
        		    fprintf(stderr, "Erreur dans virtOpen: %s\n", virtGetErrorMessage(virtGetErrorCode(NULL)));
        	    }
            printf("virtOpen worked - nothing else on Virtuose setup has been done\n");
        }

        void disconnect(){
            virtClose(VC);
        }

        void testrun() {
            printf("Doing a test run automatically\n");
            connect();  // Connect to Virtuose
            setDEFAULTPARAMS(); // Impedance mode parameters - as prescribed by the Documentation.
            engage(); // Now the Virtuose will stream data if you hold the handle.
            disengage(); // Now the Virtuose will stop streaming data even if you hold the handle.
            disconnect();
        }

        void quick_start() {
            printf("quick start entails - (i) connect, (ii) setDEFAULTPARAMS and (iii) engage.\n");
            connect();  // Connect to Virtuose
            setDEFAULTPARAMS(); // Impedance mode parameters - as prescribed by the Documentation.
            engage(); // Now the Virtuose will stream data if you hold the handle.
        }
        void quick_stop() {
            printf("quick stop entails - (i) disengage and (ii) disconnect.\n");
            disengage(); // Now the Virtuose will stop streaming data even if you hold the handle.
            disconnect();
        }

        
        // Constructors
        MyVIRTUOSE(const char* port_input, float forcefactor_input, float speedfactor_input, float dt_input) 
        {   // Constructor 1 declaration
            PORT = port_input;

            dt = dt_input;

            forcefactor = forcefactor_input;
            speedfactor = speedfactor_input;

            k = 0;
            cmd = set_CMD(k);
        }

        MyVIRTUOSE(const char* port_input)
        {   // Constructor 2  declaration
            PORT = port_input;

            k = 0;
            cmd = set_CMD(k);
        }

        MyVIRTUOSE(const char* port_input, float forcefactor_input, float speedfactor_input, float dt_input, float k_input)
        {   // Constructor 3 declaration
            PORT = port_input;

            dt = dt_input;

            forcefactor = forcefactor_input;
            speedfactor = speedfactor_input;

            k = k_input;
            cmd = set_CMD(k);
        }

};

class ARM : public MyVIRTUOSE // single colon for inheritance
{
    using MyVIRTUOSE::MyVIRTUOSE; // "using" for inheriting the constructors written for MyVIRTUOSE
public:
    std::string name;

    // can define new methods that allow to perform debugging operations on a single arm at a higher level - while having access to the lower level methods
};
