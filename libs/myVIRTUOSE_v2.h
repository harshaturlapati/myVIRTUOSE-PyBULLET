class MyVIRTUOSE {                          // The class
private:

public:                                 // Access specifier
    VirtContext VC;
    float X[7];
    float f[6] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
    const char* PORT;                                       // myAttributes
    float forcefactor, speedfactor, dt;                     // force factor, speed factor, and sampling rate - VERY IMPORTANT
    float identity[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f };
    float worldisbelow[7] = { 0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f };

    float ARM_R[3][3];

    void getR(float x_in, float y_in, float z_in, float w_in){
            float x, y, z, w, a11, a12, a13, a21, a22, a23, a31, a32, a33;
            //std::cout << "quaternion - x = " << quat_actor.getX() << ", y = " << quat_actor.getY() << ", z = " << quat_actor.getZ() << ", w = " << quat_actor.getW() << std::endl;
            x = x_in;
            y = y_in;
            z = z_in;
            w = w_in;


            float norm_quat = x * x + y * y + z * z + w * w;
            if (norm_quat == 0)
            {
                ARM_R[0][0] = 1;
                ARM_R[0][1] = 0;
                ARM_R[0][2] = 0;
                ARM_R[1][0] = 0;
                ARM_R[1][1] = 1;
                ARM_R[1][2] = 0;
                ARM_R[2][0] = 0;
                ARM_R[2][1] = 0;
                ARM_R[2][2] = 1;
            }
            else
            {
                a11 = (w * w + x * x + y * y - z * z) / norm_quat;
                a12 = 2 * (x * y - w * z) / norm_quat;
                a13 = 2 * (x * z + w * y) / norm_quat;
                a21 = 2 * (x * y + w * z) / norm_quat;
                a22 = (w * w - x * x + y * y - z * z) / norm_quat;
                a23 = 2 * (y * z - w * x) / norm_quat;
                a31 = 2 * (x * z - w * y) / norm_quat;
                a32 = 2 * (y * z + w * x) / norm_quat;
                a33 = (w * w - x * x - y * y + z * z) / norm_quat;
                ARM_R[0][0] = a11;
                ARM_R[0][1] = a12;
                ARM_R[0][2] = a13;
                ARM_R[1][0] = a21;
                ARM_R[1][1] = a22;
                ARM_R[1][2] = a23;
                ARM_R[2][0] = a31;
                ARM_R[2][1] = a32;
                ARM_R[2][2] = a33;
            }

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

        void queryPOS() {
            virtGetPosition(VC, X);
            //std::cout << "Haption position is: x = " << X[0] << ", y = " << X[1] << ", z = " << X[2] << std::endl;
        }

        float* getPOS() {
            queryPOS();
            return X;
        }

        void set_f(float f_input[6]) {
            for (int i = 0; i < 6; i++) {
                f[i] = f_input[i];
            }
        }

        void sendCMD_f(float f_input[6]) {
            set_f(f_input);
            virtSetForce(VC, f);
            //std::cout << f[0] << f[1] << f[2] << f[3] << f[4] << f[5] << std::endl;
            // reset force to 0 - after every issued command - VERY VERY IMPORTANT if UDP drops out... - discuss with dc.
            for (int i = 0; i < 6; i++) {
                f[i] = 0;
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

        //virtSetForce(VC, force);

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

        MyVIRTUOSE(const char* port_input, float forcefactor_input, float speedfactor_input, float dt_input) 
        {   // Constructor 1 declaration
            PORT = port_input;
            dt = dt_input;
            forcefactor = forcefactor_input;
            speedfactor = speedfactor_input;
        }

        MyVIRTUOSE(const char* port_input)
        {   // Constructor 2  declaration
            PORT = port_input;
        }
};

class ARM : public MyVIRTUOSE // single colon for inheritance
{
    using MyVIRTUOSE::MyVIRTUOSE; // "using" for inheriting the constructors written for MyVIRTUOSE
public:
    std::string name;

    // can define new methods that allow to perform debugging operations on a single arm at a higher level - while having access to the lower level methods
};
