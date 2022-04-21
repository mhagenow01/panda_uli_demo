#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <ratio>
#include <chrono>
#include <math.h>
#include <array>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>

#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "dhdc.h"

using namespace std;

class ForceDimensionOneDZDController{
    public:
        int init_inputDevice();
        array<double, 3> getInputDeviceVelocity();
        void run_zero_displacement_controller(ros::NodeHandle n);
        ForceDimensionOneDZDController();
};

ForceDimensionOneDZDController::ForceDimensionOneDZDController(){
}

int ForceDimensionOneDZDController::init_inputDevice() {
    cout <<"Setting up Force Dimension\n";
    int deviceID = dhdOpen();
    if (deviceID < 0) {
        cout << "Unable to open device" << endl;
        dhdSleep (1.0);
        return -1;
    }
    cout << "Force Dimension Setup Complete" << endl;

    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // Second button press to start control & forces
    bool buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Press Button to start zero displacement controller" << endl << endl;

    bool buttonPressed=false;
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
    
    buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    return deviceID;
}

 array<double, 3> ForceDimensionOneDZDController::getInputDeviceVelocity() {
    double v_x,v_y,v_z;
    dhdGetLinearVelocity(&v_x,&v_y,&v_z);
    return {v_x, v_y, v_z};
}

void ForceDimensionOneDZDController::run_zero_displacement_controller(ros::NodeHandle n){

    ros::Publisher fd_publisher;
    ros::Publisher fd_button_publisher;

    fd_publisher = n.advertise<geometry_msgs::Vector3>("/zdinput/input", 5);
    fd_button_publisher = n.advertise<std_msgs::Float64>("/zdinput/button", 5);

    double viscous_replay = 60; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 225; // for replay - everything is 2x of the falcon
    array<double, 3> forceDimensionPos = {0,0,0};

    long count = 0;

    array<double, 3> inputDevice_velocity;
    inputDevice_velocity = getInputDeviceVelocity();

    double integrator_x = 0.0;
    double integrator_y = 0.0;
    double integrator_z = 0.0;

    double x_d_zero = 0.0;
    double y_d_zero = 0.0;
    double z_d_zero = 0.0;

    double dmp_fx, dmp_fy, dmp_fz;
    
    while(ros::ok()){
        dhdGetPosition (&forceDimensionPos[0],&forceDimensionPos[1],&forceDimensionPos[2]);

        // backwards if the button is pressed
        bool backwards = (dhdGetButton(0)==1);

        count++;

        // center the device around the center of motion (to scale to -1 to) 
        double x_d = 0.0125;
        double y_d = 0.0;
        double z_d = 0.025;

        // TODO: does this need to be here?
        if(count==1000){
            x_d_zero = -forceDimensionPos[0]-x_d;
            y_d_zero = -forceDimensionPos[1]-y_d;
            z_d_zero = -forceDimensionPos[2]-z_d;
        }

        // Store forcing from device for deformations
        // Note: these should be unit-normalized (i.e., span from -1 to 1)
        
        // the coordinate frame is to correspond with teleop of the panda from behind
         dmp_fx = -(forceDimensionPos[0]-x_d)/(0.0625);
         dmp_fy = -(forceDimensionPos[1]-y_d)/(0.12);
         dmp_fz = (forceDimensionPos[2]-z_d)/(0.105);

        // the coordinate frame is to correspond with teleop of the panda from in front
//        dmp_fx = (forceDimensionPos[0]-x_d)/(0.0625);
//        dmp_fy = (forceDimensionPos[1]-y_d)/(0.12);
//        dmp_fz = (forceDimensionPos[2]-z_d)/(0.105);


        // Add integrator to the haptic profile....?

        // Custom haptic profile (note the first two have sign changes corresponding to the sign differences in dmp_fx,dmp_fy
        double fx = -7*stiffness*(forceDimensionPos[0]-x_d)-7*viscous_replay*inputDevice_velocity[0];
        double fy = -0.7*stiffness*(forceDimensionPos[1]-y_d)-viscous_replay*inputDevice_velocity[1];
        double fz = -7*stiffness*(forceDimensionPos[2]-z_d)-7*viscous_replay*inputDevice_velocity[2];
        
        dhdSetForceAndTorque(fx,fy,fz,0.0,0.0,0.0);

        dmp_fx = dmp_fx/0.30; dmp_fy = dmp_fy/0.30; dmp_fz = dmp_fz/0.30;


        geometry_msgs::Vector3 fd_input;
        double val = -dmp_fy-0.28;
        if(val>1){
            val=1;
        }
        if(val<-1){
            val = -1;
        }

        fd_input.x = 0.02*(exp(4*abs(val))-1);
        if(val<0){
            fd_input.x = -fd_input.x;
        }
        fd_input.y = 0;
        fd_input.z = 0;
        fd_publisher.publish(fd_input);
        std_msgs::Float64 button_pressed;
        if(backwards){
            button_pressed.data=1.0;
        }
        else{
            button_pressed.data = 0.0;
        }
        fd_button_publisher.publish(button_pressed);

        usleep(1000);
    }
}


int main(int argc, char **argv) {
    ForceDimensionOneDZDController* controller = new ForceDimensionOneDZDController();
    int deviceID = controller -> init_inputDevice();
    if (deviceID==-1) {
        cout << endl << "Failed to initialize input device." << endl;
        return -1;
    }
    ros::init(argc, argv, "FDOneDZDController");
    ros::NodeHandle n("~");
    controller->run_zero_displacement_controller(n);
    return 0;
}
