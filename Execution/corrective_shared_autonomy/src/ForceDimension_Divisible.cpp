#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <panda_ros_msgs/HybridPose.h>

#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>

// Force Dimension
#include "dhdc.h"

using namespace std;
using namespace std::chrono;

std::array<double, 3> frozen_position = {0.0, 0.0, 0.0};
double applied_force = 5.0;
bool divisible_on = false;

std::ofstream outputfile;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in ForceDimension.cpp\n";
    //cout << "Closing Force Dimension" << endl;
    //dhdClose();
    dhdEnableForce (DHD_OFF);
    dhdClose();
    exit(sig);
}

bool init_input() {
    cout <<"Setting up Force Dimension\n";
    if (dhdOpen () < 0) {
        cout << "Unable to open device" << endl;
        dhdSleep (2.0);
        return false;
    }

    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // Second button press to start control & forces
    bool buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Press Button again to enable Forces and Panda Control" << endl;

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

    cout << "Force Dimension Setup Complete" << endl;
}

void publishPose(ros::Publisher hybrid_pub, std::array<double, 7> panda_pose, bool divisible_on) {
    panda_ros_msgs::HybridPose hybridPose;
    hybridPose.pose.position.x = panda_pose[0];
    hybridPose.pose.position.y = panda_pose[1];
    hybridPose.pose.position.z = panda_pose[2];
    hybridPose.pose.orientation.x = panda_pose[3];
    hybridPose.pose.orientation.y = panda_pose[4];
    hybridPose.pose.orientation.z = panda_pose[5];
    hybridPose.pose.orientation.w = panda_pose[6];
    hybridPose.wrench.force.x = 0.0; hybridPose.wrench.force.y = 0.0; hybridPose.wrench.force.z = -applied_force;
    hybridPose.constraint_frame.x=0.0;
    hybridPose.constraint_frame.y=0.0;
    hybridPose.constraint_frame.z=0.0;
    hybridPose.constraint_frame.w=1.0;
    hybridPose.sel_vector = {1, 1, 1, 1, 1, 1};

    if(divisible_on){
        hybridPose.sel_vector[2] = 0;
    }

    hybrid_pub.publish(hybridPose);
}

void pollInput(ros::Publisher hybrid_pub, double* scaling_factors, double* offsets, bool* clutch, bool* reset_center, bool* freeze, bool divisible_on, bool* divisible_freeze, bool* divisible_unfreeze) {
    static bool lastCenterButton = false;
    array<double, 3> fdPos = {0,0,0};
    dhdGetPosition(&fdPos[0],&fdPos[1],&fdPos[2]);

    std::array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    // The first time the clutch is initiated, freeze the position of the falcon
    if (*freeze)
    {
        frozen_position[0] = fdPos[0];
        frozen_position[1] = fdPos[1];
        frozen_position[2] = fdPos[2];
        *freeze = false;
    }

    // First divisible freeze, save off the z position and switch to force control
    if (*divisible_freeze)
    {
        frozen_position[2] = fdPos[2];
        *divisible_freeze = false;
    }

    // If still clutching, keep the position consistent based on the frozen position
    if(*clutch)
    {
        panda_pos[0] = scaling_factors[0] * frozen_position[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * frozen_position[1] + offsets[1];
        if(!divisible_on){
            panda_pos[2] = scaling_factors[2] * frozen_position[2] + offsets[2];
        }
        panda_pos[6] = 1;
    }

    else{ // Not clutching

        // If this is the first non-clutching sample, update the center of the workspace
        // based on the movement during the clutching action
        if(*reset_center)
        {
            offsets[0] = offsets[0]-scaling_factors[0]*(fdPos[0]-frozen_position[0]);
            offsets[1] = offsets[1]-scaling_factors[1]*(fdPos[1]-frozen_position[1]);
            if(!divisible_on && !divisible_unfreeze){
                offsets[2] = offsets[2]-scaling_factors[2]*(fdPos[2]-frozen_position[2]);
            }
            *reset_center = false;
        }

        if(*divisible_unfreeze){
            offsets[2] = offsets[2]-scaling_factors[2]*(fdPos[2]-frozen_position[2]);
            *divisible_unfreeze = false;
        }

        // When not clutching, command based on the actual falcon position
        panda_pos[0] = scaling_factors[0] * fdPos[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * fdPos[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * fdPos[2] + offsets[2];
        panda_pos[6] = 1;
    }
    publishPose(hybrid_pub,panda_pos,divisible_on);
}

void feedbackInput(geometry_msgs::Wrench wrench) {
    if(!divisible_on){
        double scale = 0.0; // force reflection
        double stiffness = 200; // for replay
        double viscous = 50; // friction

        array<double, 3> forceDimensionPos = {0,0,0};
        array<double, 3> forceDimensionVel = {0,0,0};
        dhdGetLinearVelocity(&forceDimensionVel[0],&forceDimensionVel[1],&forceDimensionVel[2]);
        // Send force (bilateral + friction) to the falcon
        dhdSetForceAndTorque(-wrench.force.x * scale-viscous*forceDimensionVel[0], 
                -wrench.force.y * scale-viscous*forceDimensionVel[1], 
                wrench.force.z * scale-viscous*forceDimensionVel[2],0.0,0.0,0.0);
    }
    else{
        dhdSetForceAndTorque(0.0,0.0,0.0,0.0,0.0,0.0);
    }
}

int main(int argc, char **argv) {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    std::vector<double> scaling_factors = {-4.0, -4.0, 4.0}; //-6,6,6
    std::vector<double> offsets = {0.4, 0.0, 0.25};

    // All of the required ros topics
    ros::init(argc, argv, "ForceDimensionDMP");
    ros::NodeHandle n("~");  
    ros::Publisher gripper_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);
    ros::Subscriber force_sub = n.subscribe("/panda/wrench", 10, feedbackInput);
    ros::Publisher hybrid_pub = 
        n.advertise<panda_ros_msgs::HybridPose>("/panda/hybrid_pose", 1); 
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    if (!init_input()) {
        cout << endl << "Failed to init force dimension" << endl;
        return -1;
    }

    bool buttonPressed = false;
    bool clutch = false;
    auto start = high_resolution_clock::now(); 
    
    bool quit = false;
    bool freeze = false;
    bool reset_center = true;

    bool divisible_freeze = false;
    bool divisible_unfreeze = false;

    // Prevent Initial Discontinuity
    ros::spinOnce();

    try{
         geometry_msgs::TransformStamped transformStamped;
         transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
         offsets[0] = transformStamped.transform.translation.x;
         offsets[1] = transformStamped.transform.translation.y;
         offsets[2] = transformStamped.transform.translation.z;
       }

    catch(tf2::TransformException &ex){
        cout << "COULDN'T GET TF FROM PANDA" << endl << ex.what() << endl;
        }

    while (ros::ok() && !quit) {
        pollInput(hybrid_pub,scaling_factors.data(),offsets.data(), &clutch, &reset_center, &freeze, divisible_on, &divisible_freeze, &divisible_unfreeze);
    
        // If button pressed and released in less than 0.3 seconds,
        // it is a gripping action

        // If held greater than 0.3 seconds, it is a velocity control action

        if(!buttonPressed && dhdGetButton(0)==1) // button initially pressed
        {
            buttonPressed = true;
            start = high_resolution_clock::now(); 
        }

        
        if(buttonPressed) // button held
        {
            auto end = high_resolution_clock::now(); 
            auto duration = duration_cast<microseconds>(end - start); 

            if(duration.count()>=300000)
            {
                if (clutch==false)
                {
                    freeze = true;
                    clutch = true;
                }
               
            }
            
        }

        if(buttonPressed && dhdGetButton(0)==0) // button released
        {
            auto end = high_resolution_clock::now(); 
            auto duration = duration_cast<microseconds>(end - start); 

            if(duration.count()<300000)
            {
                
                if(!divisible_on){
                    cout << "Divisible On!" << endl;
                    divisible_on = true;
                    divisible_freeze = true;
                }
                else{
                    cout << "Divisible Off!" << endl;
                    divisible_on = false;
                    divisible_unfreeze = true;
                }
            }

            buttonPressed=false;
        
            if (clutch==true){
                clutch=false;
                reset_center=true;
            }
        }

        if (dhdKbHit()) {
            char keypress = dhdKbGet();
            // cout << "Key Press: " << keypress << endl;
            if (keypress == '5'){
                applied_force = applied_force + 1.0;
                cout << "Applied Force: " << applied_force << endl;
            }
            if (keypress == '6'){
                applied_force = applied_force - 1.0;
                if(applied_force < 0.0){
                    applied_force = 0.0;
                }
                cout << "Applied Force: " << applied_force << endl;
            }
            
            if (keypress == 'q'){
                cout << "Quitting! " << endl;
                quit = true;
            } 
        }
    
    ros::spinOnce();
    usleep(1000);
    }

    dhdEnableForce (DHD_OFF);
    dhdClose();
    return 0;
}
