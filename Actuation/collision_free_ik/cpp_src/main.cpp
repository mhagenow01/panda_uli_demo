#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "IKSolver.h"
#include <string>
#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include "franka_core_msgs/JointCommand.h"

using namespace std;


string readFile(string fp) {
    
    string line;
    string allText = "";
    ifstream myfile (fp);
    
    if (myfile.is_open()) {
        while (getline(myfile, line))
            allText += line;
        myfile.close();
    }

    return allText;
}


void poseToArray(const geometry_msgs::Pose::ConstPtr& msg, array<double, 7>& trans) {
    trans[0] = msg->position.x;
    trans[1] = msg->position.y;
    trans[2] = msg->position.z;

    trans[3] = msg->orientation.w;
    trans[4] = msg->orientation.x;
    trans[5] = msg->orientation.y;
    trans[6] = msg->orientation.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_free_ik");
    ros::NodeHandle n;

    string urdf, ee_frame, arm_colliders, environment;
    string urdf_fp, arm_collider_fp, environment_fp;
    vector<double> current_q;
    vector<string> joint_names;
    n.getParam("/ik/urdf_fp", urdf_fp);
    n.getParam("/ik/ee_frame", ee_frame);
    n.getParam("/ik/arm_collider_fp", arm_collider_fp);
    n.getParam("/ik/environment_fp", environment_fp);
    n.getParam("/ik/initial_q", current_q);
    n.getParam("/ik/joint_names", joint_names);

    urdf = readFile(urdf_fp);
    arm_colliders = readFile(arm_collider_fp);
    environment = readFile(environment_fp);
    
    IKSolver iksolver = IKSolver(urdf, ee_frame, arm_colliders, environment);
    if (current_q.size() != iksolver.dof()) {
        cout << iksolver.dof() << endl;
        return 1;
    }
    
    ros::Publisher pub = n.advertise<franka_core_msgs::JointCommand>("out", 1);
    array<double, 7> trans = array<double, 7>();
    ros::Subscriber sub = n.subscribe<geometry_msgs::Pose>("in", 1, [&](const geometry_msgs::Pose::ConstPtr& msg) {

        poseToArray(msg, trans);

        vector<double> q = vector<double>();
        for (size_t i = 0; i < iksolver.dof(); i++) {
            q.push_back(0);
        }

        if (iksolver.solve(current_q.data(), trans, q.data())) {
            for (size_t i = 0; i < current_q.size(); i++) {
                current_q[i] = q[i];
            }

            auto out_msg = franka_core_msgs::JointCommand();
            out_msg.names = joint_names;
            out_msg.position = q;
            out_msg.mode = franka_core_msgs::JointCommand::POSITION_MODE;

            pub.publish(out_msg);
            
        } else {
            ROS_WARN("Couldn't do some IK");
        }
        
    });

    ros::spin();
    return 0;
}