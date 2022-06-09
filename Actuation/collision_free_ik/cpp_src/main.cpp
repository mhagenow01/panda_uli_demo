#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "IKSolver.h"
#include <string>
#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include "franka_core_msgs/JointCommand.h"
#include "collision_free_ik/CFIK.h"

using namespace std;

bool valid_ee;
IKSolver* iksolver;

string readFile(string fp) {
    
    string line;
    string allText = "";
    ifstream myfile (fp);
    
    if (myfile.is_open()) {
        while (getline(myfile, line))
            allText += line + '\n';
        myfile.close();
    }

    return allText;
}

void poseToArray(const geometry_msgs::Pose& msg, array<double, 7>& trans) {
    trans[0] = msg.position.x;
    trans[1] = msg.position.y;
    trans[2] = msg.position.z;

    trans[3] = msg.orientation.w;
    trans[4] = msg.orientation.x;
    trans[5] = msg.orientation.y;
    trans[6] = msg.orientation.z;
}

bool solveIKSrv(collision_free_ik::CFIK::Request &req,
             collision_free_ik::CFIK::Response &res){

        geometry_msgs::Pose desired_pose = req.desired_pose;
        std_msgs::Float64MultiArray joints_in; req.starting_joints;

        vector<double> current_q = vector<double>();
        for (unsigned int j = 0; j < joints_in.data.size(); j++) {
                current_q.push_back(joints_in.data[j]);
        }

        if (!valid_ee) {
            ROS_WARN("No EE specified for IK.");
            return false;
        }

        if (iksolver->dof() != 7) {
            ROS_WARN("This only works if you are commanding 7 dof.");
            return false;
        }

        array<double, 7> trans = array<double, 7>();
        poseToArray(desired_pose, trans);

        vector<double> q = vector<double>();
        for (size_t i = 0; i < iksolver->dof(); i++) {
            q.push_back(0);
        }

        if (iksolver->solve(current_q.data(), trans, q.data())) {
            geometry_msgs::Pose pose_out;
            
            // Store joint angles in array message
            std_msgs::Float64MultiArray joints_out;
            joints_out.data.resize(q.size());
            for (unsigned int j = 0; j < q.size(); j++) {
                joints_out.data[j] = q[j];
            }
            res.soln_joints = joints_out;

            // ES: TODO put in FK from IK
            pose_out.orientation.x = 0.0;
            pose_out.orientation.y = 0.0;
            pose_out.orientation.z = 0.0;
            pose_out.orientation.w = 1.0;
            pose_out.position.x = 0.0;
            pose_out.position.y = 0.0;
            pose_out.position.z = 0.0;
            res.soln_pose = pose_out;
            
            return true;
            
        } else {
            ROS_WARN("Couldn't do some IK");
            return false;
        }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_free_ik");
    ros::NodeHandle n;

    string urdf, ee_frame, arm_colliders, environment, solver_config;
    string urdf_fp, arm_collider_fp, environment_fp, solver_config_fp;
    valid_ee = true;
    vector<double> current_q;
    vector<string> joint_names;
    n.getParam("/default_end_effector_frame", ee_frame);
    n.getParam("/collision_free_ik/urdf_fp", urdf_fp);
    n.getParam("/collision_free_ik/arm_collider_fp", arm_collider_fp);
    n.getParam("/collision_free_ik/environment_fp", environment_fp);
    n.getParam("/collision_free_ik/solver_config_fp", solver_config_fp);
    n.getParam("/collision_free_ik/initial_q", current_q);
    n.getParam("/collision_free_ik/joint_names", joint_names);

    urdf = readFile(urdf_fp);
    arm_colliders = readFile(arm_collider_fp);
    environment = readFile(environment_fp);
    solver_config = readFile(solver_config_fp);
    
    iksolver = new IKSolver(urdf, ee_frame, arm_colliders, environment, solver_config);
    if (current_q.size() != iksolver->dof()) {
        cout << "IK SOLVER NUM DOF: " << iksolver->dof() << endl;
        cout << "for file: " << urdf_fp << endl;
        return 1;
    }

    ros::ServiceServer service = n.advertiseService("/collision_free_ik/solve_ik", solveIKSrv);
    
    ros::Publisher pub = n.advertise<franka_core_msgs::JointCommand>("out", 1);
    array<double, 7> trans = array<double, 7>();

    ros::Subscriber joint_states_sub = n.subscribe<sensor_msgs::JointState>(
        "/franka_ros_interface/custom_franka_state_controller/joint_states", 
        1, 
        [&](const sensor_msgs::JointState::ConstPtr& msg){
            if (msg->position.size() == current_q.size()) {
                current_q = msg->position;
            }
        }
    );    

    ros::Subscriber ee_frame_sub = n.subscribe<std_msgs::String>("/panda/set_ee", 1, [&](const std_msgs::String::ConstPtr& msg) {
        string frame = msg->data;

        if (frame.compare(ee_frame) != 0) {
            if (!iksolver->set_ee(frame.c_str())) {
                ROS_WARN("Failed to change end effector to frame.");
                // Don't do anything if we don't know the frame.
                valid_ee = false;
            } else {
                ee_frame = frame;
                valid_ee = true;
            }
        }
        
    });

    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("in", 1, [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        auto pose = msg->pose;
        string frame = msg->header.frame_id;

        if (!valid_ee) {
            return;
        }

        if (iksolver->dof() != 7) {
            ROS_WARN("This only works if you are commanding 7 dof.");
            return;
        }

        poseToArray(pose, trans);

        vector<double> q = vector<double>();
        for (size_t i = 0; i < iksolver->dof(); i++) {
            q.push_back(0);
        }

        if (iksolver->solve(current_q.data(), trans, q.data())) {
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