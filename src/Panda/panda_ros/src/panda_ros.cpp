#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <signal.h>
#include "PandaController.h"
#include "Trajectory.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Wrench.h"
#include <relaxed_ik/JointAngles.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <deque>
#include <boost/algorithm/string.hpp>
#include "panda_ros_msgs/VelocityBoundPath.h"
#include "panda_ros_msgs/HybridPose.h"
#include "panda_ros_msgs/JointPose.h"

using namespace std;

namespace {
    PandaController::KinematicChain kinematicChain;
    PandaController::EELink eeLink;
}

ros::Publisher g_eventPub {};
ros::Publisher g_wrenchPub {};
ros::Publisher g_controlWrenchPub {};
ros::Publisher g_jointPub {};

void setKinematicChain(const std_msgs::String& msg) {
    if (msg.data == "pandaFlange") {
        kinematicChain = PandaController::KinematicChain::PandaFlange;
    }
    if (msg.data == "pandaCamera") {
        kinematicChain = PandaController::KinematicChain::PandaCamera;
    }
}

void setEELink(const std_msgs::String& msg) {
    if (msg.data == "pandaGripper") {
        eeLink = PandaController::EELink::PandaGripper;
    }
    if (msg.data == "cameraLink") {
        eeLink = PandaController::EELink::CameraLink;
    }
    if (msg.data == "pandaRoller") {
        eeLink = PandaController::EELink::PandaRoller;
    }
    if (msg.data == "pandaPolisher") {
        eeLink = PandaController::EELink::PandaPolisher;
    }
    if (msg.data == "pandaMocap") {
        eeLink = PandaController::EELink::PandaMocap;
    }
    if (msg.data == "pandaUltrasound") {
        eeLink = PandaController::EELink::PandaUltrasound;
    }
    if (msg.data == "pandaSander"){
        eeLink = PandaController::EELink::PandaSander;
    }
    if (msg.data == "pandaCaulking"){
        eeLink = PandaController::EELink::PandaCaulking;
    }

}

void setCartPos(const geometry_msgs::Pose::ConstPtr& msg){
    if (PandaController::isRunning()){
        PandaController::setKinematicChain(kinematicChain, eeLink);
        PandaController::EulerAngles angles = 
            PandaController::quaternionToEuler(Eigen::Quaternion<double>(
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z
            ));
        Eigen::VectorXd position(6);
        position << 
            msg->position.x,
            msg->position.y,
            msg->position.z,
            angles.roll,
            angles.pitch,
            angles.yaw;
        PandaController::writeCommandedPosition(position);
    }
}

void setStampedTrajectory(vector<Eigen::VectorXd> path, vector<double> timestamps) {
    PandaController::setKinematicChain(kinematicChain, eeLink);
    PandaController::setTrajectory(PandaController::Trajectory(
        PandaController::TrajectoryType::Cartesian, 
        [path, timestamps]() {
            double now_ms = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
            int goal_index = path.size();
            for(int i = 0; i < path.size(); i++) {
                if (timestamps[i] > now_ms) {
                    goal_index = i;
                    break;
                }
            }

            Eigen::VectorXd goal(6);
            Eigen::Quaterniond goal_q;

            if (goal_index == 0) {
                goal.topLeftCorner(3, 1) = path[0].topLeftCorner(3,1);
                goal_q = Eigen::Quaterniond(path[0].bottomRightCorner(4,1).data());
            } else if (goal_index == path.size()) {
                goal.topLeftCorner(3, 1) = path[goal_index-1].topLeftCorner(3,1);
                goal_q = Eigen::Quaterniond(path[goal_index-1].bottomRightCorner(4,1).data());
            } else {
                double t = (now_ms - timestamps[goal_index-1]) / (timestamps[goal_index] - timestamps[goal_index-1]);
                goal.topLeftCorner(3, 1) = path[goal_index-1].topLeftCorner(3,1) + t * (path[goal_index].topLeftCorner(3,1) - path[goal_index-1].topLeftCorner(3,1));
                goal_q = Eigen::Quaterniond(path[goal_index-1].bottomRightCorner(4,1).data()).slerp(t, Eigen::Quaterniond(path[goal_index].bottomRightCorner(4,1).data()));
            }
            auto goal_angles = PandaController::quaternionToEuler(goal_q.normalized());
            goal[3] = goal_angles.roll;
            goal[4] = goal_angles.pitch;
            goal[5] = goal_angles.yaw;
            return goal;
        }
    ));
}

void setStampedPath(const nav_msgs::Path::ConstPtr& msg) {
    if (PandaController::isRunning()){
        Eigen::VectorXd current_position(7);
        current_position.topLeftCorner(3, 1) = PandaController::getEEPos();
        current_position.bottomRightCorner(4, 1) = PandaController::getEEOrientation().coeffs();
        vector<Eigen::VectorXd> path{current_position};//{PandaController::getEEVector()};
        double now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        vector<double> timestamps{now};
        for (size_t i = 0; i < msg->poses.size(); i++) {
            auto poseStamped = msg->poses[i];
            long secs = poseStamped.header.stamp.sec;
            long nsecs = poseStamped.header.stamp.nsec;
            timestamps.push_back(secs * 1000 + nsecs / 1000000);
            path.push_back(
                (Eigen::VectorXd(7) << 
                    poseStamped.pose.position.x,
                    poseStamped.pose.position.y,
                    poseStamped.pose.position.z,
                    poseStamped.pose.orientation.x,
                    poseStamped.pose.orientation.y,
                    poseStamped.pose.orientation.z,
                    poseStamped.pose.orientation.w).finished()
            );
        }
        setStampedTrajectory(path, timestamps);
    }
}

void setVelocityBoundPath(const panda_ros_msgs::VelocityBoundPath::ConstPtr& msg) {
    if (PandaController::isRunning()) {
        Eigen::VectorXd current_position(7);
        current_position.topLeftCorner(3, 1) = PandaController::getEEPos();
        current_position.bottomRightCorner(4, 1) = PandaController::getEEOrientation().coeffs();
        double maxV = msg->maxV;

        double now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        vector<Eigen::VectorXd> path{current_position};
        vector<double> timestamps{now};
        for (size_t i = 0; i < msg->poses.size(); i++) {
            auto pose = msg->poses[i];
            Eigen::VectorXd command(7);
            command << 
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w;

            double distance = sqrt(
                (command[0] - current_position[0]) * (command[0] - current_position[0]) + 
                (command[1] - current_position[1]) * (command[1] - current_position[1]) + 
                (command[2] - current_position[2]) * (command[2] - current_position[2])
            );
            double timestamp = now + distance / (maxV / 1000);
            timestamps.push_back(timestamp);
            path.push_back(command);
            current_position = command;
            now = timestamp;
        }

        setStampedTrajectory(path, timestamps);
    }
}

void setVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (PandaController::isRunning()) {
        PandaController::setKinematicChain(kinematicChain, eeLink);
        auto twist = msg->twist;
        Eigen::VectorXd velocity(6);
        velocity << 
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z;
        double end_time = msg->header.stamp.toSec();

        PandaController::setTrajectory(PandaController::Trajectory(
            PandaController::TrajectoryType::Velocity, 
            [velocity, end_time]() {
                if (ros::Time::now().toSec() > end_time){
                    Eigen::VectorXd velocity(6);
                    velocity << 
                    ((double) rand() / (RAND_MAX))/100000.,
                    ((double) rand() / (RAND_MAX))/100000.,
                    ((double) rand() / (RAND_MAX))/100000.,
                    ((double) rand() / (RAND_MAX))/100000.,
                    ((double) rand() / (RAND_MAX))/100000.,
                    ((double) rand() / (RAND_MAX))/100000.;
                    return velocity;
                }
                else
                    return velocity;
            }
        ));
    }
}

void setJointPose(const panda_ros_msgs::JointPose::ConstPtr& msg) {
    if (PandaController::isRunning()) {
        PandaController::setKinematicChain(kinematicChain, eeLink);
        double start_time = ros::Time::now().toSec();
        double end_time = msg->header.stamp.toSec();
        Eigen::VectorXd goal(7);
        goal <<
            msg->joint_pose[0],
            msg->joint_pose[1],
            msg->joint_pose[2],
            msg->joint_pose[3],
            msg->joint_pose[4],
            msg->joint_pose[5],
            msg->joint_pose[6];

        PandaController::setTrajectory(PandaController::Trajectory(
            PandaController::TrajectoryType::Joint, 
            [goal, start_time, end_time]() {
                double progress = (ros::Time::now().toSec() - start_time) / (end_time - start_time);
                franka::RobotState robot_state = PandaController::readRobotState();
                Eigen::VectorXd q_v = Eigen::Map<Eigen::VectorXd>(robot_state.q.data(), 7);
                if (progress > 1){
                    return goal;
                }
                else
                    return (q_v + (goal - q_v) * progress).eval();
            }
        ));
    }
}

void setHybrid(const panda_ros_msgs::HybridPose::ConstPtr& msg){
    if (PandaController::isRunning()){    
        PandaController::setKinematicChain(kinematicChain, eeLink);
        Eigen::VectorXd command(23);
        command << 
            //Position
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            //Selection vector
            double(msg->sel_vector[0]),
            double(msg->sel_vector[1]),
            double(msg->sel_vector[2]),
            double(msg->sel_vector[3]),
            double(msg->sel_vector[4]),
            double(msg->sel_vector[5]),
            //Wrench
            msg->wrench.force.x,
            msg->wrench.force.y,
            msg->wrench.force.z,
            msg->wrench.torque.x,
            msg->wrench.torque.y,
            msg->wrench.torque.z,
            //Constraint frame
            msg->constraint_frame.w,
            msg->constraint_frame.x,
            msg->constraint_frame.y,
            msg->constraint_frame.z;
        PandaController::writeHybridCommand(command);
    }
}

void setHybridGain(const std_msgs::Float64& msg){
    if (PandaController::isRunning()){    
        PandaController::setHybridGain(msg.data);
    }
}

void setWrenchBias(const std_msgs::Float64& msg){
    if (PandaController::isRunning()){    
        PandaController::bias_ft();
    }
}

void callbackCommands(const std_msgs::String& msg){
    std::vector<std::string> command;
    boost::split(command, msg.data, [](char c){return c == ';';});
    std_msgs::String result;
    if(msg.data == "grasp"){
        cout<<"Grasping"<<endl;
        result.data = "grasp_finished";
        PandaController::graspObject([result](){g_eventPub.publish(result);});
    }
    if(msg.data == "release"){
        result.data = "release_finished";
        PandaController::releaseObject([result](){g_eventPub.publish(result);});
    }
    if(msg.data == "toggleGrip") {
        PandaController::toggleGrip();
    }
    if(command[0] == "setMaxForce") {
        cout<<"Setting max force to "<<command[1]<<endl;
        PandaController::writeMaxForce(stod(command[1]));
        //PandaController::writeCommandedFT({0,0,-stod(command[1]),0,0,0});
    }   
}

void publishJointState(franka::RobotState robot_state){
    const vector<string> joint_names{"panda_joint1", "panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7","panda_finger_joint1","panda_finger_joint2"};
    franka::GripperState gripperState = PandaController::readGripperState();

    sensor_msgs::JointState states;
    states.effort.resize(joint_names.size());
    states.name.resize(joint_names.size());
    states.position.resize(joint_names.size());
    states.velocity.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); i++) {
        states.name[i] = joint_names[i];
    }
    states.header.stamp = ros::Time::now();
    for (size_t i = 0; i < joint_names.size()-2; i++) {
        states.position[i] = robot_state.q[i];
        states.velocity[i] = robot_state.dq[i];
        states.effort[i] = robot_state.tau_J[i];
    }
    states.position[joint_names.size()-2] = gripperState.width/2.;
    states.position[joint_names.size()-1] = gripperState.width/2.;
    
    g_jointPub.publish(states);
}

void publishFrame(string name, PandaController::KinematicChain chain, PandaController::EELink link){
    static tf2_ros::TransformBroadcaster br;
    Eigen::Vector3d position = PandaController::getEEPos(chain, link);
    Eigen::Quaterniond orientation = PandaController::getEEOrientation(chain, link);
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "panda_link0";
    transformStamped.child_frame_id = name;
    transformStamped.transform.translation.x = position[0];
    transformStamped.transform.translation.y = position[1];
    transformStamped.transform.translation.z = position[2];
    
    transformStamped.transform.rotation.x = orientation.coeffs()[0];
    transformStamped.transform.rotation.y = orientation.coeffs()[1];
    transformStamped.transform.rotation.z = orientation.coeffs()[2];
    transformStamped.transform.rotation.w = orientation.coeffs()[3];

    br.sendTransform(transformStamped);
}
void publishTf(franka::RobotState robot_state){
    publishFrame("panda_ee", kinematicChain, eeLink);
    publishFrame("panda_camera", PandaController::KinematicChain::PandaCamera, PandaController::EELink::CameraLink);
    publishFrame("panda_gripper", PandaController::KinematicChain::PandaFlange, PandaController::EELink::PandaGripper);
}

void publishWrench(franka::RobotState robot_state){
    std::array<double, 6> forces;
    //forces = robot_state.O_F_ext_hat_K;
    Eigen::Quaterniond orientation = PandaController::getEEOrientation();
    
    forces = PandaController::readFTForces();
    
    geometry_msgs::Wrench wrench;
    wrench.force.x = forces[0];
    wrench.force.y = forces[1];
    wrench.force.z = forces[2];
    wrench.torque.x = forces[3];
    wrench.torque.y = forces[4];
    wrench.torque.z = forces[5];

    g_wrenchPub.publish(wrench);
}

void publishWrenchLocal(franka::RobotState robot_state){
    std::array<double, 6> forces;
    //forces = robot_state.O_F_ext_hat_K;
    Eigen::Quaterniond orientation = PandaController::getEEOrientation();
    forces = PandaController::readFTForces();

     // convert back into the global frame
    Eigen::Vector3d forces_global;
    forces_global << forces[0], forces[1], forces[2];
    Eigen::Vector3d torques_global;
    torques_global << forces[3], forces[4], forces[5];

    Eigen::Vector3d forces_local = orientation.inverse() * forces_global;
    Eigen::Vector3d torques_local = orientation.inverse() * torques_global;

    geometry_msgs::Wrench wrench;
    wrench.force.x = forces_local[0];
    wrench.force.y = forces_local[1];
    wrench.force.z = forces_local[2];
    wrench.torque.x = torques_local[0];
    wrench.torque.y = torques_local[1];
    wrench.torque.z = torques_local[2];

    wrench.force.x = -forces_local[0];
    wrench.force.y = -forces_local[1];
    wrench.force.z = -forces_local[2];
    wrench.torque.x = -torques_local[0];
    wrench.torque.y = -torques_local[1];
    wrench.torque.z = -torques_local[2];

    g_controlWrenchPub.publish(wrench);
}

void publishState(){
    franka::RobotState robot_state = PandaController::readRobotState();
    publishJointState(robot_state);
    publishTf(robot_state);
    publishWrench(robot_state);
    publishWrenchLocal(robot_state);
}

void signalHandler(int sig)
{
    PandaController::stopControl();
    ros::shutdown();
    exit(sig);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PandaListener");
    ros::NodeHandle n("~");
    
    //Setup the signal handler for exiting, must be called after ros is intialized
    signal(SIGINT, signalHandler); 

    PandaController::initPandaController();
    
    ros::Subscriber sub_commands = n.subscribe("/panda/commands", 10, callbackCommands);
    ros::Subscriber sub_position = n.subscribe("/panda/cart_pose", 10, setCartPos);
    ros::Subscriber sub_trajectory = n.subscribe("/panda/path", 10, setStampedPath);
    ros::Subscriber sub_vel_trajectory = n.subscribe("/panda/velocity_bound_path", 10, setVelocityBoundPath);
    ros::Subscriber sub_kinematicChain = n.subscribe("/panda/set_kinematic_chain", 10, setKinematicChain);
    ros::Subscriber sub_eeLink = n.subscribe("/panda/set_ee_link",10,setEELink);
    ros::Subscriber sub_velocity = n.subscribe("/panda/cart_velocity", 10, setVelocity);
    ros::Subscriber sub_joint_pose = n.subscribe("/panda/joint_pose", 10, setJointPose);
    ros::Subscriber sub_hybrid = n.subscribe("/panda/hybrid_pose", 10, setHybrid);
    ros::Subscriber sub_hybridgain = n.subscribe("/panda/hybrid_gain", 1, setHybridGain);
    ros::Subscriber sub_biasft = n.subscribe("panda/wrench_bias",1,setWrenchBias);

    g_wrenchPub = n.advertise<geometry_msgs::Wrench>("/panda/wrench", 10);
    g_controlWrenchPub = n.advertise<geometry_msgs::Wrench>("/panda/control_wrench", 10);
    g_jointPub = n.advertise<sensor_msgs::JointState>("/panda/joint_states", 1);
    g_eventPub = n.advertise<std_msgs::String>("/panda/events", 1);
    ros::Rate loopRate(1000);
    while (ros::ok() && PandaController::isRunning()) {
        publishState();
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
