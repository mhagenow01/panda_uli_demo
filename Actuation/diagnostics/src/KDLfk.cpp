//////////////////////////////////////////////////////////////////////////
// KDLfk.cpp
// Builds a KDL tree from a URDF and performs forward kinematics           
//////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "corrective_shared_autonomy/IK.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64MultiArray.h"

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <urdf_parser_plugin/parser.h>

using namespace std;

KDL::Tree my_tree;
urdf::Model my_model;
KDL::Chain chain;
KDL::ChainFkSolverPos_recursive fk_solver;
ros::Publisher pose_pub;

bool solversetup;

void setupFKsolver(string base, string ee){
  if (!my_model.initFile(req.urdf.data.c_str())){
    ROS_ERROR("Failed to parse urdf robot model");
    return 0;
    }
  if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
    ROS_ERROR("Failed to construct kdl tree");
    return 0;
  }

  my_tree.getChain(base,ee,chain);
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  solversetup = true;
}

void calcfk(){
  if solversetup{
  // TODO: urdf file and only do this one time?
  
  // get joint angles from jointstate message
    KDL::JntArray q(chain.getNrOfJoints());

    // Solve FK
    
    Frame F_result;
    fk_solver.JntToCart(q,F_result);

    // Publish achieved pose
    geometry_msgs::Pose pose_out;
    pose_out.position.x = F.p.;
    pose_out.position.y = 0.0;
    pose_out.position.z = 0.0;
    pose_out.orientation.x = 0.0;
    pose_out.orientation.y = 0.0;
    pose_out.orientation.z = 0.0;
    pose_out.orientation.w = 1.0;
    pose_pub.publish(pose_out);
  }
}

bool solveIK(corrective_shared_autonomy::IK::Request &req,
             corrective_shared_autonomy::IK::Response &res       ){
  if (!my_model.initFile(req.urdf.data.c_str())){
    ROS_ERROR("Failed to parse urdf robot model");
    return 0;
  }
  if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
    ROS_ERROR("Failed to construct kdl tree");
    return 0;
  }

  // Set up solver
  my_tree.getChain(req.base.data.c_str(),req.ee.data.c_str(),chain);
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
  KDL::ChainIkSolverPos_NR_JL ik_solver(chain, fk_solver,
            ik_solver_vel, 1000, 1e-6); //max 1000 iterations and stop by an accuracy of 1e-6
  
  // Set up the joint limits
  int num_joints = chain.getNrOfJoints();
  if(num_joints==0){
    cout << "Found no joints" << endl;
    return 0;
  }

  KDL::JntArray q_init(chain.getNrOfJoints());
  KDL::JntArray q_out(chain.getNrOfJoints());

  KDL::JntArray q_min(chain.getNrOfJoints());
  KDL::JntArray q_max(chain.getNrOfJoints());

  for(int i=0; i<chain.getNrOfJoints();i++){
    q_min(i) = my_model.joints_[req.joint_names[i].data.c_str()]->limits->lower;
    q_max(i) = my_model.joints_[req.joint_names[i].data.c_str()]->limits->upper;
    q_init(i) = (q_min(i)+q_max(i))/2.0;
  }

  ik_solver.setJointLimits(q_min,q_max);

  // Run the Inverse Kinematics
  geometry_msgs::Pose pose_in = req.desired_pose;
  KDL::Frame F_dest = KDL::Frame(KDL::Rotation::Quaternion(pose_in.orientation.x, pose_in.orientation.y,
   pose_in.orientation.z, pose_in.orientation.w), KDL::Vector(pose_in.position.x, pose_in.position.y, pose_in.position.z));
  
  int ikresult = ik_solver.CartToJnt(q_init, F_dest, q_out);
  if (ikresult==0 || ikresult==-5) {
    std_msgs::Float64MultiArray array_msg;
    array_msg.data.resize(q_out.rows());
    // parse output of ik_solver to the robot
    for (unsigned int j = 0; j < q_out.rows(); j++) {
        array_msg.data[j] = q_out(j);
    }
    
    // also send back the reached EE pose
    KDL::Frame F_found;
    fk_solver.JntToCart(q_out,F_found);
    double qx; double qy; double qz; double qw;
    F_found.M.GetQuaternion(qx,qy,qz,qw);
    double x; double y; double z;
    x = F_found.p.data[0];
    y = F_found.p.data[1];
    z = F_found.p.data[2];

    geometry_msgs::Pose pose_out;
    pose_out.orientation.x = qx;
    pose_out.orientation.y = qy;
    pose_out.orientation.z = qz;
    pose_out.orientation.w = qw;
    pose_out.position.x = x;
    pose_out.position.y = y;
    pose_out.position.z = z;
    res.soln_pose = pose_out;
    res.soln_joints = array_msg;
    return 1;
  } 
  else {
    return 0;
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kdl_fk");
  ros::NodeHandle nh;

  solversetup = false;

  // TODO: set up solver once based on params on the server
  // TODO: subscribe to ee like the ik

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber subq = nh.subscribe ("/franka_ros_interface/motion_controller/arm/joint_commands", 1, calcfk);
  pose_pub = nh.advertise<geometry_msgs::Pose>("/ik/pose_out", 1);
  ros::ServiceServer service = nh.advertiseService("/corrective_shared_autonomy/solve_ik", solveIK);
  
  ros::spin();
}
