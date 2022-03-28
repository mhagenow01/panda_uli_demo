#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/log.h>
#include "PandaController.h"
#include "Kinematics.h"
#include "Trajectory.h"
#include "Common.h"
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <thread>
#include <cmath>
#include <deque>
#include <functional>
#include <array>

using namespace std;

namespace PandaController {
    namespace {
        thread ft_listener;
        thread controller;
        thread gripper;
        bool running = false;
        
    } 

    //Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    Eigen::Quaterniond eulerToQuaternion(EulerAngles angle) // roll (X), pitch (Y), yaw (Z)
    {
        // Abbreviations for the various angular functions
        double cy = cos(angle.yaw * 0.5);
        double sy = sin(angle.yaw * 0.5);
        double cp = cos(angle.pitch * 0.5);
        double sp = sin(angle.pitch * 0.5);
        double cr = cos(angle.roll * 0.5);
        double sr = sin(angle.roll * 0.5);

        Eigen::Quaterniond q(cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr, sy * cp * sr + cy * sp * cr, sy * cp * cr - cy * sp * sr);

        return q;
    }
    
    //Adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    EulerAngles quaternionToEuler(Eigen::Quaterniond q){
        EulerAngles angle;
        double q0=q.coeffs()[3];
        double q1=q.coeffs()[0];
        double q2=q.coeffs()[1];
        double q3=q.coeffs()[2];

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
        double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
        angle.roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q0 * q2 - q3 * q1);
        
        if (abs(sinp) >= 1)
            angle.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angle.pitch = asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q0 * q3 + q1 * q2);
        double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        angle.yaw = atan2(siny_cosp, cosy_cosp);
        
        return angle;
    }

    // See: https://github.com/frankaemika/franka_ros/issues/35
    // This adds a tiny bit of random noise to each component to ensure they aren't 0.
    // Only works with column vectors, which seems to be the normal kind.
    void addNoise(Eigen::VectorXd & v) {
        double epsilon = 0.00001;
        for(int i = 0; i < v.rows(); i++) {
            v[i] += rand() % 2 == 0 ? epsilon : - epsilon;
        }
    }

    void constrainForces(Eigen::VectorXd & velocity, const franka::RobotState & robot_state) {
        double maxForce = readMaxForce();
        for (int i = 0; i < 3; i++){
            // If the force is too high, and the velocity would increase the force,
            // Then remove that velocity.
            // Doesn't account for RPY forces - 
            // I.E. velocity in x direction could increase twist on the EE.
            if (abs(robot_state.O_F_ext_hat_K[i]) > maxForce
                && velocity[i] * robot_state.O_F_ext_hat_K[i] >= 0){
                if (abs(velocity[i]) >= 0.0001){
                    velocity[i] = 0;//- 0.05 * velocity[i] / abs(velocity[i]);
                }
            } 
        }
    }

    void commandPositionFromState(const franka::RobotState & state) {
        Eigen::VectorXd positionArray(6);
        Eigen::Affine3d transformMatrix(Eigen::Matrix4d::Map(state.O_T_EE.data()));
        Eigen::Vector3d positionVector(transformMatrix.translation());
        for (size_t i = 0; i < 3; i++) {
            positionArray[i] = positionVector[i];
        }
        for (size_t i = 0; i < 3; i++) {
            positionArray[3+i] = 0;
        }
        writeCommandedPosition(positionArray);
    }

    franka::JointVelocities positionControlLoop(const franka::RobotState& robot_state, Eigen::VectorXd commandedPosition) {

        Eigen::Affine3d transform(getEETransform());
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());
        orientation.normalize();
        
        EulerAngles desired_a;
        desired_a.roll = commandedPosition[3];
        desired_a.pitch = commandedPosition[4];
        desired_a.yaw = commandedPosition[5];
        auto current_a = quaternionToEuler(orientation);
        Eigen::Quaterniond desired_q=eulerToQuaternion(desired_a);
        
        Eigen::Quaterniond difference(desired_q*orientation.inverse());
        
        EulerAngles difference_a = quaternionToEuler(difference.normalized());

        double scaling_factor = 5;
        double v_x = (commandedPosition[0] - position[0]) * scaling_factor;
        double v_y = (commandedPosition[1] - position[1]) * scaling_factor;
        double v_z = (commandedPosition[2] - position[2]) * scaling_factor;
        double v_roll = difference_a.roll * scaling_factor;
        double v_pitch = difference_a.pitch * scaling_factor;
        double v_yaw = difference_a.yaw * scaling_factor;
        Eigen::VectorXd v(6);
        v << v_x, v_y, v_z, v_roll, v_pitch, v_yaw;

        constrainForces(v, robot_state);
        Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(readJacobian().data(), 6, 7).completeOrthogonalDecomposition().solve(v);
        franka::JointVelocities output = {{
            jointVelocities[0], 
            jointVelocities[1], 
            jointVelocities[2], 
            jointVelocities[3], 
            jointVelocities[4], 
            jointVelocities[5], 
            jointVelocities[6]
        }};
        return output;
    }

    franka::JointVelocities velocityControlLoop(const franka::RobotState& robot_state, Eigen::VectorXd commandedVelocity) {
        double v_x = commandedVelocity[0];
        double v_y = commandedVelocity[1];
        double v_z = commandedVelocity[2];
        double v_roll = commandedVelocity[3];
        double v_pitch = commandedVelocity[4];
        double v_yaw = commandedVelocity[5];
        Eigen::VectorXd v(6);
        v << v_x, v_y, v_z, v_roll, v_pitch, v_yaw;

        constrainForces(v, robot_state);
        Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(readJacobian().data(), 6, 7).completeOrthogonalDecomposition().solve(v);
        franka::JointVelocities output = {{
            jointVelocities[0], 
            jointVelocities[1], 
            jointVelocities[2], 
            jointVelocities[3], 
            jointVelocities[4], 
            jointVelocities[5], 
            jointVelocities[6]
        }};
        return output;
    }

    franka::JointVelocities hybridControlLoop(const franka::RobotState& robot_state, Eigen::VectorXd command) {
        // Panda hybrid controller using an admittance type architecture -> ultimately commanded as a P-law on
        // joint velocities

        // User command
        Eigen::Vector3d commandedPosition;
        commandedPosition << command[0], command[1], command[2];
        Eigen::Vector3d commandedForce;
        commandedForce << command[13], command[14], command[15];
        Eigen::Vector3d commandedTorque;
        commandedTorque << command[16], command[17], command[18];

        Eigen::Quaterniond desired_qCF = Eigen::Quaterniond(command[3], command[4], command[5], command[6]);
        Eigen::Quaterniond constraint_frame = Eigen::Quaterniond(command[19], command[20], command[21], command[22]);
        Eigen::VectorXd selection_vector(3);
        selection_vector << command[7], command[8], command[9]; // Currently just forces are used in hybrid
        
        // Current robot state
        auto position = getEEPos();
        auto orientation = getEEOrientation();
        array<double, 6> currentWrench = readFTForces();

        // Rotate current command (positions and forces [todo: generalwrenches]) into the constraint frame to compute control law
        Eigen::Vector3d currentPositionCF = constraint_frame.inverse() * position;
        Eigen::Vector3d currentForce;
        currentForce << currentWrench[0], currentWrench[1], currentWrench[2];
        // current force is in the global frame
        Eigen::Vector3d currentForceCF = constraint_frame.inverse() *  currentForce;

        // Compute Position/Force Hybrid Control Law
        double scaling_factor = 5;
        double Kfp = PandaController::getHybridGain();
        double v_x_CF = selection_vector[0]*(commandedPosition[0] - currentPositionCF[0]) * scaling_factor + (1-selection_vector[0])*Kfp*(commandedForce[0]+currentForceCF[0]);
        double v_y_CF = selection_vector[1]*(commandedPosition[1] - currentPositionCF[1]) * scaling_factor + (1-selection_vector[1])*Kfp*(commandedForce[1]+currentForceCF[1]);
        double v_z_CF = selection_vector[2]*(commandedPosition[2] - currentPositionCF[2]) * scaling_factor + (1-selection_vector[2])*Kfp*(commandedForce[2]+currentForceCF[2]);
        
        // Rotate the desired orientation and cartesian velocity (difference) out of the constraint frame
        Eigen::Quaterniond desired_q = constraint_frame * desired_qCF;     
        Eigen::Vector3d v_CF;
        v_CF << v_x_CF, v_y_CF, v_z_CF;
        Eigen::Vector3d v_global = constraint_frame * v_CF;
        
        // Orientation control law
        Eigen::Quaterniond difference((desired_q*orientation.inverse()).normalized());
        EulerAngles difference_a = quaternionToEuler(difference);
        
        double v_roll = difference_a.roll * scaling_factor;
        double v_pitch = difference_a.pitch * scaling_factor;
        double v_yaw = difference_a.yaw * scaling_factor;


        Eigen::VectorXd v_hybrid_expanded(6);
        v_hybrid_expanded << v_global[0], v_global[1], v_global[2], v_roll, v_pitch, v_yaw;


        //constrainForces(v_hybrid_expanded, robot_state);
        Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(readJacobian().data(), 6, 7).completeOrthogonalDecomposition().solve(v_hybrid_expanded);
        
        franka::JointVelocities output = {
            jointVelocities[0], 
            jointVelocities[1], 
            jointVelocities[2], 
            jointVelocities[3], 
            jointVelocities[4], 
            jointVelocities[5], 
            jointVelocities[6]
        };
        return output;
    }

    franka::JointVelocities jointPositionControlLoop(const franka::RobotState& robot_state, Eigen::VectorXd joint_angles) {
        double scale = 5;
        Eigen::VectorXd joint_velocity(7);
        joint_velocity << 
            (joint_angles[0] - robot_state.q[0]) * scale,
            (joint_angles[1] - robot_state.q[1]) * scale,
            (joint_angles[2] - robot_state.q[2]) * scale,
            (joint_angles[3] - robot_state.q[3]) * scale,
            (joint_angles[4] - robot_state.q[4]) * scale,
            (joint_angles[5] - robot_state.q[5]) * scale,
            (joint_angles[6] - robot_state.q[6]) * scale;
        addNoise(joint_velocity);

        // Take the desired joint velocities. Convert to cartesian space.
        // Attempt to shift the target cartesian velocity to avoid a collision.
        // Go back to joint space using a simplified IK via the Jacobian. <-- This part might be a little bit sketchy.
        Eigen::VectorXd v = Eigen::Map<Eigen::MatrixXd>(readJacobian().data(), 6, 7) * joint_velocity;
        constrainForces(v, robot_state);
        joint_velocity = Eigen::Map<Eigen::MatrixXd>(readJacobian().data(), 6, 7).completeOrthogonalDecomposition().solve(v);
        franka::JointVelocities velocities = {
            joint_velocity[0],
            joint_velocity[1],
            joint_velocity[2],
            joint_velocity[3],
            joint_velocity[4],
            joint_velocity[5],
            joint_velocity[6]
        };
        return velocities;
    }

    franka::JointVelocities controlLoop(int & iteration,
                                        const franka::RobotState& robot_state, franka::Duration duration) {
        PandaController::writeRobotState(robot_state);
        TrajectoryType t;
        Eigen::VectorXd command = getNextCommand(t);
        franka::JointVelocities velocities{0,0,0,0,0,0,0};
        switch (t) {
            case TrajectoryType::Cartesian:
                velocities = positionControlLoop(robot_state, command);
                break;
            case TrajectoryType::Velocity:
                velocities = velocityControlLoop(robot_state, command);
                break;
            case TrajectoryType::Joint:
                velocities = jointPositionControlLoop(robot_state, command);
                break;
            case TrajectoryType::Hybrid:
                velocities = hybridControlLoop(robot_state, command);
                break;
        }
        iteration++;
        if (!isRunning()) {
            return franka::MotionFinished(velocities);
        }
        if (iteration < 5) {
            return {0,0,0,0,0,0,0};
        }
        return velocities;
    }

    void simulateControl(franka::Robot & robot) {
        auto state = robot.readOnce();
        franka::JointVelocities velocities{0,0,0,0,0,0,0};
        int iteration = 0;
        auto innerLoop = bind(controlLoop, iteration, std::placeholders::_1, std::placeholders::_2);
        do {
            velocities = innerLoop(state, franka::Duration(1));
            state.dq = velocities.dq;
            Eigen::Map<Eigen::VectorXd>(state.q.data(), 7) = Eigen::Map<Eigen::VectorXd>(state.q.data(), 7) + 0.001*Eigen::Map<Eigen::VectorXd>(state.dq.data(), 7);
            Eigen::Map<Eigen::Matrix4d>(state.O_T_EE.data()) = getEETransform();//calculatePandaEE(state.q);
        } while (!velocities.motion_finished);
    }

    void robotControl(franka::Robot & robot) {
        int iteration = 0;
        // MH Play with joint impedance and collision behavior for better contact control
        robot.setJointImpedance({{400, 400, 400, 400, 400, 400, 400}});
        // Set the collision behavior
        //array<double,7> lower_torque_thresholds = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}; previous values
        array<double,7> lower_torque_thresholds = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
        array<double,7> upper_torque_thresholds = {40.0, 40.0, 36.0, 36.0, 10.0, 10.0, 10.0};
        array<double,6> lower_force_thresholds = {20.0, 20.0, 35.0, 25.0, 25.0, 25.0};
        array<double,6> upper_force_thresholds = {50.0, 50.0, 60.0, 25.0, 25.0, 25.0};
        robot.setCollisionBehavior(lower_torque_thresholds,upper_torque_thresholds,lower_force_thresholds,upper_force_thresholds);
        robot.control(bind(controlLoop, iteration, std::placeholders::_1, std::placeholders::_2));
    }

    void resetRobot(franka::Robot & robot) {
        robot.automaticErrorRecovery();
        array<double,7> q_goal = {{0.0754606,-0.337453,0.150729,-2.46194,0.0587094,2.12597,0.972193}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);
        setDefaultBehavior(robot);
    }

    void runController(char * ip, bool simulate) {
        running = true;
        try {
            cout << "YO " << endl;
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            // franka::Robot robot(ip);
            cout << "YO2 " << endl;
            resetRobot(robot);
            writeRobotState(robot.readOnce());
            auto control = simulate ? simulateControl : robotControl;
            control(robot);
        } catch(const exception& e) {
            cout << e.what() << endl;
        }
        stopControl();
    }

    void stopControl() {
        cout << "Control stopping" << endl;
        running = false;
        // Hey. Make sure your listeners respect the running flag.
        // Also - don't try to join to yourself.
        if (controller.joinable()) controller.join();
        if (ft_listener.joinable()) ft_listener.join();
    }

    bool isRunning() {
        return running;
    }

    void initPandaController(bool simulate) {
        char * ip = getenv("PANDA_IP");
        cout << "Panda ip is " << ip << endl;
        // Gripper thread will hang forever on a blocking tcp call
        // if the gripper is not attached. 
        controller = thread(runController, ip, simulate);
        gripper = thread(initGripper, ip); 
        ft_listener = thread(forceTorqueListener);
    }
}
