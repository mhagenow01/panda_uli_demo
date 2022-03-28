#ifndef PANDA_CONTROLLER_H
#define PANDA_CONTROLLER_H

#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <franka/robot_state.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <franka/gripper.h>
#include "Trajectory.h"


namespace PandaController {
    enum KinematicChain {PandaFlange,PandaCamera};
    enum EELink {PandaGripper,PandaRoller,PandaPolisher,PandaMocap,PandaSander,PandaUltrasound,CameraLink,PandaFT,PandaCaulking};
    struct EulerAngles {
        double roll, pitch, yaw;
    };

    void stopControl();
    void initPandaController(bool = false);

    Eigen::VectorXd getNextCommand(TrajectoryType & t);
    Eigen::Matrix4d getEETransform();
    Eigen::VectorXd getEEPos();
    Eigen::Quaterniond getEEOrientation();
    Eigen::VectorXd getEEPos(PandaController::KinematicChain chain, PandaController::EELink link);
    Eigen::Quaterniond getEEOrientation(PandaController::KinematicChain chain, PandaController::EELink link);
    Eigen::Quaterniond getFTOrientation();

    // Set different hybrid gains for different tool stiffness
    double getHybridGain();
    void setHybridGain(double gain);
    void bias_ft();
    

    void setTrajectory(Trajectory);
    void writeCommandedPosition(Eigen::VectorXd data);
    EulerAngles quaternionToEuler(Eigen::Quaterniond q);
    Eigen::Quaterniond eulerToQuaternion(EulerAngles angle);
    
    std::array<double, 6> readFTForces();
    void writeFTForces(std::array<double, 6> data);

    std::array<double, 7> readPoseGoal();
    void writePoseGoal(std::array<double, 7> data);

    std::array<double, 7> readJointAngles();
    void writeJointAngles(Eigen::VectorXd data);

    void writeHybridCommand(Eigen::VectorXd data);

    franka::RobotState readRobotState();
    void writeRobotState(franka::RobotState data);
    std::array<double, 42> readJacobian();
    void printRobotJointAngles(franka::RobotState data);

    void startLogging();
    bool isRunning();

    void writeMaxForce(double val);
    double readMaxForce();
    
    franka::GripperState readGripperState();
    void initGripper(const char * ip);
    void toggleGrip(std::function<void ()> onToggle = NULL);
    void graspObject(std::function<void ()> onGrasp = NULL);
    void releaseObject(std::function<void ()> onRelease = NULL);

    void forceTorqueListener();
    void setKinematicChain(KinematicChain chain = KinematicChain::PandaFlange, EELink link = EELink::PandaGripper);

} //end namespace PandaController

#endif

