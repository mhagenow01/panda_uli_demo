#include "PandaController.h"
#include "Kinematics.h"
#include "Trajectory.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <array>
#include <deque>
#include "DHA.h"

using namespace std;

namespace PandaController {
    Trajectory motionlessTrajectory();
    namespace {
        boost::mutex mutex;
        boost::mutex trajectory_mutex;
        Trajectory trajectory = motionlessTrajectory();
        franka::RobotState current_state;

        array<double, 7> pose_goal{}; //<x, y, z, x, y, z, w>

        array<double, 42> jacobian{};
        
	    double maxForce = 15;

        double hybridGain = 0.002;

        vector<DHA> PandaFlangeDHA{
            DHA(       0,   0.333,       0,  0),
            DHA(       0,       0, -M_PI/2,  1),
            DHA(       0, 0.31599,  M_PI/2,  2),
            DHA( 0.08249,       0,  M_PI/2,  3),
            DHA(-0.08249,   0.384, -M_PI/2,  4),
            DHA(       0,       0,  M_PI/2,  5),
            DHA(  0.0879,       0,  M_PI/2,  6),
            DHA(       0,  0.1069,       0, -1),
        };
        
        vector<DHA> PandaCameraDHA{
            DHA(       0,   0.333,       0,  0),
            DHA(       0,       0, -M_PI/2,  1),
            DHA(       0, 0.31599,  M_PI/2,  2),
            DHA( 0.08249,       0,  M_PI/2,  3),
            DHA(-0.08249,   0.384, -M_PI/2,  4),
            DHA(       0,       0,  M_PI/2,  5),
            DHA(  0.0879,       0,       0, -1),
        };

        Eigen::Matrix4d pandaGripperEELink = (
            Eigen::Matrix4d() << 
                 0.7071, -0.7071,  0,       0, 
                -0.7071, -0.7071,  0,       0, 
                      0,       0, -1, 0.16874, 
                      0,       0,  0,       1
        ).finished();

        // Mocap calibration
        // Just force torque now!!
        // use same rotation as gripper (doesn't matter)
        Eigen::Matrix4d pandaMocapEELink = (
            Eigen::Matrix4d() << 
                 0.7071, -0.7071,  0,       0, 
                -0.7071, -0.7071,  0,       0, 
                      0,       0, -1, 0.0634, 
                      0,       0,  0,       1
        ).finished();


        // 1 in roller installed + 0.1 mm for tape
        // 3.5 mm for rubber casing
        // center hole to mating surface is 0.08938m
        // force torque sensor is 0.06534 m
        Eigen::Matrix4d pandaRollerEELink = (
            Eigen::Matrix4d() << 
                 0.0,  -1.0,     0,       0, 
                 -1.0,  0.0,     0,       0, 
                    0,     0,  -1.0, 0.17102, 
                    0,     0,     0,       1
        ).finished();

        // 1 in roller installed + 0.1 mm for tape
        // 3.5 mm for rubber casing
        // center hole to mating surface is 0.08938m
        // force torque sensor is 0.06534 m
        Eigen::Matrix4d pandaUltrasoundEELink = (
            Eigen::Matrix4d() << 
                 0.0,  -1.0,     0,       0, 
                 -1.0,  0.0,     0,       0, 
                    0,     0,  -1.0, 0.2178, 
                    0,     0,     0,       1
        ).finished();

        // Actuated sanding tool
        // force torque sensor is 0.06534m
        // ISO interface to sander - 0.05595m
        // Thickness of scotchbrite ~0.003m
        // Total = 0.12429m
        Eigen::Matrix4d pandaSanderEELink = (
            Eigen::Matrix4d() << 
                 0.0,  -1.0,     0,        0, 
                 -1.0,  0.0,     0,   -0.0812, 
                    0,     0,  -1.0, 0.12429, 
                    0,     0,     0,        1
        ).finished();

        // Caulking Gun
        // force torque sensor is 0.06534m
        // PLA thinkness is 0.032m
        // PLA to end of caulking tube 0.112m
        // Total = 0.20934m
        // Horizontal offset: 100mm (at 45 degree angle in +X,-Y of prev transform)
        Eigen::Matrix4d pandaCaulkingEELink = (
            Eigen::Matrix4d() << 
                 0.7071,  -0.7071,     0,        0.07071, 
                 -0.7071,  -0.7071,     0,       -0.07071, 
                    0,     0,  -1.0, 0.20934, 
                    0,     0,     0,        1
        ).finished();

        // center hole to mating surface is 0.090988m
        // force torque sensor is 0.06534 m
        // offset in non-vertical is 63mm polisher + 15 mm interface = 78mm (-x of previous transform)
        Eigen::Matrix4d pandaPolisherEELink = (
            Eigen::Matrix4d() << 
                  0.0,   0.0,   1.0,    -0.078, 
                 -1.0,   0.0,   0.0,       0.0, 
                  0.0,  -1.0,   0.0,    0.1563, 
                    0,     0,     0,       1
        ).finished();

        // force torque sensor
        // sensor is aligned with gripper transform
        // really only the orientation is important!!
        // TODO: remove 30 degrees and put it here!!
        Eigen::Matrix4d pandaFTLink = (
            Eigen::Matrix4d() << 
                  0.7071, -0.7071,  0,       0.0 
                 -0.7071, -0.7071,  0,       0.0, 
                  0,       0,      -1,   0.06534, 
                  0,     0,     0,           1
        ).finished();

        vector<DHA> ee_chain = PandaFlangeDHA;
        Eigen::Matrix4d ee_link = pandaGripperEELink;
        
        Eigen::Matrix4d cameraLink = (
            Eigen::Matrix4d() << 
                  1.0,    .0,    0,      .065, 
                    0,    .0,    1,     -.038, 
                    0,    -1,   .0,      .037, 
                    0,     0,    0,         1
        ).finished();

    }

    std::vector<DHA> getChain(PandaController::KinematicChain chain){
        switch (chain){
            case KinematicChain::PandaFlange:
                return PandaFlangeDHA;
            case KinematicChain::PandaCamera:
                return PandaCameraDHA;
            default:
                return PandaFlangeDHA;
        }
    }

    Eigen::Matrix4d getEELink(PandaController::EELink link){
        switch (link) {
            case EELink::PandaGripper:
                return pandaGripperEELink;
            case EELink::PandaRoller:
                return pandaRollerEELink;
            case EELink::PandaPolisher:
                return pandaPolisherEELink;
            case EELink::PandaMocap:
                return pandaMocapEELink;
            case EELink::PandaSander:
                return pandaSanderEELink;
            case EELink::PandaCaulking:
                return pandaCaulkingEELink;
            case EELink::PandaUltrasound:
                return pandaUltrasoundEELink;
            case EELink::CameraLink:
                return cameraLink;
            default:
                return pandaGripperEELink;
        }
    }

    void setKinematicChain(KinematicChain chain, EELink link) {
        ee_chain = getChain(chain);
        ee_link = getEELink(link);
    }

    // Set different hybrid gains for different tool stiffness
    void setHybridGain(double gain){
        boost::lock_guard<boost::mutex> guard(mutex);
        hybridGain=gain;
    }

    double getHybridGain(){
        return hybridGain;
    }

    void setTrajectory(Trajectory t) {
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        trajectory = t;
    }

    Eigen::VectorXd getNextCommand(TrajectoryType & t) {
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        t = trajectory.type;
        auto v = trajectory();
        return v;
    }

    Trajectory motionlessTrajectory() {
        return Trajectory(
            TrajectoryType::Joint,
            []() {
                auto q = readRobotState().q;
                return Eigen::Map<const Eigen::VectorXd>(q.data(), 7).eval();
            }
        );
    }

    void dontMove() {
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        auto t = motionlessTrajectory();
        trajectory = t;
    }

    bool isMotionDone(array<double, 7> command) {
        long timePoint = (long)command[0];
        long deltaT = chrono::duration_cast<chrono::milliseconds>(
                chrono::time_point<chrono::system_clock>(chrono::milliseconds(timePoint)) - chrono::system_clock::now()
        ).count();
        // If the waypoint is in the future. 
        // Then we aren't done yet
        if (deltaT > 0) {
            return false;
        }
        // If it is sufficiently far in the past
        // Then lets forget about it.
        return true;
    }

    void updateInterpolationCoefficients() {
        // double now_ms = chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1);
        // // Fit to poly of the form w0 + w1 t + w2 t^2 + ...
        // // This has 3 parameters from the position, velocity and acceleration at the beginning.
        // // + 1 parameter for each positional waypoint in the middle.
        // // + 3 more parameters for the end goal position, end velocity (0) and and acceleration (0);
        // // Only interpolate between the next few points.
        // int m = 6;
        // Eigen::MatrixXd A(m,m); A.fill(0);
        // Eigen::MatrixXd B(m, 6); B.fill(0);
        // // t goes from 0 (now) to 1 (last point)
        // array<double, 7> command = commanded_position.front();
        // double timeWidth = command[0] - now_ms;
        // // First 3 rows look like:
        // // 1 0 0 0 ...
        // // 0 1 0 0 ...
        // // 0 0 2 0 ...
        // A(0,0) = 1;
        // A(1,1) = 1;
        // A(2,2) = 2;

        // Eigen::Affine3d transformMatrix(Eigen::Matrix4d::Map(current_state.O_T_EE.data()));
        // Eigen::Vector3d positionVector(transformMatrix.translation());
        // Eigen::Quaterniond orientationVector(transformMatrix.linear());
        // EulerAngles euler = quaternionToEuler(orientationVector);
        // for (size_t i = 0; i < 3; i++) {
        //     B(0,i) = positionVector[i];
        // }
        // B(0,3) = euler.roll - M_PI;
        // if (B(0,3) < -M_PI) B(0,3) += 2* M_PI;
        // else if (B(0,3) > M_PI) B(0,3) -= 2 * M_PI;
        // B(0,4) = euler.pitch;
        // B(0,5) = euler.yaw;
        // //B.row(1) = Eigen::Map<const Eigen::VectorXd>(currentVelocity.data(), 6);
        // //B.row(2) = Eigen::Map<const Eigen::VectorXd>(currentAcceleration.data(), 6); // currentAcceleration is currently not set.

        // A.row(3) << 1, 1, 1, 1, 1, 1;
        // B.row(3) << command[1], command[2], command[3], command[4], command[5], command[6];
        // //Huh?
        // A.row(4) << 0, 1, 2, 3, 4, 5;
        // A.row(4) << 0, 0, 6, 12, 20, 30;
        // Eigen::MatrixXd W = A.completeOrthogonalDecomposition().solve(B);
        // for (size_t i = 0; i < (m * 6); i++) {
        //     interpolationCoefficients[i] = W.data()[i];
        // }
        // pathStartTime_ms = (long)now_ms;
        // pathEndTime_ms = (long)command[0];
    }

    void writeCommandedPosition(Eigen::VectorXd data){
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        trajectory = Trajectory(
            TrajectoryType::Cartesian,
            [data]() {
                return data;
            }
        );
    }

    void writeHybridCommand(Eigen::VectorXd data){
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        trajectory = Trajectory(
            TrajectoryType::Hybrid,
            [data]() {
                return data;
            }
        );
    }
    void writeJointAngles(Eigen::VectorXd data){
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        trajectory = Trajectory(
            TrajectoryType::Joint,
            [data]() {
                return data;
            }
        );
    }
    
    Eigen::Matrix4d getEETransform() {
        boost::lock_guard<boost::mutex> guard(mutex);
        return EEFromDHA(current_state.q, ee_chain, ee_link);
    }

    Eigen::VectorXd getEEPos() {
        Eigen::Affine3d transform(getEETransform());
        return Eigen::Vector3d(transform.translation());
    }

    Eigen::VectorXd getEEPos(PandaController::KinematicChain chain, PandaController::EELink link) {
        std::vector<DHA> temp_ee_chain = getChain(chain);
        Eigen::Matrix4d temp_ee_link = getEELink(link);

        boost::lock_guard<boost::mutex> guard(mutex);
        Eigen::Affine3d transform(EEFromDHA(current_state.q, temp_ee_chain, temp_ee_link));
        return Eigen::Vector3d(transform.translation());
    }

    Eigen::Quaterniond getEEOrientation() {
        Eigen::Affine3d transform(getEETransform());
        return Eigen::Quaterniond(transform.linear()).normalized();
    }

    Eigen::Quaterniond getFTOrientation() {
        // This computes the orientation for the given configuration of the force torque sensor
        // using a static link and chain choice
        boost::lock_guard<boost::mutex> guard(mutex);
        Eigen::Affine3d transform(EEFromDHA(current_state.q, PandaFlangeDHA,pandaFTLink));
        return Eigen::Quaterniond(transform.linear()).normalized();
    }
    
    Eigen::Quaterniond getEEOrientation(PandaController::KinematicChain chain, PandaController::EELink link) {
        std::vector<DHA> temp_ee_chain = getChain(chain);
        Eigen::Matrix4d temp_ee_link = getEELink(link);

        boost::lock_guard<boost::mutex> guard(mutex);
        Eigen::Affine3d transform(EEFromDHA(current_state.q, temp_ee_chain, temp_ee_link));
        return Eigen::Quaterniond(transform.linear()).normalized();
    }    


    franka::RobotState readRobotState(){
        boost::lock_guard<boost::mutex> guard(mutex);
        return current_state;
    }
    void writeRobotState(franka::RobotState data){
        boost::lock_guard<boost::mutex> guard(mutex);
        jacobian = jacobianFromDHA(data.q, ee_chain, ee_link);//pandaJacobian(data.q);
        current_state = data;
    }
    array<double, 42> readJacobian() {
        boost::lock_guard<boost::mutex> guard(mutex);
        return jacobian;
    }
    double readMaxForce() {
        boost::lock_guard<boost::mutex> guard(mutex);
        return maxForce;
    }
    void writeMaxForce(double val) {
        boost::lock_guard<boost::mutex> guard(mutex);
        maxForce = val;
    }
}