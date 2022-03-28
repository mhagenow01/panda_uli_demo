#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <functional>
#include <vector>
#include <eigen3/Eigen/Core>

namespace PandaController {
    enum TrajectoryType {Cartesian, Joint, Hybrid, Velocity};
    class Trajectory {
    private:
        std::function<Eigen::VectorXd ()> trajectory_generator;
    public:
        TrajectoryType type;
        Trajectory(TrajectoryType t, std::function<Eigen::VectorXd ()> trajectory_generator);

        Eigen::VectorXd operator()();
    };
}
#endif