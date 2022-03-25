#include <functional>
#include <vector>
#include "Trajectory.h"
#include <eigen3/Eigen/Core>

namespace PandaController {
    Trajectory::Trajectory(TrajectoryType t, std::function<Eigen::VectorXd ()> trajectory_generator) {
        this->type = t;
        this->trajectory_generator = trajectory_generator;
    }
    Eigen::VectorXd Trajectory::operator()() {
        return this->trajectory_generator();
    }
}