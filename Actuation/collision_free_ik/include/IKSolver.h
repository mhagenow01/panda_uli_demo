#include "IKSolverInterface.h"
#include <string>

class IKSolver {
private:
    void* solver_ptr;
    
public:
    IKSolver(const std::string urdf_ptr, const std::string ee_frame_ptr, const std::string arm_colliders_ptr, const std::string environment_ptr) {
        solver_ptr = IKSolverInterface::new_solver(urdf_ptr.c_str(), ee_frame_ptr.c_str(), arm_colliders_ptr.c_str(), environment_ptr.c_str());
    }

    int dof() {
        return IKSolverInterface::dof(solver_ptr);
    }

    bool solve(double* current_q, const std::array<double, 7>& trans, double* q) {
        return IKSolverInterface::solve(solver_ptr, current_q, &trans, q);
    }

    bool set_ee(const char* ee_frame_ptr) {
        return IKSolverInterface::set_ee(solver_ptr, ee_frame_ptr);
    }
    ~IKSolver() {
        IKSolverInterface::deallocate(solver_ptr);
    }
};