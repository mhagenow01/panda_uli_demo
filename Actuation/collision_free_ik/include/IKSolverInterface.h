#include <array>

namespace IKSolverInterface {
    extern "C" {
        void* new_solver(const char* urdf_ptr, const char* ee_frame_ptr, const char* arm_colliders_ptr, const char* environment_ptr);
        int dof(void* iksolver);
        bool solve(void* iksolver, double* current_q_ptr, const std::array<double, 7>* trans_ptr, double* q_ptr);
        void deallocate(void* iksolver);
    }
}