// File name:  cppregistration.cpp
// Description: main registration routine called by python/cython to perform parallel ICP registration of models
// Authors: Mike Hagenow, Kevin Macauley, Nicole Gathman
// Date: 11/05/21

// File name:  cppregistration.cpp
// Description: main registration routine called by python/cython to perform parallel ICP registration of models
// Authors: Mike Hagenow, Kevin Macauley, Nicole Gathman
// Date: 11/05/21

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "Model.h"
//#include "urdf_helpers.cpp"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry> 
#include <vector>
#include <iterator>
#include <chrono>
#include <thread>
#include <pico_tree/eigen.hpp>
#include <pico_tree/kd_tree.hpp>
#include <omp.h>
#include <nlopt.hpp>

using namespace std;

// int counter {};

struct closest_alignment_model{
    Eigen::Matrix3Xd s_vals;
    Eigen::Matrix3Xd t_vals;
    Eigen::VectorXd dists;
    std::vector<int> close_enough_inds;

    closest_alignment_model(Eigen::Matrix3Xd s, Eigen::Matrix3Xd t, std::vector<double> d, std::vector<int> c){
        s_vals=s;
        t_vals=t;
        dists=Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(d.data(), d.size());
        close_enough_inds=c;
    }
};

struct opt_data_registration_residual{
    std::vector<double> x;
    Model* model;
    Eigen::Matrix3Xd t_vals;
    std::vector<int> close_enough_inds;
    Eigen::MatrixXd weights;
    Eigen::Matrix3Xd R_final;
    Eigen::Matrix3Xd t_final;
    opt_data_registration_residual(std::vector<double> x_temp,
                                    Model* model_temp,
                                    Eigen::Matrix3Xd t_vals_temp,
                                    std::vector<int> close_enough_inds_temp,
                                    Eigen::MatrixXd weights_temp,
                                    Eigen::Matrix3Xd R_final_temp,
                                    Eigen::Matrix3Xd t_final_temp){
        x=x_temp;
        model=model_temp;
        t_vals=t_vals_temp;
        close_enough_inds=close_enough_inds_temp;
        weights=weights_temp;
        R_final=R_final_temp;
        t_final=t_final_temp;
    }
};


struct icp_model{
    Eigen::Matrix3Xd R;
    Eigen::Matrix3Xd t;
    std::vector<double> angles;
    double scale;
    double error;

    icp_model(Eigen::Matrix3Xd R_final, Eigen::Matrix3Xd t_final, std::vector<double> angles_final, double scale_final, double error_final){
        R=R_final;
        t=t_final;
        angles=angles_final;
        scale=scale_final;
        error=error_final;        
    }
};

// Function for aligning closest points between source and target to be used for SVD
closest_alignment_model get_closest_alignment(Eigen::Matrix3Xd aligned_source, Eigen::Matrix3Xd target, pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>>* target_tree, double max_distance_threshold){
    pico_tree::Neighbor<int, Eigen::Matrix3Xd::Scalar> nn;
    std::vector<int> indices;
    std::vector<int> indices_orig;
    std::vector<double> dists; 
    for(int i=0; i< aligned_source.cols(); i++){
        target_tree->SearchNn(aligned_source.col(i), &nn);  
        if(nn.distance<max_distance_threshold){
            indices.push_back(nn.index);
            indices_orig.push_back(i);
            dists.push_back(nn.distance);
        }
    }
    return closest_alignment_model(aligned_source(Eigen::placeholders::all,indices_orig), target(Eigen::placeholders::all, indices), dists, indices_orig);
}

// returns the rotation matrix from the x vector
Eigen::Matrix3Xd get_R_from_X(const std::vector<double> &x){
    Eigen::Quaterniond q;
    q.x() = x[0];
    q.y() = x[1];
    q.z() = x[2];
    q.w() = x[3];
    return q.normalized().toRotationMatrix();
}

// returns the translation matrix from the x vector
Eigen::Matrix3Xd get_trans_from_X(const std::vector<double> &x){
    Eigen::Matrix3Xd t = Eigen::MatrixXd::Zero(3,1);

    for(int i = 4; i < 7;i++){
        t(i-4,0) = x[i];
    }
    return t;
}

// returns the angles vector from the x vector
std::vector<double> get_angles_from_x(const std::vector<double> &x){
    std::vector<double> angles_final;
    for(size_t i = 7; i < x.size();i++){ // TODO: missing the scale
        angles_final.push_back(x[i]);
    }
    return angles_final;
}

// Calculates just the residual for a set of inputs (used both for residual and gradient calculations)
double calc_residual(const std::vector<double> &x, void *data){
    // counter++;
    opt_data_registration_residual *d = (opt_data_registration_residual *) data;

    Eigen::Matrix3Xd R = get_R_from_X(x);

    if(isnan(R(0,0))){
        printf("X: %f %f %f %f %f\n",x[0],x[1],x[2],x[3],x[4]);
    }

    Eigen::Matrix3Xd t = get_trans_from_X(x);
    std::vector<double> angles_final = get_angles_from_x(x);

    // If scaling allowed, it is the last element of the optimization
    double scale;
    d->model->scaling_allowed ? scale = x[-1] : scale = 1.0;

    Eigen::Matrix3Xd R_temp = R*d->R_final;
    Eigen::Matrix3Xd trans_temp = R*d->t_final + t;

    Eigen::Matrix3Xd aligned_source = d->model->getPointsWithAngles(angles_final, R_temp, trans_temp, scale);

    Eigen::Matrix3Xd s_vals = aligned_source(Eigen::placeholders::all,d->close_enough_inds);
    Eigen::VectorXd non_weighted_residual = (s_vals-d->t_vals).colwise().norm();
    Eigen::VectorXd weights_vec = d->weights.col(0);
    double residual = (weights_vec.dot(non_weighted_residual))/scale;
    // std::cout << "Iteration: " << counter << " residual: " << residual << std::endl;
    return residual;
}

//  Calculates a residual and gradient for nonlinear fitting of R, t, angles, and scale if applicable 
double registration_residual(const std::vector<double> &x, std::vector<double> &grad, void *data){
    

    // // Numerically approximate the gradient
    // double dirac = 1e-4;
    // if (!grad.empty()) {
    //     for(int i=0;i<int(x.size());i++){
    //         std::vector<double> temp = x;
    //         std::vector<double> temp2 = x;
    //         temp[i] -= dirac;
    //         temp2[i] += dirac;
    //         double bwd = calc_residual(temp,data);
    //         double fwd = calc_residual(temp2,data);
    //         grad[i] = (fwd-bwd)/(2*dirac);
    //     }
    // }
    
    // Calculate residual for current values
    return calc_residual(x,data);
}

//  quaternion must have magnitude 1 -- used in NL optimization 
double quat_con(const std::vector<double> &x, std::vector<double> &grad, void *data){
    double dirac = 1e-4;
    
    if (!grad.empty()) {
        for(int i=0;i<4;i++){
            std::vector<double> temp = x;
            temp[i]-=dirac;
            Eigen::Quaterniond q;
            q.x() = temp[0];
            q.y() = temp[1];
            q.z() = temp[2];
            q.w() = temp[3];
            double bwd = 1.0 - q.norm();
            temp[i]+=2.0*dirac;
            q.x() = temp[0];
            q.y() = temp[1];
            q.z() = temp[2];
            q.w() = temp[3];
            double fwd = 1.0 - q.norm();
            grad[i] = (fwd-bwd)/(2*dirac);
        }
    }
    // TODO: get rid of eigen
    Eigen::Quaterniond q;
    q.x() = x[0];
    q.y() = x[1];
    q.z() = x[2];
    q.w() = x[3];
    return 1.0 - q.norm();
}

// Take two arrays of points in 3d space and computes the registration.
//  source refers to mesh/model to be fit, target refers to scene/point cloud
//  source and target dimensions are 3xa and 3xb
//  Note: target should be a region of interest (~ centered wrt the selected point) 
icp_model icp(Model model, 
            Eigen::Matrix3Xd target, 
            pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>>* target_tree, 
            int n,
            bool artic_svd_inital = false,
            Eigen::Matrix3Xd R_initial = Eigen::Matrix3d::Identity(),
            Eigen::Matrix3Xd t_initial = Eigen::MatrixXd::Zero(3,1),
            std::vector<double> angles_initial = {},
            double scale_initial = 1.0,
            double diameter = 1.0){

    bool converged = false;
    double max_distance_threshold = diameter;
    int num_iters = 0;
    
    // For the idependent values calculated in one run of the loop 
    Eigen::MatrixXd R;
    Eigen::MatrixXd trans;

    // Updated after every run of the loop
    Eigen::Matrix3Xd R_final = R_initial;
    Eigen::Matrix3Xd trans_final = t_initial;
    std::vector<double> angles_final = angles_initial;
    double prev_error = std::numeric_limits<double>::max();
    double scale_final = scale_initial;

    // Get initial sampling with angles
    Eigen::Matrix3Xd aligned_source = model.getPointsWithAngles(angles_final, R_final, trans_final, scale_final);

    // calculate the initial alignment
    closest_alignment_model ca = get_closest_alignment(aligned_source, target, target_tree, max_distance_threshold);
    
    if(ca.s_vals.cols()<3){
        // insufficient points for registration, return initial pose
        return icp_model(R_final,trans_final,angles_final,scale_final,std::numeric_limits<double>::max());
    }

    while(!converged && num_iters<n){
        // Weighting is based on the inverse of distances (with some normalization)
        Eigen::MatrixXd weights = (1.0+(ca.dists.array())).pow(-1).matrix();
        Eigen::MatrixXd weights_matrix = weights.asDiagonal();
        
        if(!model.requires_NL_fitting() or artic_svd_inital){ // use SVD if not fitting articulations/scale
            // Following this formulation: https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
            Eigen::MatrixXd w_cent_s = (ca.s_vals*weights)/weights.sum();
            Eigen::MatrixXd w_cent_t = (ca.t_vals*weights)/weights.sum();
            
            for(int i = 0; i < ca.s_vals.cols(); i++){
                ca.s_vals.col(i)-=w_cent_s;
            }
            for(int i = 0; i < ca.t_vals.cols(); i++){
                ca.t_vals.col(i)-=w_cent_t;
            }

            Eigen::MatrixXd S = ca.s_vals * weights_matrix * ca.t_vals.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3Xd V = svd.matrixV();

            if(svd.matrixU().determinant()*V.determinant()<0){
                for(int i = 0; i<V.rows();i++){
                    V(i,2) *= -1;
                }
            }

            R = V*svd.matrixU().transpose();
            trans = (w_cent_t - (R * w_cent_s));

            R_final = R * R_final;
            trans_final = R * trans_final + trans;

        } else {    // articulations, need to use NL optimization
            // structure of minimization: R as quaternion, t as (3x), angles as list
            // initialize parameters


            std::vector<double> x0((7+angles_final.size()+int(model.scaling_allowed)), 0); // 2nd arg -- initialize to zero
            std::vector<double> quat_temp{0.0, 0.0, 0.0, 1.0};

            // Add initial quaternion
            for(int i = 0; i < 4; i++){
                x0[i] = quat_temp[i];
            }

            // Add initial translation
            for(int i = 4; i < 7;i++){
                x0[i] = 0.0;
            }

            // Add initial angles
            for(size_t i = 7; i < 7+angles_final.size();i++){
                x0[i] = angles_final[i-7];
            }

            // bounds for pose (quaterions and translation)
            // Lower bound
            std::vector<double> lb{-1.0, -1.0, -1.0, -1.0, -model.diameter/2, -model.diameter/2, -model.diameter/2};
            // Upper bound
            std::vector<double> ub{1.0, 1.0, 1.0, 1.0, model.diameter/2, model.diameter/2, model.diameter/2};
            
            // Add bounds for all articulations
            for(size_t i = 0; i < angles_final.size(); i++){
                lb.push_back(model.joints[i].min_angle);
                ub.push_back(model.joints[i].max_angle);
            }

            // Add bounds for scaling if enabled
            if(model.scaling_allowed){
                lb.push_back(model.min_scale);
                ub.push_back(model.max_scale);
            }
    
            opt_data_registration_residual data_min_obj(x0, &model, ca.t_vals, ca.close_enough_inds, weights, R_final, trans_final);
            
            nlopt::opt opt(nlopt::algorithm::LN_COBYLA, x0.size());
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
            opt.set_min_objective(registration_residual, &data_min_obj);
            opt.set_maxeval(500);
            opt.add_equality_constraint(quat_con, NULL, 0.0001);
            opt.set_ftol_rel(0.0001);
            // counter = 0;
            double minf;

            try{
                nlopt::result result = opt.optimize(x0, minf);
                // cout << "RESULT: " << result << endl;
                if(result < 1){ // positive values are successful
                    return icp_model(R_final,trans_final,angles_final,scale_final,std::numeric_limits<double>::max());
                }
            }
            catch(nlopt::roundoff_limited const&){
                cout << "roundoff limited" << endl;
            }

            // Capture output from minimization
            R = get_R_from_X(x0);
            trans = get_trans_from_X(x0);
            angles_final = get_angles_from_x(x0);

            if(model.scaling_allowed){
                scale_final = x0[x0.size()-1];
            }

            R_final = R * R_final;
            trans_final = R * trans_final + trans;
        }

        // Transform source points
        aligned_source = model.getPointsWithAngles(angles_final,R_final, trans_final, scale_final);
        
        // Check if done - calculate residual based on next set of pt alignments
        ca = get_closest_alignment(aligned_source,target,target_tree,max_distance_threshold);
        // need 3 points
        if(ca.s_vals.cols()<3){ 
            // insufficient points for registration, return initial pose
            // return as failure (infinite error)
            std::cout << "Exiting Early" << std::endl;
            return icp_model(R_final,trans_final,angles_final,scale_final,std::numeric_limits<double>::max());
        }

        // // Calculate residual
        Eigen::Matrix3Xd t_vals_adjust = ca.t_vals;

        for(int i = 0; i < ca.t_vals.cols(); i++){
            t_vals_adjust.col(i) = trans - t_vals_adjust.col(i);
        }
        double error  = ((R * ca.s_vals) + t_vals_adjust).norm();
        
        // Check if converged (changing by small amount -- empiricially determined)
        double error_per_pt = 0.000000025;
        if(std::abs(prev_error-error)<(error_per_pt*ca.s_vals.cols())){
            converged = true;
        }
        prev_error = error;
        num_iters++;
    }
    return icp_model(R_final,trans_final,angles_final,scale_final, prev_error);
}

// Run ICP multiple times to avoid local minima (random rotations, translations, angles, scale).
icp_model icp_random_restarts(Model model, 
                                Eigen::Matrix3Xd target,
                                int num_restarts,
                                int n,
                                double diameter,
                                Eigen::Vector3d ref_point,
                                bool artic_svd_inital){
    
    double min_error = std::numeric_limits<double>::max();

    // initial value for scale
    double scale_final = 1.0;

    // Store target in KDTree for fast lookup of closest points
    int const kMaxLeafCount = 16;
    pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>> tree(target, kMaxLeafCount);
    pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>> *treePtr = &tree;

    // Default return values
    Eigen::Matrix3Xd R_final = Eigen::Matrix3d::Identity();
    Eigen::Matrix3Xd t_final(3,1);
    t_final.col(0) = ref_point;
    std::vector<double> angles_final(model.joints.size(), 0.0);

    icp_model icp_model_final = icp_model(R_final,t_final,angles_final,scale_final, min_error);

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.33);

    for(int ii = 0; ii < num_restarts; ii++){
        // std::cout<<"RR: " << ii <<std::endl;
        // Get random start for orientation, translation, and all relevant ariculations
        Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
        Eigen::Matrix3Xd R_init = q.normalized().toRotationMatrix();
        q = Eigen::Quaterniond::UnitRandom();
        Eigen::Matrix3Xd t_init(3,1);
        t_init.col(0) = ((diameter/2.0)*distribution(generator)*q.normalized().toRotationMatrix().row(0).transpose()+ref_point);

        // Model articulation
        std::vector<double> angles;
        for(size_t jj = 0; jj < model.joints.size(); jj++){
            angles.push_back(model.joints[jj].min_angle + (model.joints[jj].max_angle-model.joints[jj].min_angle)*((double)rand()/(double)RAND_MAX));
        }

        double scale_init = 1.0;
        if(model.scaling_allowed){
            scale_init = model.min_scale + (model.max_scale-model.min_scale)*((double)rand()/(double)RAND_MAX);
        }

        icp_model icp_model_temp = icp(model,target,treePtr,n, artic_svd_inital, R_init, t_init, angles, scale_init, diameter);

        if(icp_model_temp.error<icp_model_final.error){
            icp_model_final = icp_model_temp;
        }        
    }

    // if skipping articulations on first, refit best fit    
    if(artic_svd_inital && model.joints.size()>0){
        icp_model_final = icp(model,target,treePtr,n,false, icp_model_final.R, icp_model_final.t, icp_model_final.angles, icp_model_final.scale, diameter);
    }
    return icp_model_final;
}

Eigen::Matrix3Xd get_cloud_interest(Eigen::Matrix3Xd scene, Eigen::Vector3d ref_point, double diameter){
    int const kMaxLeafCount = 16;
    std::vector<pico_tree::Neighbor<int, Eigen::Matrix3Xd::Scalar>> nn;
    pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>> tree(scene, kMaxLeafCount);
    tree.SearchRadius(ref_point, diameter/2.0, &nn);

    std::vector<int> indices;     
    for(size_t i = 0; i<nn.size(); i++){
    indices.push_back(nn[i].index);
    }

    return scene(Eigen::placeholders::all,indices);
}
