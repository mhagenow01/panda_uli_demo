// File name:  cppregistration.cpp
// Description: main registration routine called by python/cython to perform parallel ICP registration of models
// Authors: Mike Hagenow, Kevin Macauley, Nicole Gathman
// Date: 11/05/21

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "../../affordance_helpers/cppfitting/Model.h"
#include "../../affordance_helpers/cppfitting/registration.cpp"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry> 
#include <vector>
#include <iterator>
#include <chrono>
#include <pico_tree/eigen.hpp>
#include <pico_tree/kd_tree.hpp>
#include <omp.h>
#include <nlopt.hpp>

using namespace std;

struct ModelInfo{
  //int num_meshes;
  double*** mesh_samples;
  Link* links;
  Joint* joints;
  int num_links;
  int num_joints;
  //int num_pts;
  int num_mesh_points;
  double diameter;
};

struct FittingInputs{
  // all inputs needed for getFitsForModels
  int num_scene_pts;
  double** scene; // scene is num_pts x 3
  double ref_point[3];
  int num_models;
  char** models; 
  ModelInfo ** model_info;
  bool artic_svd_initial;

};

// This makes a C++ function name have C++ linkage so that a client can link
// Only need function called using Cython function within this scope
extern "C"{

  struct Fitholder* getFitsForModels(struct FittingInputs* fitinputs){
    Model model = Model(fitinputs->model_info[0]->mesh_samples,fitinputs->model_info[0]->num_mesh_points,fitinputs->models[0],fitinputs->model_info[0]->links, fitinputs->model_info[0]->joints, fitinputs->model_info[0]->num_links, fitinputs->model_info[0]->num_joints,fitinputs->model_info[0]->diameter);
    
    vector<double> angles;
    angles.push_back(0.1);
    Eigen::Matrix3d R_init = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd t_init = Eigen::MatrixXd::Zero(3,1);
    t_init(0,0) = 0.005;
    t_init(1,0) = 0.005;
    t_init(2,0) = 0.005;
    R_init(0,0) = 0.9800666;
    R_init(0,1) = -0.1986693;
    R_init(0,2) = 0.0000000;
    R_init(1,0) = 0.1986693;
    R_init(1,1) = 0.9800666;
    R_init(1,2) =0.0000000;
    R_init(2,0) = 0.0000000;
    R_init(2,1) = 0.0000000;
    R_init(2,2) = 1.0000000;

    std::vector<double> tempPts;
    for(int point = 0; point<fitinputs->num_scene_pts; point++){
      for(int coord = 0; coord < 3; coord++){
        tempPts.push_back(fitinputs->scene[point][coord]); 
      } 
    }

    // n x 3 matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> scene_input;
    scene_input = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(tempPts.data(), fitinputs->num_scene_pts, 3);

    // 3 x n matrix (this is needed by multiple functions in the ICP algorithim)
    Eigen::Matrix3Xd scene = scene_input.transpose();

    Eigen::Vector3d ref_point(fitinputs->ref_point);

    ///////////////////////////////////
    // Cull search space and run ICP //
    ///////////////////////////////////
    Eigen::Matrix3Xd cloud_trimmed = get_cloud_interest(scene, ref_point, model.diameter);

    bool artic_svd_initial = false;

    pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>> tree(cloud_trimmed, 16);
    pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>> *treePtr = &tree;
    
    icp_model icp_model_temp = icp(model,cloud_trimmed,treePtr,100, artic_svd_initial, R_init, t_init, angles, 1.0, model.diameter);

    cout << icp_model_temp.R << "\n" << icp_model_temp.t << "\n" << icp_model_temp.angles[0] << endl;

  }
}
