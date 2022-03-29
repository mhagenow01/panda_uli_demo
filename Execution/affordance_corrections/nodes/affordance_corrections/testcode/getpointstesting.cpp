// File name:  cppregistration.cpp
// Description: main registration routine called by python/cython to perform parallel ICP registration of models
// Authors: Mike Hagenow, Kevin Macauley, Nicole Gathman
// Date: 11/05/21

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "../affordance_helpers/cppfitting/Model.h"
#include "../affordance_helpers/cppfitting/registration.cpp"
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
    angles.push_back(0.5);
    Eigen::Matrix3d R_base = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd t_base = Eigen::MatrixXd::Zero(3,1);
    t_base(0,0) = 0.05;
    t_base(1,0) = 0.03;
    t_base(2,0) = 0.02;
    R_base(0,0) = -0.680375;
    R_base(0,1) = 0.471684;
    R_base(0,2) = -0.560895;
    R_base(1,0) = 0.658096;
    R_base(1,1) = 0.730009;
    R_base(1,2) = -0.184381;
    R_base(2,0) = 0.322489;
    R_base(2,1) = -0.494571;
    R_base(2,2) = -0.807094;

    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix3Xd temp_pts = model.getPointsWithAngles(angles,R_base,t_base, 1.0);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time (microseconds): " << duration.count() << std::endl;
    
    cout << "Temp pts dims: " << temp_pts.transpose().rows() << " " << temp_pts.transpose().cols() << endl;
    cout << "Temp pts\n" << temp_pts.transpose() << endl;
  }
}
