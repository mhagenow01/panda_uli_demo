// File name:  cppregistration.cpp
// Description: main registration routine called by python/cython to perform parallel ICP registration of models
// Authors: Mike Hagenow, Kevin Macauley, Nicole Gathman
// Date: 11/05/21

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "Model.h"
#include "registration.cpp"
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

struct FitInfo{
  // used to keep track of the details of a particular model's fit
  double pos[3];
  double rot[3][3];
  int num_angles;
  double *angles;
  double scale;
  double residual;
};

struct Fitholder{
  // container holding output of fits
  int num_fits;
  FitInfo *fits;
};

struct ModelInfo{
  //int num_meshes;
  double*** mesh_samples;
  Link* links;
  Joint* joints;
  int num_links;
  int num_joints;
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

struct RefittingInputs{
  // all inputs needed for getFitsForModels
  int num_scene_pts;
  double** scene; // scene is num_pts x 3
  double ref_point[3];
  char* model; 
  ModelInfo *model_info;
  FitInfo initial_fit;
};

void loadFitIntoHolder(Fitholder* fitholder, icp_model icp_final, int i){
    // Takes ICP results and loads into fitholder to be returned to python
    int num_angles = icp_final.angles.size();

    // Position (3x1 vector)
    fitholder->fits[i].pos[0] = icp_final.t(0,0);
    fitholder->fits[i].pos[1] = icp_final.t(1,0);
    fitholder->fits[i].pos[2] = icp_final.t(2,0);

    // Rotation (3x3 matrix)
    for(int ii=0;ii<3;ii++){
        for(int jj=0;jj<3;jj++){
            fitholder->fits[i].rot[ii][jj] = icp_final.R(ii,jj);
        }
    }

    // Angles (array of floats)
    fitholder->fits[i].num_angles = num_angles;
    fitholder->fits[i].angles = (double *)malloc(sizeof(double)*num_angles);
    for(int j=0;j<num_angles;j++){
        fitholder->fits[i].angles[j] = icp_final.angles[j];
    }

    // Scale and Residual (both floats)
    fitholder->fits[i].scale = icp_final.scale;
    fitholder->fits[i].residual = icp_final.error;
}

// This makes a C++ function name have C++ linkage so that a client can link
// Only need function called using Cython function within this scope
extern "C"{

  struct Fitholder* getFitsForModels(struct FittingInputs* fitinputs){
    // Runs ICP random restarts for each of the provided models and returns a list of the fits for each model
    int num_models = fitinputs->num_models;
    Fitholder *fitholder;
    fitholder = (Fitholder *)malloc(sizeof(Fitholder));
    fitholder->num_fits = num_models;
    fitholder->fits = (FitInfo *)malloc(sizeof(FitInfo)*num_models);

    // make ref_point/scene Eigen Vector/Matrix respectively 
    Eigen::Vector3d ref_point(fitinputs->ref_point);

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

    omp_set_num_threads(8);        
    #pragma omp parallel for
    for(int i=0;i<num_models;i++){
        ////////////////////////
        // Build model object //
        ////////////////////////
        Model model_temp = Model(fitinputs->model_info[i]->mesh_samples,fitinputs->models[i],fitinputs->model_info[i]->links, fitinputs->model_info[i]->joints, fitinputs->model_info[i]->num_links, fitinputs->model_info[i]->num_joints,fitinputs->model_info[i]->diameter);

        ///////////////////////////////////
        // Cull search space and run ICP //
        ///////////////////////////////////
        Eigen::Matrix3Xd cloud_trimmed = get_cloud_interest(scene, ref_point, model_temp.diameter);
        icp_model icp_final = icp_random_restarts(model_temp, cloud_trimmed,25,100,model_temp.diameter,ref_point, fitinputs->artic_svd_initial);

        ///////////////////////////////
        // Load data into fitholder  //
        ///////////////////////////////
        loadFitIntoHolder(fitholder,icp_final, i);
    }

    return fitholder;
    }

    struct Fitholder* getRefitsForModel(struct RefittingInputs* refitinputs){
        // Runs ICP random restarts for each of the provided models and returns a list of the fits for each model
        Fitholder *fitholder;
        fitholder = (Fitholder *)malloc(sizeof(Fitholder));
        fitholder->num_fits = 1;
        fitholder->fits = (FitInfo *)malloc(sizeof(FitInfo)*1);

        // make ref_point/scene Eigen Vector/Matrix respectively 
        Eigen::Vector3d ref_point(refitinputs->ref_point);

        std::vector<double> tempPts;
        for(int point = 0; point<refitinputs->num_scene_pts; point++){
            for(int coord = 0; coord < 3; coord++){
                tempPts.push_back(refitinputs->scene[point][coord]); 
            }
        }

        // n x 3 matrix
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> scene_input;
        scene_input = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(tempPts.data(), refitinputs->num_scene_pts, 3);

        // 3 x n matrix (this is needed by multiple functions in the ICP algorithim)
        Eigen::Matrix3Xd scene = scene_input.transpose();

        ////////////////////////
        // Build model object //
        ////////////////////////
        Model model_temp = Model(refitinputs->model_info->mesh_samples,refitinputs->model,refitinputs->model_info->links, refitinputs->model_info->joints, refitinputs->model_info->num_links, refitinputs->model_info->num_joints,refitinputs->model_info->diameter);

        ///////////////////////////////////
        // Cull search space and run ICP //
        ///////////////////////////////////
        Eigen::Matrix3Xd cloud_trimmed = get_cloud_interest(scene, ref_point, model_temp.diameter);

        // Store target in KDTree for fast lookup of closest points
        int const kMaxLeafCount = 16;
        pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>> tree(cloud_trimmed, kMaxLeafCount);
        pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xd>> *treePtr = &tree;

        Eigen::Matrix3Xd R_init(3,3);
        R_init << refitinputs->initial_fit.rot[0][0], refitinputs->initial_fit.rot[0][1], refitinputs->initial_fit.rot[0][2],
                  refitinputs->initial_fit.rot[1][0], refitinputs->initial_fit.rot[1][1], refitinputs->initial_fit.rot[1][2],  
                  refitinputs->initial_fit.rot[2][0], refitinputs->initial_fit.rot[2][1], refitinputs->initial_fit.rot[2][2];
        Eigen::Matrix3Xd t_init(3,1);
        t_init << refitinputs->initial_fit.pos[0], refitinputs->initial_fit.pos[1], refitinputs->initial_fit.pos[2];
        
        std::vector<double> angles;
        for(int v=0;v < refitinputs->initial_fit.num_angles; v++){
            angles.push_back(refitinputs->initial_fit.angles[v]);
        }

        double scale_init = refitinputs->initial_fit.scale;

        icp_model icp_final = icp(model_temp,cloud_trimmed,treePtr,100, false, R_init, t_init, angles, scale_init, refitinputs->model_info->diameter); //artic svd false for refitting

        ///////////////////////////////
        // Load data into fitholder  //
        ///////////////////////////////
        loadFitIntoHolder(fitholder,icp_final,0);

        return fitholder;
    }

    void freeFits(Fitholder* fitholder){
        if(fitholder!=NULL){
            int num_fits = fitholder->num_fits;
            for(int i=0;i<num_fits;i++){
                // Free angles arrays
                if(fitholder->fits[i].angles!=NULL){
                    free(fitholder->fits[i].angles);
                }
            }
            free(fitholder->fits); // Free fit objects
            free(fitholder); //Free holder container
        }
    }
}
