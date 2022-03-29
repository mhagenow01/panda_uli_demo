#ifndef MODEL_H
#define MODEL_H

#include <string>
#include <regex>
#include <string>
#include <stdbool.h>
#include <list>
#include "NumCpp.hpp"
//#include "urdf_helpers.cpp"

#include "Eigen/Dense"

struct Joint{
    
  double axis[3];
  //double* axis;
  double min_angle;
  double max_angle;
  double ** R;
  //double** R;
  //double* t;
  double * t;
  
  /*
   nc::NdArray<double> axis;
  double min_angle;
  double max_angle;
  nc::NdArray<double> R; 
  nc::NdArray<double> t; 
  */
  
    
};

struct Link{
    
  //double pts[400][3]; 
  //double** R
  double * t;
  double ** R;
  int num_mesh_points;
  //double* t;
  
  
  /*
    nc::NdArray<double> pts; //this was called pts and still is in Model
  nc::NdArray<double> R;
  nc::NdArray<double> t;
  */
  
};
class Model {
    // ''' A model contains information such as meshes, links, joints, and transforms '''
    public:
        //std::string name;
        std::string inputfile;
        //int num_pts;
        bool scaling_allowed;
        double min_scale;
        double max_scale;
       // int flip_number;
        int artic_id;
        double diameter;
        //principal_axes is a 2D array
        //nc::NdArray<float> principal_axes; 
        //nc::NdArray<double> mesh_pts;
        std::vector<Eigen::MatrixXd> mesh_pts;
        std::vector<Joint> joints;
        std::vector<Link> links;
        Model(double*** mesh_points, char* inputfile,Link links[50],  Joint joints[50], int num_links, int num_joints, double diameter);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> getPointsWithAngles(std::vector<double> angles, Eigen::Matrix3d R_base, Eigen::MatrixXd t_base, double scale=1);
        // void calculate_diameter(void);
        void allowMeshScaling(int min_scale, int max_scale);
        //void calculate_principal_axes(void);
        bool requires_NL_fitting(void);
        //void resetFlip(void);
        //rotation method
    private:

};


#endif /*MODEL_H*/
