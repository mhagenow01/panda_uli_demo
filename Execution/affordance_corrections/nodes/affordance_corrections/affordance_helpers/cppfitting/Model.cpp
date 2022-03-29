//Model.cpp
#include <string>
#include <regex>
#include <string>
#include <stdbool.h>
#include <list>
#include <iterator>
#include "Model.h"
#include <chrono>
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenDoubleMatrix;
typedef Eigen::Map<EigenDoubleMatrix> EigenDoubleMatrixMap;


using namespace std;

Model::Model (double*** mesh_points,char * inputfile, Link* links,  Joint* joints, int num_links, int num_joints, double diameter){
    //convert links and joints array to vector
    vector<Link> links_vector;
    vector<Joint> joints_vector;

    for(int j = 0; j < num_links; j++){
        links_vector.push_back(links[j]); 
    }
    for(int i = 0; i < num_joints; i++){
        joints_vector.push_back(joints[i]);
    }
    
    //fix this next when debug avail
    this->links = links_vector;
    this->joints = joints_vector;

    /*
        Note: mesh_samples is a 3D array that contains a matrix
        of points for every link in the model
    */
    vector<Eigen::MatrixXd> temp3dVect;

    //iterate through the links
    for(int link_index = 0; link_index < num_links; link_index++){
        Eigen::MatrixXd temp(links[link_index].num_mesh_points,3);
        for(int point = 0; point<links[link_index].num_mesh_points;point++){
            for(int coord = 0; coord < 3; coord++){
                temp(point,coord) = mesh_points[link_index][point][coord];
            }
         }
         //end of link's matrix, add matrix to outer vector of matricies
        temp3dVect.push_back(temp);
    }

    /*
        nc::NdArray<double> mesh_pts;
        this->mesh_pts = nc::NdArray<double>(temp3dVect);
    */  // The next error is caused by mesh_pts being uninitialized. Putting these two lines back in throws another error when rebuilding libregistration.so
    this->mesh_pts = temp3dVect;

    this->diameter = diameter;

    this->scaling_allowed = false;

    //flip_number = 0;
    this->artic_id = 0; // which articulation (e.g., joints, scaling) is active
}

void Model::allowMeshScaling(int min_scale, int max_scale){
   // ''' if set, a model can also update its scale during NL fitting (e.g., different size valves)''' 
    this->scaling_allowed = true;
    this->min_scale = min_scale;
    this->max_scale = max_scale;
}

bool Model::requires_NL_fitting(void){
   // ''' Need NL fitting if articulations or scaling allowed'''
    return (this->joints.size() > 0 ) || this->scaling_allowed;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Model::getPointsWithAngles(std::vector<double> angles, Eigen::Matrix3d R_base, Eigen::MatrixXd t_base, double scale){
   // gets the full point sampled model for a model with given joint angles
    EigenDoubleMatrix pts_arr_return = Eigen::MatrixXd::Zero(0,3);

    for(long unsigned int ii = 0; ii<this->links.size(); ii++){
        
        Eigen::MatrixXd pts_temp = this->mesh_pts[ii]; // get link points
        Eigen::VectorXd centroid_before = pts_temp.colwise().mean(); // 3x1
        pts_temp = (double)scale * pts_temp;
        Eigen::VectorXd centroid_after = pts_temp.colwise().mean(); // 3x1
        Eigen::VectorXd delta = centroid_after-centroid_before; //3x1
        pts_temp.rowwise()-=(centroid_after-centroid_before).transpose(); // subtract 1x3;

        for(int jj = int(ii); jj>=0; jj--){ // count from ii down to 0
            
            ///////////////////////////////////////////////
            // LINK TRANSFORM (needs to be subtracted)   //
            ///////////////////////////////////////////////
            double ** temp_R = this->links[jj].R;
            Eigen::Matrix3d R_temp;
            R_temp << temp_R[0][0], temp_R[0][1], temp_R[0][2], temp_R[1][0], temp_R[1][1], temp_R[1][2], temp_R[2][0], temp_R[2][1], temp_R[2][2];
            Eigen::Vector3d t_temp;
            t_temp << this->links[jj].t[0], this->links[jj].t[1], this->links[jj].t[2];   

            Eigen::MatrixXd pts_temp_T =  pts_temp.transpose();
            pts_temp = (R_temp * pts_temp_T).transpose();
            pts_temp.rowwise()-=scale*t_temp.transpose();

            // JOINT TRANSFORM
            if((jj<int(this->joints.size())) &&  ii!= 0){
                //////////////////////////////////////
                // Transform from the Joint Itself  //
                //////////////////////////////////////
                double ** temp_R_joint = this->joints[jj].R;
                Eigen::Matrix3d R_temp_j;
                R_temp_j << temp_R_joint[0][0], temp_R_joint[0][1], temp_R_joint[0][2], temp_R_joint[1][0], temp_R_joint[1][1], temp_R_joint[1][2], temp_R_joint[2][0], temp_R_joint[2][1], temp_R_joint[2][2];
                Eigen::Vector3d t_temp_j;
                t_temp_j << this->joints[jj].t[0], this->joints[jj].t[1], this->joints[jj].t[2];  
                auto R_t_prod = R_temp_j * pts_temp.transpose();
                pts_temp = R_t_prod.transpose();
                pts_temp.rowwise()+=scale*t_temp_j.transpose();

                ////////////////////////////////////////
                // Transform from the Joint Rotation  //
                ////////////////////////////////////////
                Eigen::Vector3d axis_vector;
                axis_vector << this->joints[jj].axis[0], this->joints[jj].axis[1], this->joints[jj].axis[2];
                Eigen::Matrix3d R_angle;
                R_angle = Eigen::AngleAxisd(angles[jj], axis_vector);
                Eigen::Vector3d origin_before = - scale*t_temp;
                Eigen::Vector3d origin_after = R_angle * origin_before;
                pts_temp = (R_angle * pts_temp.transpose()).transpose(); // rotate
                pts_temp.rowwise()-=(origin_after-origin_before).transpose(); // translate
            }
        }

        // Base rotation and translation
        Eigen::MatrixXd pts_temp_matrixT = pts_temp.transpose();
        pts_temp = (R_base * pts_temp_matrixT).transpose();
        pts_temp.rowwise()+=t_base.col(0).transpose(); 
        
        // Concatenate return points along row (https://newbedev.com/eigen-how-to-concatenate-matrix-along-a-specific-dimension)
        Eigen::MatrixXd temp(pts_arr_return.rows()+pts_temp.rows(),pts_arr_return.cols());
        temp << pts_arr_return, pts_temp;
        pts_arr_return = temp;

    }
    
    // Return as 3 x (num_pts)
    return pts_arr_return.transpose();
}
