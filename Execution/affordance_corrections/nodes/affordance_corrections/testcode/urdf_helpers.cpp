

#include </opt/ros/noetic/include/ros/package.h>
 
#include <string>
#include <regex>
#include <string>
#include <stdbool.h>
#include <list>
#include <tuple>
#include "NumCpp.hpp"


using namespace std; 
//using namespace trimesh;
class Joint{
       // ''' used in model'''
        public:
            nc::NdArray<float> axis;
            float min_angle;
            float max_angle;
            nc::NdArray<float> R; //don't know what type to put for R
            nc::NdArray<float> t; 
            Joint(nc::NdArray<float> axis,float min_angle,float max_angle,nc::NdArray<float> R,nc::NdArray<float> t);
};
Joint::Joint(nc::NdArray<float> axis,float min_angle,float max_angle,nc::NdArray<float> R,nc::NdArray<float> t){
        this->axis = axis;
        this->min_angle = max_angle;
        this->max_angle = max_angle;
        this->R = R;
        this->t = t;
}

class Link{
   // ''' used in model'''
    public:
        nc::NdArray<float> pts;
        nc::NdArray<float> R;
        nc::NdArray<float> t;
        Link(nc::NdArray<float> mesh_points, nc::NdArray<float> R, nc::NdArray<float> t, int num_pts);
};
Link::Link(nc::NdArray<float> mesh_points, nc::NdArray<float> R, nc::NdArray<float> t, int num_pts){

    this->R = R;
    this->t = t;
    this->pts = mesh_points;
}

//will be using the original python methods for this function

/*
static  std::tuple<std::vector<Link>, std::vector<Joint>> getLinksAndJoints(nc::NdArray<float> mesh_points, std::string urdffile){
    //''' From the URDF, load link and joint objects for fitting, plotting, etc.
    //Note: an object without articulations would be expected to have one link'''
     nc::NdArray<float> axis;
     nc::NdArray<float> R;
     nc::NdArray<float> t;
     float min_angle;
     float max_angle;
     std::string mesh_file_full;
    rapidxml::xml_document<> xmltree;
    //have to xmltree in buffer!!!!
    char *urdffile_param;
    strcpy(urdffile_param, urdffile.c_str());
    
    xmltree.parse<0>(urdffile_param); //should represent the root after this line
    rapidxml::xml_node<> * root = xmltree.first_node();
    std::vector<Link> links;
    std::vector<Joint> joints;
   

    //The following is mainly traversing the XML structure
    // to pull out key information needed to define the full mesh
    //check for 0 returns!!!!!
    for(rapidxml::xml_node<> *child = root->first_node(); child; child = child->next_sibling()){
         //TODO: only supports revolute currently
         std::string childTag (child->name());
         std::string childAttribute(child->first_attribute("type")->value());
        if((childTag.compare("joint") == 0) && (childAttribute.compare("revolute") == 0)){
            for(rapidxml::xml_node<> *attr = child->first_node(); attr; attr = attr->next_sibling()){
                std::string attrTag (attr->name());
                std::string::size_type size;
                if(attrTag.compare("limit") == 0){
                    min_angle = std::stof((attr->first_attribute("lower")->value()), &size );
                    max_angle = std::stof((attr->first_attribute("upper")->value()), &size );
                }
                if(attrTag.compare("axis") == 0){
                    std::string string = attr->first_attribute("xyz")->value();
                    int index;
                    std::vector<float> tempAxis; 
                    float coordin_float;
                    while((index = string.find(' ')) != std::string::npos){
                        std::string coordin = string.substr(0,index);
                        coordin_float = std::stof(coordin);
                        //add to vector then create ndarray? 
                        tempAxis.push_back(coordin_float);
                        string.erase(0, index + 1);
                    }
                    //need to get z coordinate 
                    coordin_float = std::stof(string);
                    tempAxis.push_back(coordin_float);
                    axis = nc::NdArray<float>(tempAxis);
                    auto axis_double = nc::norm(axis);  //normalize if necessary
                    axis = axis_double.astype<float>();
                }
                if(attrTag.compare("origin") == 0){
                    nc::NdArray<float> rpy; 
                    std::string string = attr->first_attribute("rpy")->value();
                    int index;
                    std::vector<float> tempRpy;
                    float coordin_float;
                    while((index = string.find(' ')) != std::string::npos){
                        std::string coordin = string.substr(0,index);
                        coordin_float = std::stof(coordin);
                        //add to vector then create ndarray? 
                        tempRpy.push_back(coordin_float);
                        string.erase(0, index + 1);
                    }
                    //need to get last coorindate 
                    coordin_float = std::stof(string);
                    tempRpy.push_back(coordin_float);
                    rpy = nc::NdArray<float>(tempRpy);
                    
                     nc::NdArray<float> R;
                    
                    R =  nc::eye<float>(3);
                   ////////////////////////////////////////////////////////////
                    std::string stringT = attr->first_attribute("xyz")->value();
                    int indexT;
                    std::vector<float> tempT;
                    float coordin_floatT;
                    while((indexT = stringT.find(' ')) != std::string::npos){
                        std::string coordin = string.substr(0,indexT);
                        coordin_floatT = std::stof(coordin);
                        //add to vector then create ndarray? 
                        tempT.push_back(coordin_floatT);
                        stringT.erase(0, index + 1);
                    }
                    //need to get z coordinate 
                    coordin_floatT = std::stof(stringT);
                    tempT.push_back(coordin_floatT);
                    auto t = nc::NdArray<float>(tempT);
                }
            }
            Joint joint_temp(axis,min_angle,max_angle,R,t);
            
            joints.push_back(joint_temp);
        }
        if(childTag.compare("link") == 0){
            rapidxml::xml_node<>* visual = (child->first_node("visual"));
            for(rapidxml::xml_node<> *attr = child->first_node(); attr; attr = attr->next_sibling()){
                std::string attrTag (attr->name());
                if(attrTag.compare("origin")){
                    nc::NdArray<float> rpy;
                    std::string string = attr->first_attribute("rpy")->value();
                    int index;
                    std::vector<float> tempRpy;
                    float coordin_float;
                    while((index = string.find(' ')) != std::string::npos){
                        std::string coordin = string.substr(0,index);
                        coordin_float = std::stof(coordin);
                        //add to vector then create ndarray? 
                        tempRpy.push_back(coordin_float);
                        string.erase(0, index + 1);
                    }
                    //need to get last coorindate 
                    coordin_float = std::stof(string);
                    tempRpy.push_back(coordin_float);
                    rpy = nc::NdArray<float>(tempRpy);

                    nc::NdArray<float> R;
                    R =  nc::eye<float>(3);
                   ////////////////////////////////////////////////////////////
                    std::string stringT = attr->first_attribute("xyz")->value();
                    int indexT;
                    std::vector<float> tempT;
                    float coordin_floatT;
                    while((indexT = stringT.find(' ')) != std::string::npos){
                        std::string coordin = string.substr(0,indexT);
                        coordin_floatT = std::stof(coordin);
                        //add to vector then create ndarray? 
                        tempT.push_back(coordin_floatT);
                        stringT.erase(0, index + 1);
                    }
                    //need to get z coordinate 
                    coordin_floatT = std::stof(stringT);
                    tempT.push_back(coordin_floatT);
                    auto t = nc::NdArray<float>(tempT);

                }
                if(attrTag.compare("geometry")){
                    for(rapidxml::xml_node<> *geo = child->first_node(); geo; geo = geo->next_sibling()){
                        std::string geo_attr_tag (geo->name());
                        if(geo_attr_tag.compare("mesh")){
                            std::string mesh_file (geo->first_attribute("filename")->value());
                            //resolve full filename
                            int double_slash_index = mesh_file.find_first_of("//");
                            std::string string_path = mesh_file.substr(double_slash_index);
                            int first_slash_index = string_path.find_first_of("/");
                            std::string path_within_package = "";
                            //Python code has for loop, but I don't think it is necessary
                            path_within_package = string_path;
                            std::string pkg_temp  = string_path.substr(0, first_slash_index);
                            //mesh_file_full = ros::package::getPath(pkg_temp)+ path_within_package;
                        }
                    }
                }
            }
            Link link_temp(mesh_points, R, t,400);
            links.push_back(link_temp);
        }
    }
    std::tuple<std::vector<Link>, std::vector<Joint>> links_joints (links, joints);
    return links_joints;
}

static std::tuple<std::vector<Link>, std::vector<Joint>>  getLinksAndJointsFromSTL(nc::NdArray<float> mesh_points, std::string stlfile){
    //''' An STL file can be interpreted as a single link'''
    std::vector<Link> links;
    std::vector<Joint> joints;
    
    nc::NdArray<float> R = nc::eye<float>(3); //creates diagonal matrix
    nc::NdArray<float> t = nc::zeros<float>(3);
    Link link_temp(mesh_points,R, t,400);
    links.push_back(link_temp);
    std::tuple<std::vector<Link>, std::vector<Joint>> links_joints (links, joints);
    return links_joints;
}*/
//2D array

