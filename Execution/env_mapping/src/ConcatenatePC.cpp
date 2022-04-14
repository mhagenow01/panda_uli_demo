//////////////////////////////////////////////////////////////////////////
// ConcatenatePC.cpp
// Takes PointCloud2 from Rtabmap and concatenates               
// https://answers.ros.org/question/243588/how-to-use-concatenatepointcloud/
//////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/String.h>
// PCL-specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <tf2_ros/transform_listener.h>

#include <boost/thread.hpp>

using namespace std;

// Global members
sensor_msgs::PointCloud2* PCAgg;
pcl::PCLPointCloud2* pcAgg;
ros::Publisher pcpub;
ros::Publisher pc_cm_pub;

double cam_x;
double cam_y;
double cam_z;

bool currmapping;

boost::mutex mutex;

// When new PC2 messages come in, add them to the aggregate
void addNewPC(const sensor_msgs::PointCloud2ConstPtr& msg){
  if (currmapping){
    boost::lock_guard<boost::mutex> lock(mutex);

    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl_conversions::toPCL(*msg,*cloud2);
    
    if (cam_z != -999){
      pcl::fromPCLPointCloud2(*cloud2,*cloud);
      pcl::CropBox<pcl::PointXYZRGB> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(cam_x-0.25, cam_y-0.25, cam_z-0.25, 1.0));
      boxFilter.setMax(Eigen::Vector4f(cam_x+0.25, cam_y+0.25, cam_z+0.25, 1.0));
      boxFilter.setNegative(true);
      boxFilter.setInputCloud(cloud);
      boxFilter.filter(*cloud);

      pcl::toPCLPointCloud2(*cloud,*cloud2);
    }

    pcAgg = cloud2;
    PCAgg = new sensor_msgs::PointCloud2;
    pcl_conversions::fromPCL(*cloud2, *PCAgg); 
    pc_cm_pub.publish(*PCAgg);
    delete PCAgg;
  }
}

// When mapping is done, publish the final aggregated cloud to the filtered topic
void mappingToggle(std_msgs::String msg){
  if(msg.data=="on"){ // starting mapping, clear aggregate
    boost::lock_guard<boost::mutex> lock(mutex);
    cout << "WOWOWO" << endl;
    currmapping = true;

    // Send blank message
    PCAgg = new sensor_msgs::PointCloud2;
    PCAgg->header.frame_id = "map";
    PCAgg->header.seq = 0;
    PCAgg->header.stamp = ros::Time::now();
    PCAgg->height = 1;
    PCAgg->point_step = 32;
    PCAgg->row_step = 32;
    PCAgg->is_bigendian = false;
    PCAgg->is_dense = true;

    sensor_msgs::PointField p1;
    p1.name = "x";
    p1.offset = 0;
    p1.datatype = 7;
    p1.count = 1;
    sensor_msgs::PointField p2;
    p2.name = "y";
    p2.offset = 4;
    p2.datatype = 7;
    p2.count = 1;
    sensor_msgs::PointField p3;
    p3.name = "z";
    p3.offset = 8;
    p3.datatype = 7;
    p3.count = 1;
    sensor_msgs::PointField p4;
    p4.name = "rgb";
    p4.offset = 16;
    p4.datatype = 7;
    p4.count = 1;
    vector <sensor_msgs::PointField> points;
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    PCAgg->fields = points;
    
    pc_cm_pub.publish(*PCAgg);
    pcpub.publish(*PCAgg);
    delete PCAgg;
  }
  if(msg.data=="off"){ // mapping done, export aggregate
    boost::lock_guard<boost::mutex> lock(mutex);
    currmapping = false;
    cout << "YOYOYO" << endl;
    PCAgg = new sensor_msgs::PointCloud2;
    pcl_conversions::fromPCL(*pcAgg, *PCAgg); 
    pcpub.publish(*PCAgg);
    delete PCAgg;
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_concatenation");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  currmapping = false;


  cam_z = -999;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber subpc = nh.subscribe ("/rtabmap/cloud_map", 1, addNewPC);
  ros::Subscriber subtoggle = nh.subscribe ("/mappingToggle", 1, mappingToggle);

  // Create a ROS publisher for the output point cloud
  pcpub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud",1,true); // latch
  pc_cm_pub = nh.advertise<sensor_msgs::PointCloud2> ("/rtabmap/cloud_map2",1,false);

  // Spin
  while (ros::ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("panda_link0", "camera_base",ros::Time(0));
        cam_x = transformStamped.transform.translation.x;
        cam_y = transformStamped.transform.translation.y;
        cam_z = transformStamped.transform.translation.z;
    }

    catch (tf2::TransformException &ex) {
      // ROS_WARN("%s",ex.what());
      ros::Duration(0.5).sleep();
      continue;
    }
  
    ros::spinOnce();
    ros::Duration(0.2).sleep();

  }
}
