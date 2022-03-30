//////////////////////////////////////////////////////////////////////////
// ConcatenatePC.cpp
// Takes PointCloud2 from Rtabmap and concatenates               
// https://answers.ros.org/question/243588/how-to-use-concatenatepointcloud/
//////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/String.h>
// PCL-specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>


void addNewPC(sensor_msgs::PointCloud2 msg){

}

void mappingToggle(std_msgs::String msg){

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_concatenation");
  ros::NodeHandle nh;

//   Create a ROS subscriber for the input point cloud
  ros::Subscriber subpc = nh.subscribe ("/rtabmap/cloud_map", 1, addNewPC);
  ros::Subscriber subtoggle = nh.subscribe ("/mappingToggle", 1, mappingToggle);

  // Create a ROS publisher for the output point cloud
  pcpub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud",1);

  // Spin
  ros::spin ();
}
