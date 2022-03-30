//////////////////////////////////////////////////////////////////////////
// ConcatenatePC.cpp
// Takes PointCloud2 from Rtabmap and concatenates               
// https://answers.ros.org/question/243588/how-to-use-concatenatepointcloud/
//////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/String.h>
// PCL-specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

// Global members
static sensor_msgs::PointCloud2 PCAgg;
static pcl::PCLPointCloud2 pcAgg;
ros::Publisher pcpub;

// When new PC2 messages come in, add them to the aggregate
void addNewPC(sensor_msgs::PointCloud2 msg){
  pcl::PCLPointCloud2 pclmsg;
  pcl_conversions::toPCL(msg,pclmsg);
  pcl::concatenatePointCloud (pcAgg, pclmsg, pcAgg);
  
}

// When mapping is done, publish the final aggregated cloud to the filtered topic
void mappingToggle(std_msgs::String msg){
  if(msg.data=="on"){ // starting mapping, clear aggreagate
    // pcl::clear(pcAgg);
  }
  if(msg.data=="off"){ // mapping done, export aggregate
    pcl_conversions::fromPCL(pcAgg, PCAgg); 
    pcpub.publish(PCAgg);
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_concatenation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber subpc = nh.subscribe ("/rtabmap/cloud_map", 1, addNewPC);
  ros::Subscriber subtoggle = nh.subscribe ("/mappingToggle", 1, mappingToggle);

  // Create a ROS publisher for the output point cloud
  pcpub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud",1,true); // latch

  // Spin
  ros::spin();
}
