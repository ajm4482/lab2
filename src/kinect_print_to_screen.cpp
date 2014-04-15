#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cstdio>

#include <pcl_conversions/pcl_conversions.h>

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  try
  {
    //std::cout<<msg->height<<" ";
    pcl::PCLPointCloud2 pcl_pc2;
    //pcl_conversions::toPCL((sensor_msgs::PointCloud2&)msg, pcl_pc2);
    pcl_conversions::toPCL(*msg, pcl_pc2);
     //Actually convert the PointCloud2 message into a type we can reason about
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
   
    pcl::PointCloud < pcl::PointXYZ >::iterator myIterator;
    for(myIterator = pcl_cloud.begin();  
        myIterator != pcl_cloud.end();
        myIterator++)
    {
        std::cout<<*myIterator<<"\n";
    }


  }
  catch (pcl::PCLException& ex)
  {
    ROS_ERROR("Failed to convert a message to a pcl type, dropping observation: %s", ex.what());
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, chatterCallback);
  //ros::Subscriber sub = n.subscribe("/nav_kinect/depth/points", 1000, chatterCallback);
  ros::spin();
  return 0;
}
