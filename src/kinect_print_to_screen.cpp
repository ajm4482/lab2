#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cstdio>
#include <pcl_conversions/pcl_conversions.h>

struct Point{
  float z;
  float y;
  float x;
};

Point min = {0,0,0};

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  try
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
   
    pcl::PointCloud < pcl::PointXYZ >::iterator myIterator;
    // double z = 100;
    // double x;
    // double y;
    Point current = {1024,128,128};

    for(myIterator = pcl_cloud.begin();myIterator != pcl_cloud.end();myIterator++)
    {
      if (myIterator->z < current.z && myIterator->z > 0.005) {
        current.z = myIterator->z;
        current.x = -1 * myIterator->x;
        current.y = myIterator->y;
        std::cout << "Min X: " << current.x << " Min Y: " << current.y << " Min Z: " << current.z << std::endl;
      }
    }

    min = current;
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
  tf::TransformListener listener;
  ros::Subscriber sub = n.subscribe("/nav_kinect/depth/points", 1000, chatterCallback);
// ros::Subscriber sub = n.subscribe("/camera/depth/points",1000,chatterCallback);
  ros::Publisher vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  geometry_msgs::Twist velocity_msg;

  ros::Rate r(10);
  while(ros::ok()) {

    tf::StampedTransform trans;
    try{
      listener.lookupTransform("/base_link", "/nav_kinect_depth_optical_frame", ros::Time(0), trans);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    min.x -= trans.getOrigin().x();

    if ((min.x > 0.1 || min.x < -0.1) && min.z < 0.7) 
    {
      velocity_msg.angular.z = 0.2 * atan2(min.x, min.z) ;
      velocity_msg.linear.x = 0;
    } else 
    if (min.z > 0.65 && min.z < 1.3) 
    {  
      velocity_msg.linear.x = 0.6 * sqrt(min.z*min.z + min.x*min.x);
      velocity_msg.angular.z = 0;
      if (min.z < 0.5) 
        velocity_msg.linear.x = -0.5;
    } else 
    {
      velocity_msg.linear.x = .1;
      velocity_msg.angular.z = 0;
    }
    std::cout << "Angular Z: " << velocity_msg.angular.z << "\tLinear X: " << velocity_msg.linear.x << std::endl;
    vel.publish(velocity_msg);
    ros::spinOnce();
    r.sleep();
  }
  velocity_msg.linear.x = -0.6;
  velocity_msg.angular.z = 0;
  return 0;
}
