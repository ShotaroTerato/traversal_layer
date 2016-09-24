#include <ros/ros.h>
#include "ImageToGridmap.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "image_to_gridmap");
  ros::NodeHandle nh("~");
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(5.0);
  ros::Rate loop_rate(10);

  while(ros::Time::now() - start_time < timeout)
  {
    grid_map::ImageToGridmap imageToGridmap(nh);
    loop_rate.sleep();
  }

  return 0;
}
