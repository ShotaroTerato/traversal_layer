#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>

class ShutdownNode{
public:
  ShutdownNode(){
  map_sub = nh.subscribe("/map_for_costmap", 1, &ShutdownNode::mapCallBack, this);
}

private:
  ros::NodeHandle nh;
  ros::Subscriber map_sub;

  void mapCallBack(const nav_msgs::OccupancyGridConstPtr& map)
  {
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(5.0);
    ros::Rate loop_rate(1);
    while(ros::Time::now() - start_time < timeout)
    {
      ROS_INFO_STREAM("wait");
      loop_rate.sleep();
    }
    ROS_INFO_STREAM("shutdown");
    ros::shutdown();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ShutdownNode");
  ShutdownNode shutdown;
  shutdown;

  ros::spin();
}
