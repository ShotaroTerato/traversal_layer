#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class TraversalFilter{
public:
  TraversalFilter(){
  pointcloud_sub = nh.subscribe("/grid_map_visualization/elevation_points", 1, &TraversalFilter::filterCallBack, this);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/untraversal_cloud", 400, 1);
}

private:
  ros::NodeHandle nh;
  ros::Subscriber pointcloud_sub;
  ros::Publisher pub;
  sensor_msgs::PointCloud2 cloud_in;
  sensor_msgs::PointCloud2 cloud_out;

  void filterCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_in = *cloud;

    pcl::fromROSMsg(cloud_in, *pcl_cloud_in);

    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud(pcl_cloud_in);
    z_pass.setFilterFieldName("z");
    z_pass.setFilterLimits(-1.0, -0.03);
    z_pass.filter(*pcl_cloud_out);

    pcl::toROSMsg(*pcl_cloud_out, cloud_out);

    pub.publish(cloud_out);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TraversalFilter");
  TraversalFilter filter;
  filter;

  ros::spin();
}
