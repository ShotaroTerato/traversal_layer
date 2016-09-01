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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>

class ConvertPC{
public:
  ConvertPC(){
  point_cloud_sub = nh.subscribe("/cloud_pcd", 1, &ConvertPC::ConvertPCCallBack, this);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/pointcloud_rgb", 400, 1);
  pub_image = nh.advertise<sensor_msgs::Image> ("/cloud_image", 1);
  ros::Rate loop_rate(10);
}

private:
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub;
  ros::Publisher pub_cloud;
  ros::Publisher pub_image;
  sensor_msgs::PointCloud2 cloud_in;
  sensor_msgs::PointCloud2 cloud_out;

  void ConvertPCCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud_in = *cloud;

    pcl::fromROSMsg(cloud_in, *pcl_cloud_xyzi);

    int i_max = 0;
    int i_min = 0;
    int max = 255;
    int min = 0;

    for(int i=0; i<pcl_cloud_xyzi->points.size(); i++){
      if(pcl_cloud_xyzi->points[i].intensity>=i_max){
        i_max = pcl_cloud_xyzi->points[i].intensity;
      }
      if(pcl_cloud_xyzi->points[i].intensity<=i_min){
        i_min = pcl_cloud_xyzi->points[i].intensity;
      }
    }

    int i_range = abs(i_max - i_min);
    int range = abs(max - min);

    pcl_cloud_xyzrgb->points.resize(pcl_cloud_xyzi->points.size());
    for(int j=0; j<pcl_cloud_xyzi->points.size(); j++){
      int normalize_num = max - floor((range/i_range) * pcl_cloud_xyzi->points[j].intensity);
      pcl_cloud_xyzrgb->points[j].x = pcl_cloud_xyzi->points[j].x; pcl_cloud_xyzrgb->points[j].y = pcl_cloud_xyzi->points[j].y; pcl_cloud_xyzrgb->points[j].z = pcl_cloud_xyzi->points[j].z; pcl_cloud_xyzrgb->points[j].r = normalize_num;
      pcl_cloud_xyzrgb->points[j].g = normalize_num;
      pcl_cloud_xyzrgb->points[j].b = normalize_num;
    }

    pcl_cloud_xyzrgb->height = 1984;
    pcl_cloud_xyzrgb->width = 1984;
    cv::Mat cloud_image;
    cloud_image = cv::Mat(pcl_cloud_xyzrgb->height, pcl_cloud_xyzrgb->width, CV_8UC3);
    ROS_INFO("height = %d, width = %d", pcl_cloud_xyzrgb->height, pcl_cloud_xyzrgb->width);
    ROS_INFO("rows = %d, cols = %d", cloud_image.rows, cloud_image.cols);
    for(int h=0; h<cloud_image.rows; h++){
      for(int w=0; w<cloud_image.cols; w++){
        pcl::PointXYZRGB point = pcl_cloud_xyzrgb->at(w, h);
        Eigen::Vector3i rgb = point.getRGBVector3i();
        cloud_image.at<cv::Vec3b>(h,w)[0] = rgb[2];
        cloud_image.at<cv::Vec3b>(h,w)[1] = rgb[1];
        cloud_image.at<cv::Vec3b>(h,w)[2] = rgb[0];
      }
    }

    cv_bridge::CvImage out_image;
    out_image.encoding = "bgr8";
    out_image.image = cloud_image;

    sensor_msgs::Image ros_image;
    out_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "/map";

    pcl::toROSMsg(*pcl_cloud_xyzrgb, cloud_out);

    cloud_out.header = cloud_in.header;

    pub_cloud.publish(cloud_out);
    pub_image.publish(ros_image);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ConvertPC");
  ConvertPC convertpc;
  convertpc;

  ros::spin();
}
