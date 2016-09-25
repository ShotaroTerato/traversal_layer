#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(10.0);

  cv::Mat cv_image;
  //cv_image = cv::imread("/home/tera/catkin_ws/src/traversal_layer/maps/mymap_for_costmap.png",CV_LOAD_IMAGE_COLOR);
  cv_image = cv::imread("/home/tera/catkin_ws/src/traversal_layer/maps/gaisyuu_for_costmap_edit.png",CV_LOAD_IMAGE_COLOR);
  float angle = 90, scale = 1.0;
  cv::Point2f center(cv_image.cols*0.5+7, cv_image.rows*0.5);
  const cv::Mat affine_matrix = cv::getRotationMatrix2D(center, angle, scale);
  cv::Mat rot_cv_image;
  cv::warpAffine(cv_image, rot_cv_image, affine_matrix, cv_image.size());

  int origin_x, origin_y, cut_x, cut_y;
  origin_x = rot_cv_image.cols*0.5-cv_image.rows*0.5;
  origin_y = 0;
  cut_x = cv_image.rows;
  cut_y = cv_image.rows;
  cv::Rect rect(origin_x, origin_y, cut_y, cut_x);
  cv::Mat cut_image(rot_cv_image, rect);

  cv_bridge::CvImage out_image;
  out_image.encoding = "bgr8";
//  out_image.image = rot_cv_image;
  out_image.image = cut_image;

  sensor_msgs::Image ros_image;
  out_image.toImageMsg(ros_image);
  ros_image.header.frame_id = "/map";

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/map_image", 1);
  ros::Rate loop_rate(10);

  while(ros::Time::now() - start_time < timeout)
  {
    pub.publish(ros_image);
    loop_rate.sleep();
  }
}
