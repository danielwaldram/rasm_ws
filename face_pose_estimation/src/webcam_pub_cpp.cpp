#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <dlib/opencv.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_pub_cpp");
  ros::NodeHandle nh;

  const int CAMERA_INDEX = 0;
  cv::VideoCapture capture(CAMERA_INDEX);
  if(!capture.isOpened()){
    ROS_ERROR_STREAM("Failed to open camera with index " << CAMERA_INDEX << "!");
    ros::shutdown();
  }

  //image transport is responsible for publishing and subscribing to Images
  image_transport::ImageTransport it(nh);

  //Publish to the /camera topic
  image_transport::Publisher pub_frame = it.advertise("camera", 1);

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(10);

  while(nh.ok()){
    //Load image
    capture >> frame;

    //Check if grabbed frame has content
    if(frame.empty()) {
      ROS_ERROR_STREAM("Failed to capture image!");
      ros::shutdown();
    }

    //Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type and publish
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub_frame.publish(msg);
    cv::waitKey(1);

    ros::spinOnce();
    loop_rate.sleep();
  }

  capture.release();
}
