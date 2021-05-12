#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("face_pointcloud", 1);
  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "face_pose";
  msg->height = 1;
  msg->width = 125;
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
        for(int k = 0; k < 5; k++){
            msg->points.push_back (pcl::PointXYZ(-.125 + .05*i, -.125 + .05*j, -.125 + .05*k));
        }
    }
  }
  //msg->points.push_back (pcl::PointXYZ(0.0, 0.0, 0.0));
 //msg->points.push_back (pcl::PointXYZ(0.12, 0.12, 0.0));
  //msg->points.push_back (pcl::PointXYZ(0.12, -0.12, 0.0));
  //msg->points.push_back (pcl::PointXYZ(-0.12, 0.12, 0.0));
  //msg->points.push_back (pcl::PointXYZ(-0.12, -0.12, 0.0));

  ros::Rate loop_rate(10);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
