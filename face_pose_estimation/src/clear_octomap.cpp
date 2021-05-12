#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "clear_octomap_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("clear_octomap");
  std_srvs::Empty srv;

  ros::Rate loop_rate(1);
  while (nh.ok())
  {
    client.call(srv);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
