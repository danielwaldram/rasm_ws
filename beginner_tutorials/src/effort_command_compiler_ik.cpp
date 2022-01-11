#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <beginner_tutorials/pid_effort_commands.h>

static beginner_tutorials::pid_effort_commands pid_efforts;

void effort_callback(const std_msgs::Float64::ConstPtr& effort, int index){
    pid_efforts.commands[index] = effort->data;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "effort_command_compiler_node");
    ros::NodeHandle n;

    //Initializing the pid values as zero
    for(int i = 0; i < 6; i++){
		pid_efforts.commands.push_back(0);
	}

    ros::Subscriber sub_z_1 = n.subscribe<std_msgs::Float64>("/z_pid/control_effort", 1, boost::bind(effort_callback, _1, 0));
    ros::Subscriber sub_elbow = n.subscribe<std_msgs::Float64>("/elbow_3_pid/control_effort",1, boost::bind(effort_callback, _1, 2));
    ros::Subscriber sub_shoulder = n.subscribe<std_msgs::Float64>("/shoulder_2_pid/control_effort", 1, boost::bind(effort_callback, _1, 1));
    ros::Subscriber sub_y_4 = n.subscribe<std_msgs::Float64>("/y_pid/control_effort", 1, boost::bind(effort_callback, _1, 3));
    ros::Subscriber sub_p_5 = n.subscribe<std_msgs::Float64>("/p_pid/control_effort", 1, boost::bind(effort_callback, _1, 4));
    ros::Subscriber sub_r_6 = n.subscribe<std_msgs::Float64>("/r_pid/control_effort", 1, boost::bind(effort_callback, _1, 5));

    ros::Publisher pub = n.advertise<beginner_tutorials::pid_effort_commands>("pid_effort_commands", 10);

    ros::Rate loop_rate(225);

    while(ros::ok()){
        ros::spinOnce();
        pub.publish(pid_efforts);
        loop_rate.sleep();
    }
    return 0;

}
