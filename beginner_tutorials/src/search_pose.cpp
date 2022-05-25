#include <ros/ros.h>
#include <time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"

double current_joint_positions[6] = {0, 0, 0, 0, 0, 0};
void callback_joint_positions(const sensor_msgs::JointState& msg){
     for(int i=0; i < 6; i++){
        if(msg.name[i] == "link_z_1"){
           current_joint_positions[0] = msg.position[i];
        }
        if(msg.name[i] == "link_s_2"){
           current_joint_positions[1] = msg.position[i];
        }
        if(msg.name[i] == "link_e_3"){
           current_joint_positions[2] = msg.position[i];
        }
        if(msg.name[i] == "link_y_4"){
           current_joint_positions[3] = msg.position[i];
        }
        if(msg.name[i] == "link_p_5"){
           current_joint_positions[4] = msg.position[i];
        }
        if(msg.name[i] == "link_r_6"){
           current_joint_positions[5] = msg.position[i];
        }
     }
     //ROS_INFO("position: [%f, %f, %f, %f, %f, %f]\n", current_joint_positions[0], current_joint_positions[1], current_joint_positions[2], current_joint_positions[3], current_joint_positions[4], current_joint_positions[5]);

}


int main(int argc, char** argv){
	ros::init(argc, argv, "search_pose_node");
	ros::NodeHandle nh;
	ros::Subscriber sub;
	sub = nh.subscribe("joint_states",100, callback_joint_positions);
	double hz = 300;
	ros::Rate rate(hz);
	KDL::Tree my_tree;
	tf2_ros::TransformBroadcaster tfb;
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped stamped_base_to_eef;
    tf2::Transform base_to_eef;
    // Step size defines the speed of the end effector
    double step_sizes[6] {0.1/hz, 0.25/hz, 0.25/hz, 0.5/hz, 0.3/hz, 0.3/hz};
    double scaled_step_sizes[6] {0, 0, 0, 0, 0, 0};
    double distance_array[6] {0.01, 0.25, 0.25, 0.05, 0.25, 0.25};
    double temp_step_size;
    // PID Publishers
    ros::Publisher pub_elbow_state = nh.advertise<std_msgs::Float64>("/elbow_3_pid/state",1);
	ros::Publisher pub_elbow_setpoint = nh.advertise<std_msgs::Float64>("/elbow_3_pid/setpoint",1);
    ros::Publisher pub_shoulder_state = nh.advertise<std_msgs::Float64>("/shoulder_2_pid/state",1);
	ros::Publisher pub_shoulder_setpoint = nh.advertise<std_msgs::Float64>("/shoulder_2_pid/setpoint",1);
	ros::Publisher pub_z_1_state = nh.advertise<std_msgs::Float64>("/z_pid/state",1);
	ros::Publisher pub_z_1_setpoint = nh.advertise<std_msgs::Float64>("/z_pid/setpoint",1);
	ros::Publisher pub_y_4_state = nh.advertise<std_msgs::Float64>("/y_pid/state",1);
	ros::Publisher pub_y_4_setpoint = nh.advertise<std_msgs::Float64>("/y_pid/setpoint",1);
	ros::Publisher pub_p_5_state = nh.advertise<std_msgs::Float64>("/p_pid/state",1);
	ros::Publisher pub_p_5_setpoint = nh.advertise<std_msgs::Float64>("/p_pid/setpoint",10);
	ros::Publisher pub_r_6_state = nh.advertise<std_msgs::Float64>("/r_pid/state",10);
	ros::Publisher pub_r_6_setpoint = nh.advertise<std_msgs::Float64>("/r_pid/setpoint",10);
	// Using kdl parser to get the tree from the parameter server
    std::string robot_desc_string;
    nh.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    //Converting the tree to a chain of serial links
    KDL::Chain chain;
    my_tree.getChain("base_link","link_r_6", chain);

    // Create joint array
    KDL::JntArray q_current(chain.getNrOfJoints());
    KDL::JntArray q_prev(chain.getNrOfJoints());
    KDL::JntArray q_goal(chain.getNrOfJoints());
    KDL::JntArray q_search(chain.getNrOfJoints()); // joint array for the search pose
    q_search(0) = 0.1;// 0.19;     // prismatic
    q_search(1) = 1.4868;    // shoulder (prev: 1.309)
    q_search(2) = -2.4795;   // elbow (prev: -2.164)
    q_search(3) = -0.6873912129674569;    // yaw(prev: -0.94)
    q_search(4) = -1.5708; //-1.38; // pitch
    q_search(5) = 0;        // roll
    bool looking_for_pos = true;

    // First find the current joint positions
    while(looking_for_pos){
        // Wait until the current joint positions are up to date
        if(current_joint_positions[0] != 0){
            q_current(0) = current_joint_positions[0];
            q_current(1) = current_joint_positions[1];
            q_current(2) = current_joint_positions[2];
            q_current(3) = current_joint_positions[3];
            q_current(4) = current_joint_positions[4];
            q_current(5) = current_joint_positions[5];
            q_prev(0) = current_joint_positions[0];
            q_prev(1) = current_joint_positions[1];
            q_prev(2) = current_joint_positions[2];
            q_prev(3) = current_joint_positions[3];
            q_prev(4) = current_joint_positions[4];
            q_prev(5) = current_joint_positions[5];
            q_goal(0) = current_joint_positions[0];
            q_goal(1) = current_joint_positions[1];
            q_goal(2) = current_joint_positions[2];
            q_goal(3) = current_joint_positions[3];
            q_goal(4) = current_joint_positions[4];
            q_goal(5) = current_joint_positions[5];
            pub_z_1_state.publish(current_joint_positions[0]);
            pub_shoulder_state.publish(current_joint_positions[1]);
            pub_elbow_state.publish(current_joint_positions[2]);
            pub_y_4_state.publish(current_joint_positions[3]);
            pub_p_5_state.publish(current_joint_positions[4]);
            pub_r_6_state.publish(current_joint_positions[5]);
            pub_z_1_setpoint.publish(current_joint_positions[0]);
            pub_shoulder_setpoint.publish(current_joint_positions[1]);
            pub_elbow_setpoint.publish(current_joint_positions[2]);
            pub_y_4_setpoint.publish(current_joint_positions[3]);
            pub_p_5_setpoint.publish(current_joint_positions[4]);
            pub_r_6_setpoint.publish(current_joint_positions[5]);
            looking_for_pos = false;
            ROS_INFO("Done Looking");
        }else{
            //ROS_INFO("Still Looking");
        }
        ros::spinOnce();
         //ROS_INFO("In Loop");
    }

    while(nh.ok()){
    	try{
            stamped_base_to_eef = tfBuffer.lookupTransform("base_link", "link_r_6", ros::Time(0));
            base_to_eef.setRotation(tf2::Quaternion(stamped_base_to_eef.transform.rotation.x,stamped_base_to_eef.transform.rotation.y, stamped_base_to_eef.transform.rotation.z, stamped_base_to_eef.transform.rotation.w));
            base_to_eef.setOrigin(tf2::Vector3(stamped_base_to_eef.transform.translation.x, stamped_base_to_eef.transform.translation.y, stamped_base_to_eef.transform.translation.z));
        	// setting the initial joint positions for the ik solver
            q_current(0) = current_joint_positions[0];
            q_current(1) = current_joint_positions[1];
            q_current(2) = current_joint_positions[2];
            q_current(3) = current_joint_positions[3];
            q_current(4) = current_joint_positions[4];
            q_current(5) = current_joint_positions[5];


            // Check the position relative to the previous q
            for(int i = 0; i < chain.getNrOfJoints(); i++){
                // Check if the goal is within the deceleration distance of the current position
                temp_step_size = step_sizes[i];
                if(abs(q_search(i) - q_current(i)) < distance_array[i]){
                    temp_step_size = step_sizes[i]*abs(q_search(i) - q_current(i))/distance_array[i];
                }
                // Check if the goal is in the same direction relative to the current position
                if(signbit(q_search(i) - q_current(i)) == signbit(q_prev(i) - q_current(i))){
                    // See if the overall goal is greater than a step away from the previous adjusted goal
                    if(abs(q_search(i) - q_goal(i)) > temp_step_size){
                        if(i == 2){
                            ROS_INFO("Greater than a step\n");
                        }
                        // If it is, then increase the adjusted goal by the step size in the appropriate direction
                        if(q_search(i) - q_goal(i) > temp_step_size){
                            q_goal(i) = q_goal(i) + temp_step_size;
                        }else if(q_search(i) - q_goal(i) < temp_step_size){
                            q_goal(i) = q_goal(i) - temp_step_size;
                        }
                    }else{
                        // If it isn't, then set the current goal to be the overall goal q
                        q_goal(i) = q_search(i);
                    }
                }else{
                    // If it isn't, check if the goal is within a step of the current position q_init
                    if(abs(q_search(i) - q_current(i)) > temp_step_size){
                        if(q_search(i) - q_current(i) > temp_step_size){
                            q_goal(i) = q_current(i) + temp_step_size;
                        }else if(q_search(i) - q_current(i) < temp_step_size){
                            q_goal(i) = q_current(i) - temp_step_size;
                        }
                    }else{
                        q_goal(i) = q_search(i);
                    }
                }

                //if(abs(q_goal(i) - q_goal_prev(i)) < theta_threshold){
                   // q_goal(i) = q_goal_prev(i);
                //}
                //q_goal_prev(i) = q_goal(i);

                // update the previous goal to be equal to the current goal for the next loop of fun
                q_prev(i) = q_goal(i);
            }

        }catch(tf2::TransformException &ex){
        }
        pub_z_1_state.publish(current_joint_positions[0]);
        pub_shoulder_state.publish(current_joint_positions[1]);
        pub_elbow_state.publish(current_joint_positions[2]);
        pub_y_4_state.publish(current_joint_positions[3]);
        pub_p_5_state.publish(current_joint_positions[4]);
        pub_r_6_state.publish(current_joint_positions[5]);
        pub_z_1_setpoint.publish(double(q_goal(0)));
        pub_shoulder_setpoint.publish(double(q_goal(1)));
        pub_elbow_setpoint.publish(double(q_goal(2)));
        pub_y_4_setpoint.publish(double(q_goal(3)));
        pub_p_5_setpoint.publish(double(q_goal(4)));
        pub_r_6_setpoint.publish(double(q_goal(5)));
        /*
        pub_z_1_setpoint.publish(current_joint_positions[0]);
        pub_shoulder_setpoint.publish(current_joint_positions[1]);
        pub_elbow_setpoint.publish(current_joint_positions[2]);
        pub_y_4_setpoint.publish(current_joint_positions[3]);
        pub_p_5_setpoint.publish(current_joint_positions[4]);
        pub_r_6_setpoint.publish(current_joint_positions[5]);
        */
        rate.sleep();
        ros::spinOnce();
    }
    return 0;

}