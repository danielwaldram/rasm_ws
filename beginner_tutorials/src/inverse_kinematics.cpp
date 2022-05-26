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
#include "std_msgs/Int32.h"
#include <algorithm>


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
    //double theta_threshold = 0.025;
    ros::init(argc, argv, "ik_node");
	ros::NodeHandle nh;
    // publisher to indicate when the test has started
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("time_stamp", 1000);
    ros::Publisher traj_pub = nh.advertise<std_msgs::Float64>("traj_shoulder", 1000);
    ros::Publisher goal_pub = nh.advertise<std_msgs::Float64>("goal_shoulder", 1000);
    std_msgs::Int32 msg;
    msg.data = 1;
    std_msgs::Float64 traj_data;
    traj_data.data = 0.0;
	ros::Subscriber sub;
	sub = nh.subscribe("joint_states",100, callback_joint_positions);
	double hz = 300;
	ros::Rate rate(hz);
	KDL::Tree my_tree;
	tf2_ros::TransformBroadcaster tfb;
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped stamped_base_to_eef;
    geometry_msgs::TransformStamped stamped_base_to_goal;
    geometry_msgs::TransformStamped stamped_base_to_face;
    geometry_msgs::TransformStamped stamped_base_to_normal;
    stamped_base_to_normal.header.frame_id = "base_link";
    stamped_base_to_normal.child_frame_id = "normal";
    tf2::Transform base_to_goal;
    tf2::Transform base_to_eef;
    tf2::Transform base_to_face;
    tf2::Transform cam_to_face_normal;
    tf2::Transform base_to_cam;
    tf2::Vector3 EEF_Origin;
    tf2::Vector3 cam_to_face;
    tf2::Vector3 cam_to_face_zy;
    tf2::Transform eef_to_cam;
    tf2::Quaternion q_eef_to_cam;
    q_eef_to_cam.setRPY(0, 0, 0);
    eef_to_cam.setRotation(q_eef_to_cam);
    float eef_to_cam_x = 0.086;//0.121 for the tablet; 0.086 for the surface
    float eef_to_cam_z = -0.145;//-0.136 for the tablet; -0.145 for the surface
    eef_to_cam.setOrigin(tf2::Vector3(eef_to_cam_x, 0.0, eef_to_cam_z));
    KDL::Vector EEF_pos_vector;
    KDL::Rotation EEF_rot_vector;
    tf2::Quaternion quat;
    tf2::Matrix3x3 m;
    double roll, pitch, yaw;
    double step_sizes[6] {0.1/hz, 0/hz, 0/hz, 2.5/hz, 1.0/hz, 0.5/hz}; //{0.03, 0.065, 0.07, 0.085, 0.05, 0.06}; // step size x hz is the velocity in rad or m/s
    double scaled_step_sizes[6] {0, 0, 0, 0, 0, 0};
    double temp_step_size_prev[6] {0, 0, 0, 0, 0, 0};
    //double distance_array[6] {0.01, 0.25, 0.25, 0.15, 0.15, 0.15};
    double step_increase[6] {1.0/pow(hz,2), 1.0/pow(hz,2), 1.0/pow(hz,2), 2.0/pow(hz,2), 0.75/pow(hz,2), 1.0/pow(hz,2)}; // step_size_increasexhzxhz is the acceleration in rad or m/s^2
    double temp_step_size;
    bool accel = false;
    bool switch_dir = false;
    bool deccel = false;
    ROS_INFO("%s", argv[1]);
    if (std::string(argv[1]) == "s1"){
        ROS_INFO("Slow Speed Profile");
        step_sizes[1] = 0.1/hz;
        step_sizes[2] = 0.1/hz;
    }else if(std::string(argv[1]) == "s2"){
        ROS_INFO("Med Speed Profile");
        step_sizes[1] = 0.2/hz;
        step_sizes[2] = 0.2/hz;
    }else if (std::string(argv[1]) == "s3"){
        ROS_INFO("High Speed Profile");
        step_sizes[1] = 0.3/hz;
        step_sizes[2] = 0.3/hz;
    }else if (std::string(argv[1]) == "s4"){
        ROS_INFO("Higher Speed Profile");
        step_sizes[1] = 0.35/hz;
        step_sizes[2] = 0.35/hz;
    }else if (std::string(argv[1]) == "s5"){
        ROS_INFO("Higher Speed Profile");
        step_sizes[1] = 0.4/hz;
        step_sizes[2] = 0.4/hz;
    }else if (std::string(argv[1]) == "s6"){
        ROS_INFO("S6 Speed Profile");
        step_sizes[1] = 0.45/hz;
        step_sizes[2] = 0.45/hz;
    }else if (std::string(argv[1]) == "s7"){
        ROS_INFO("S7 Profile");
        step_sizes[1] = 0.5/hz;
        step_sizes[2] = 0.5/hz;
    }else if (std::string(argv[1]) == "s8"){
        ROS_INFO("S8 Profile");
        step_sizes[1] = 0.45/hz;
        step_sizes[2] = 0.45/hz;
        step_increase[1] =  2.0/pow(hz,2);
        step_increase[2] =  2.0/pow(hz,2);

    }else if (std::string(argv[1]) == "s9"){
        ROS_INFO("S9 Profile");
        step_sizes[1] = 0.5/hz;
        step_sizes[2] = 0.5/hz;
        step_increase[1] =  2.0/pow(hz,2);
        step_increase[2] =  2.0/pow(hz,2);

    }else if (std::string(argv[1]) == "s10"){
        ROS_INFO("S10 Profile");
        step_sizes[1] = 0.6/hz;
        step_sizes[2] = 0.6/hz;
        step_increase[1] =  2.0/pow(hz,2);
        step_increase[2] =  2.0/pow(hz,2);

    }else{
        ROS_INFO("Error Selecting Speed");
    }
    // The distance is calculated based on the step size and the step_increase which are stand ins for velocity and acceleration 
    double distance_array[6] {0.5*pow(step_sizes[0], 2)/step_increase[0], 0.5*pow(step_sizes[1], 2)/step_increase[1], 0.5*pow(step_sizes[2], 2)/step_increase[2], 0.5*pow(step_sizes[3], 2)/step_increase[3], 0.5*pow(step_sizes[4], 2)/step_increase[4], 0.5*pow(step_sizes[5], 2)/step_increase[5]};

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

    // Grab the eef relative to base
    //tfBuffer.waitForTransform("base_link", "link_r_6", ros::Time(0), ros::Duration(3.0));

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
    KDL::JntArray q(chain.getNrOfJoints()); // The final goal for the first three dof combined with the goal for the screen to be pointed at the user's face
    KDL::JntArray q_prev(chain.getNrOfJoints());
    KDL::JntArray q_init(chain.getNrOfJoints());
    KDL::JntArray q_current(chain.getNrOfJoints());
    KDL::JntArray q_goal(chain.getNrOfJoints());
    KDL::JntArray q_traj_prev(chain.getNrOfJoints());
    KDL::JntArray q_goal_prev(chain.getNrOfJoints());    // previous goal that was planned to where the goal has been scaled to a certian step size
    KDL::JntArray q_switch_pos(chain.getNrOfJoints());

    // Testing values, should be near zero position after ik
    KDL::Vector zero_pos_v(0.4, 0.4, 1.0);
    KDL::Frame zero_pos_f(zero_pos_v);

    KDL::Frame frame = chain.getSegment(0).getFrameToTip();

    KDL::JntArray qmin(chain.getNrOfJoints());
    qmin(0) = 0.0;
    qmin(1) = -3.0;
    qmin(2) = -2.75;
    qmin(3) = -2.53;
    qmin(4) = -2.13;
    qmin(5) = -1.57;

    KDL::JntArray qmax(chain.getNrOfJoints());
    qmax(0) = 1.00;
    qmax(1) = 3.0;
    qmax(2) = 2.75;
    qmax(3) = 2.53;
    qmax(4) = 0;
    qmax(5) = 1.57;

    KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
    KDL::ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
    KDL::ChainIkSolverPos_NR_JL iksolver1(chain,fksolver1,iksolver1v,100,1e-6);

    iksolver1.setJointLimits(qmin, qmax);

    int ret = iksolver1.CartToJnt(q_init, zero_pos_f, q);
    /*
    // Convert to betwen -2pi and 2pi
    for(int i = 1; i < chain.getNrOfJoints(); i++){
        int overshoot = q(i)/(2*M_PI);
        if(abs(q(i)/(2*M_PI)) - abs(overshoot) > 0.5 && overshoot > 0){
            q(i) = q(i) - (overshoot + 1)*2*M_PI;
        }else if(abs(q(i)/(2*M_PI)) - abs(overshoot) > 0.5 && overshoot < 0){
            q(i) = q(i) - (overshoot - 1)*2*M_PI;
        }
        else{
            q(i) = q(i) - overshoot*2*M_PI;
        }
    }
    */
    bool looking_for_pos = true;
    // First find the current joint positions
    while(looking_for_pos){
        // Wait until the current joint positions are up to date
        if(current_joint_positions[0] != 0){
            q_goal(0) = current_joint_positions[0];
            q_goal(1) = current_joint_positions[1];
            q_goal(2) = current_joint_positions[2];
            q_goal(3) = current_joint_positions[3];
            q_goal(4) = current_joint_positions[4];
            q_goal(5) = current_joint_positions[5];
            q_current(0) = current_joint_positions[0];
            q_current(1) = current_joint_positions[1];
            q_current(2) = current_joint_positions[2];
            q_current(3) = current_joint_positions[3];
            q_current(4) = current_joint_positions[4];
            q_current(5) = current_joint_positions[5];
            q_goal_prev(0) = current_joint_positions[0];
            q_goal_prev(1) = current_joint_positions[1];
            q_goal_prev(2) = current_joint_positions[2];
            q_goal_prev(3) = current_joint_positions[3];
            q_goal_prev(4) = current_joint_positions[4];
            q_goal_prev(5) = current_joint_positions[5];
            q_prev(0) = current_joint_positions[0];
            q_prev(1) = current_joint_positions[1];
            q_prev(2) = current_joint_positions[2];
            q_prev(3) = current_joint_positions[3];
            q_prev(4) = current_joint_positions[4];
            q_prev(5) = current_joint_positions[5];
            q_traj_prev(0) = current_joint_positions[0];
            q_traj_prev(1) = current_joint_positions[1];
            q_traj_prev(2) = current_joint_positions[2];
            q_traj_prev(3) = current_joint_positions[3];
            q_traj_prev(4) = current_joint_positions[4];
            q_traj_prev(5) = current_joint_positions[5];
            q_switch_pos(0) = current_joint_positions[0];
            q_switch_pos(1) = current_joint_positions[1];
            q_switch_pos(2) = current_joint_positions[2];
            q_switch_pos(3) = current_joint_positions[3];
            q_switch_pos(4) = current_joint_positions[4];
            q_switch_pos(5) = current_joint_positions[5];
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
        for(int i=0; i < 75; i++){
            ROS_INFO("Publishing Search Message");
            pub.publish(msg);
            rate.sleep();
            ros::spinOnce();
        }
        break;
    }
    ROS_INFO("dist array: [%f, %f, %f, %f, %f, %f]\n", distance_array[0], distance_array[1], distance_array[2], distance_array[3], distance_array[4], distance_array[5]);


    while(nh.ok()){
        try{
            // Find the base to the end effector and camera
            stamped_base_to_eef = tfBuffer.lookupTransform("base_link", "link_r_6", ros::Time(0));
            base_to_eef.setRotation(tf2::Quaternion(stamped_base_to_eef.transform.rotation.x,stamped_base_to_eef.transform.rotation.y, stamped_base_to_eef.transform.rotation.z, stamped_base_to_eef.transform.rotation.w));
            base_to_eef.setOrigin(tf2::Vector3(stamped_base_to_eef.transform.translation.x, stamped_base_to_eef.transform.translation.y, stamped_base_to_eef.transform.translation.z));
            base_to_cam = base_to_eef*eef_to_cam;
            EEF_Origin = base_to_eef.getOrigin();

        }catch(tf2::TransformException &ex){
        }
        try{
            // Find the base to goal and face transforms
            stamped_base_to_goal = tfBuffer.lookupTransform("base_link", "goal_pose", ros::Time(0));
            stamped_base_to_face = tfBuffer.lookupTransform("base_link", "face_pose", ros::Time(0));
            base_to_face.setRotation(tf2::Quaternion(stamped_base_to_face.transform.rotation.x,stamped_base_to_face.transform.rotation.y, stamped_base_to_face.transform.rotation.z, stamped_base_to_face.transform.rotation.w));
            base_to_face.setOrigin(tf2::Vector3(stamped_base_to_face.transform.translation.x, stamped_base_to_face.transform.translation.y, stamped_base_to_face.transform.translation.z));
            
            // Cam to face transform
            cam_to_face_normal = base_to_cam.inverse()*base_to_face;
            // get the vector from the cam to the face: This defines the z-axis of the rotation vector pointing at a user's face
            cam_to_face = cam_to_face_normal.getOrigin();
            cam_to_face = cam_to_face.normalize();
            tf2::Vector3 z_normal;
            z_normal[0] = -cam_to_face[0];
            z_normal[1] = -cam_to_face[1];
            z_normal[2] = -cam_to_face[2];

            tf2::Quaternion quat_cam_to_face;
            quat_cam_to_face = cam_to_face_normal.getRotation();
            tf2::Matrix3x3 cam_to_face_rot;
            cam_to_face_rot.setRotation(quat_cam_to_face);
            tf2::Vector3 cam_to_face_x;
            tf2::Vector3 y_normal;
            tf2::Vector3 x_normal;
            cam_to_face_x = cam_to_face_rot.getColumn(0);
            x_normal = tf2::tf2Cross(z_normal, tf2::tf2Cross(cam_to_face_x, z_normal));
            x_normal = x_normal.normalize();
            y_normal = tf2::tf2Cross(z_normal, x_normal);
            y_normal = y_normal.normalize();
            //ROS_INFO("x y z normal: [%f, %f, %f] [%f, %f, %f] [%f, %f, %f]\n",  x_normal[0], x_normal[1], x_normal[2], y_normal[0], y_normal[1], y_normal[2], z_normal[0], z_normal[1], z_normal[2]);
            tf2::Matrix3x3 cam_to_normal;
            cam_to_normal.setValue(x_normal[0], y_normal[0], z_normal[0], x_normal[1], y_normal[1], z_normal[1], x_normal[2], y_normal[2], z_normal[2]);
            tf2::Transform cam_to_normal_trans;
            tf2::Quaternion cam_to_normal_quat;
            cam_to_normal.getRotation(cam_to_normal_quat);
            cam_to_normal_trans.setRotation(cam_to_normal_quat);
            cam_to_normal_trans.setOrigin(tf2::Vector3(0, 0, 0));
            tf2::Transform base_to_normal;
            base_to_normal = base_to_cam*cam_to_normal_trans;


            base_to_goal.setRotation(tf2::Quaternion(stamped_base_to_goal.transform.rotation.x,stamped_base_to_goal.transform.rotation.y, stamped_base_to_goal.transform.rotation.z, stamped_base_to_goal.transform.rotation.w));
            base_to_goal.setOrigin(tf2::Vector3(stamped_base_to_goal.transform.translation.x, stamped_base_to_goal.transform.translation.y, stamped_base_to_goal.transform.translation.z));
            EEF_Origin = base_to_eef.getOrigin();


            EEF_pos_vector[0] = EEF_Origin[0];
            EEF_pos_vector[1] = EEF_Origin[1];
            EEF_pos_vector[2] = EEF_Origin[2];
            
            quat = base_to_normal.getRotation();
            stamped_base_to_normal.header.stamp = ros::Time::now();
            stamped_base_to_normal.transform.rotation.x = quat.x();
            stamped_base_to_normal.transform.rotation.y = quat.y();
            stamped_base_to_normal.transform.rotation.z = quat.z();
            stamped_base_to_normal.transform.rotation.w = quat.w();
            stamped_base_to_normal.transform.translation.x = EEF_pos_vector[0];
            stamped_base_to_normal.transform.translation.y = EEF_pos_vector[1];
            stamped_base_to_normal.transform.translation.z = EEF_pos_vector[2];
            tfb.sendTransform(stamped_base_to_normal);
            m.setRotation(quat);
            m.getRPY(roll, pitch, yaw);
            KDL::Rotation eef_goal_rotation = KDL::Rotation::RPY(roll, pitch, yaw);

            KDL::Frame EEF_frame(eef_goal_rotation, EEF_pos_vector);
            // setting the initial joint positions for the ik solver
            q_init(0) = current_joint_positions[0];
            q_init(1) = current_joint_positions[1];
            q_init(2) = current_joint_positions[2];
            q_init(3) = current_joint_positions[3];
            q_init(4) = current_joint_positions[4];
            q_init(5) = current_joint_positions[5];
            KDL::JntArray q_first(chain.getNrOfJoints());
            // Solving the inverse Kinematics for the final 3 joints of the RASM, which correspond to its wrist joints
            int ret = iksolver1.CartToJnt(q_init, EEF_frame, q_first);

            // Convert to betwen -2pi and 2pi
            for(int i = 1; i < chain.getNrOfJoints(); i++){
                int overshoot = q_first(i)/(2*M_PI);
                if(abs(q_first(i)/(2*M_PI)) - abs(overshoot) > 0.5 && overshoot > 0){
                    q_first(i) = q_first(i) - (overshoot + 1)*2*M_PI;
                }else if(abs(q_first(i)/(2*M_PI)) - abs(overshoot) > 0.5 && overshoot < 0){
                    q_first(i) = q_first(i) - (overshoot - 1)*2*M_PI;
                }
                else{
                    q_first(i) = q_first(i) - overshoot*2*M_PI;
                }
            }


            // If the motion plan didn't fail, then the goal is updated for the wrist joints
            if(ret == 0){
                q(3) = q_first(3);
                q(4) = q_first(4);
                q(5) = q_first(5);
            }

            // Now finding the joint goals for the first 3 joints
            EEF_Origin = base_to_goal.getOrigin();
            EEF_pos_vector[0] = EEF_Origin[0];
            EEF_pos_vector[1] = EEF_Origin[1];
            EEF_pos_vector[2] = EEF_Origin[2];
            KDL::Frame EEF_pos_frame(eef_goal_rotation, EEF_pos_vector);
            int ret_2 = iksolver1.CartToJnt(q_init, EEF_pos_frame, q_first);
            for(int i = 1; i < chain.getNrOfJoints(); i++){
                int overshoot = q_first(i)/(2*M_PI);
                if(abs(q_first(i)/(2*M_PI)) - abs(overshoot) > 0.5 && overshoot > 0){
                    q_first(i) = q_first(i) - (overshoot + 1)*2*M_PI;
                }else if(abs(q_first(i)/(2*M_PI)) - abs(overshoot) > 0.5 && overshoot < 0){
                    q_first(i) = q_first(i) - (overshoot - 1)*2*M_PI;
                }
                else{
                    q_first(i) = q_first(i) - overshoot*2*M_PI;
                }
            }
            // If the motion plan didn't fail, then the goal is updated for only the first 3 joints
            if(ret == 0 && ret_2 == 0){
                q(0) = q_first(0);
                q(1) = q_first(1);
                q(2) = q_first(2);
            }


            // CREATE THE TRAJECTORY FOR THE RASM'S JOINTS
            // Check the position relative to the previous q
            switch_dir = false;
            accel = false;
            for(int i = 0; i < chain.getNrOfJoints(); i++){
                temp_step_size = step_sizes[i];
                // reset the direction switch position indicator to the current joint position if a switch has been detected
                if (signbit(q(i) - q_init(i)) != signbit(q_prev(i) - q_init(i))){
                    //COMMENT OUT BELOW
                    //q_switch_pos(i) = q_init(i);
                    q_switch_pos(i) = q_traj_prev(i);
                    switch_dir = true;
                    // the trajectory position must also be updated to be the current position
                    //COMMENT OUT BELOW
                    //q_goal(i) = q_init(i);
                }

                // Check if we have a new goal in the same direction
                if (switch_dir == false && q_prev(i)!= q(i)){
                    // if the previous trajectory was in the decceleration region a new switch position will need to be defined so that there is a smooth transition
                    //      from one decelleration to acceleration
                    if (deccel == true){
                        float new_switch_dist = (pow(2*(temp_step_size_prev[i] - step_increase[i])/step_increase[i] + 1,2) - 1)*step_increase[i]/8;
                        // if traj_prev is greater then we are headed in the positive direction so traj_switch should be less
                        if(q_traj_prev(i) > q_switch_pos(i)){
                            q_switch_pos(i) = q_traj_prev(i) - new_switch_dist;
                        }else{
                            q_switch_pos(i) = q_traj_prev(i) + new_switch_dist;
                        }
                    }
                }

                deccel = false;

                // Checkif the trajectory position is within the distance array value of the joint position at the last switch
                if (abs(q_goal(i) - q_switch_pos(i)) < distance_array[i]){
                    temp_step_size = step_increase[i] + step_increase[i]*(0.5*pow((1 + 8*abs(q_goal(i) - q_switch_pos(i))/step_increase[i]), 0.5) - 0.5);
                    accel == true;
                    if(i == 2){
                        //ROS_INFO("step_size: %f\n", step_sizes[i]);
                        ROS_INFO("accel seperation: %f\n", abs(q_goal(i) - q_switch_pos(i)));
                    }
                }
                // Check if the goal is within the deceleration distance of the trajectory position
                if(abs(q(i) - q_goal(i)) < distance_array[i]){
                    temp_step_size = step_increase[i] + step_increase[i]*(0.5*pow((1 + 8*abs(q(i) - q_goal(i))/step_increase[i]), 0.5) - 0.5);
                    deccel = true;
                    //temp_step_size = step_sizes[i]*abs(q(i) - q_init(i))/distance_array[i];
                    ROS_INFO("decel seperation: %f\n", abs(q(i) - q_goal(i)));
                }
                if(deccel == true && accel == true){
                    temp_step_size = std::min(step_increase[i] + step_increase[i]*(0.5*pow((1 + 8*abs(q(i) - q_goal(i))/step_increase[i]), 0.5) - 0.5), step_increase[i] + step_increase[i]*(0.5*pow((1 + 8*abs(q_goal(i) - q_switch_pos(i))/step_increase[i]), 0.5) - 0.5));
                }

                // adjust the temp_step_size to be no higher than the step size
                if(temp_step_size > step_sizes[i]){
                    temp_step_size = step_sizes[i];
                }
                ROS_INFO("step_size: %f\n", step_sizes[i]);
                ROS_INFO("temp_size: %f\n", temp_step_size);
                ROS_INFO("--------------");


                // Check if the goal is in the same direction relative to the current position
                if(signbit(q(i) - q_init(i)) == signbit(q_prev(i) - q_init(i))){
                    // See if the overall goal is greater than a step away from the previous adjusted goal
                    if(abs(q(i) - q_goal(i)) > temp_step_size){
                        if(i == 2){
                            //ROS_INFO("Greater than a step\n");
                        }
                        // If it is, then increase the adjusted goal by the step size in the appropriate direction
                        if(q(i) - q_goal(i) > temp_step_size){
                            q_goal(i) = q_goal(i) + temp_step_size;
                        }else if(q(i) - q_goal(i) < temp_step_size){
                            q_goal(i) = q_goal(i) - temp_step_size;
                        }
                    }else{
                        // If it isn't, then set the current goal to be the overall goal q
                        q_goal(i) = q(i);
                    }
                }else{
                    // If it isn't, check if the goal is within a step of the current position q_init
                    if(abs(q(i) - q_init(i)) > temp_step_size){
                        if(q(i) - q_init(i) > temp_step_size){
                            q_goal(i) = q_init(i) + temp_step_size;
                        }else if(q(i) - q_init(i) < temp_step_size){
                            q_goal(i) = q_init(i) - temp_step_size;
                        }
                    }
                    else{
                        q_goal(i) = q(i);
                    }
                }

                //if(abs(q_goal(i) - q_goal_prev(i)) < theta_threshold){
                   // q_goal(i) = q_goal_prev(i);
                //}
                //q_goal_prev(i) = q_goal(i);

                // update the previous goal to be equal to the current goal for the next loop of fun
                q_prev(i) = q(i);
                q_traj_prev(i) = q_goal(i);
            }
            //ROS_INFO("q_goal: [%f, %f, %f, %f, %f, %f]\n", q_goal(0), q_goal(1), q_goal(2), q_goal(3), q_goal(4), q_goal(5));
            //ROS_INFO("current_joint_positions: [%f, %f, %f, %f, %f, %f]\n", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5));
            //ROS_INFO("q: [%f, %f, %f, %f, %f, %f]\n", q(0), q(1), q(2), q(3), q(4), q(5));

        }catch(tf2::TransformException &ex){
        }
        //ROS_INFO("Z position: %f\n", q[2]);
        //ROS_INFO("q: [%f, %f, %f, %f, %f, %f]\n", q_goal(0), q_goal(1), q_goal(2), q_goal(3), q_goal(4), q_goal(5));
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

        // publishing the trajectory data for the shoulder so it can be evaluated
        traj_data.data = q_goal(1);
        traj_pub.publish(traj_data);
        traj_data.data = q(1);
        goal_pub.publish(traj_data);


        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
