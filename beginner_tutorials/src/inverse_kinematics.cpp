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
    std_msgs::Int32 msg;
    msg.data = 1;
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
    double step_sizes[6] {0.1/hz, 0/hz, 0/hz, 0.85/hz, 0.5/hz, 0.5/hz}; //{0.03, 0.065, 0.07, 0.075, 0.05, 0.06}; // step size x hz is the velocity in rad or m/s
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
    }else{
        ROS_INFO("Error Selecting Speed");
    }
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
    KDL::JntArray q_goal_prev(chain.getNrOfJoints());    // previous goal that was planned to where the goal has been scaled to a certian step size

    // Testing values, should be near zero position after ik
    KDL::Vector zero_pos_v(0.4, 0.4, 1.0);
    KDL::Frame zero_pos_f(zero_pos_v);

    KDL::Frame frame = chain.getSegment(0).getFrameToTip();

    KDL::JntArray qmin(chain.getNrOfJoints());
    qmin(0) = 0.0;
    qmin(1) = -3.0;
    qmin(2) = -3.1;
    qmin(3) = -2.53;
    qmin(4) = -2.13;
    qmin(5) = -1.57;

    KDL::JntArray qmax(chain.getNrOfJoints());
    qmax(0) = 1.00;
    qmax(1) = 3.0;
    qmax(2) = 3.1;
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
        pub.publish(msg);
        break;
    }


    while(nh.ok()){
        try{
            stamped_base_to_eef = tfBuffer.lookupTransform("base_link", "link_r_6", ros::Time(0));

            base_to_eef.setRotation(tf2::Quaternion(stamped_base_to_eef.transform.rotation.x,stamped_base_to_eef.transform.rotation.y, stamped_base_to_eef.transform.rotation.z, stamped_base_to_eef.transform.rotation.w));
            base_to_eef.setOrigin(tf2::Vector3(stamped_base_to_eef.transform.translation.x, stamped_base_to_eef.transform.translation.y, stamped_base_to_eef.transform.translation.z));
            base_to_cam = base_to_eef*eef_to_cam;


            EEF_Origin = base_to_eef.getOrigin();
            //ROS_INFO("Origin: [%f, %f, %f]\n", EEF_Origin[0], EEF_Origin[1], EEF_Origin[2]);
            //ROS_INFO("Solution? %i\n", ret);
            //ROS_INFO("z: %f\n", q(0));
            //ROS_INFO("shoulder: %f\n", q(1));
            //ROS_INFO("elbow: %f\n", q(2));
            //ROS_INFO("yaw: %f\n", q(3));
            //ROS_INFO("pitch: %f\n", q(4));
            //ROS_INFO("roll: %f\n", q(5));
        }catch(tf2::TransformException &ex){
        }
        try{
            stamped_base_to_goal = tfBuffer.lookupTransform("base_link", "goal_pose", ros::Time(0));
            stamped_base_to_face = tfBuffer.lookupTransform("base_link", "face_pose", ros::Time(0));
            base_to_face.setRotation(tf2::Quaternion(stamped_base_to_face.transform.rotation.x,stamped_base_to_face.transform.rotation.y, stamped_base_to_face.transform.rotation.z, stamped_base_to_face.transform.rotation.w));
            base_to_face.setOrigin(tf2::Vector3(stamped_base_to_face.transform.translation.x, stamped_base_to_face.transform.translation.y, stamped_base_to_face.transform.translation.z));
            cam_to_face_normal = base_to_cam.inverse()*base_to_face;
            /*
            tf2::Quaternion quat_cam_to_face;
            quat_cam_to_face = cam_to_face_normal.getRotation();
            tf2::Matrix3x3 m_cam_to_face_normal(quat);
            double cam_to_face_roll, cam_to_face_pitch, cam_to_face_yaw;

            m_cam_to_face_normal.getRPY(cam_to_face_yaw, cam_to_face_pitch, cam_to_face_roll);
            */
            // get the vector from the cam to the face
            cam_to_face = cam_to_face_normal.getOrigin();
            //ROS_INFO("original: [%f, %f, %f]\n", cam_to_face[0], cam_to_face[1], cam_to_face[2]);
            cam_to_face = cam_to_face.normalize();
            //ROS_INFO("normalized: [%f, %f, %f]\n", cam_to_face[0], cam_to_face[1], cam_to_face[2]);
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
            ROS_INFO("x y z normal: [%f, %f, %f] [%f, %f, %f] [%f, %f, %f]\n",  x_normal[0], x_normal[1], x_normal[2], y_normal[0], y_normal[1], y_normal[2], z_normal[0], z_normal[1], z_normal[2]);
            tf2::Matrix3x3 cam_to_normal;
            cam_to_normal.setValue(x_normal[0], y_normal[0], z_normal[0], x_normal[1], y_normal[1], z_normal[1], x_normal[2], y_normal[2], z_normal[2]);
            tf2::Transform cam_to_normal_trans;
            tf2::Quaternion cam_to_normal_quat;
            cam_to_normal.getRotation(cam_to_normal_quat);
            cam_to_normal_trans.setRotation(cam_to_normal_quat);
            cam_to_normal_trans.setOrigin(tf2::Vector3(0, 0, 0));
            tf2::Transform base_to_normal;
            base_to_normal = base_to_cam*cam_to_normal_trans;










            /*

            // Project onto the zy axis
            cam_to_face_zy[0] = 0;
            cam_to_face_zy[1] = cam_to_face[1];
            cam_to_face_zy[2] = cam_to_face[2];
            // getting the yaw and the pitch rotations that would be necessary to get from where we are to where we want to be for the camera_dof
            double cam_to_normal_goal_pitch = std::atan2(-cam_to_face_zy[2], -cam_to_face_zy[1]); // pitch because it is about the x-axis
            double cam_to_normal_goal_yaw = std::atan2(-cam_to_face[0], -cam_to_face[2]); // yaw because it is about the y-axis
            double cam_to_normal_goal_roll = std::atan2(-cam_to_face[1], -cam_to_face[0]);
            // define a new coordinate system rotated relative to the camera
            tf2::Transform cam_to_normal;
            tf2::Quaternion quat_cam_to_normal;
            quat_cam_to_normal.setRPY(0, cam_to_normal_goal_pitch, cam_to_normal_goal_yaw);
            cam_to_normal.setRotation(quat_cam_to_normal);
            cam_to_normal.setOrigin(tf2::Vector3(0, 0, 0));
            tf2::Transform base_to_normal;
            base_to_normal = base_to_cam*cam_to_normal;
            tf2::Quaternion quat_base_to_normal;
            quat_base_to_normal = base_to_normal.getRotation();
            m.setRotation(quat_base_to_normal);
            double base_to_normal_roll, base_to_normal_pitch, base_to_normal_yaw;
            m.getRPY(base_to_normal_roll, base_to_normal_pitch, base_to_normal_yaw);
            */


            base_to_goal.setRotation(tf2::Quaternion(stamped_base_to_goal.transform.rotation.x,stamped_base_to_goal.transform.rotation.y, stamped_base_to_goal.transform.rotation.z, stamped_base_to_goal.transform.rotation.w));
            base_to_goal.setOrigin(tf2::Vector3(stamped_base_to_goal.transform.translation.x, stamped_base_to_goal.transform.translation.y, stamped_base_to_goal.transform.translation.z));
            //EEF_Origin = base_to_goal.getOrigin();
            EEF_Origin = base_to_eef.getOrigin();
            //ROS_INFO("Origin Goal: [%f, %f, %f]\n", EEF_Origin[0], EEF_Origin[1], EEF_Origin[2]);
            //ROS_INFO("Solution? %i\n", ret);
            //ROS_INFO("z: %f\n", q(0));
            //ROS_INFO("shoulder: %f\n", q(1));
            //ROS_INFO("elbow: %f\n", q(2));
            //ROS_INFO("yaw: %f\n", q(3));
            //ROS_INFO("pitch: %f\n", q(4));
            //ROS_INFO("roll: %f\n", q(5));
            EEF_pos_vector[0] = EEF_Origin[0];
            EEF_pos_vector[1] = EEF_Origin[1];
            EEF_pos_vector[2] = EEF_Origin[2];
            //TEMPORARY!!!!!!!!!!!!quat = base_to_goal.getRotation();
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
            //ROS_INFO("base_to_normal: [%f, %f, %f]\n",  roll, pitch, yaw);
            KDL::Rotation eef_goal_rotation = KDL::Rotation::RPY(roll, pitch, yaw);
            //KDL::Rotation eef_goal_rotation = KDL::Rotation::RPY(base_to_normal_roll, base_to_normal_pitch, base_to_normal_yaw);
            //EEF_Origin = base_to_goal.getOrigin();
            //EEF_pos_vector[0] = EEF_Origin[0];
            //EEF_pos_vector[1] = EEF_Origin[1];
            //EEF_pos_vector[2] = EEF_Origin[2];
            KDL::Frame EEF_frame(eef_goal_rotation, EEF_pos_vector);
            // setting the initial joint positions for the ik solver
            q_init(0) = current_joint_positions[0];
            q_init(1) = current_joint_positions[1];
            q_init(2) = current_joint_positions[2];
            q_init(3) = current_joint_positions[3];
            q_init(4) = current_joint_positions[4];
            q_init(5) = current_joint_positions[5];
            KDL::JntArray q_first(chain.getNrOfJoints());
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
            q(3) = q_first(3);
            q(4) = q_first(4);
            q(5) = q_first(5);
            EEF_Origin = base_to_goal.getOrigin();
            EEF_pos_vector[0] = EEF_Origin[0];
            EEF_pos_vector[1] = EEF_Origin[1];
            EEF_pos_vector[2] = EEF_Origin[2];
            KDL::Frame EEF_pos_frame(eef_goal_rotation, EEF_pos_vector);
            iksolver1.CartToJnt(q_init, EEF_pos_frame, q_first);
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
            q(0) = q_first(0);
            q(1) = q_first(1);
            q(2) = q_first(2);


            ROS_INFO("q_prev: [%f, %f, %f, %f, %f, %f]\n", q_prev(0), q_prev(1), q_prev(2), q_prev(3), q_prev(4), q_prev(5));
            // Check the position relative to the previous q
            for(int i = 0; i < chain.getNrOfJoints(); i++){
                // Check if the goal is in the same direction relative to the current position
                if(signbit(q(i) - q_init(i)) == signbit(q_prev(i) - q_init(i))){
                    if(i == 2){
                        ROS_INFO("Same direction\n");
                    }
                    // See if the overall goal is greater than a step away from the previous adjusted goal
                    if(abs(q(i) - q_goal(i)) > step_sizes[i]){
                        if(i == 2){
                            ROS_INFO("Greater than a step\n");
                        }
                        // If it is, then increase the adjusted goal by the step size in the appropriate direction
                        if(q(i) - q_goal(i) > step_sizes[i]){
                            q_goal(i) = q_goal(i) + step_sizes[i];
                        }else if(q(i) - q_goal(i) < step_sizes[i]){
                            q_goal(i) = q_goal(i) - step_sizes[i];
                        }
                    }else{
                        // If it isn't, then set the current goal to be the overall goal q
                        q_goal(i) = q(i);
                    }
                }else{
                    // If it isn't, check if the goal is within a step of the current position q_init
                    if(abs(q(i) - q_init(i)) > step_sizes[i]){
                        if(q(i) - q_init(i) > step_sizes[i]){
                            q_goal(i) = q_init(i) + step_sizes[i];
                        }else if(q(i) - q_init(i) < step_sizes[i]){
                            q_goal(i) = q_init(i) - step_sizes[i];
                        }
                    }else{
                        q_goal(i) = q(i);
                    }
                }

                //if(abs(q_goal(i) - q_goal_prev(i)) < theta_threshold){
                   // q_goal(i) = q_goal_prev(i);
                //}
                //q_goal_prev(i) = q_goal(i);

                // update the previous goal to be equal to the current goal for the next loop of fun
                q_prev(i) = q(i);
            }
            ROS_INFO("q_goal: [%f, %f, %f, %f, %f, %f]\n", q_goal(0), q_goal(1), q_goal(2), q_goal(3), q_goal(4), q_goal(5));
            ROS_INFO("current_joint_positions: [%f, %f, %f, %f, %f, %f]\n", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5));
            ROS_INFO("q: [%f, %f, %f, %f, %f, %f]\n", q(0), q(1), q(2), q(3), q(4), q(5));
            /*
            // using the step size to scale down movements
            for(int i =0; i < chain.getNrOfJoints(); i++){
                if(abs(q(i) - q_init(i)) > step_sizes[i]){
                    if(q(i) - q_init(i) > step_sizes[i]){
                        q_goal(i) = q_init(i) + step_sizes[i];
                    }else if(q(i) - q_init(i) < step_sizes[i]){
                        q_goal(i) = q_init(i) - step_sizes[i];
                    }else{
                        q_goal(i) = q(i);
                    }
                }
                if(abs(q_goal(i) - q_goal_prev(i)) < theta_threshold){
                    q_goal(i) = q_goal_prev(i);
                }
                q_goal_prev(i) = q_goal(i);
            }


            pub_z_1_setpoint.publish(double(q_goal(0)));
            pub_shoulder_setpoint.publish(double(q_goal(1)));
            pub_elbow_setpoint.publish(double(q_goal(2)));
            pub_y_4_setpoint.publish(double(q_goal(3)));
            pub_p_5_setpoint.publish(double(q_goal(4)));
            pub_r_6_setpoint.publish(double(q_goal(5)));
            // publish joint states
            pub_z_1_state.publish(current_joint_positions[0]);
            pub_shoulder_state.publish(current_joint_positions[1]);
            pub_elbow_state.publish(current_joint_positions[2]);
            pub_y_4_state.publish(current_joint_positions[3]);
            pub_p_5_state.publish(current_joint_positions[4]);
            pub_r_6_state.publish(current_joint_positions[5]);
            //ROS_INFO("current_joint_positions: [%f, %f, %f, %f, %f, %f]\n", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5));
            //ROS_INFO("q: [%f, %f, %f, %f, %f, %f]\n", q(0), q(1), q(2), q(3), q(4), q(5));
            */

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