#include <beginner_tutorials/rasm_hardware_interface.h>
#include <beginner_tutorials/Sensor_set_values.h>


MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {

	//Declare all JointHandles, JointInterfaces, and JointLimitInterfaces of the robot.
	init();

	// Create the controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

	//Set the frequency of the control loop
	loop_hz_ = 75;
	ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

	pub = nh.advertise<beginner_tutorials::Sensor_set_values>("rasm_effort_commands", 10);
	sub = nh.subscribe("hardware_joint_positions",10, &MyRobot::callback_joint_positions, this);

	//Run the control loop. This will make the update() method (PID control loop) loop periodically at a specified frequency
	my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot(){
}

void MyRobot::init(){
	//Create joint_state_interface for link_z_1
	hardware_interface::JointStateHandle jointStateHandle_z_1("link_z_1", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
	joint_state_interface_.registerHandle(jointStateHandle_z_1);
	//Create effort joint interface as link_z_1 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_z_1(jointStateHandle_z_1, &joint_effort_command_[0]);
	effort_joint_interface_.registerHandle(jointEffortHandle_z_1);


	//Create joint_state_interface for link_s_2
	hardware_interface::JointStateHandle jointStateHandle_s_2("link_s_2", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
	joint_state_interface_.registerHandle(jointStateHandle_s_2);
	//Create effort joint interface as link_s_2 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_s_2(jointStateHandle_s_2, &joint_effort_command_[1]);
	effort_joint_interface_.registerHandle(jointEffortHandle_s_2);

	//Create joint_state_interface for link_e_3
	hardware_interface::JointStateHandle jointStateHandle_e_3("link_e_3", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
	joint_state_interface_.registerHandle(jointStateHandle_e_3);
	//Create effort joint interface as link_e_3 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_e_3(jointStateHandle_e_3, &joint_effort_command_[2]);
	effort_joint_interface_.registerHandle(jointEffortHandle_e_3);

	//Create joint_state_interface for link_y_4
	hardware_interface::JointStateHandle jointStateHandle_y_4("link_y_4", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
	joint_state_interface_.registerHandle(jointStateHandle_y_4);
	//Create effort joint interface as link_s_2 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_y_4(jointStateHandle_y_4, &joint_effort_command_[3]);
	effort_joint_interface_.registerHandle(jointEffortHandle_y_4);

	//Create joint_state_interface for link_p_5
	hardware_interface::JointStateHandle jointStateHandle_p_5("link_p_5", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
	joint_state_interface_.registerHandle(jointStateHandle_p_5);
	//Create effort joint interface as link_p_5 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_p_5(jointStateHandle_p_5, &joint_effort_command_[4]);
	effort_joint_interface_.registerHandle(jointEffortHandle_p_5);

	//Create joint_state_interface for link_r_6
	hardware_interface::JointStateHandle jointStateHandle_r_6("link_r_6", &joint_position_[5], &joint_velocity_[5], &joint_effort_[5]);
	joint_state_interface_.registerHandle(jointStateHandle_r_6);
	//Create effort joint interface as link_r_6 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_r_6(jointStateHandle_r_6, &joint_effort_command_[5]);
	effort_joint_interface_.registerHandle(jointEffortHandle_r_6);



	joint_limits_interface::getJointLimits("link_s_2", nh_, limits[1]);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle_s_2(jointEffortHandle_s_2,limits[1]);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle_s_2);

	joint_limits_interface::getJointLimits("link_e_3", nh_, limits[2]);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle_e_3(jointEffortHandle_e_3, limits[2]);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle_e_3);

	joint_limits_interface::getJointLimits("link_y_4", nh_, limits[3]);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle_y_4(jointEffortHandle_y_4, limits[3]);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle_y_4);

	joint_limits_interface::getJointLimits("link_p_5", nh_, limits[4]);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle_p_5(jointEffortHandle_p_5, limits[4]);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle_p_5);

	joint_limits_interface::getJointLimits("link_r_6", nh_, limits[5]);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle_r_6(jointEffortHandle_r_6, limits[5]);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle_r_6);


	//Register all joints interfaces
	registerInterface(&joint_state_interface_);
	registerInterface(&position_joint_interface_);
	registerInterface(&effort_joint_interface_);
	registerInterface(&effortJointSaturationInterface);

}

//Control loop
void MyRobot::update(const ros::TimerEvent& e) {
	ROS_INFO("Control loop called");
	elapsed_time_ = ros::Duration(e.current_real - e.last_real);
	read();
	controller_manager_->update(ros::Time::now(), elapsed_time_);
	write(elapsed_time_);
}

void MyRobot::read(){
	ROS_INFO("Control loop called");
}

void MyRobot::write(ros::Duration elapsed_time){
	effort_values.sensor_values.clear();
	for(int i = 0; i < 6; i++){
		effort_values.sensor_values.push_back(joint_effort_command_[i]);
	}
	pub.publish(effort_values);
	effortJointSaturationInterface.enforceLimits(elapsed_time);
	positionJointSaturationInterface.enforceLimits(elapsed_time);
}

int main(int argc, char** argv){
	// Initialize ros node
	ros::init(argc, argv, "MyRobot_hardware_interface_node");
	ros::NodeHandle nh;
	// Seperate spinner thread for the non-real time callbacks such as service callbacks to load controllers
	ros::MultiThreadedSpinner spinner(2);
	// Create the object of the robot hardware interface class and spin the thread
	MyRobot ROBOT(nh);
	spinner.spin();
	return 0;
}
