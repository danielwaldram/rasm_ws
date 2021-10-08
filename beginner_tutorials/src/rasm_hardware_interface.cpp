#include <beginner_tutorials/rasm_hardware_interface.h>
#include <beginner_tutorials/Sensor_set_values.h>
#include <math.h>

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {

	//Declare all JointHandles, JointInterfaces, and JointLimitInterfaces of the robot.
	init();

	// Create the controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

	//Set the frequency of the control loop
	loop_hz_ = 225;
	ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

	pub = nh.advertise<beginner_tutorials::Sensor_set_values>("rasm_effort_commands", 1);
	pub_elbow_state = nh.advertise<std_msgs::Float64>("/elbow_pid/state",1);
	pub_elbow_setpoint = nh.advertise<std_msgs::Float64>("/elbow_pid/setpoint",1);
    pub_shoulder_state = nh.advertise<std_msgs::Float64>("/shoulder_pid/state",1);
	pub_shoulder_setpoint = nh.advertise<std_msgs::Float64>("/shoulder_pid/setpoint",1);
	pub_z_1_state = nh.advertise<std_msgs::Float64>("/z_1_pid/state",1);
	pub_z_1_setpoint = nh.advertise<std_msgs::Float64>("/z_1_pid/setpoint",1);
	pub_y_4_state = nh.advertise<std_msgs::Float64>("/y_4_pid/state",1);
	pub_y_4_setpoint = nh.advertise<std_msgs::Float64>("/y_4_pid/setpoint",1);
	pub_p_5_state = nh.advertise<std_msgs::Float64>("/p_5_pid/state",1);
	pub_p_5_setpoint = nh.advertise<std_msgs::Float64>("/p_5_pid/setpoint",1);
	pub_r_6_state = nh.advertise<std_msgs::Float64>("/r_6_pid/state",1);
	pub_r_6_setpoint = nh.advertise<std_msgs::Float64>("/r_6_pid/setpoint",1);
	// FOR VELOCITY TEST
	pub_velocity_filter = nh.advertise<beginner_tutorials::pid_effort_commands>("velocity_filtered",1);
	sub = nh.subscribe("hardware_joint_positions",1, &MyRobot::callback_joint_positions, this);
	pid_subscriber = nh.subscribe("pid_effort_commands",1, &MyRobot::callback_pid_values ,this);//sub = nh.subscribe("/elbow_pid/setpoint",10, &MyRobot::,this);

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
	hardware_interface::PosVelAccJointHandle jointPositionHandle_z_1(jointStateHandle_z_1, &joint_position_command_[0], &joint_velocity_command_[0], &joint_acceleration_command_[0]);
	position_joint_interface_.registerHandle(jointPositionHandle_z_1);
	effort_joint_interface_.registerHandle(jointEffortHandle_z_1);


	//Create joint_state_interface for link_s_2
	hardware_interface::JointStateHandle jointStateHandle_s_2("link_s_2", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
	joint_state_interface_.registerHandle(jointStateHandle_s_2);
	hardware_interface::PosVelAccJointHandle jointPositionHandle_s_2(jointStateHandle_s_2, &joint_position_command_[1], &joint_velocity_command_[1], &joint_acceleration_command_[1]);
	position_joint_interface_.registerHandle(jointPositionHandle_s_2);
	//Create effort joint interface as link_s_2 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_s_2(jointStateHandle_s_2, &joint_effort_command_[1]);
	effort_joint_interface_.registerHandle(jointEffortHandle_s_2);

	//Create joint_state_interface for link_e_3
	hardware_interface::JointStateHandle jointStateHandle_e_3("link_e_3", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
	joint_state_interface_.registerHandle(jointStateHandle_e_3);
	hardware_interface::PosVelAccJointHandle jointPositionHandle_e_3(jointStateHandle_e_3, &joint_position_command_[2], &joint_velocity_command_[2], &joint_acceleration_command_[2]);
	position_joint_interface_.registerHandle(jointPositionHandle_e_3);
	//Create effort joint interface as link_e_3 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_e_3(jointStateHandle_e_3, &joint_effort_command_[2]);
	effort_joint_interface_.registerHandle(jointEffortHandle_e_3);

	//Create joint_state_interface for link_y_4
	hardware_interface::JointStateHandle jointStateHandle_y_4("link_y_4", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
	joint_state_interface_.registerHandle(jointStateHandle_y_4);
	hardware_interface::PosVelAccJointHandle jointPositionHandle_y_4(jointStateHandle_y_4, &joint_position_command_[3], &joint_velocity_command_[3], &joint_acceleration_command_[3]);
	position_joint_interface_.registerHandle(jointPositionHandle_y_4);
	//Create effort joint interface as link_s_2 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_y_4(jointStateHandle_y_4, &joint_effort_command_[3]);
	effort_joint_interface_.registerHandle(jointEffortHandle_y_4);

	//Create joint_state_interface for link_p_5
	hardware_interface::JointStateHandle jointStateHandle_p_5("link_p_5", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
	joint_state_interface_.registerHandle(jointStateHandle_p_5);
	hardware_interface::PosVelAccJointHandle jointPositionHandle_p_5(jointStateHandle_p_5, &joint_position_command_[4], &joint_velocity_command_[4], &joint_acceleration_command_[4]);
	position_joint_interface_.registerHandle(jointPositionHandle_p_5);
	//Create effort joint interface as link_p_5 accepts effort command
	hardware_interface::JointHandle jointEffortHandle_p_5(jointStateHandle_p_5, &joint_effort_command_[4]);
	effort_joint_interface_.registerHandle(jointEffortHandle_p_5);

	//Create joint_state_interface for link_r_6
	hardware_interface::JointStateHandle jointStateHandle_r_6("link_r_6", &joint_position_[5], &joint_velocity_[5], &joint_effort_[5]);
	joint_state_interface_.registerHandle(jointStateHandle_r_6);
	hardware_interface::PosVelAccJointHandle jointPositionHandle_r_6(jointStateHandle_r_6, &joint_position_command_[5], &joint_velocity_command_[5], &joint_acceleration_command_[5]);
	position_joint_interface_.registerHandle(jointPositionHandle_r_6);
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
    double Vc_array[2] = {0,0};
	//ROS_INFO("I heard:[%f ,%f ,%f ,%f ,%f ,%f ] /n", joint_position_command_[0],joint_position_command_[1],joint_position_command_[2],joint_position_command_[3],joint_position_command_[4],joint_position_command_[5]);
	//ROS_INFO("  v:[%f ,%f ,%f ,%f ,%f ,%f ] ", joint_velocity_command_[0],joint_velocity_command_[1],joint_velocity_command_[2],joint_velocity_command_[3],joint_velocity_command_[4],joint_velocity_command_[5]);
        pub_z_1_setpoint.publish(joint_position_command_[0]);
        pub_shoulder_setpoint.publish(joint_position_command_[1]);
        pub_elbow_setpoint.publish(joint_position_command_[2]);
        pub_y_4_setpoint.publish(joint_position_command_[3]);
        pub_p_5_setpoint.publish(joint_position_command_[4]);
        pub_r_6_setpoint.publish(joint_position_command_[5]);

	//feedback_calc(Vc_array);
	feedforward_torque(Vc_array);
    //inverse_dynamic_calc(Vc_array);

	elapsed_time_ = ros::Duration(e.current_real - e.last_real);
	velocity_calc(elapsed_time_);
	read();
	controller_manager_->update(ros::Time::now(), elapsed_time_);
	//inv_dyn_write(elapsed_time_,Vc_array);
	write(elapsed_time_,Vc_array);
}

void MyRobot::read(){
	//ROS_INFO("Control loop called");
}




void MyRobot::write(ros::Duration elapsed_time, double *Vc_array){
	effort_values.sensor_values.clear();
	//this for loop converts the doubles saved in the pid_commands array to ints so that they can be sent to the Arduino
	//for(int i = 0; i < 6; i++){
	//	effort_values.sensor_values.push_back((int)pid_commands[i]);
	//}
	//z_1 joint which does not have feedforward
	effort_values.sensor_values.push_back((int)pid_commands[0]);
	//shoulder and elbow, which do have feedforward
	effort_values.sensor_values.push_back((int)pid_commands[1] + (int)Vc_array[0]);
	effort_values.sensor_values.push_back((int)pid_commands[2] + (int)Vc_array[1]);
	//effort_values.sensor_values.push_back((int)Vc_feedforward[0]);
	//effort_values.sensor_values.push_back((int)Vc_feedforward[1]);
	// Including the torque required to overcome colomb friction for the yaw joint
	//if(joint_velocity_command_[3] > 0){
     //   effort_values.sensor_values.push_back((int)pid_commands[3] + 400);
	//}else if(joint_velocity_command_[3] < 0){
      //   effort_values.sensor_values.push_back((int)pid_commands[3] - 400);
	//}else{
     //   effort_values.sensor_values.push_back((int)pid_commands[3]);
	//}
	for(int i = 3; i < 6; i++){
		effort_values.sensor_values.push_back((int)pid_commands[i]);
	}


	pub.publish(effort_values);
	effortJointSaturationInterface.enforceLimits(elapsed_time);
	positionJointSaturationInterface.enforceLimits(elapsed_time);
}


void MyRobot::feedforward_torque(double *Vc_array){
    //ROS_INFO("Feedforward Calc");
    //Gear ratio
    double N[2] = {1.5*96, 96};
    //motor damping
    double b = 0.000003716;
    //motor coulomb friction
    double c = 0.00404055;
    //motor inertia
    double J = 0.0000057904;
    //gearbox efficiency
    double eff = 0.73;
    //amp gain
    double gain = 1/15.8856;
    //motor constant
    double kt = 0.048;
    //armature resistance
    double Ra = 4.04;

    //FRICTION CALC
    double F1,F2;
    int sign_1 = 1;
    int sign_2 = 1;
    if(joint_velocity_command_[1] > 0){
        sign_1 = 1;
    }if(joint_velocity_command_[1] < 0){
        sign_1 = -1;
    }else{
        sign_1 = 0;
    }
    if(joint_velocity_command_[2] > 0){
        sign_2 = 1;
    }if(joint_velocity_command_[2] < 0){
        sign_2 = -1;
    }else{
        sign_2 = 0;
    }
    F1 = pow(N[0],2)*b*joint_velocity_command_[1] + N[0]*c*sign_1;
    F2 = pow(N[1],2)*b*joint_velocity_command_[2] + N[1]*c*sign_2;
    //ROS_INFO("  F:[%f ,%f] ", F1, F2);
    // INERTIAL CALC
    double H1, H2, H1_prime, H2_prime;
    H1 = (2.1347 + 1.9865*cos(joint_position_command_[2]))*joint_acceleration_command_[1] + (1.2328 + 0.99326*cos(joint_position_command_[2]))*joint_acceleration_command_[2];
    H2 = (1.2328 + 0.99326*cos(joint_position_command_[2]))*joint_acceleration_command_[1] + 1.2328*joint_acceleration_command_[2];
    H1_prime = H1 + pow(N[0],2)*J*joint_acceleration_command_[1];
    H2_prime = H2 + pow(N[1],2)*J*joint_acceleration_command_[2];
    //ROS_INFO("  H:[%f ,%f] ", H1_prime, H2_prime);
    // CENTRIPETAL/CORIOLIS CALC
    double h, V1, V2;
    h = 0.99326*sin(joint_position_command_[2]);
    V1 = -2*h*joint_velocity_command_[1]*joint_velocity_command_[2] - h*pow(joint_velocity_command_[2],2);
    V2 = h*pow(joint_velocity_command_[1],2);
    //ROS_INFO("  C:[%f ,%f] ", V1, V2);
    //TORQUE AT JOINTS CALC
    double T1, T2;
    T1 = H1_prime + F1 + V1;
    T2 = H2_prime + F2 + V2;

    //CONTROL VOLTAGE CALC
    double Vc1, Vc2;
    Vc1 = (1/gain)*(Ra/kt)*((1/eff)*(1/N[0])*T1 + (pow(kt,2)/Ra)*N[0]*joint_velocity_command_[1]);
    Vc2 = (1/gain)*(Ra/kt)*((1/eff)*(1/N[1])*T2 + (pow(kt,2)/Ra)*N[1]*joint_velocity_command_[2]);
    Vc_array[0] = Vc1;
    Vc_array[1] = Vc2;
    //ROS_INFO("  Vc:[%f ,%f] ", Vc_array[0], Vc_array[1]);
    //ROS_INFO("  P:[%f ,%f] ", joint_position_command_[1], joint_position_command_[2]);
    //ROS_INFO("  V:[%f ,%f] ", joint_velocity_command_[1], joint_velocity_command_[2]);
    //ROS_INFO("  A:[%f ,%f] ", joint_acceleration_command_[1], joint_acceleration_command_[2]);
    return;
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
