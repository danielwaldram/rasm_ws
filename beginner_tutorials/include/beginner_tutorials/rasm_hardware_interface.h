#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "beginner_tutorials/Sensor_set_values.h"
#include <hardware_interface/posvelacc_command_interface.h>
#include "std_msgs/Float64.h"
#include "beginner_tutorials/pid_effort_commands.h"



// This is the class of the robots hardware which consists of the methods for
// reading the joint sensor data, sending commands to the motor, and  the joint
// interface data
class MyRobot: public hardware_interface::RobotHW
{
public:
  // Constructor
  MyRobot(ros::NodeHandle& nh);
  // Destructor
  ~MyRobot();
  // init method in which we define the joint handle, joint's interfaces, and joint
  // limits interfaces
  void init();
  // Update method is the control loop()
  void update(const ros::TimerEvent& e);
  // method for reading joint sensor data
  void read();
  // method for sending commnad to motors
  void write(ros::Duration elapsed_time, double *Vc_feedforward);
  //method for computing the feedforward torque
  void feedforward_torque(double *Vc_array);

  void callback_pid_values(const beginner_tutorials::pid_effort_commands& msg){
    pid_commands[0] = msg.commands[0];
    pid_commands[1] = msg.commands[1];
    pid_commands[2] = msg.commands[2];
    pid_commands[3] = msg.commands[3];
    pid_commands[4] = msg.commands[4];
    pid_commands[5] = msg.commands[5];
  }

  void callback_joint_positions(const beginner_tutorials::Sensor_set_values& msg){
    //ROS_INFO("I heard: [%i]", msg.sensor_values[0]);cat

  	// PREVIOUS
  	//current_position[0] = msg.sensor_values[0]/1000.0;
    //current_position[1] = ((msg.sensor_values[1] - 5) - 495)*M_PI/501.5;
    //current_position[2] = ((msg.sensor_values[2] -5) - 514)*M_PI/501.5;
    //current_position[3] = ((msg.sensor_values[3] - 5)- 535)*M_PI/501.5;
    //current_position[4] = ((msg.sensor_values[4] - 5) - 800)*M_PI/501.5;
    //current_position[5] = -((msg.sensor_values[5] - 5) - 555)*M_PI/501.5;

    current_position[0] = msg.sensor_values[0]/1000.0;
    current_position[1] = (msg.sensor_values[1] - 1895)*2*M_PI/4095;
    current_position[2] = (msg.sensor_values[2] - 2101)*2*M_PI/4095;
    current_position[3] = (msg.sensor_values[3]- 2164)*2*M_PI/4095;
    current_position[4] = (msg.sensor_values[4] - 3359)*2*M_PI/4095;
    current_position[5] = -(msg.sensor_values[5] - 2240)*2*M_PI/4095;

    low_pass_filter();
  };
//-(msg.sensor_values[1] - 500)*M_PI/512;
  ros::Publisher pub;
  ros::Publisher pub_elbow_state;
  ros::Publisher pub_elbow_setpoint;
  ros::Publisher pub_shoulder_state;
  ros::Publisher pub_shoulder_setpoint;
  ros::Publisher pub_z_1_state;
  ros::Publisher pub_z_1_setpoint;
  ros::Publisher pub_y_4_state;
  ros::Publisher pub_y_4_setpoint;
  ros::Publisher pub_p_5_state;
  ros::Publisher pub_p_5_setpoint;
  ros::Publisher pub_r_6_state;
  ros::Publisher pub_r_6_setpoint;
  ros::Publisher pub_velocity_filter;
  ros::Subscriber sub;
  ros::Subscriber pid_subscriber;
  beginner_tutorials::Sensor_set_values joint_read;
  beginner_tutorials::Sensor_set_values effort_values;
  beginner_tutorials::pid_effort_commands velocity_filtered;
  // I may need these two below but for now I will try to use a subscriber instead
  //ros::ServiceClient client;
  //beginner_tutorials::HardwareJointPositions joint_read;
protected:
  // The joint state interface is for getting feedback on position
  // The joint and limit interfaces for position and effort are declared
  // Velocity could be as well, but I don't think it is needed for the RASM
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  hardware_interface::PosVelAccJointInterface position_joint_interface_;
 // hardware_interface::PosVelAccJointInterface pos_vel_acc_joint_interface_;

  joint_limits_interface::JointLimits limits[6];
  joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
  joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;

  // The joint position velocity and effort arrays are for reading
  // position effort and velocity from the robot. I don't know if I will have
  // anything to read except for position.
  double joint_position_[6] = {0,0,0,0,0,0};
  double joint_velocity_[6]= {0,0,0,0,0,0};
  double joint_effort_[6];
  double joint_effort_command_[6];
  double joint_position_command_[6] = {0,0,0,0,0,0};
  double joint_velocity_command_[6]= {0,0,0,0,0,0};
  double joint_acceleration_command_[6] = {0,0,0,0,0,0};
  double pid_commands[6] = {0,0,0,0,0,0};
  bool first_time_filter_flag = 1;
  double previous_position[6] = {0,0,0,0,0,0};
  double current_position[6] = {0,0,0,0,0,0};
  double velocity_filter_test[6] = {0,0,0,0,0,0};
  double velocity_filter_test_prev[6] = {0,0,0,0,0,0};
  bool zero_position = 1;
    void velocity_calc(const ros::Duration& elapsed_time_){
        joint_velocity_[1] = (joint_position_[1] - previous_position[1])/elapsed_time_.toSec();//Shoulder velocity
        joint_velocity_[2] = (joint_position_[2] - previous_position[2])/elapsed_time_.toSec();//Elbow velocity
        //ROS_INFO("current: %f, previous: %f, velocity: %lf", current_position[1], previous_position[1], joint_velocity_[1]);
    }

    void inverse_dynamic_calc(double *Vc_array){
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
        //effective acceleration term
        double u[2];

        u[0] = pid_commands[1] + joint_acceleration_command_[1]; //Shoulder u value
        u[1] = pid_commands[2] + joint_acceleration_command_[2]; //Elbow u value

         // INERTIAL CALC
        double H1, H2, H1_prime, H2_prime;
        H1 = (2.1347 + 1.9865*cos(joint_position_[2]))*u[0] + (1.2328 + 0.99326*cos(joint_position_[2]))*u[1];
        H2 = (1.2328 + 0.99326*cos(joint_position_[2]))*u[0] + 1.2328*u[1];
        H1_prime = H1 + pow(N[0],2)*J*u[0];
        H2_prime = H2 + pow(N[1],2)*J*u[1];

        //FRICTION CALC
        double F1,F2;
        int sign_1 = 1;
        int sign_2 = 1;
        if(joint_velocity_[1] > 0){
            sign_1 = 1;
        }if(joint_velocity_[1] < 0){
            sign_1 = -1;
        }else{
            sign_1 = 0;
        }
        if(joint_velocity_[2] > 0){
            sign_2 = 1;
        }if(joint_velocity_[2] < 0){
            sign_2 = -1;
        }else{
            sign_2 = 0;
        }
        F1 = pow(N[0],2)*b*joint_velocity_[1] + N[0]*c*sign_1;
        F2 = pow(N[1],2)*b*joint_velocity_[2] + N[1]*c*sign_2;
        //ROS_INFO("  F:[%f ,%f] ", F1, F2);

        // CENTRIPETAL/CORIOLIS CALC
        double h, V1, V2;
        h = 0.99326*sin(joint_position_[2]);
        V1 = -2*h*joint_velocity_[1]*joint_velocity_[2] - h*pow(joint_velocity_[2],2);
        V2 = h*pow(joint_velocity_[1],2);
        //ROS_INFO("  C:[%f ,%f] ", V1, V2);
        //TORQUE AT JOINTS CALC
        double T1, T2;
        T1 = F1 + V1 + H1_prime;
        T2 = F2 + V2 + H2_prime;

        //CONTROL VOLTAGE CALC
        double Vc1, Vc2;
        Vc1 = (1/gain)*(Ra/kt)*((1/eff)*(1/N[0])*T1 + (pow(kt,2)/Ra)*N[0]*joint_velocity_[1]);
        Vc2 = (1/gain)*(Ra/kt)*((1/eff)*(1/N[1])*T2 + (pow(kt,2)/Ra)*N[1]*joint_velocity_[2]);
        Vc_array[0] = Vc1;
        Vc_array[1] = Vc2;
        //ROS_INFO("  Vc:[%f ,%f] ", Vc_array[0], Vc_array[1]);
        //ROS_INFO("  P:[%f ,%f] ", joint_position_command_[1], joint_position_command_[2]);
        //ROS_INFO("  V:[%f ,%f] ", joint_velocity_command_[1], joint_velocity_command_[2]);
        //ROS_INFO("  A:[%f ,%f] ", joint_acceleration_command_[1], joint_acceleration_command_[2]);
        return;

    }

    void feedback_calc(double *Vc_array){
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
        if(joint_velocity_[1] > 0){
            sign_1 = 1;
        }if(joint_velocity_[1] < 0){
            sign_1 = -1;
        }else{
            sign_1 = 0;
        }
        if(joint_velocity_[2] > 0){
            sign_2 = 1;
        }if(joint_velocity_[2] < 0){
            sign_2 = -1;
        }else{
            sign_2 = 0;
        }
        F1 = pow(N[0],2)*b*joint_velocity_[1] + N[0]*c*sign_1;
        F2 = pow(N[1],2)*b*joint_velocity_[2] + N[1]*c*sign_2;
        //ROS_INFO("  F:[%f ,%f] ", F1, F2);

        // CENTRIPETAL/CORIOLIS CALC
        double h, V1, V2;
        h = 0.99326*sin(joint_position_[2]);
        V1 = -2*h*joint_velocity_[1]*joint_velocity_[2] - h*pow(joint_velocity_[2],2);
        V2 = h*pow(joint_velocity_[1],2);
        //ROS_INFO("  C:[%f ,%f] ", V1, V2);
        //TORQUE AT JOINTS CALC
        double T1, T2;
        T1 = F1 + V1;
        T2 = F2 + V2;

        //CONTROL VOLTAGE CALC
        double Vc1, Vc2;
        Vc1 = (1/gain)*(Ra/kt)*((1/eff)*(1/N[0])*T1 + (pow(kt,2)/Ra)*N[0]*joint_velocity_[1]);
        Vc2 = (1/gain)*(Ra/kt)*((1/eff)*(1/N[1])*T2 + (pow(kt,2)/Ra)*N[1]*joint_velocity_[2]);
        Vc_array[0] = Vc1;
        Vc_array[1] = Vc2;
        //ROS_INFO("  Vc:[%f ,%f] ", Vc_array[0], Vc_array[1]);
        //ROS_INFO("  P:[%f ,%f] ", joint_position_command_[1], joint_position_command_[2]);
        //ROS_INFO("  V:[%f ,%f] ", joint_velocity_command_[1], joint_velocity_command_[2]);
        //ROS_INFO("  A:[%f ,%f] ", joint_acceleration_command_[1], joint_acceleration_command_[2]);
        return;
    }

    void inv_dyn_write(ros::Duration elapsed_time, double *Vc_array){
	effort_values.sensor_values.clear();
	//this for loop converts the doubles saved in the pid_commands array to ints so that they can be sent to the Arduino
	//for(int i = 0; i < 6; i++){
	//	effort_values.sensor_values.push_back((int)pid_commands[i]);
	//}
	//z_1 joint which does not have feedforward
	effort_values.sensor_values.push_back((int)pid_commands[0]);
	//shoulder and elbow, which do have feedforward
	effort_values.sensor_values.push_back((int)Vc_array[0]);
	effort_values.sensor_values.push_back((int)Vc_array[1]);
	//effort_values.sensor_values.push_back((int)Vc_feedforward[0]);
	//effort_values.sensor_values.push_back((int)Vc_feedforward[1]);
	for(int i = 3; i < 6; i++){
		effort_values.sensor_values.push_back((int)pid_commands[i]);
	}

	pub.publish(effort_values);
	effortJointSaturationInterface.enforceLimits(elapsed_time);
	positionJointSaturationInterface.enforceLimits(elapsed_time);
}



  void low_pass_filter(){
    double alpha_z = 0.1;
    double alpha = 0.8;
    if(first_time_filter_flag == 1){
        for(int i = 0; i<6; i++){
            joint_position_[i] = (current_position[i]);
            previous_position[i] = (joint_position_[i]);
        }
        first_time_filter_flag = 0;
        pub_z_1_state.publish(joint_position_[0]);
        pub_elbow_state.publish(joint_position_[2]);
        pub_shoulder_state.publish(joint_position_[1]);
        pub_y_4_state.publish(joint_position_[3]);
        pub_p_5_state.publish(joint_position_[4]);
        pub_r_6_state.publish(joint_position_[5]);
    }else{
        for(int i = 0; i<6; i++){
            previous_position[i] = joint_position_[i];
        }
        velocity_filtered.commands.clear();
        joint_position_[0] = alpha_z*current_position[0] + (1-alpha_z)*previous_position[0];
        for(int i = 1; i<6; i++){
            joint_position_[i] = alpha*current_position[i] + (1-alpha)*previous_position[i];
            // DATA FOR TEST OF VELOCITY FILTER
            velocity_filter_test[i] = alpha*joint_velocity_[i] + (1-alpha)*velocity_filter_test_prev[i];
            velocity_filter_test_prev[i] = velocity_filter_test[i];
            velocity_filtered.commands.push_back(velocity_filter_test[i]);
        }
        // This blocks the position from being sent to the PID if the robot is commanded to the
        //  zero position to prevent the arm from going to that position at startup
        for(int i = 0; i<6; i++){
            if(joint_position_command_[i]!= 0){
                zero_position = 0;
            }
        }
        if(zero_position == 0){
            pub_z_1_state.publish(joint_position_[0]);
            pub_elbow_state.publish(joint_position_[2]);
            pub_shoulder_state.publish(joint_position_[1]);
            pub_y_4_state.publish(joint_position_[3]);
            pub_p_5_state.publish(joint_position_[4]);
            pub_r_6_state.publish(joint_position_[5]);
        }
        pub_velocity_filter.publish(velocity_filtered);
    }

  }

  ros::NodeHandle nh_;
  // my_control_loop_ is a timer which calls control loop (update method) at a
  // set frequency (loop_hz_)
  ros::Timer my_control_loop_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
