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
  void write(ros::Duration elapsed_time);

  void callback_joint_positions(const beginner_tutorials::Sensor_set_values& msg){
    ROS_INFO("I heard: [%i]", msg.sensor_values[0]);
  	joint_position_[0] = msg.sensor_values[0]/1000.0;
    joint_position_[1] = -(msg.sensor_values[1] - 500)*M_PI/512;
    joint_position_[2] = (msg.sensor_values[2] - 492)*M_PI/512;
    joint_position_[3] = (msg.sensor_values[3] - 352)*M_PI/512;
    joint_position_[4] = (msg.sensor_values[4] - 558)*M_PI/512;
    joint_position_[5] = -(msg.sensor_values[5] - 717)*M_PI/512;
  };
//-(msg.sensor_values[1] - 500)*M_PI/512;
  ros::Publisher pub;
  ros::Subscriber sub;
  beginner_tutorials::Sensor_set_values joint_read;
  beginner_tutorials::Sensor_set_values effort_values;
  // I may need these two below but for now I will try to use a subscriber instead
  //ros::ServiceClient client;
  //beginner_tutorials::HardwareJointPositions joint_read;
protected:
  // The joint state interface is for getting feedback on position
  // The joint and limit interfaces for position and effort are declared
  // Velocity could be as well, but I don't think it is needed for the RASM
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  joint_limits_interface::JointLimits limits[6];
  joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
  joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;

  // The joint position velocity and effort arrays are for reading
  // position effort and velocity from the robot. I don't know if I will have
  // anything to read except for position.
  double joint_position_[6];
  double joint_velocity_[6];
  double joint_effort_[6];
  double joint_effort_command_[6];
  double joint_position_command_[6];


  ros::NodeHandle nh_;
  // my_control_loop_ is a timer which calls control loop (update method) at a
  // set frequency (loop_hz_)
  ros::Timer my_control_loop_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
