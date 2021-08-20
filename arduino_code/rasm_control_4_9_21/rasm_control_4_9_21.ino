/*
  
*/
#include "ros.h"
//#include <beginner_tutorials/rasm_arduino_commands.h>
#include <beginner_tutorials/Sensor_set_values.h>
#include <rasm_control_loop.h>
#include <pid_loop.h>
//#include <std_msgs/Float64.h>
#include "DualMC33926MotorShield_Modified.h"


int roll_speed = 0;
int base_speed = 0;
unsigned long loop_time = 16;
unsigned long min_loop_time = 16;
unsigned long last_time;


//defining member of class rasm_control_loop; rasm_control_loop library contains the functions necessary to control the robot
rasm_control_loop rasm_loop;

//const long interval = 1000;           // interval at which to blink (milliseconds)
int loop_num = 0;

//setup of ros node
ros::NodeHandle nh;
beginner_tutorials::Sensor_set_values SensorValues; //message that holds the positions to be sent to ros

//Callback function that gets called when a message position command is recieved from ros
void messageCb( const beginner_tutorials::Sensor_set_values& commands){
  //if goal_a is true then a is the goal pose, else b is the goal pose
  // this is necessary so that the control loop ISR getting called in the middle of a goal update
  // doesn't result in a corrupted goal state
  if(rasm_loop.goal_a == true){
    for(int i = 0; i < 6; i++){
      rasm_loop.goal_pos_b[i] = commands.sensor_values[i]/1000.0;
      rasm_loop.goal_a = false;
    }
  }else{
    for(int i = 0; i < 6; i++){
      rasm_loop.goal_pos_a[i] = commands.sensor_values[i]/1000.0;
      rasm_loop.goal_a = true;
    }
  }
  
}


//setting up the publisher and subscriber for the node
ros::Publisher hardware_joint_positions("hardware_joint_positions", &SensorValues);
ros::Subscriber<beginner_tutorials::Sensor_set_values> sub("rasm_position_commands", &messageCb);


void setup() {
  analogReference(EXTERNAL);
  //PIN SETUP
  //Serial.begin(9600);
  
  //Setting up the timer for the control loop interrupt in Hz
  rasm_loop.setup_timer(100);

 
  //initiate the motor drivers
  rasm_loop.initiate_motor_drivers();

  //BASE
  //set the basePID values
  rasm_loop.basePID.SetTunings(5000, 100, 0);
  //set the basePID sample times in milliseconds
  rasm_loop.basePID.SetSampleTime(10);
  //set the output limits
  rasm_loop.basePID.SetOutputLimits(-400,400);
  //picking a setpoint
  rasm_loop.base_setpoint = 0;
  //start the PID
  rasm_loop.basePID.SetMode(AUTOMATIC);
  
  //ROLL
  //set the rollPID values
  rasm_loop.rollPID.SetTunings(900, 200, 0);
  //set therollPID sample time in milliseconds
  rasm_loop.rollPID.SetSampleTime(10);
  rasm_loop.rollPID.SetOutputLimits(-400, 400);
  rasm_loop.roll_setpoint = 0.0;
  rasm_loop.rollPID.SetMode(AUTOMATIC);

  //ELBOW
  //set the elbowPID values
  rasm_loop.elbowPID.SetTunings(1200, 10, 350);//int 50
  //set therollPID sample time in milliseconds
  rasm_loop.elbowPID.SetSampleTime(10);
  rasm_loop.elbowPID.SetOutputLimits(-800, 800);
  rasm_loop.elbow_setpoint = 0.0;
  rasm_loop.elbowPID.SetMode(AUTOMATIC);

  //SHOULDER
  rasm_loop.shoulderPID.SetTunings(700, 0, 120);//int 50
  rasm_loop.shoulderPID.SetSampleTime(10);
  rasm_loop.shoulderPID.SetOutputLimits(-800,800);
  rasm_loop.shoulder_setpoint = 0.0;
  rasm_loop.shoulderPID.SetMode(AUTOMATIC);

  //PITCH
  rasm_loop.pitchPID.SetTunings(2500, 100, 100);
  rasm_loop.pitchPID.SetSampleTime(10);
  rasm_loop.pitchPID.SetOutputLimits(-750,750);
  rasm_loop.pitch_setpoint = 0.0;
  rasm_loop.pitchPID.SetMode(AUTOMATIC);

  //YAW
  rasm_loop.yawPID.SetTunings(1500, 50, 0);
  rasm_loop.yawPID.SetSampleTime(10);
  rasm_loop.yawPID.SetOutputLimits(-750,750);
  rasm_loop.yaw_setpoint = 0.0;
  rasm_loop.yawPID.SetMode(AUTOMATIC);
  
  nh.initNode();
  nh.advertise(hardware_joint_positions);
  nh.subscribe(sub); 
  last_time = millis();
}

ISR(TIMER4_COMPA_vect){//timer 4 interrupt
  //Control Loop
  //Needs to call PID and whatever will run alongside the PID (inv-dyn, feedforward)
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  rasm_loop.pid_computation();
}

void loop() {
  loop_time = millis() - last_time;
  if(loop_time > min_loop_time){
    SensorValues.sensor_values = rasm_loop.encoder_values;
    SensorValues.sensor_values_length = 6;
    hardware_joint_positions.publish(&SensorValues);
   last_time = millis();
   nh.spinOnce();
  }
}
