#include <ros.h>
#include <beginner_tutorials/Sensor_set_values.h>
#include "DualMC33926MotorShield_Modified.h"
#include "Wire.h"
#include <AS5600.h>

#define TCAADDR 0x70
#define setRollSpeed(speed) md.setM1Speed(speed)
#define setBaseSpeed(speed) md.setM2Speed(speed)
#define setElbowSpeed(speed) md2.setM1Speed(speed)
#define setShoulderSpeed(speed) md2.setM2Speed(speed)
void setPitchSpeed(int speed);
void setYawSpeed(int speed);
int position_average(int pin);
void setup_pwm();

//#define setYawSpeed(speed) md3.setM2Speed(speed)

DualMC33926MotorShield md(7, 12, A0, 8, 13, A1, 4, 14);
DualMC33926MotorShield md2(41, 44, A2, 42, 45, A3, 40, 43); //M1DIR, M1PWM, M1FB, M2DIR, M2PWM, M2FB, nD2, nSF
//DualMC33926MotorShield md3(53, 13, A14, 52, 46, A15, 51, 50);

int elbow_speed = 0;
int shoulder_speed = 0;
int pitch_speed = 0;
int yaw_speed = 0;
int roll_speed = 0;
int base_speed = 0;

//pins for the nonmotorshield driver: Yaw and Pitch
const int D2_y = 2;//yaw pwm
const int D2_p = 3;
const int IN2_p =22; //direction pin2 pitch
const int IN2_y = 6; //direction pin 2 yaw
const int IN1_y = 11;//
const int IN1_p = 23;

//int roll_encoder_pin = A5;
//int elbow_encoder_pin = A8;
//int pitch_encoder_pin = A6;
//int yaw_encoder_pin = A7;
unsigned long time_of_last_effort_message = 0;
AMS_5600 ams5600_shoulder;
AMS_5600 ams5600_elbow;
AMS_5600 ams5600_pitch;
AMS_5600 ams5600_yaw;
AMS_5600 ams5600_roll;

ros::NodeHandle nh;
beginner_tutorials::Sensor_set_values SensorValues;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void messageCb( const beginner_tutorials::Sensor_set_values& effort_values){
  time_of_last_effort_message = millis();
  String effort_command_shoulder = String((int) effort_values.sensor_values[3]);
  const char *c = effort_command_shoulder.c_str();
  //nh.loginfo(c);

  base_speed = effort_values.sensor_values[0];
  
  if(base_speed < 100 && base_speed > -100){
    base_speed = 0;
  }else if(base_speed <= -100){
    base_speed = -400; 
  }else if(base_speed >= 100){
    base_speed = 400;
  }
  
  
  elbow_speed = (effort_values.sensor_values[2]);
  if(elbow_speed > 350){
    elbow_speed = 350;
  }
  if(elbow_speed < -350){
    elbow_speed = -350;
  }

  shoulder_speed = (effort_values.sensor_values[1]);
  if(shoulder_speed > 1200){
    shoulder_speed = 1200;
  }
  if(shoulder_speed < -1200){
    shoulder_speed = -1200;
  }
  // Needed to flip the sign on the yaw motor to get the correct direction
  yaw_speed = (effort_values.sensor_values[3]);
  if(yaw_speed > 750){
    yaw_speed = 750;
  }
  if(yaw_speed < -750){
    yaw_speed = -750;
  }

  pitch_speed = (effort_values.sensor_values[4]);
  if(pitch_speed > 750){
    pitch_speed = 750;
  }
  if(pitch_speed < -750){
    pitch_speed = -750;
  }

  roll_speed = (-effort_values.sensor_values[5]);
  if(roll_speed > 400){
    roll_speed = 400;
  }
  if(roll_speed < -400){
    roll_speed = -400;
  }
  String effort_command_shoulder_adjusted = String(yaw_speed);
  const char *d = effort_command_shoulder_adjusted.c_str();
  //nh.loginfo(d);
  setElbowSpeed(elbow_speed);
  setShoulderSpeed(shoulder_speed);
  setYawSpeed(yaw_speed);
  setPitchSpeed(pitch_speed);
  setRollSpeed(roll_speed);
  setBaseSpeed(base_speed);

  
}

ros::Publisher hardware_joint_positions("hardware_joint_positions", &SensorValues);
ros::Subscriber<beginner_tutorials::Sensor_set_values> sub("rasm_effort_commands", &messageCb);
int populate_values[] = {0,0,0,0,0,0};


void setup() {
  delay(1000);

  Wire.begin();
  Wire.setClock(1000);
  // put your setup code here, to run once:
  setup_pwm();
  md.init();
  md2.init();
  pinMode(D2_y, OUTPUT);
  pinMode(IN2_y, OUTPUT);
  pinMode(IN1_y, OUTPUT);
  pinMode(D2_p, OUTPUT);
  pinMode(IN2_p, OUTPUT);
  pinMode(IN1_p, OUTPUT);
    md.init();
  //Serial.begin(57600);
  nh.initNode();
  nh.advertise(hardware_joint_positions);
  nh.subscribe(sub);
  //analogReference(EXTERNAL);
}

void loop() {
  nh.spinOnce();
  
  if(millis() - time_of_last_effort_message > 1000){
    setElbowSpeed(0);
  setShoulderSpeed(0);
  setYawSpeed(0);
  setPitchSpeed(0);
  setRollSpeed(0);
  setBaseSpeed(0);
  }
  populate_values[0] = int(1000*(0.455 - 0.455*(868 - position_average(A9))/723));
  
  //UPDATE
  tcaselect(1);
  populate_values[1] = ams5600_shoulder.getRawAngle();
  tcaselect(4);
  populate_values[2] = ams5600_elbow.getRawAngle();
  tcaselect(0);
  populate_values[3] = ams5600_yaw.getRawAngle();
  tcaselect(2);
  populate_values[4] = ams5600_pitch.getRawAngle();
  tcaselect(3);
  populate_values[5] = ams5600_roll.getRawAngle();
  SensorValues.sensor_values=populate_values;
  SensorValues.sensor_values_length = 6;
  hardware_joint_positions.publish(&SensorValues);
  //delay(10);
}

void setPitchSpeed(int speed){
  if(speed >= 0){
    digitalWrite(IN2_p, HIGH);
    digitalWrite(IN1_p, LOW);
  }else{
    speed = -speed;
    digitalWrite(IN2_p, LOW);
    digitalWrite(IN1_p, HIGH);
  }
  OCR3C = speed;
}

void setYawSpeed(int speed){
  if(speed >= 0){
    digitalWrite(IN2_y, LOW);
    digitalWrite(IN1_y,HIGH);
  }else{
    speed = -speed;
    digitalWrite(IN2_y, HIGH);
    digitalWrite(IN1_y, LOW);
  }
  OCR3B = speed;
}

int position_average(int pin){
    int sum = 0;
    for(int i=0; i < 5; i++){
        sum = sum + analogRead(pin);
    }
    return sum/5;
}

//Function to setup the timer that determines the frequency of the control loop
void setup_pwm(){
    //Serial.println("Timers Initialiing");
    //TIMER SETUP
    cli(); //stops interrupts
    //Timer 1
    //From ATmega640 data sheet and https://www.arduino.cc/en/pmwiki.php?n=Tutorial/SecretsOfArduinoPWM
    //TCCR1A = _BV(COM1B1) | _BV(COM1C1) | _BV(WGM11);//| _BV(WGM50);
    //TCCR1B = _BV(WGM13) | _BV(WGM12)| _BV(CS11);
    //ICR1 = 149;
    //Timer 1
    TCCR1A = 0b00101000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
    //Timer 3: Pitch and Yaw motors
    TCCR3A = _BV(COM3C1) | _BV(COM3B1) | _BV(WGM31);
    TCCR3B = _BV(WGM33) | _BV(WGM32)| _BV(CS30);
    ICR3 = 750;

    //Timer 5
    TCCR5A = _BV(COM5C1) | _BV(COM5B1) | _BV(WGM51);//| _BV(WGM50);
    TCCR5B = _BV(WGM53) | _BV(WGM52)| _BV(CS50);
    ICR5 = 400;

    //TCCR5A = 0b10100000;
    //TCCR5B = 0b00010001;
    //ICR5 = 400;

    sei(); //allow interrupts again
}
