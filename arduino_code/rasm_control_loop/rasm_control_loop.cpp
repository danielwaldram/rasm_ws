#include "Arduino.h"
#include "rasm_control_loop.h"
#include "math.h"
#include <pid_loop.h>
#include "DualMC33926MotorShield_Modified.h"



//rasm_control_loop::rasm_control_loop() {
//}


//Function to setup the timer that determines the frequency of the control loop
void rasm_control_loop::setup_timer(int Hz){
    //Serial.println("Timers Initializing");
    //TIMER SETUP
    cli(); //stops interrupts
    //setting up timer4 for control loop
    TCCR4A = 0; //set entire TCCR4A register to 0
    TCCR4B = 0; //same for TCCR4B
    TCNT4 = 0; //initialize counter value to 0
    //set compare match register for Hz
    OCR4A = (int)(16*pow(10,6)/(256*Hz) - 1);
    //Serial.print("OCR4A: ");
    //Serial.println(OCR4A);
    //turn on CTC mode and set prescaler to 256
    TCCR4B |= (1 << CS42)|(1 << WGM42); //|(1 <<CS40);
    //enable timer compare interrupt
    TIMSK4 |= (1 << OCIE4A);

    //Timer 1
    //From ATmega640 data sheet and https://www.arduino.cc/en/pmwiki.php?n=Tutorial/SecretsOfArduinoPWM
    //TCCR1A = _BV(COM1B1) | _BV(COM1C1) | _BV(WGM11);//| _BV(WGM50);
    //TCCR1B = _BV(WGM13) | _BV(WGM12)| _BV(CS11);
    //ICR1 = 149;
    //Timer 1
    TCCR1A = 0b00101000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
    //Timer 3
    TCCR3A = _BV(COM3C1) | _BV(COM3B1) | _BV(WGM31);
    TCCR3B = _BV(WGM33) | _BV(WGM32)| _BV(CS30);
    ICR3 = 750;

    //Timer 5
    TCCR5A = _BV(COM5C1) | _BV(COM5B1) | _BV(WGM51);//| _BV(WGM50);
    TCCR5B = _BV(WGM53) | _BV(WGM52)| _BV(CS50);
    ICR5 = 1200;

    //TCCR5A = 0b10100000;
    //TCCR5B = 0b00010001;
    //ICR5 = 400;

    sei(); //allow interrupts again
}

void rasm_control_loop::pid_computation(void){
//Analog every joints position and save to a variable that is shared in the class
    encoder_values[0] = position_average(base_encoder_pin);
    encoder_values[1] = position_average(shoulder_encoder_pin);
    encoder_values[2] = position_average(elbow_encoder_pin);
    encoder_values[3] = analogRead(yaw_encoder_pin);
    encoder_values[4] = position_average(pitch_encoder_pin);
    encoder_values[5] = analogRead(roll_encoder_pin);

    //Convert the encoder values to degrees for rotary joints
    base_input = encoder_values[0]/1000.0;
    shoulder_input = ((encoder_values[1] - 5) - 490)*M_PI/501.5;
    elbow_input = ((encoder_values[2] -5) - 520)*M_PI/501.5;
    yaw_input = ((encoder_values[3] - 5)- 510)*M_PI/501.5;
    pitch_input = ((encoder_values[4] - 5) - 820)*M_PI/501.5;
    roll_input = ((encoder_values[5] - 5) - 560)*M_PI/501.5;

    if (goal_a == true){
        base_setpoint = goal_pos_a[0];
        shoulder_setpoint = goal_pos_a[1];
        elbow_setpoint = goal_pos_a[2];
        yaw_setpoint = goal_pos_a[3];
        pitch_setpoint = goal_pos_a[4];
        roll_setpoint = goal_pos_a[5];
    }else{
        base_setpoint = goal_pos_b[0];
        shoulder_setpoint = goal_pos_b[1];
        elbow_setpoint = goal_pos_b[2];
        yaw_setpoint = goal_pos_b[3];
        pitch_setpoint = goal_pos_b[4];
        roll_setpoint = goal_pos_b[5];
    }

    basePID.Compute();
    shoulderPID.Compute();
    elbowPID.Compute();
    rollPID.Compute();
    yawPID.Compute();
    pitchPID.Compute();

    if(abs(base_output) < 20){
        base_output = 0;
    }else if(base_output< 0){
        base_output = -400;
    }else{
        base_output = 400;
    }
    setBaseSpeed(base_output);
    setShoulderSpeed(shoulder_output);
    setElbowSpeed(elbow_output);
    setRollSpeed((int)roll_output);
    setYawSpeed(yaw_output);
    setPitchSpeed(pitch_output);
}

void rasm_control_loop::inverse_dynamics(void){
//take the joint position which has already been calculated by the PID
//take the PID command and add it to the joint acceleration command from ROS
//Calc inertia matrix
//Calc friction matrix
//Calc centripetal matrix
//Calc torque needed at each joint
//determine final control voltage needed
//send this voltage command to the PWMs by using setspeed and analog write functions
}


PID rasm_control_loop::create_pid(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd,int POn, int ControllerDirection, double beta){
   PID new_PID(Input, Output, Setpoint, Kp, Ki, Kd, POn, ControllerDirection, beta);
    return new_PID;
}

void rasm_control_loop::initiate_motor_drivers(void){
    //yaw and pitch pins
    pinMode(D2_y, OUTPUT);
    pinMode(IN2_y, OUTPUT);
    pinMode(IN1_y, OUTPUT);
    pinMode(D2_p, OUTPUT);
    pinMode(IN2_p, OUTPUT);
    pinMode(IN1_p, OUTPUT);
    md.init();
    md2.init();
}

void rasm_control_loop::setPitchSpeed(int speed){
  if(speed >= 0){
    digitalWrite(IN2_p, LOW);
    digitalWrite(IN1_p, HIGH);
  }else{
    speed = -speed;
    digitalWrite(IN2_p, HIGH);
    digitalWrite(IN1_p, LOW);
  }
  OCR3C = speed;
}

void rasm_control_loop::setYawSpeed(int speed){
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

int rasm_control_loop::position_average(int pin){
    int sum = 0;
    for(int i=0; i < 3; i++){
        sum = sum + analogRead(pin);
    }
    return sum/3;
}
