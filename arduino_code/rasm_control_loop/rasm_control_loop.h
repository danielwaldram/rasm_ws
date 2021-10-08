#ifndef rasm_control_loop_h
#define rasm_control_loop_h

#include "Arduino.h"
#include "pid_loop.h"
#include "DualMC33926MotorShield_Modified.h"

#define setRollSpeed(speed) md.setM1Speed(speed)
#define setBaseSpeed(speed) md.setM2Speed(speed)
#define setElbowSpeed(speed) md2.setM1Speed(speed)
#define setShoulderSpeed(speed) md2.setM2Speed(speed)

class rasm_control_loop{
    private:
        void setYawSpeed(int speed);
        void setPitchSpeed(int speed);
        void inverse_dynamics(void);
        int position_average(int pin);
        //Function to create a new pid loop using the pid library
        PID create_pid(double*, double*, double*, double, double, double, int, int, double);
    public:
        // These hold the goal pose for the RASM, if goal_a is true then a is the goal pose, else b is the goal pose
        volatile bool goal_a = true;
        volatile float goal_pos_a[6] = {0,0,0,0,0,0};    // one of two arrays to hold the goal position
        volatile float goal_pos_b[6] = {0,0,0,0,0,0};    // second of two arrays to hold the goal position

        int encoder_values[6] = {0,0,0,0,0,0};  //holds the encoder readings

        //Motor shield #1 & 2
        DualMC33926MotorShield md;
        DualMC33926MotorShield md2;
        rasm_control_loop() : md(7, 12, A0, 8, 13, A1, 4, 14), md2(41, 44, A2, 42, 45, A3, 40, 43){};
        void pid_computation(void);
        void setup_timer(int Hz);
        double input, output, setpoint, Kp, Ki, Kd;
        double elbow_setpoint, elbow_input, elbow_output;
        double shoulder_setpoint, shoulder_input, shoulder_output;
        double base_setpoint, base_input, base_output;
        double yaw_setpoint, yaw_input, yaw_output;
        double pitch_setpoint, pitch_input, pitch_output;
        double roll_setpoint, roll_input, roll_output;
        double beta = 1;
        double yaw_beta = 0.25;
        double elbow_beta = 0.1;
        double shoulder_beta = 0.2;

        int elbow_encoder_pin = A8;
        int roll_encoder_pin = A5;
        int base_encoder_pin = A9;
        int yaw_encoder_pin = A7;
        int shoulder_encoder_pin = A4;
        int pitch_encoder_pin = A6;

        //YAW and PITCH PINS
        const int D2_y = 2;//yaw pwm
        const int D2_p = 3;
        const int IN2_p =22; //direction pin2 pitch
        const int IN2_y = 6; //direction pin 2 yaw
        const int IN1_y = 11;//
        const int IN1_p = 23;

        PID elbowPID = create_pid(&elbow_input, &elbow_output, &elbow_setpoint,Kp, Ki, Kd, P_ON_E, DIRECT, elbow_beta);
        PID rollPID = create_pid(&roll_input,&roll_output,&roll_setpoint,Kp,Ki,Kd, P_ON_E, DIRECT, beta);
        PID basePID = create_pid(&base_input,&base_output,&base_setpoint,Kp,Ki,Kd, P_ON_E, DIRECT, beta);
        PID shoulderPID = create_pid(&shoulder_input,&shoulder_output,&shoulder_setpoint,Kp,Ki,Kd, P_ON_E, DIRECT, shoulder_beta);
        PID pitchPID = create_pid(&pitch_input,&pitch_output,&pitch_setpoint,Kp,Ki,Kd, P_ON_E, DIRECT, beta);
        PID yawPID = create_pid(&yaw_input,&yaw_output,&yaw_setpoint,Kp,Ki,Kd, P_ON_E, DIRECT, yaw_beta);
        void initiate_motor_drivers(void);
};

#endif
