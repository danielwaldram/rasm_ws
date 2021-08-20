#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher start_capture("start_rec", &str_msg);

char start[5] = "go";
const int buttonPin = 3;
const int vicon_startPin = 7;
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int ledState = LOW; 
const int ledPin =  13;  


void setup()
{
  nh.initNode();
  nh.advertise(start_capture);
  pinMode(buttonPin, INPUT);
  pinMode(vicon_startPin, OUTPUT);
  digitalWrite(vicon_startPin, LOW);
  pinMode(ledPin, OUTPUT);
  str_msg.data = start;
}

void loop()
{
  int reading = digitalRead(buttonPin);
  //determine time the lst change of state
  if(reading != lastButtonState){
    lastDebounceTime = millis();
  }
  if((millis() - lastDebounceTime) > debounceDelay){
    if(reading != buttonState){
      buttonState = reading;
    }
    if(buttonState == HIGH){
      start_capture.publish( &str_msg);
      ledState = !ledState;
      digitalWrite(vicon_startPin, HIGH);
    }else{
      digitalWrite(vicon_startPin, LOW);
    }
  }
  digitalWrite(ledPin, ledState);
  nh.spinOnce();
  lastButtonState = reading;
}
