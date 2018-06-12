#include <Arduino.h>
#include <VarSpeedServo.h>    // Variable speed control for servo motors using PWM
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  arduino;

VarSpeedServo servo9;
VarSpeedServo servo10;
VarSpeedServo servo11;

long feedback4 = 0; 
long feedback5 = 0; 
long feedback6 = 0; 

int spd = 50;
float ratio_short = 0.25;

std_msgs::UInt16 start_val; // Start value used as command for initial start value in degrees
std_msgs::UInt16 max_val;
std_msgs::UInt16 min_val;

//---------------------------------------------------------------------------------------------------------------
void open_shortG( const std_msgs::UInt16& cmd_msg){          // Callback function for PWM control for D9,D10,D11  
  digitalWrite(A0, HIGH);
   servo9.slowmove(min_val.data, spd);
  servo10.slowmove(min_val.data, spd);
  servo11.slowmove(min_val.data, spd);  
  
  delay(3000);                                               // Delay before reading feedback
 
  for (int i=0; i<100; i++){                                 // Calculates the average feedback signal
    feedback4 = feedback4 + analogRead(4);
    feedback5 = feedback5 + analogRead(5);
    feedback6 = feedback6 + analogRead(6);
  }
  feedback4=feedback4/100;
  feedback5=feedback5/100;
  feedback6=feedback6/100;
  
  servo9.slowmove((feedback4*ratio_short), spd);                 // set new position of pwm signal on D9
  servo10.slowmove((feedback5*ratio_short), spd);               // set new position of pwm signal on D10
  servo11.slowmove((feedback6*ratio_short), spd);               // set new position of pwm signal on D11 
}

//---------------------------------------------------------------------------------------------------------------
void close_shortG( const std_msgs::UInt16& cmd_msg){          // Callback function for PWM control for D9,D10,D11  
  digitalWrite(A0, HIGH);
   servo9.slowmove(max_val.data, spd);
  servo10.slowmove(max_val.data, spd);
  servo11.slowmove(max_val.data, spd);  
  
  delay(3000);                                               // Delay before reading feedback
 
  for (int i=0; i<100; i++){                                 // Calculates the average feedback signal
    feedback4 = feedback4 + analogRead(4);
    feedback5 = feedback5 + analogRead(5);
    feedback6 = feedback6 + analogRead(6);
  }
  feedback4=feedback4/100;
  feedback5=feedback5/100;
  feedback6=feedback6/100;
  
  servo9.slowmove((feedback4*ratio_short), spd);                 // set new position of pwm signal on D9
  servo10.slowmove((feedback5*ratio_short), spd);               // set new position of pwm signal on D10
  servo11.slowmove((feedback6*ratio_short), spd);               // set new position of pwm signal on D11 
}

//---------------------------------------------------------------------------------------------------------------
void manual_shortG( const std_msgs::UInt16& cmd_msg){          // Callback function for PWM control for D9,D10,D11  
  digitalWrite(A0, HIGH);
   servo9.slowmove(cmd_msg.data, spd);
  servo10.slowmove(cmd_msg.data, spd);
  servo11.slowmove(cmd_msg.data, spd);  
  
  delay(3000);                                               // Delay before reading feedback
 
  for (int i=0; i<100; i++){                                 // Calculates the average feedback signal
    feedback4 = feedback4 + analogRead(4);
    feedback5 = feedback5 + analogRead(5);
    feedback6 = feedback6 + analogRead(6);
  }
  feedback4=feedback4/100;
  feedback5=feedback5/100;
  feedback6=feedback6/100;
  
  servo9.slowmove((feedback4*ratio_short), spd);                 // set new position of pwm signal on D9
  servo10.slowmove((feedback5*ratio_short), spd);               // set new position of pwm signal on D10
  servo11.slowmove((feedback6*ratio_short), spd);               // set new position of pwm signal on D11 
}

//---------------------------------------------------------------------------------------------------------------
void Goff( const std_msgs::UInt16& cmd_msg){
    digitalWrite(A0, LOW);
}

//---------------------------------------------------------------------------------------------------------------
// ROS subscriber
ros::Subscriber<std_msgs::UInt16> sub1("open_shortG", open_shortG);
ros::Subscriber<std_msgs::UInt16> sub2("close_shortG", close_shortG);
ros::Subscriber<std_msgs::UInt16> sub3("Goff", Goff);
ros::Subscriber<std_msgs::UInt16> sub4("manual_shortG", manual_shortG);

void setup(){   // Initiation of node, subscriber and pins
  pinMode(A0, OUTPUT);   // sets the digital pin 0 as output
  
  arduino.initNode();
  arduino.subscribe(sub1);
  arduino.subscribe(sub2);
  arduino.subscribe(sub3);
  arduino.subscribe(sub4);
  
  servo9.attach(9);     //attach it to pin 9
  servo10.attach(10);   //attach it to pin 10
  servo11.attach(11);   //attach it to pin 11
  
  Serial.begin(57600); // opens serial port, sets data rate to 57600 bps

  min_val.data = 45;
  max_val.data = 145;
  open_shortG(min_val);      // Sets the short grippers start value
  digitalWrite(A0, LOW);
}

//---------------------------------------------------------------------------------------------------------------
void loop(){
  arduino.spinOnce();
  delay(30000); 
  close_shortG(min_val);
  Goff(min_val);
  delay(5000);
  open_shortG(min_val);
  Goff(min_val);
}
