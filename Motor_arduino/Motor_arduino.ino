/* 
 * Phototronic bot!
 */

#include <ros.h>
#include <geometry_msgs/Point.h>

ros::NodeHandle  nh;
const int
M11  = 8,
M12  = 9,
M21  = 10,
M22  = 11,
ENABLER_1 = 5,
ENABLER_2 = 6;
int M11_frwrd = 0,
M12_bcwrd = 0,
M21_frwrd = 0,
M22_bcwrd = 0;


//to control velocity

void velControl(const geometry_msgs::Point& vel){
  //vel in form (vel_value,1/-1,0)
  // 1 means move frwrd, -1
  int velocity;
  float motor1 = 0.92;
  float motor2 = 1.1;
  
  //convert velocities greater than 220 to 220
  if(vel.x>220){
     velocity = 220;
  }else {
     velocity = vel.x;
    }
  
  if(vel.y==1){
    M11_frwrd = HIGH;
    M12_bcwrd = LOW;
    M21_frwrd = HIGH;
    M22_bcwrd = LOW; 
    analogWrite(ENABLER_1,int(velocity*motor1));
    analogWrite(ENABLER_2,int(velocity*motor2));
    digitalWrite(M11,M11_frwrd);
    digitalWrite(M12,M12_bcwrd);
    digitalWrite(M21,M21_frwrd);
    digitalWrite(M22,M22_bcwrd);
  }
  else if(vel.y==-1){
    M11_frwrd = LOW;
    M12_bcwrd = HIGH;
    M21_frwrd = LOW;
    M22_bcwrd = HIGH;
    analogWrite(ENABLER_1,int(velocity*motor1));
    analogWrite(ENABLER_2,int(velocity*motor2));
    digitalWrite(M11,M11_frwrd);
    digitalWrite(M12,M12_bcwrd);
    digitalWrite(M21,M21_frwrd);
    digitalWrite(M22,M22_bcwrd);
  }
}

ros::Subscriber<geometry_msgs::Point> motor("motor_vel", &velControl );

void setup()
{ 
  pinMode(ENABLER_1, OUTPUT);  
  pinMode(ENABLER_2, OUTPUT);
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);
  nh.initNode();
  nh.subscribe(motor);

}

void loop()
{ 
  nh.spinOnce();
  delay(0);

}


