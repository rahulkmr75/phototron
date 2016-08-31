/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Byte.h>
#include <geometry_msgs/Point.h>

ros::NodeHandle  nh;
const int
M11  = 5,
M12  = 6,
M21  = 10,
M22  = 11,
ENABLER_1 = 8,
ENABLER_2 = 9,
LED  = 13;
int M11_frwrd = 0,
M12_bcwrd = 0,
M21_frwrd = 0,
M22_bcwrd = 0;

void messageCb( const std_msgs::Byte& LED_state){
  if(LED_state.data==0){
    digitalWrite(LED, LOW);
  }
  else{
    digitalWrite(LED,HIGH);
  }
}

//to control velocity

void velControl(const geometry_msgs::Point& vel){
  //vel in form (1/-1,vel_value,0)
  // 1 means move frwrd, -1
  int velocity;
  float alpha_bcwrd = 1.099;
  float alpha_frwrd = 1.16;
  //int velocity = normalisedVel(vel.y);
  
  //convert velocities greater than 150 to 150
  if(vel.y>220){
     velocity = 220;
  }else if(vel.y<=220 && vel.y>=90){
     velocity = vel.y;
    }else{
      velocity=vel.y;
      }
  
  if(vel.x==2){
    M11_frwrd = velocity;
    M12_bcwrd = 0;
    M21_frwrd = (int)velocity/alpha_frwrd;
    M22_bcwrd = 0; 
    digitalWrite(ENABLER_1,HIGH);
    digitalWrite(ENABLER_2,HIGH);
    analogWrite(M11,M11_frwrd);
    analogWrite(M12,M12_bcwrd);
    analogWrite(M21,M21_frwrd);
    analogWrite(M22,M22_bcwrd);
  }
  else if(vel.x==1){
    M11_frwrd = 0;
    M12_bcwrd = velocity;
    M21_frwrd = 0;
    M22_bcwrd = (int)velocity/alpha_bcwrd;
    digitalWrite(ENABLER_1,HIGH);
    digitalWrite(ENABLER_2,HIGH);
    analogWrite(M11,M11_frwrd);
    analogWrite(M12,M12_bcwrd);
    analogWrite(M21,M21_frwrd);
    analogWrite(M22,M22_bcwrd);
  }
  else if(vel.x==0){
    M11_frwrd = 0;
    M12_bcwrd = 0;
    M21_frwrd = 0;
    M22_bcwrd = 0; 
    digitalWrite(ENABLER_1,HIGH);
    digitalWrite(ENABLER_2,HIGH);
    analogWrite(M11,M11_frwrd);
    analogWrite(M12,M12_bcwrd);
    analogWrite(M21,M21_frwrd);
    analogWrite(M22,M22_bcwrd);
  }

}

/*int normalisedVel(int vel){
  float alpha = 1; //alpha = w1 / w2
  int M1 = vel;       //slower motor
  int M2 = vel/alpha; //faster motor
  return(M2);
}*/

ros::Subscriber<std_msgs::Byte> led("LED", &messageCb );
ros::Subscriber<geometry_msgs::Point> motor("motor_vel", &velControl );

void setup()
{ 
  pinMode(LED, OUTPUT);
  pinMode(ENABLER_1, OUTPUT);  
  pinMode(ENABLER_2, OUTPUT);
  nh.initNode();
  nh.subscribe(led);
  nh.subscribe(motor);

}

void loop()
{ 
  nh.spinOnce();
  delay(0);

}


