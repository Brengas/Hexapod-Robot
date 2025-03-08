//This is the code which controls the hexapod and moves it in a straight forward direction, nevertheless its and ongoing project,
//so in the future i will add more movements and features
//This code initialize the two drivers to control the 18 servomotors, implement an inverse kinematics to move the end effector of the
//leg to the desired position as well as a function to make the robot move forward

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

//Initialize the two drivers

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver servos2 = Adafruit_PWMServoDriver(0x41);

//Generate all the proper variables to the functions inside the code
int pos0 = 110;
int pos180 = 512;
// these constants correspond to the lenght of each link of the leg
const float a1 = 28;
const float a2 = 55;
const float a3 = 95;
//This arrays contains the position of important points of interest, like zero position, where the leg is completely extended
float restPosition[3] = {90,0,-75};
float zeroPosition[3] = {178,0,0};
float elevateLeg[3] = {90,0,-30};
float forwardLeg[3] = {90,-40,-75};
float backwardsLeg[3] = {90,40,-75};
float forwardFrontLeg[3] = {120,-40,-75};
float backwardsBackLeg[3] = {120,40,-75};
int j = 1;

float theta1, theta2, theta3;

uint8_t servonum = 0;

void setup(){
  //Initialize the servos and serial print for debugging
  Serial.begin(115200);
  Serial.println("poio");
  servos.begin();
  servos.setPWMFreq(50);
  servos2.begin();
  servos2.setPWMFreq(50);
  delay(200);
  //ZeroPosition();
  //all legs to extended position
  RestPosition();
  
}

void loop(){

  //Start the walking cycle
  moveForward();

  }
  

  
//---------------------------------------------------------------------------------------------------------
//This function is used to apply inverse kinematics, it receive a certain coordinate and then transform it into
//Angle values which correspond to each link
void Inverse_Kinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3 ){
  theta1 = atan(y/x);
  float L = sqrt(x*x + y*y);
  float r = sqrt(z*z + (L-a1)*(L-a1));
  float phi = atan(z/(L-a1));
  float beta = acos((r*r + a2*a2 - a3*a3)/ (2*r*a2));
  theta2 = beta + phi;
  theta3 = acos((a3*a3 + a2*a2 - r*r)/(2*a3*a2))-(M_PI);

  theta1 = (theta1 * (180.0 / M_PI));
  theta2 = theta2 * (180.0 / M_PI);
  theta3 = theta3 * (180.0 / M_PI);
}

//------------------------------------------------------------------------------
//Function to establish the right pwm for the servo and match them with degree angles,it receives the number of servo
//you want to move and the desire angle

void setServo(uint8_t n_servo, int angulo){
  int duty;
  duty = map(angulo,0,180,pos0,pos180);
  if (n_servo > 8){
    n_servo = n_servo - 9;
    servos2.setPWM(n_servo,0,duty);
  }else{
    servos.setPWM(n_servo,0,duty);
  }
  
}

//---------------------------------------------------------------------------------------
//This function is used to move an entire leg including all the joints to a desired position
//after calculating the inverse kinematics of that position

void moveLeg(float x, float y,float z, int LegNum){

  Inverse_Kinematics(x, y, z, theta1, theta2, theta3);
  theta1 = theta1 + 90;
  theta2 = 180 - (theta2 + 90);
  theta3 = 180 - (theta3 + 120);
  //Serial.println(theta1);
  //Serial.println(theta2);
  //Serial.println(theta3);
  switch (LegNum){
    case 1:
    setServo(0,theta1);
    setServo(1,theta2);
    setServo(2,theta3);
    break;
    case 2:
    setServo(3,theta1);
    setServo(4,theta2);
    setServo(5,theta3);
    break;
    case 3:
    setServo(6,theta1);
    setServo(7,theta2);
    setServo(8,theta3);
    break;
    case 4:
    setServo(9,theta1);
    setServo(10,theta2);
    setServo(11,theta3);
    break;
    case 5:
    setServo(12,theta1);
    setServo(13,theta2);
    setServo(14,theta3);
    break;
    case 6:
    setServo(15,theta1);
    setServo(16,theta2);
    setServo(17,theta3);
    break;
  } 
}


//------------------------------------------------------------------------------------
//This function creates an interpolation between to coordinate points to make a smooth the leg movement
//It receives starting and end points, the number of points between coordinates and the numer o leg you want to move

void interpolation(float start[3], float end[3], int steps, int legNum){
  for(int i = 0; i <= steps; i++){
    float t = (float)i / steps;
    float xf = start[0] + t*(end[0]-start[0]);
    float yf = start[1] + t*(end[1]-start[1]);
    float zf = start[2] + t*(end[2]-start[2]);
    moveLeg(xf,yf,zf,legNum);
    delay(10);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//This is an interpolation function as well, the main difference is that this one moves 3 legs simultaneously

void SimuInterpolation(float start1[3], float end1[3], float start2[3], float end2[3], float start3[3], float end3[3], int steps, int leg1, int leg2, int leg3){
  for(int i = 0; i <= steps; i++){
    float t = (float)i / steps;
    float xf1 = start1[0] + t*(end1[0]-start1[0]);
    float xf2 = start2[0] + t*(end2[0]-start2[0]);
    float xf3 = start3[0] + t*(end3[0]-start3[0]);
    float yf1 = start1[1] + t*(end1[1]-start1[1]);
    float yf2 = start2[1] + t*(end2[1]-start2[1]);
    float yf3 = start3[1] + t*(end3[1]-start3[1]);
    float zf1 = start1[2] + t*(end1[2]-start1[2]);
    float zf2 = start2[2] + t*(end2[2]-start2[2]);
    float zf3 = start3[2] + t*(end3[2]-start3[2]);
    
    moveLeg(xf1,yf1,zf1,leg1);
    moveLeg(xf2,yf2,zf2,leg2);
    moveLeg(xf3,yf3,zf3,leg3);

    delay(10);
  }
}

//---------------------------------------------------------------------------
//This uses the "SimuInterpolation" function to generate the walking cycle, first it moves 3 legs, 2 from one side and one from the opossite side
//then repeat the same with the other 3 legs

void moveForward(){
  

  SimuInterpolation(backwardsLeg,elevateLeg,forwardLeg,elevateLeg,backwardsBackLeg,elevateLeg,20,1,3,5);
  SimuInterpolation(elevateLeg, forwardFrontLeg,elevateLeg, backwardsLeg,elevateLeg, forwardLeg,20,1,3,5);
  SimuInterpolation(forwardFrontLeg,backwardsLeg,backwardsLeg,forwardLeg,forwardLeg,backwardsBackLeg,20,1,3,5);
  delay(20);
  SimuInterpolation(forwardFrontLeg,elevateLeg,forwardLeg,elevateLeg,backwardsLeg,elevateLeg,20,2,4,6);
  SimuInterpolation(elevateLeg,backwardsLeg,elevateLeg,backwardsLeg,elevateLeg,forwardFrontLeg,20,2,4,6);
  SimuInterpolation(backwardsLeg,forwardFrontLeg,backwardsLeg,forwardLeg,forwardFrontLeg,backwardsLeg,20,2,4,6);
  delay(20);
  


}
//----------------------------------------------------------------------------
//This function put all the legs in the Ready-to-walk position of the robot

void RestPosition(){
  Serial.println("Moving leg to rest position");
      for (int i = 1; i < 7; i++){
        moveLeg(90,0,-75,i);
        delay(100);
      }
  Serial.println("All legs have been moved");
  delay(1000);
}

//------------------------------------------------------------------------------
//This function put all the legs in the total-extended position of the robot

void ZeroPosition(){
  Serial.println("Moving leg to 0 position");
      for(int i = 1; i < 7; i++){
        moveLeg(178,0,0,i);
        delay(200);
      }
      Serial.println("All legs have been moved");
}