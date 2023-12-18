#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;

int right_speed = 0;
int left_speed = 0;

void messageCb(const std_msgs::Int32MultiArray& received_msg) {
  right_speed = received_msg.data[0];
  left_speed = received_msg.data[1];
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("to_arduino", messageCb);

// Define motor driver pins //L
// Motor A
const int motorA1 = 8;  // Connect to the IN1 pin on L298
const int motorA2 = 9;  // Connect to the IN2 pin on L298
const int motorAEnable = 6; // Connect to the ENA pin on L298
const byte encoder0PinA1 = 2; //A Pin -> interrupt pin 0
const byte encoder0PinA2 = 4; //B Pin -> digital pin 4
byte encoder0pinALast;
double durationA=0, rpmA;
boolean DirectionA;
double val_outputA;
double SetpointA;
double Kp_A = 0.6, Ki_A = 5, Kd_A = 0;
PID APID(&rpmA, &val_outputA, &SetpointA, Kp_A, Ki_A, Kd_A, DIRECT);


// Motor B    //R
const int motorB1 = 10;   // Connect to the IN3 pin on L298
const int motorB2 = 11;   // Connect to the IN4 pin on L298
const int motorBEnable = 5; // Connect to the ENB pin on L298
const byte encoder0PinB1 = 3; //A Pin -> interrupt pin 1
const byte encoder0PinB2 = 12;  //B Pin -> digital pin 12
byte encoder0pinBLast;
double durationB=0, rpmB;
boolean DirectionB;
double val_outputB;
double SetpointB;
double Kp_B = 0.4, Ki_B = 5, Kd_B = 0;
PID BPID(&rpmB, &val_outputB, &SetpointB, Kp_B, Ki_B, Kd_B, DIRECT);


void motor_setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAEnable, OUTPUT);

  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBEnable, OUTPUT);
}

void EncoderInit(){
  DirectionA = true; //default -> forward
  pinMode(encoder0PinA2 ,INPUT);
  attachInterrupt(0, WheelSpeed_A, CHANGE);

  DirectionB = true; //default -> forward
  pinMode(encoder0PinB2 ,INPUT);
  attachInterrupt(1, WheelSpeed_B, CHANGE);
}

void WheelSpeed_A(){
  int Lstate = digitalRead(encoder0PinA1);
  if((encoder0pinALast == LOW) && Lstate == HIGH){
    int valA = digitalRead(encoder0PinA2);
    if(valA == LOW && DirectionA){
      DirectionA = false;
    }
    else if(valA == HIGH && !DirectionA){
      DirectionA = true;
    }
  }
  encoder0pinALast = Lstate;

  if(!DirectionA)  durationA++;
  else durationA--;
}

//void WheelSpeed_A(){
//  durationA++;
//}

void WheelSpeed_B(){
  int Rstate = digitalRead(encoder0PinB1);
  if((encoder0pinBLast == LOW) && Rstate == HIGH){
    int valB = digitalRead(encoder0PinB2);
    if(valB == LOW && DirectionB){
      DirectionB = false;
      
    }
    else if(valB == HIGH && !DirectionB){
      DirectionB = true;
    }
  }
  encoder0pinBLast = Rstate;

  if(!DirectionB)  durationB++;
  else durationB--;
}

//void WheelSpeed_B(){
//  durationB++;
//}

void controlMotor(int pwm, int enPin, int in1Pin, int in2Pin) {
    if (pwm >= 0) {
        analogWrite(enPin, pwm);
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else if (pwm < 0) {
        analogWrite(enPin, abs(pwm));
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }
}

void motorA(int pwmA) {
    controlMotor(pwmA, motorAEnable, motorA1, motorA2);
}

void motorB(int pwmB) {
    controlMotor(pwmB, motorBEnable, motorB1, motorB2);
}


void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  motor_setup();
//  APID.SetMode(AUTOMATIC);
//  APID.SetSampleTime(100);
//  BPID.SetMode(AUTOMATIC);
//  BPID.SetSampleTime(100);
//  EncoderInit();
//  motorA(left_speed);
//  motorB(right_speed);
}

void loop() {
  nh.spinOnce();
//  SetpointA = abs(left_speed);
//  SetpointB = abs(right_speed);
//  
//  //motorA(left_speed);
//  rpmA = abs(durationA)/2;
//  APID.Compute();
//  durationA = 0;
//  if(left_speed < 0) val_outputA = -1*val_outputA;
//  motorA(val_outputA);
//  
//  
//  //motorB(right_speed);
//  rpmB = abs(durationB)/2;
//  BPID.Compute();
//  durationB = 0;
//  if(right_speed < 0) val_outputB = -1*val_outputB;
//  motorB(val_outputB);
  motorA(left_speed);
  motorB(right_speed);
 
  delay(10);
}

