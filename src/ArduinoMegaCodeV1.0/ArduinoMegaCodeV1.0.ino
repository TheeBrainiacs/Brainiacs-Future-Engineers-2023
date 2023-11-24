#include <Wire.h>
#include <MPU6050_tockn.h>
#include<Servo.h>

MPU6050 mpu6050(Wire);
Servo myservo;

long error=0;
int MP = 3;
int MN=2;
int A = 4;

// PID constants
double Kp = 1.33;   // Proportional gain
double Ki = 0;   // Integral gain
double Kd = 2;   // Derivative gain
long lastError = 0;
long de;
long Pr;
int p=0;
int e=0;
int sum =0;
long integral =0 ;
double setpoint = 0.0;  // Target angle
int mangle;

void setup() {
  Serial.begin(9600);
  pinMode(9, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(MP, OUTPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  pinMode(MN, OUTPUT);
  pinMode(A, OUTPUT);
 //digitalWrite(MP , HIGH);
 // digitalWrite(MN, LOW );
 analogWrite(A, 145);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
 myservo.attach(5);
 
}

void loop() {
  int jetson = digitalRead(7);
   int orangePin1 = digitalRead(10); ///orange pin
int bluePin2 = digitalRead(11); ///blue pin
  mpu6050.update();
///////////////////////////////////pid
 if( jetson ==0){
 Serial.println(error);
 error=mpu6050.getAngleZ();
 error = setpoint - error;
 Pr= error;
 de = error - lastError;
 error = lastError;
 integral = integral + error;
 sum = Ki*integral + Kp*Pr + Kd*de;
 if (sum > 30){ 
   sum =30;
 }
  if (sum < -30){
   sum =-30;
 }
  myservo.write(-sum + 80);
 }
 //////////////////////////////////////////pidEnd
 error = -error - setpoint;
if(orangePin1==1 && e==0){
  while(bluePin2==0 || error > -85 ){
    myservo.write(30 + 80);
      mpu6050.update();
    error=mpu6050.getAngleZ();
    Serial.println("error in orange");
    if(bluePin2 ==0){
    bluePin2 = digitalRead(11);
    }
  }
  p=1;
  setpoint =setpoint - 85;
  myservo.write(-30 - 80);
}
/////////////////////////////////orangeend
else if(bluePin2==1 && p==0) {
  Serial.print("error in blue");
while(orangePin1==0 || error < 85){
    myservo.write(-30 + 80);
      mpu6050.update();
    error=mpu6050.getAngleZ();
    Serial.println("HJHH");
    if(orangePin1==0){
    orangePin1 = digitalRead(10);
    }
  }
  e=1;
 setpoint =setpoint + 85;
  myservo.write(30+ 80);


}
if(jetson==1){
  if (Serial.available()) {
    mangle = Serial.parseInt();
    mangle = constrain(mangle, -30, 30);
    myservo.write(mangle + 80);
  }
}
}
