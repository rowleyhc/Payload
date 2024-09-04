#include <PWMServo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static PWMServo leftServo;
static PWMServo rightServo;
float goal;
float delta;
float angle;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void) 
{
  /*
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);
  delay(1000);
  digitalWrite(12,LOW);
  */
  delay(1000);
  Serial.begin(9600);
  
  //while (!Serial) ;
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  while(!bno.begin())
  Serial.println("Complete");

  
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
  // put your setup code here, to run once:
  
  //uint8_t bno_cal_system, bno_cal_gyro, bno_cal_accel, bno_cal_mag = 0;
  //while ((bno_cal_system!=3) || (bno_cal_gyro!=3) || (bno_cal_accel!=3) || (bno_cal_mag!=3)){
  //  bno.getCalibration(&bno_cal_system, &bno_cal_gyro, &bno_cal_accel, &bno_cal_mag);
  //}
  
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  leftServo.attach(3);//10
  rightServo.attach(2);//9
  //Serial.println("setup");
  
  delay(500);
  leftServo.write(0);
  rightServo.write(0);
  delay(1000);  
  leftServo.write(180);
  rightServo.write(180);
  delay(1000);
  leftServo.write(90);
  rightServo.write(90);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);  
  // Display the floating point data
  Serial.print("X: ");
  Serial.println(euler.x());
  angle=euler.x()
  */
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  angle=orientationData.orientation.x;
  Serial.println(angle);

  goal=270;
  delta=(goal-angle)*3.14/180;
  if(angle==0){
    leftServo.write(0);
    rightServo.write(0);
  }
  else{
    leftServo.write(int(90*(1+cos(3.14/2-delta))));
    rightServo.write(int(90*(1+cos(-delta))));
  }
}
