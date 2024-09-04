#include <Arduino.h>
#include "TERPState.h"
#include <Kalman_Filter.h>


int LIGHT_THRESHOLD = 300;  // TODO set at launch // in lx

int TADPOL_POWER_PIN = 41;

Barometer TERPBarometer("BMP280");
IMU TERPIMU("BNO055");
LightSensor TERPLightSensor("BH1750");
TERPState TERPSTATE;

double KF_PREVIOUS_ITER_TIME; //in seconds
double dt = 0.01;
Config LinearKFConfig = initialize_filter(dt);
LinearKalmanFilter kf(LinearKFConfig);

void setup() {
  TERPSTATE.buzzerPin = 17;
  buzz(TERPSTATE.buzzerPin, 1000, 1);
  Serial.begin(115200);

  //add sensors
  TERPSTATE.addBarometer(TERPBarometer);
  TERPSTATE.addIMU(TERPIMU);
  TERPSTATE.addLightSensor(TERPLightSensor);

  //setup steps
  TERPSTATE.stateBarometer.setupBarometer();
  TERPSTATE.stateIMU.setupIMU();
  TERPSTATE.stateLightSensor.setupLightSensor();

  //setup datalogging
  TERPSTATE.setcsvHeader();
  setupPSRAM(TERPSTATE.csvHeader);
  bool sdSuccess = setupSDCard(TERPSTATE.csvHeader);

  if (sdSuccess && TERPSTATE.successfulSetup()) {buzz(TERPSTATE.buzzerPin, 3000, 1);}
  else {buzz(TERPSTATE.buzzerPin, 250, 4);}

  powerOnTADPOL(TADPOL_POWER_PIN);

  KF_PREVIOUS_ITER_TIME = millis()/1000;
}

void loop() {
  TERPSTATE.settimeAbsolute();
  TERPSTATE.stateBarometer.readBarometer();
  TERPSTATE.stateIMU.readIMU();
  TERPSTATE.stateLightSensor.readLightSensor();
  //Run the KF
  dt = TERPSTATE.timeAbsolute - KF_PREVIOUS_ITER_TIME;
  double* meas_arr = new double[3] {0.0, 0.0, TERPSTATE.stateBarometer.relativeAltitude};
  double* control_arr = new double[3] {TERPSTATE.stateIMU.acceleration.x(), TERPSTATE.stateIMU.acceleration.y(), TERPSTATE.stateIMU.acceleration.z() - 9.8};
  LinearKFConfig = iterate_filter(dt, meas_arr, control_arr, 0);
  Matrix X = kf.iterate(LinearKFConfig);

  double* KFOutputState = X.getArr();

  //Set the kf output data in the TERPSTATE
  TERPSTATE.position.x() = KFOutputState[0]; TERPSTATE.position.y() = KFOutputState[1]; TERPSTATE.position.z() = KFOutputState[2];
  TERPSTATE.velocity.x() = KFOutputState[3]; TERPSTATE.velocity.y() = KFOutputState[4]; TERPSTATE.velocity.z() = KFOutputState[5];
  TERPSTATE.acceleration.x() = TERPSTATE.stateIMU.acceleration.x(); TERPSTATE.acceleration.y() = TERPSTATE.stateIMU.acceleration.y(); TERPSTATE.acceleration.z() = TERPSTATE.stateIMU.acceleration.z();
  TERPSTATE.determineTERPStage();

  // if (TERPSTATE.stage == "Drogue" && TERPSTATE.stateBarometer.relativeAltitude < 475 && TERPSTATE.stateLightSensor.lux > LIGHT_THRESHOLD){
  //   releaseTADPOL();
  // }

  TERPSTATE.setdataString();
  recordData(TERPSTATE.getdataString(), TERPSTATE.getrecordDataStage());

  KF_PREVIOUS_ITER_TIME = TERPSTATE.timeAbsolute;
}

void powerOnTADPOL(int power_on_pin) {
  Serial.println("Turning on TADPOL");
  pinMode(power_on_pin, OUTPUT);
  digitalWrite(power_on_pin, HIGH);
  delay(10000);
  digitalWrite(power_on_pin, LOW);
  Serial.println("Done turning on TADPOL");
}

void releaseTADPOL(){
  // TODO write this
  // Add some absolute checks
  // Cut both nichrome
}
