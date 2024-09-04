#include <Arduino.h>
#include "TERPState.h"
#include <Kalman_Filter.h>

int BUZZER_PIN = 17;

Barometer TERPBarometer("BMP280");
IMU TERPIMU("BNO055");
LightSensor TERPLightSensor("BH1750");
TERPState TERPSTATE;

KFState KFSTATE;

void setup() {
  buzz(BUZZER_PIN, 1000, 1);
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

  if (sdSuccess) {
    buzz(BUZZER_PIN, 3000 ,1);
  }
  else {
    buzz(BUZZER_PIN, 250, 4);
  }
}

void loop() {
  TERPSTATE.settimeAbsolute();
  TERPSTATE.stateBarometer.readBarometer();
  TERPSTATE.stateIMU.readIMU();
  TERPSTATE.stateLightSensor.readLightSensor();

  Serial.print("Lux Value: "); Serial.println(TERPSTATE.stateLightSensor.lux);

  TERPSTATE.setdataString();
  recordData(TERPSTATE.getdataString(), "Test"); // TODO create a "test" that allows straight to sd card

}
