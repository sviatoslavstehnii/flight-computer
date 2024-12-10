#include <Arduino.h>
#include <FCMS.h>

#define BUZZER_PIN 28

BMP388 bmp388;
GPS gps;
IMU9DOF bmp;

void setup() {

  Wire.begin();
  Serial.begin(115200);
  Serial.println("STAAART");

  bmp388.setup();
  Serial.println("BMP388 setup done");
  gps.setup();
  Serial.println("GPS setup done");
  bmp.setup();
  Serial.println("BMP setup done");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("setup done");

}


void loop() {
  bmp388.update();
  bmp.update();

  std::stringstream ss;
  ss << "{";
  ss << std::fixed << std::setprecision(2);
  ss << "\"roll\":" << bmp.getRollRate() << ",\"pitch\":" << bmp.getPitchRate() << ",\"yaw\":" << bmp.getYawRate();
  ss << ",\"alt\":"<< bmp388.getAltitude() << ",\"lat\":" << gps.getLatitude() << ",\"lon\":" << gps.getLongitude();
  ss << "}";
  Serial.println(ss.str().c_str());
  delay(50);
}
