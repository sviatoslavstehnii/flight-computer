#include <Arduino.h>
#include <FCMS.h>

// FCMS fcms{};

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("Starting...");
//   #ifdef MASTER_MODE
//     Serial.println("Running in MASTER mode");
//   #elif defined(SLAVE_MODE)
//     Serial.println("Running in SLAVE mode");
//   #else
//     Serial.println("No mode defined. Please set build flag!");
//   #endif

//   Wire.begin();
//   delay(200);

//   fcms.setup();
//   fcms.checkHealth();

//   Serial.println("STAAART");
// }


// void loop() {
//   fcms.step();
// }


// Flash flash_{10};

// void setup() {

//     Serial.begin(115200);
//   while (!Serial);
//   Serial.println("Starting...");
//   #ifdef MASTER_MODE
//     Serial.println("Running in MASTER mode");
//   #elif defined(SLAVE_MODE)
//     Serial.println("Running in SLAVE mode");
//   #else
//     Serial.println("No mode defined. Please set build flag!");
//   #endif

//   Wire.begin();
//   delay(200);
//   flash_.setup(16777216, false);



//   size_t bytes_read = 0;
//   while (true) {
//     char read_data[1000] = "";
//     flash_.readDJ(read_data, 1000, bytes_read);
//     if ( strlen(read_data) == 0 ) {
//       break;
//     }
//     Serial.println(read_data);
//     bytes_read += strlen(read_data);
//   }
// }

// void loop() {}


IMU9DOF imu9dof_{};
GPS gps_{};
BMP388 bmp388_{};


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting...");
  #ifdef MASTER_MODE
    Serial.println("Running in MASTER mode");
  #elif defined(SLAVE_MODE)
    Serial.println("Running in SLAVE mode");
  #else
    Serial.println("No mode defined. Please set build flag!");
  #endif

  Wire.begin();
  delay(200);

  imu9dof_.setup();
  bmp388_.setup();

  // fcms.setup();
  // fcms.checkHealth();

  Serial.println("STAAART");
}


void loop() {
    imu9dof_.update();
    float roll = imu9dof_.getRollRate();
    float pitch = imu9dof_.getPitchRate();
    float yaw = imu9dof_.getYawRate();

    float lat = gps_.getLatitude();
    float lon = gps_.getLongitude();

    bmp388_.update();
    float altitude = bmp388_.getAltitude();

    // Serial.print("Roll: ");
    // Serial.print(roll);
    // Serial.print(" Pitch: ");
    // Serial.print(pitch);
    // Serial.print(" Yaw: ");
    // Serial.println(yaw);
    // std::ostringstream data_msg;
    // data_msg << "Roll:" << std::fixed << std::setprecision(2) << roll << ",Pitch: " << std::fixed << std::setprecision(2) << pitch << ",Yaw: " << std::fixed << std::setprecision(2) << yaw;
    // std::string data_msg_str = data_msg.str();
    // char data_buf[data_msg_str.size() + 1];
    // std::copy(data_msg_str.begin(), data_msg_str.end(), data_buf);
    // data_buf[data_msg_str.size()] = '\0';
    // if (!sdmc_.write("test.txt", data_buf)) {
    //   Serial.println("error while writing to file");
    // }

    Serial.print("{\"roll\":");
    Serial.print(roll, 2);
    Serial.print(",\"pitch\":");
    Serial.print(pitch, 2);
    Serial.print(",\"yaw\":");
    Serial.print(yaw, 2);
    Serial.print(",\"lat\":");
    Serial.print(lat, 6);
    Serial.print(",\"lon\":");
    Serial.print(lon, 6);
    Serial.print(",\"alt\":");
    Serial.print(altitude, 2);
    Serial.println("}");
    delay(100);
}
