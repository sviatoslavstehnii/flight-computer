#include <Arduino.h>
#include <FCMS.h>

#define BUZZER_PIN 28

FCMS fcms{};


void setup() {
  // Serial.begin(115200);
  // while (!Serial);
  // Serial.println("Starting...");
  // #ifdef MASTER_MODE
  //   Serial.println("Running in MASTER mode");
  // #elif defined(SLAVE_MODE)
  //   Serial.println("Running in SLAVE mode");
  // #else
  //   Serial.println("No mode defined. Please set build flag!");
  // #endif

  Wire.begin();
  delay(200);

  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  delay(140);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  fcms.setup();

  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  delay(140);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  fcms.checkHealth();

  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  delay(140);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);

  // Serial.println("STAAART");
}


void loop() {
  fcms.step();
}



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
