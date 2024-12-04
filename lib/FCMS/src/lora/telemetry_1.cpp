#include "LoRa_E32.h"

#define START_BYTE 0x7E
#define SENDER 0x01
#define RECEIVER 0x00

#define LORA_AUX_PIN 4
LoRa_E32 loraFCMS(&Serial2, LORA_AUX_PIN);

#define PACKET_TYPE 0x01 // send 1 packet of telemetry fc > gs

struct ImuData {
  uint16_t yaw;         // Курс
  uint16_t pitch;       // Тангаж
  uint16_t roll;        // Крен
  int16_t accel_x;      // Прискорення по X
  int16_t accel_y;      // Прискорення по Y
  int16_t accel_z;      // Прискорення по Z
  int16_t velocity_x;   // Швидкість по X
  int16_t velocity_y;   // Швидкість по Y
  int16_t velocity_z;   // Швидкість по Z
  int16_t position_x;   // Позиція по X
  int16_t position_y;   // Позиція по Y
  int16_t position_z;   // Позиція по Z
};

void receiveTelemetry(uint8_t *packet, size_t packetSize) {
  if (packetSize < 58) { // Мінімальна довжина пакету
    Serial.println("Invalid packet size!");
    return;
  }

  // Розбір заголовку
  uint8_t startByte = packet[0];
  uint8_t packetType = packet[1];
  uint8_t sender = packet[2];
  uint8_t receiver = packet[3];
  uint32_t timestamp;
  memcpy(&timestamp, &packet[4], sizeof(uint32_t));

  Serial.println("Header:");
  Serial.print("Start Byte: "); Serial.println(startByte, HEX);
  Serial.print("Packet Type: "); Serial.println(packetType, HEX);
  Serial.print("Sender: "); Serial.println(sender);
  Serial.print("Receiver: "); Serial.println(receiver);
  Serial.print("Timestamp: "); Serial.println(timestamp);

  // Розбір тіла
  Serial.println("\nBody:");

  uint8_t flightState = packet[8] & 0x0F;
  uint8_t additionalState = (packet[8] >> 4) & 0x0F;
  Serial.print("Flight State: "); Serial.println(flightState);
  Serial.print("Additional State: "); Serial.println(additionalState);

  int16_t barometricAltitude;
  memcpy(&barometricAltitude, &packet[9], sizeof(int16_t));
  Serial.print("Barometric Altitude: "); Serial.println(barometricAltitude);

  uint16_t yaw, pitch, roll;
  memcpy(&yaw, &packet[16], sizeof(uint16_t));
  memcpy(&pitch, &packet[18], sizeof(uint16_t));
  memcpy(&roll, &packet[20], sizeof(uint16_t));
  Serial.print("Yaw: "); Serial.println(yaw);
  Serial.print("Pitch: "); Serial.println(pitch);
  Serial.print("Roll: "); Serial.println(roll);

  int16_t accelX, accelY, accelZ;
  memcpy(&accelX, &packet[22], sizeof(int16_t));
  memcpy(&accelY, &packet[24], sizeof(int16_t));
  memcpy(&accelZ, &packet[26], sizeof(int16_t));
  Serial.print("Accel X: "); Serial.println(accelX);
  Serial.print("Accel Y: "); Serial.println(accelY);
  Serial.print("Accel Z: "); Serial.println(accelZ);

  int16_t velocityX, velocityY, velocityZ;
  memcpy(&velocityX, &packet[28], sizeof(int16_t));
  memcpy(&velocityY, &packet[30], sizeof(int16_t));
  memcpy(&velocityZ, &packet[32], sizeof(int16_t));
  Serial.print("Velocity X: "); Serial.println(velocityX);
  Serial.print("Velocity Y: "); Serial.println(velocityY);
  Serial.print("Velocity Z: "); Serial.println(velocityZ);

  int16_t positionX, positionY, positionZ;
  memcpy(&positionX, &packet[34], sizeof(int16_t));
  memcpy(&positionY, &packet[36], sizeof(int16_t));
  memcpy(&positionZ, &packet[38], sizeof(int16_t));
  Serial.print("Position X: "); Serial.println(positionX);
  Serial.print("Position Y: "); Serial.println(positionY);
  Serial.print("Position Z: "); Serial.println(positionZ);

  uint8_t fin[4] = {packet[40], packet[41], packet[42], packet[43]};
  for (int i = 0; i < 4; i++) {
    Serial.print("Fin "); Serial.print(i); Serial.print(": ");
    Serial.println(fin[i]);
  }

  uint16_t event0Time, event1Time;
  memcpy(&event0Time, &packet[52], sizeof(uint16_t));
  memcpy(&event1Time, &packet[54], sizeof(uint16_t));
  Serial.print("Event 0 Time: "); Serial.println(event0Time);
  Serial.print("Event 1 Time: "); Serial.println(event1Time);

  uint8_t pyroFlags = packet[56];
  uint8_t pyroFlagsAd = packet[57];
  Serial.print("Pyro Flags: "); Serial.println(pyroFlags, BIN);
  Serial.print("Additional Flags: "); Serial.println(pyroFlagsAd, BIN);
}


void sendTelemetry() {
  /*
  Header 
  Byte	Part	  Type
  0	    bin	    Start Byte
  1		  bin	    Packet Type
  2		  bin	    Sender
  3		  bin   	Receiver
  4-7		uint32	Timestamp	
  */
  uint8_t packet[58] = {START_BYTE, PACKET_TYPE, SENDER, RECEIVER};

  // timestamp
  uint32_t timestamp = millis();
  memcpy(&packet[4], &timestamp, sizeof(uint32_t));

  // check if size of packet is less than 58 bytes
  if (sizeof(packet) > MAX_SIZE_TX_PACKET) {
    Serial.println("Packet size is too large...");
    return;
  }

  /*
  Body
  8		    bin	Flight State + Additional State
  9-10		int16	Barometric Altitude	
  11-15   ...				
  16-17		uint16	Yaw
  18-19		uint16	Pitch
  20-21		uint16	Roll		
  22-23		int16	Accel X			
  24-25		int16	Accel Y		
  26-27		int16	Accel Z
  28-29		int16	Velocity X
  30-31		int16	Velocity Y
  32-33		int16	Velocity Z
  34-35		int16	Position X
  36-37		int16	Position Y
  38-39		int16	Position Z
  40		  uint8_t	Fin 0
  41	  	uint8_t	Fin 1
  42		  uint8_t	Fin 2
  43		  uint8_t	Fin 3
  44-47   ...			
  48	  	Bin	Events 0-3
  49		  Bin	Events 4-7
  50		  Bin	Events 8-11
  51		  Bin	Events 12-15
  52-53		uint16	Event 0 Time		
  54-55		uint16	Event 1 Time
  56	  	Bin	Pyro Flag 0
          Bin	Pyro Flag 1
          Bin	Pyro Flag 2
          Bin	Pyro Flag 3
          Bin	Pyro Flag 4
          Bin	Pyro Flag 5
          Bin	Pyro Flag 6
          Bin	Pyro Flag 7
  57		  Bin	Pyro Flag 8
          Bin	Pyro Flag 9
          Bin	Pyro Flag 10
          Bin	Pyro Flag 11
          Bin	Additional Flag 0
          Bin	Additional Flag 1
          Bin	Additional Flag 2
          Bin	Additional Flag 3
  */

  // body  
  uint8_t FLIGHT_STATE = 0x01;      // Політний стан (1 біт або більше)
  uint8_t ADDITIONAL_STATE = 0x02;  // Додатковий стан (1 біт або більше)

  // Записуємо обидва стани в один байт
  packet[8] = (FLIGHT_STATE & 0x0F) | ((ADDITIONAL_STATE & 0x0F) << 4);

  uint16_t BAROMETRIC_ALTITUDE = 0x1234;  // Барометрична висота
  memcpy(&packet[9], &BAROMETRIC_ALTITUDE, sizeof(uint16_t));

  // 4 bytes reserved

  uint16_t yaw = 0x5678;    // Курс
  uint16_t pitch = 0x9ABC;  // Тангаж
  uint16_t roll = 0xDEF0;   // Крен

  int16_t accel_x = 0x1234;  // Прискорення по X
  int16_t accel_y = 0x5678;  // Прискорення по Y
  int16_t accel_z = 0x9ABC;  // Прискорення по Z

  int16_t velocity_x = 0x1234;  // Швидкість по X
  int16_t velocity_y = 0x5678;  // Швидкість по Y
  int16_t velocity_z = 0x9ABC;  // Швидкість по Z

  int16_t position_x = 0x1234;  // Позиція по X
  int16_t position_y = 0x5678;  // Позиція по Y
  int16_t position_z = 0x9ABC;  // Позиція по Z

  ImuData imuData = {yaw, pitch, roll, accel_x, accel_y, accel_z, velocity_x, velocity_y, velocity_z, position_x, position_y, position_z};
  memcpy(&packet[16], &imuData, sizeof(ImuData));

  // fin for 1 byte
  uint8_t fin0 = 0x01;  // Фін 0
  uint8_t fin1 = 0x02;  // Фін 1
  uint8_t fin2 = 0x03;  // Фін 2
  uint8_t fin3 = 0x04;  // Фін 3

  packet[40] = fin0;
  packet[41] = fin1;
  packet[42] = fin2;
  packet[43] = fin3;

  // 4 bytes reserved

  // Масив із 16 значень по 2 біти
  uint8_t events_data[16] = {0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x00,
                      0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x00};


  // Записуємо по 4 значення в кожен байт
  for (int i = 0; i < 4; i++) {
    packet[48 + i] = (events_data[i * 4] & 0x03) |
      ((events_data[i * 4 + 1] & 0x03) << 2) |
      ((events_data[i * 4 + 2] & 0x03) << 4) |
      ((events_data[i * 4 + 3] & 0x03) << 6);
  }

  uint16_t event0_time = 0x1234;  // Час події 0
  uint16_t event1_time = 0x5678;  // Час події 1

  memcpy(&packet[52], &event0_time, sizeof(uint16_t));
  memcpy(&packet[54], &event1_time, sizeof(uint16_t));


  uint8_t pyroFlags = 0;  // Один байт для зберігання всіх Pyro Flags
  bool pyroFlagsArray[8] = {true, false, true, false, true, false, true, false};

  for (int i = 0; i < 8; i++) {
      pyroFlags |= (pyroFlagsArray[i] << i);
  }
  packet[56] = pyroFlags;


  uint8_t pyroFlags_ad = 0;
  bool pyroFlags_adArray[8] = {true, false, true, false, true, false, true, false};

  for (int i = 0; i < 8; i++) {
      pyroFlags_ad |= (pyroFlags_adArray[i] << i);
  }
  packet[57] = pyroFlags_ad;


  // send packet
  ResponseStatus response = loraFCMS.sendMessage(packet, sizeof(packet));
  if (response.code != 1) {
    Serial.println(response.getResponseDescription());
  } else {
    Serial.println("Packet sent...");
  }
}
  

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  loraFCMS.begin();
  Serial.println("LoRa module initialized...");
}

void loop() {
  sendTelemetry();
  delay(1000);

  // receive telemetry
  ResponseContainer response = loraFCMS.receiveMessage();
  uint8_t* dataBuffer = (uint8_t*)response.data.c_str();
  size_t dataSize = response.data.length();

  receiveTelemetry(dataBuffer, dataSize);
}
