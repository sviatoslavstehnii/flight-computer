 #include "LoRa_E32.h"

#define START_BYTE 0x7E
#define SENDER 0x01
#define RECEIVER 0x00

#define LORA_AUX_PIN 4
LoRa_E32 loraFCMS(&Serial2, LORA_AUX_PIN);

#define PACKET_TYPE 0x03 // send telemetry fs > gs

void receiveStandTelemetry(uint8_t* packet, size_t packetSize) {
  if (packetSize != 58) {
    Serial.println("Invalid packet size");
    return;
  }

  // Header
  uint8_t startByte = packet[0];
  uint8_t packetType = packet[1];
  uint8_t sender = packet[2];
  uint8_t receiver = packet[3];
  uint32_t timestamp;
  memcpy(&timestamp, &packet[4], sizeof(uint32_t));

  // Validate header
  if (startByte != START_BYTE || receiver != SENDER) {
    Serial.println("Invalid header or receiver mismatch");
    return;
  }

  // Body
  uint8_t flightState = packet[8] & 0x0F;
  uint8_t additionalState = (packet[8] >> 4) & 0x0F;

  uint16_t V_BAT, CURR_BAT, loadCell;
  memcpy(&V_BAT, &packet[9], sizeof(uint16_t));
  memcpy(&CURR_BAT, &packet[11], sizeof(uint16_t));
  memcpy(&loadCell, &packet[13], sizeof(uint16_t));

  uint8_t eventsData[4];
  for (int i = 0; i < 4; i++) {
    eventsData[i] = packet[48 + i];
  }

  uint16_t event0Time, event1Time;
  memcpy(&event0Time, &packet[52], sizeof(uint16_t));
  memcpy(&event1Time, &packet[54], sizeof(uint16_t));

  uint8_t pyroFlags = packet[56];
  uint8_t additionalPyroFlags = packet[57];

  // Output parsed data
  Serial.println("=== Telemetry Data ===");
  Serial.print("Start Byte: "); Serial.println(startByte, HEX);
  Serial.print("Packet Type: "); Serial.println(packetType, HEX);
  Serial.print("Sender: "); Serial.println(sender, HEX);
  Serial.print("Receiver: "); Serial.println(receiver, HEX);
  Serial.print("Timestamp: "); Serial.println(timestamp);

  Serial.print("Flight State: "); Serial.println(flightState, HEX);
  Serial.print("Additional State: "); Serial.println(additionalState, HEX);

  Serial.print("V_BAT: "); Serial.println(V_BAT);
  Serial.print("CURR_BAT: "); Serial.println(CURR_BAT);
  Serial.print("Load Cell: "); Serial.println(loadCell);

  Serial.println("Events Data:");
  for (int i = 0; i < 16; i++) {
    uint8_t value = (eventsData[i / 4] >> ((i % 4) * 2)) & 0x03;
    Serial.print("Event "); Serial.print(i); Serial.print(": ");
    Serial.println(value);
  }

  Serial.print("Event 0 Time: "); Serial.println(event0Time);
  Serial.print("Event 1 Time: "); Serial.println(event1Time);

  Serial.println("Pyro Flags:");
  for (int i = 0; i < 8; i++) {
    bool flag = (pyroFlags >> i) & 0x01;
    Serial.print("Flag "); Serial.print(i); Serial.print(": ");
    Serial.println(flag);
  }

  Serial.println("Additional Pyro Flags:");
  for (int i = 0; i < 8; i++) {
    bool flag = (additionalPyroFlags >> i) & 0x01;
    Serial.print("Flag "); Serial.print(i + 8); Serial.print(": ");
    Serial.println(flag);
  }
  Serial.println("======================");
}


void sendStandTelemetry() {
  /*
  Header 
  Byte	Part	Type	Description
  0	    bin	Start Byte
  1		  bin	Packet Type
  2		  bin	Sender
  3		  bin	Receiver
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
    body
    8		      bin	Stand State + Additional State	
    9-10		  uint16	V_BAT	mV			
    11-12		  uint16	CURR_BAT	mA				
    13-14		  uint16	load cell	g
    15-47     ...				
    48	  	  Bin	Events 0-3
    49		    Bin	Events 4-7
    50		    Bin	Events 8-11
    51		    Bin	Events 12-15
    52-53		  uint16	Event 0 Time		
    54-55		  uint16	Event 1 Time
    56	  	  Bin	Pyro Flag 0
              Bin	Pyro Flag 1
              Bin	Pyro Flag 2
              Bin	Pyro Flag 3
              Bin	Pyro Flag 4
              Bin	Pyro Flag 5
              Bin	Pyro Flag 6
              Bin	Pyro Flag 7
    57		    Bin	Pyro Flag 8
              Bin	Pyro Flag 9
              Bin	Pyro Flag 10
              Bin	Pyro Flag 11
              Bin	Additional Flag 0
              Bin	Additional Flag 1
              Bin	Additional Flag 2
              Bin	Additional Flag 3
    */
  uint8_t FLIGHT_STATE = 0x01;      // Політний стан (1 біт або більше)
  uint8_t ADDITIONAL_STATE = 0x02;  // Додатковий стан (1 біт або більше)

  // Записуємо обидва стани в один байт
  packet[8] = (FLIGHT_STATE & 0x0F) | ((ADDITIONAL_STATE & 0x0F) << 4);

  uint16_t V_BAT = 0x1234;  // Напруга батареї
  uint16_t CURR_BAT = 0x5678;  // Струм батареї
  uint16_t load_cell = 0x9ABC;  // Дані load cell

  memcpy(&packet[9], &V_BAT, sizeof(uint16_t));
  memcpy(&packet[11], &CURR_BAT, sizeof(uint16_t));
  memcpy(&packet[13], &load_cell, sizeof(uint16_t));



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
  sendStandTelemetry();
  delay(1000);

  // receive telemetry
  ResponseContainer response = loraFCMS.receiveMessage();
  uint8_t* dataBuffer = (uint8_t*)response.data.c_str();
  size_t dataSize = response.data.length();

  receiveStandTelemetry(dataBuffer, dataSize);
}

