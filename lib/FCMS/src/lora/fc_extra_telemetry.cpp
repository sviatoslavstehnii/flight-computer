#include "LoRa_E32.h"

#define START_BYTE 0x7E
#define SENDER 0x01
#define RECEIVER 0x00

#define LORA_AUX_PIN 4
LoRa_E32 loraFCMS(&Serial2, LORA_AUX_PIN);

#define PACKET_TYPE 0x02 // send extra telemetry fc > gs


void receiveExtraTelemetry(const uint8_t *packet, size_t packetSize) {
    // Перевірка розміру пакету
    if (packetSize < 33) {
        Serial.println("Received packet is too small.");
        return;
    }

    // Перевірка стартового байта
    if (packet[0] != START_BYTE) {
        Serial.println("Invalid start byte.");
        return;
    }

    // Отримання заголовку
    uint8_t packetType = packet[1];
    uint8_t sender = packet[2];
    uint8_t receiver = packet[3];

    uint32_t timestamp;
    memcpy(&timestamp, &packet[4], sizeof(uint32_t));

    Serial.print("Packet Type: "); Serial.println(packetType, HEX);
    Serial.print("Sender: "); Serial.println(sender, HEX);
    Serial.print("Receiver: "); Serial.println(receiver, HEX);
    Serial.print("Timestamp: "); Serial.println(timestamp);

    // Отримання тіла пакету
    uint16_t V_BAT;
    uint16_t CURR_BAT;
    memcpy(&V_BAT, &packet[8], sizeof(uint16_t));
    memcpy(&CURR_BAT, &packet[10], sizeof(uint16_t));

    int8_t Temp = packet[14];
    uint8_t MEL_Data = packet[15];
    uint8_t CL_Data = packet[16];
    uint8_t Datalog_Data = packet[17];

    int32_t GPS_Latitude;
    int32_t GPS_Longitude;
    int16_t GPS_Altitude;

    memcpy(&GPS_Latitude, &packet[18], sizeof(int32_t));
    memcpy(&GPS_Longitude, &packet[22], sizeof(int32_t));
    memcpy(&GPS_Altitude, &packet[26], sizeof(int16_t));

    uint8_t takeoffDetection = packet[28];
    uint8_t apogeeDetection = packet[29];
    uint8_t landingDetection = packet[30];

    uint16_t Apogee;
    memcpy(&Apogee, &packet[31], sizeof(uint16_t));

    // Виведення отриманих даних
    Serial.print("V_BAT: "); Serial.println(V_BAT, HEX);
    Serial.print("CURR_BAT: "); Serial.println(CURR_BAT, HEX);
    Serial.print("Temp: "); Serial.println(Temp);
    Serial.print("MEL Data: "); Serial.println(MEL_Data);
    Serial.print("CL Data: "); Serial.println(CL_Data);
    Serial.print("Datalog Data: "); Serial.println(Datalog_Data);
    Serial.print("GPS Latitude: "); Serial.println(GPS_Latitude, HEX);
    Serial.print("GPS Longitude: "); Serial.println(GPS_Longitude, HEX);
    Serial.print("GPS Altitude: "); Serial.println(GPS_Altitude, HEX);
    Serial.print("Takeoff Detection: "); Serial.println(takeoffDetection);
    Serial.print("Apogee Detection: "); Serial.println(apogeeDetection);
    Serial.print("Landing Detection: "); Serial.println(landingDetection);
    Serial.print("Apogee: "); Serial.println(Apogee, HEX);
}


void sendExtraTelemetry() {
    /*
    Header 
    Byte	Part	Type	Description
    0	    bin	Start Byte
    1		  bin	Packet Type
    2		  bin	Sender
    3		  bin	Receiver
    4-7		uint32	Timestamp	
    */
    uint8_t packet[33] = {START_BYTE, PACKET_TYPE, SENDER, RECEIVER};
    
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
    8-9		    uint16	V_BAT
    10-11		uint16	CURR_BAT		
    12-13       ...					
    14	    	int8	Temp
    15		    uint8	MEL Data %
    16		    uint8	CL Data %
    17		    uint8	Datalog Data %
    18-21		int32	GPS Latitude	
    22-25		int32	GPS Longitude	
    26-27		int16	GPS Altitude
    28		    bin	    take off detection
    29		    bin	    apogee off detection
    30		    bin	    landing off detection
    31-32		uint16	Apogee
    */
    
    uint16_t V_BAT = 0x1234;  // Напруга батареї
    uint16_t CURR_BAT = 0x5678;  // Струм батареї

    memcpy(&packet[8], &V_BAT, sizeof(uint16_t));
    memcpy(&packet[10], &CURR_BAT, sizeof(uint16_t));

    int8_t Temp = 0x12;  // Температура
    uint8_t MEL_Data = 0x34;  // Дані MEL
    uint8_t CL_Data = 0x56;  // Дані CL
    uint8_t Datalog_Data = 0x78;  // Дані Datalog

    packet[14] = Temp;
    packet[15] = MEL_Data;
    packet[16] = CL_Data;
    packet[17] = Datalog_Data;

    int32_t GPS_Latitude = 0x12345678;  // Широта GPS
    int32_t GPS_Longitude = 0x9ABCDEF0;  // Довгота GPS
    int16_t GPS_Altitude = 0x1234;  // Висота GPS

    memcpy(&packet[18], &GPS_Latitude, sizeof(int32_t));
    memcpy(&packet[22], &GPS_Longitude, sizeof(int32_t));
    memcpy(&packet[26], &GPS_Altitude, sizeof(int16_t));

    uint8_t detections[3] = {0x01, 0x02, 0x03};  // Виявлення злітання
    for (int i = 0; i < 3; i++) {
        packet[28 + i] = detections[i];
    }

    uint16_t Apogee = 0x1234;  // max height
    memcpy(&packet[31], &Apogee, sizeof(uint16_t));
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  loraFCMS.begin();
  Serial.println("LoRa module initialized...");
}

void loop() {
  sendExtraTelemetry();
  delay(1000);

  // Receive telemetry
  ResponseContainer response = loraFCMS.receiveMessage();
  uint8_t* dataBuffer = (uint8_t*)response.data.c_str();
  size_t dataSize = response.data.length();

  receiveExtraTelemetry(dataBuffer, dataSize);
}