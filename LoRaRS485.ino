#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module (ESP32)
#define ss 5
#define rst 15
#define dio0 4


void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  Serial2.begin(19200);
  while (!Serial)
    ;
  Serial.println("LoRa Sender");
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {


  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print("LoRa: ");
      Serial.println(LoRaData);
    }
  }
  if (Serial2.available() > 0) {
    Serial.print("RS485: ");
    String bacaSerial2 = Serial2.readString();
    Serial.println(bacaSerial2);
    LoRa.beginPacket();
    LoRa.print(bacaSerial2);
    LoRa.endPacket();
  }



  if (Serial.available() > 0) {
    String bacaSerial = Serial.readString();
    LoRa.beginPacket();
    Serial2.print(bacaSerial);
    LoRa.print(bacaSerial);
    Serial.println(bacaSerial);

    LoRa.endPacket();
  }
}