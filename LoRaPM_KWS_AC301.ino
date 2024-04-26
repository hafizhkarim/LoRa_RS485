//memanggil library yang dipakai
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <LoRa.h>
#include <SPI.h>

//define pin LoRa
#define ss 5
#define rst 15
#define dio0 4

//define ID alat
#define ID_alat "AC301"

//define spesifikasi komunikasi alat
#define baud 9600
#define ID_meter 2

//siapkan variabel penddukung
uint16_t Reg_addr;
float DATA_METER;
String ID_sensor;


ModbusMaster node; //define kode perintah modbus

float Read_Meter_float(char addr, uint16_t REG) {
  float i = 0;
  uint8_t result, j;

  uint16_t data[2];
  double value = 0;
  node.begin(ID_meter, Serial2);

  result = node.readHoldingRegisters(REG, 2);  ///< Modbus function 0x03 Read Holding Registers
  delay(500);
  if (result == node.ku8MBSuccess) {
    for (j = 0; j < 2; j++) {
      data[j] = (node.getResponseBuffer(j));
    }

    // Serial.print(data[1]);
    // Serial.println(data[0]);

    value = data[0];
    if (Reg_addr == 3 || Reg_addr == 14 || Reg_addr == 17 || Reg_addr == 21 || Reg_addr == 27 || Reg_addr == 28 || Reg_addr == 30){
      value = value/10;
    }else{
      if (Reg_addr == 1 || Reg_addr == 19 || Reg_addr == 29){
        value = value/100;
      }else{
        if (Reg_addr == 2 || Reg_addr == 15 || Reg_addr == 23) {
          value=value/1000;
        }
      }
    }
    //Serial.println(value);
    return value;
  } else {
    Serial.print("Connect modbus fail. REG >>> ");
    Serial.println(REG);  // Debug
    delay(1000);
    return 0;
  }
}

void GET_METER(int regist) {  // Update read all data
  delay(100);
  DATA_METER = Read_Meter_float(ID_meter, regist);
}

//**************************************************************************************************************
void setup() {
  Serial.begin(115200);
  Serial2.begin(baud);  // Serial 8E1 menunjukkan parity even 8 data bit
  LoRa.setPins(ss, rst, dio0);
  LoRa.setGain(6);
  LoRa.setTxPower(20);
  pinMode(22, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  while (!LoRa.begin(915E6)) {  //915 adalah frekuensi LoRa dalam MHz
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xF3);  // samakan ke reciever / transmitter
  Serial.println("LoRa Initializing OK!");
}

void loop() {

  //membaca jika ada data masuk dari LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print("LoRa: ");
      Serial.println(LoRaData);
      Reg_addr = LoRaData.toInt();
      tone(22, 1000);  // Send 1KHz sound signal...
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);  // ...for 1 sec
      noTone(22);
      digitalWrite(LED_BUILTIN, LOW);
      if (Reg_addr) {
        GET_METER(Reg_addr);
        Serial.println();
        Serial.print("Data = ");
        Serial.println(DATA_METER, 3);
        if (DATA_METER > 0.001) {
          LoRa.beginPacket();
          LoRa.print(DATA_METER);
          LoRa.endPacket();
        } else {
          Serial.println("No data");
          LoRa.beginPacket();
          LoRa.print("No Data");
          LoRa.endPacket();
        }
      } else {
        Serial.println("Input error");
      }
    }
  }

  if (millis()%1000 == 0){
    Reg_addr = 17; //address power
    GET_METER(Reg_addr);
    String kirimData = String(ID_alat) +";"+ String(DATA_METER);
    LoRa.beginPacket();
    LoRa.print(kirimData);
    LoRa.endPacket();
  }



  if (Serial.available() > 0) { // 14 = voltage, 15 = Current, 17 = Power, 23 = Energy, 26 = Temp , 29 = Power Factor, 30 = Freq
    String bacaSerial = Serial.readString();
    // LoRa.beginPacket();
    // LoRa.print(bacaSerial);
    Serial.println(bacaSerial);
    Reg_addr = bacaSerial.toInt();
    // LoRa.endPacket();
    GET_METER(Reg_addr);
    //Serial.println();
    Serial.print("Data = ");
    Serial.println(DATA_METER, 3);
  }
}
