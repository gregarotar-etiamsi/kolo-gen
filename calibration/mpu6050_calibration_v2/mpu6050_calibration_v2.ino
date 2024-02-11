// Author: Grega Rotar
// Date: 20240205
// FIXME: dodaj da se izbira med opcijami kot je deg/s in g

#include "secrets.h"

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Wi-Fi
const char *ssid = W_SSID;
const char *pass = W_PASS;
WiFiClient wifiClient;

// MQTT Broker
const char *mqtt_broker = MQTT_BROKER;
const int mqtt_port = MQTT_PORT;
const char *topic = "mpu6050";
const char *mqtt_username = MQTT_USER;
const char *mqtt_password = MQTT_PASS;
PubSubClient client(wifiClient);

int mpuAddr = 0x68;
int16_t accelX, accelY, accelZ;  // raw vrednosti
float gForceX, gForceY, gForceZ;

int16_t gyroX, gyroY, gyroZ;  // raw vrednosti
float rotX, rotY, rotZ;

unsigned int steviloCiklov = 1000;  // vecje boljše vendar počasnejše
unsigned int counter = 0;

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  Wire.begin();  // defaultni 21 (SDA) in 22 (SCL) drugace dodaj kot parameter
  setupMpu();
  Serial.println("place device on a level surface with back plate leveled");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("Connected to WiFi");
  // connecting to a MQTT broker START
  client.setServer(mqtt_broker, mqtt_port);

  // loop until client is connected
  while (!client.connected()) {
    String client_id = "mpu6050_calibration_device";

    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}


int64_t sumAccelX, sumAccelY, sumAccelZ;
int64_t sumGyroX, sumGyroY, sumGyroZ;
int16_t offsetAccelX, offsetAccelY, offsetAccelZ;
int16_t offsetGyroX, offsetGyroY, offsetGyroZ;


void loop() {

  digitalWrite(2, LOW);
  delay(10000);
  digitalWrite(2, HIGH);
  Serial.print("Started!");
  while (counter < steviloCiklov) {
    recordAccelRegisters();
    sumAccelX += accelX;
    sumAccelY += accelY;
    sumAccelZ += accelZ;
    recordGyroRegisters();
    sumGyroX += gyroX;
    sumGyroY += gyroY;
    sumGyroZ += gyroZ;
    counter++;
    delay(2);
  }
  offsetAccelX = sumAccelX / counter;
  offsetAccelY = sumAccelY / counter;
  // FIME samo pri +-2G
  offsetAccelZ = sumAccelZ / counter; // ERROR: pazi ker je tako ok in če odšteješ potem ne bo ok ker je to samo takrat kadar je kot offset
  offsetGyroX = sumGyroX / counter;
  offsetGyroY = sumGyroY / counter;
  offsetGyroZ = sumGyroZ / counter;

  sumAccelX = 0;
  sumAccelY = 0;
  sumAccelZ = 0;
  sumGyroX = 0;
  sumGyroY = 0;
  sumGyroZ = 0;
  counter = 0;

  Serial.print("offsetAccelX: ");
  Serial.println(offsetAccelX);
  Serial.print("offsetAccelY: ");
  Serial.println(offsetAccelY);
  Serial.print("offsetAccelZ: ");
  Serial.println(offsetAccelZ);
  Serial.print("offsetGyroX: ");
  Serial.println(offsetGyroX);
  Serial.print("offsetGyroY: ");
  Serial.println(offsetGyroY);
  Serial.print("offsetGyroZ: ");
  Serial.println(offsetGyroZ);
  // String msg = String(offsetAccelX) + " " + String(offsetAccelY) + " " + String(offsetAccelZ); // only calibrating accelometer // + "," + String(offsetGyroX) + "," + String(offsetGyroY) + "," + String(offsetGyroZ);
  String msg = String(offsetGyroX) + "," + String(offsetGyroY) + "," + String(offsetGyroZ);
  client.publish(topic, msg.c_str());

  // izpis raw offset vrednosti
}

void setupMpu() {
  // FIXME: dodaj support za to da se nastavi žiroskop in pospeškomer
  // konfiguracija power-ja
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);        // register za power management
  Wire.write(0b00000000);  // nastavi SLEEP register na 0 (glej 4.28)
  Wire.endTransmission();

  // konfiguracija žiroskopa
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1B);
  // Bit 3 in 4 za konfiguracijo gyroscopa (glej 4.4)
  // 00 (0) +/- 250deg/s
  // 01 (1) +/- 500deg/s
  // 10 (2) +/- 1000deg/s
  // 11 (3) +/- 2000deg/s
  Wire.write(0b00011000);  // nastavljen +/- 2000deg/s
  Wire.endTransmission();

  // konfiguracija pospeškomera
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1C);
  // 00 (0) +/- 2g
  // 01 (1) +/- 4g
  // 10 (2) +/- 8g
  // 11 (3) +/- 16g
  Wire.write(0b00000000);  // nastavljen na +/- 2g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);  // izbran začetni register
  Wire.endTransmission();
  Wire.requestFrom(mpuAddr, 6);  // zahteva accel registre (3B - 40)
  while (Wire.available() < 6)
    ;  // program caka da je na voljo vseh 6 bytov
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();

  processAccelData();
}

void processAccelData() {
  // LSB/g glej poglavje 4.17
  // FIXME: vrednost je določena le ko je accel nastavljen na 2G
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x43);                   //izbran začeten register
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);  //zahteva gyro registere (43 - 48)
  while (Wire.available() < 6)
    ;
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
  processGyroData();
}

void processGyroData() {
  // LSB/deg/s glej poglavje 4.19
  // FIXME: vrednost je določena le ko je 250deg/s
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}