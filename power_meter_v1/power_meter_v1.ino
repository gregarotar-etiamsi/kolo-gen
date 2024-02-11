#include "secrets.h"
#include "mpu6050_const.h"

#include <Wire.h>
#include "HX711.h"
#include <WiFi.h>
#include <PubSubClient.h>

// Wi-Fi
const char *ssid = W_SSID;
const char *pass = W_PASS;
WiFiClient wifiClient;

// MQTT Broker
const char *mqtt_broker = MQTT_BROKER;
const int mqtt_port = MQTT_PORT;
const char *topic = "test/secure";
const char *mqtt_username = MQTT_USER;
const char *mqtt_password = MQTT_PASS;
PubSubClient client(wifiClient);

// MPU-6050
// acceleration
int16_t accelX, accelY, accelZ;
double gForceX, gForceY, gForceZ;
// gyroscope
int16_t gyroX, gyroY, gyroZ;
double rotX, rotY, rotZ;
// I2C details
int mpuAddr = 0x68;
// calibration numbers
// fixme only for 2G raw data
typedef float Matrix3x3[3][3];
typedef float Matrix3x1[3][3];
Matrix3x3 mpuCalibMatrix = {
  { 1.000542, 0.000006, 0.002136 },
  { 0.000006, 0.999202, 0.001130 },
  { 0.002136, 0.001130, 0.980568 }
};

struct MpuData {
  double gForceXAverage;
  double gForceYAverage;
  double gForceZAverage;
  double rotXAverage;
  double rotYAverage;
  double rotZAverage;
};

// HX711
#define SCALE_DOUT_PIN 16  // Replace with your chosen GPIO pin
#define SCALE_SCK_PIN 4    // Replace with your chosen GPIO pin
HX711 scale;
long rawWeight;

void setup() {
  Serial.begin(115200);

  // MPU-6050
  Wire.begin();  // defaultni 21 (SDA) in 22 (SCL) drugace dodaj kot parameter
  setupMpu();

  // hx711
  scale.begin(SCALE_DOUT_PIN, SCALE_SCK_PIN);
  scale.tare();

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
    String client_id = generateRandomClientId();

    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}


String mqttMsg = "";
MpuData mpuData;

void loop() {
  // bere podatke MPU-6050
  recordAccelRegisters();
  recordGyroRegisters();
  // mpuData = mpuAverageForTime(100);

  getWeight();
  // mqttMsg = String(mpuData.gForceXAverage) + "," + String(mpuData.gForceYAverage) + "," + String(mpuData.gForceZAverage) + "," + String(mpuData.rotXAverage) + "," + String(mpuData.rotYAverage) + "," + String(mpuData.rotZAverage) + "," + String(rawWeight);
  mqttMsg = String(gForceX) + "," + String(gForceY) + "," + String(gForceZ) + "," + String(rotX) + "," + String(rotY) + "," + String(rotZ) + "," + String(rawWeight);
  client.publish(topic, mqttMsg.c_str());
  delay(500);
}

// functions for HX711
// hmt -> how many times
// FIXME -> depends on global variable
// WARNING: be careful when using because it will not neccesseraly update
void getWeight() {
  if (scale.is_ready()) {
    rawWeight = scale.get_units(1);
  }
}

// functions for MPU-6050
// mpu6050_const.h contains lots of variables for this
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

  uint8_t gyro_config;
  switch (FS_SEL) {
    case 0:
      gyro_config = 0b00000000;  // nastavljen na +/- 250deg/sec
      break;
    case 1:
      gyro_config = 0b00001000;  // nastavljen na +/- 500deg/sec
      break;
    case 2:
      gyro_config = 0b00010000;  // nastavljen na +/- 1000deg/sec
      break;
    case 3:
      gyro_config = 0b00011000;  // nastavljen na +/- 2000deg/sec
      break;
  }
  Wire.write(gyro_config);
  Wire.endTransmission();

  // konfiguracija pospeškomera
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1C);
  // 00 (0) +/- 2g
  // 01 (1) +/- 4g
  // 10 (2) +/- 8g
  // 11 (3) +/- 16g
  uint8_t accel_config;
  switch (AFS_SEL) {
    case 0:
      accel_config = 0b00000000;  // nastavljen na +/- 250deg/sec
      break;
    case 1:
      accel_config = 0b00001000;  // nastavljen na +/- 500deg/sec
      break;
    case 2:
      accel_config = 0b00010000;  // nastavljen na +/- 1000deg/sec
      break;
    case 3:
      accel_config = 0b00011000;  // nastavljen na +/- 2000deg/sec
      break;
  }
  Wire.write(accel_config);
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

// FIXME: function stores calibrated values inside global gForceX,Y,Z and so you cannot access uncalibrated data (you can but just from raw)
void processAccelData() {
  // LSB/g glej poglavje 4.17
  // FIXME: vrednost je določena le ko je accel nastavljen na 2G
  // kalibracija z magneto 1.2 100 podatkov
  // 16384
  // what this is doing is basicly just dividing bias so that it matches current setting of deg/sec
  // FIXME: not the best method. Calibrate it to current settings
  gForceX = (accelX - ((BIAS_ACCEL_X) / (LSB_G_CALIBRATED / LSB_G)));
  gForceY = (accelY - ((BIAS_ACCEL_Y) / (LSB_G_CALIBRATED / LSB_G)));
  gForceZ = (accelZ - ((BIAS_ACCEL_Z) / (LSB_G_CALIBRATED / LSB_G)));

  gForceX = mpuCalibMatrix[0][0] * gForceX + mpuCalibMatrix[0][1] * gForceY + mpuCalibMatrix[0][2] * gForceZ;
  gForceY = mpuCalibMatrix[1][0] * gForceX + mpuCalibMatrix[1][1] * gForceY + mpuCalibMatrix[1][2] * gForceZ;
  gForceZ = mpuCalibMatrix[2][0] * gForceX + mpuCalibMatrix[2][1] * gForceY + mpuCalibMatrix[2][2] * gForceZ;

  gForceX = gForceX / LSB_G;
  gForceY = gForceY / LSB_G;
  gForceZ = gForceZ / LSB_G;
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
  // what this is doing is basicly just dividing bias so that it matches current setting of deg/sec
  // FIXME: not the best method. Calibrate it to current settings
  // FIXME: i dont know if this matrix is the best for gyro data to
  rotX = (gyroX - ((BIAS_GYRO_X) / (LSB_DEG_CALIBRATED / LSB_DEG)));
  rotY = (gyroY - ((BIAS_GYRO_Y) / (LSB_DEG_CALIBRATED / LSB_DEG)));
  rotZ = (gyroZ - ((BIAS_GYRO_Z) / (LSB_DEG_CALIBRATED / LSB_DEG)));
  Serial.println((BIAS_GYRO_X) / (LSB_DEG_CALIBRATED / LSB_DEG));
  Serial.println((LSB_DEG_CALIBRATED / LSB_DEG));
  Serial.println(gyroX);

  rotX = mpuCalibMatrix[0][0] * rotX + mpuCalibMatrix[0][1] * rotY + mpuCalibMatrix[0][2] * rotZ;
  rotY = mpuCalibMatrix[1][0] * rotX + mpuCalibMatrix[1][1] * rotY + mpuCalibMatrix[1][2] * rotZ;
  rotZ = mpuCalibMatrix[2][0] * rotX + mpuCalibMatrix[2][1] * rotY + mpuCalibMatrix[2][2] * rotZ;

  rotX = rotX / LSB_DEG;
  rotY = rotY / LSB_DEG;
  rotZ = rotZ / LSB_DEG;
}

MpuData mpuAverageForTime(int timeMS) {
  MpuData mpuData;
  unsigned int timeStart = millis();
  unsigned int counter = 0;
  double gForceXTotal = 0, gForceYTotal = 0, gForceZTotal = 0;
  double rotXTotal = 0, rotYTotal = 0, rotZTotal = 0;
  while (millis() - timeStart < timeMS) {
    recordAccelRegisters();
    recordGyroRegisters();
    gForceXTotal += gForceX;
    gForceYTotal += gForceY;
    gForceZTotal += gForceZ;
    rotXTotal += rotX;
    rotYTotal += rotY;
    rotZTotal += rotZ;
    counter++;
    delay(2);
  }
  if (counter != 0) {
    mpuData.gForceXAverage = gForceXTotal / counter;
    mpuData.gForceYAverage = gForceYTotal / counter;
    mpuData.gForceZAverage = gForceZTotal / counter;
    mpuData.rotXAverage = rotXTotal / counter;
    mpuData.rotYAverage = rotYTotal / counter;
    mpuData.rotZAverage = rotZTotal / counter;
  } else {
    mpuData.gForceXAverage = gForceX;
    mpuData.gForceYAverage = gForceY;
    mpuData.gForceZAverage = gForceZ;
    mpuData.rotXAverage = rotX;
    mpuData.rotYAverage = rotY;
    mpuData.rotZAverage = rotZ;
  }
  return mpuData;
}

char generateRandomChar() {
  const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  const int charsetSize = sizeof(charset) - 1;

  // Generate a random index within the character set
  int randomIndex = rand() % charsetSize;

  // Return the random character
  return charset[randomIndex];
}

String generateRandomClientId() {
  String clientId = "ESP32Client-";
  for (int i = 0; i < 8; ++i) {
    clientId += generateRandomChar();
  }
  return clientId;
}
