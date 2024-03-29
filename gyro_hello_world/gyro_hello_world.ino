#include <Wire.h>

int16_t accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

int16_t gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

int mpuAddr = 0x68;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // defaultni 21 (SDA) in 22 (SCL) drugace dodaj kot parameter
  Serial.print("Hello");
  setupMpu();
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();

  printData();
  // Serial.println(accelX);

  delay(100);
}

void setupMpu() {
  // FIXME: dodaj support za to da se nastavi žiroskop in pospeškomer
  // konfiguracija power-ja
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B); // register za power management
  Wire.write(0b00000000); // nastavi SLEEP register na 0 (glej 4.28)
  Wire.endTransmission();
  
  // konfiguracija žiroskopa
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1B);
  // Bit 3 in 4 za konfiguracijo gyroscopa (glej 4.4)
  // 00 (0) +/- 250deg/s
  // 01 (1) +/- 500deg/s
  // 10 (2) +/- 1000deg/s
  // 11 (3) +/- 2000deg/s
  Wire.write(0b00000000); // nastavljen +/- 250deg/s
  Wire.endTransmission();

  // konfiguracija pospeškomera
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1C);
  // 00 (0) +/- 2g
  // 01 (1) +/- 4g
  // 10 (2) +/- 8g
  // 11 (3) +/- 16g
  Wire.write(0b00000000); // nastavljen na +/- 2g
  Wire.endTransmission();
}


void recordAccelRegisters() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B); // izbran začetni register
  Wire.endTransmission();
  Wire.requestFrom(mpuAddr, 6); // zahteva accel registre (3B - 40)
  while (Wire.available() < 6); // program caka da je na voljo vseh 6 bytov
  accelX = (Wire.read()<<8)|Wire.read();
  accelY = (Wire.read()<<8)|Wire.read();
  accelZ = (Wire.read()<<8)|Wire.read();

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
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //izbran začeten register
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //zahteva gyro registere (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); 
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();
  processGyroData();
}

void processGyroData() {
  // LSB/deg/s glej poglavje 4.19
  // FIXME: vrednost je določena le ko je 250deg/s
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}


void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}


void printRawData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(gyroX);
  Serial.print(" Y=");
  Serial.print(gyroY);
  Serial.print(" Z=");
  Serial.print(gyroZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(accelX);
  Serial.print(" Y=");
  Serial.print(accelY);
  Serial.print(" Z=");
  Serial.println(accelZ);
}



