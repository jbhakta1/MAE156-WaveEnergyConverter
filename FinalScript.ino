//Initialize Libraries
#include <Wire.h>
#include <SPI.h>
#include <LSM6.h>
#include "MS5837.h"
#include <TinyGPS++.h>
#include <SD.h>

//IMU 1: LSM6 (I2C) 
LSM6 imu1;

//IMU 2: ICM-42688-P (SPI)
#define IMU2_CS 10
#define WHO_AM_I       0x75
#define PWR_MGMT0      0x4E
#define ACCEL_CONFIG0  0x50
#define GYRO_CONFIG0   0x4F
#define ACCEL_X1       0x1F
#define ACCEL_Y1       0x21
#define ACCEL_Z1       0x23
#define GYRO_X1        0x25
#define GYRO_Y1        0x27
#define GYRO_Z1        0x29

// Pressure Sensor (I2C)
MS5837 pressureSensor;

// GPS (UART)
TinyGPSPlus gps;

// SD Logging (SPI)
#define SD_CS 53
File logfile;

// Voltage Sensor (Turbine)
#define VOLTAGE_PIN A0
const float VREF = 5.0;
const float R1 = 10000.0;
const float R2 = 5100.0;

// Logging Control
#define BUTTON_PIN 7
bool isLogging = false;

// LEDs
#define RED_LED     4
#define GREEN_LED   5
#define BLUE_LED    6

bool initFailed = false;

// TIMING
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 100;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  SPI.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(IMU2_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(IMU2_CS, HIGH);

  for (int i = 0; i < 6; i++) {
    digitalWrite(RED_LED, HIGH);
    delay(150);
    digitalWrite(RED_LED, LOW);
    delay(150);
  }

  Serial.println("Initializing Sensors");

  Serial.print("Initializing IMU 1 (LSM6)... ");
  if (!imu1.init()) {
    Serial.println("FAILED");
  } else {
    imu1.enableDefault();
    imu1.writeReg(LSM6::CTRL1_XL, 0x28);
    imu1.writeReg(LSM6::CTRL2_G,  0x48);
    Serial.println("Locked IN");
  }

  Serial.print("Initializing IMU 2 (ICM-42688-P)... ");
  delay(100);
  uint8_t whoami = readRegister(IMU2_CS, WHO_AM_I);
  Serial.print("WHO_AM_I = 0x"); Serial.println(whoami, HEX);
  if (whoami != 0x47) {
    Serial.println("FAILED");
    digitalWrite(RED_LED, HIGH);
    initFailed = true;
  } else {
    writeRegister(IMU2_CS, PWR_MGMT0, 0x0F);
    writeRegister(IMU2_CS, ACCEL_CONFIG0, 0x00);
    writeRegister(IMU2_CS, GYRO_CONFIG0,  0x01);
    Serial.println("Locked IN");
  }

  Serial.print("Initializing Pressure Sensor... ");
  if (!pressureSensor.init()) {
    Serial.println("FAILED");
    digitalWrite(RED_LED, HIGH);
    initFailed = true;
  } else {
    pressureSensor.setModel(MS5837::MS5837_02BA);
    pressureSensor.setFluidDensity(1029);
    Serial.println("Locked IN 2BA, Seawater");
  }

  Serial.print("Initializing GPS... ");
  Serial1.begin(9600);
  Serial.println("Locked IN");

  Serial.print("Initializing SD card... ");
  digitalWrite(IMU2_CS, HIGH);
  if (!SD.begin(SD_CS)) {
    Serial.println("FAILED");
    digitalWrite(RED_LED, HIGH);
    initFailed = true;
  } else {
    char filename[15];
    for (int i = 1; i < 1000; i++) {
      sprintf(filename, "log_%03d.csv", i);
      if (!SD.exists(filename)) {
        logfile = SD.open(filename, FILE_WRITE);
        break;
      }
    }
    if (!logfile) {
      Serial.println("File creation FAILED");
      digitalWrite(RED_LED, HIGH);
      initFailed = true;
    } else {
      Serial.print("Logging to: "); Serial.println(filename);
      logfile.println("time_ms,gps_time,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,pressure,depth,temp,lat,lon,sats,voltage");
      logfile.flush();
    }
  }

  if (initFailed) {
    Serial.println("Sensor init failure. Stopped. Reset required.");
    digitalWrite(RED_LED, HIGH);
    while (1);
  }

  digitalWrite(GREEN_LED, HIGH);
  Serial.println("System ready. Waiting for button press...");
  while (digitalRead(BUTTON_PIN) == HIGH);
  delay(50);
  digitalWrite(GREEN_LED, LOW); // Turn off green LED
  digitalWrite(BLUE_LED, HIGH); // Turn on blue LED
  isLogging = true;
  Serial.println("Logging started.");
}

void loop() {
  while (Serial1.available()) gps.encode(Serial1.read());

  if (millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();

    imu1.readAcc();
    imu1.readGyro();
    float ax1 = imu1.a.x * 0.122 * 9.80665 / 1000.0;
    float ay1 = imu1.a.y * 0.122 * 9.80665 / 1000.0;
    float az1 = imu1.a.z * 0.122 * 9.80665 / 1000.0;
    float gx1 = imu1.g.x * 0.00875;
    float gy1 = imu1.g.y * 0.00875;
    float gz1 = imu1.g.z * 0.00875;

    digitalWrite(SD_CS, HIGH);
    delayMicroseconds(2);
    digitalWrite(IMU2_CS, LOW);
    delayMicroseconds(2);

    float ax2 = read16(IMU2_CS, ACCEL_X1) * (9.80665 / 2048.0);
    float ay2 = read16(IMU2_CS, ACCEL_Y1) * (9.80665 / 2048.0);
    float az2 = read16(IMU2_CS, ACCEL_Z1) * (9.80665 / 2048.0) * (9.80665 / 10.16);
    float gx2 = read16(IMU2_CS, GYRO_X1) * (500.0 / 32768.0);
    float gy2 = read16(IMU2_CS, GYRO_Y1) * (500.0 / 32768.0);
    float gz2 = read16(IMU2_CS, GYRO_Z1) * (500.0 / 32768.0);

    digitalWrite(IMU2_CS, HIGH);
    delayMicroseconds(2);

    pressureSensor.read();
    float pressure = pressureSensor.pressure();
    float depth = pressureSensor.depth();
    float temp = pressureSensor.temperature();

    float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    float lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    String gps_time = gps.time.isValid() ? String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) : "00:00:00";

    int rawADC = analogRead(VOLTAGE_PIN);
    float v_out = rawADC * (VREF / 1023.0);
    float voltage = v_out * (R1 + R2) / R2;

    if (isLogging) {
      logfile.print(millis()); logfile.print(",");
      logfile.print(gps_time); logfile.print(",");
      logfile.print(ax1); logfile.print(","); logfile.print(ay1); logfile.print(","); logfile.print(az1); logfile.print(",");
      logfile.print(gx1); logfile.print(","); logfile.print(gy1); logfile.print(","); logfile.print(gz1); logfile.print(",");
      logfile.print(ax2); logfile.print(","); logfile.print(ay2); logfile.print(","); logfile.print(az2); logfile.print(",");
      logfile.print(gx2); logfile.print(","); logfile.print(gy2); logfile.print(","); logfile.print(gz2); logfile.print(",");
      logfile.print(pressure); logfile.print(","); logfile.print(depth); logfile.print(",");
      logfile.print(temp); logfile.print(",");
      logfile.print(lat, 6); logfile.print(","); logfile.print(lon, 6); logfile.print(",");
      logfile.print(sats); logfile.print(",");
      logfile.println(voltage, 2);
      logfile.flush();

      Serial.print("[LOG] ");
      Serial.print(gps_time); Serial.print(" | ");
      Serial.print(ax1); Serial.print(", "); Serial.print(ay1); Serial.print(", "); Serial.print(az1); Serial.print(" | ");
      Serial.print(gx1); Serial.print(", "); Serial.print(gy1); Serial.print(", "); Serial.print(gz1); Serial.print(" | ");
      Serial.print(ax2); Serial.print(", "); Serial.print(ay2); Serial.print(", "); Serial.print(az2); Serial.print(" | ");
      Serial.print(gx2); Serial.print(", "); Serial.print(gy2); Serial.print(", "); Serial.print(gz2); Serial.print(" | ");
      Serial.print(pressure); Serial.print(" mbar, "); Serial.print(depth); Serial.print(" m, "); Serial.print(temp); Serial.print(" C | ");
      Serial.print(lat, 6); Serial.print(", "); Serial.print(lon, 6); Serial.print(" | sats: "); Serial.print(sats); Serial.print(" | V: ");
      Serial.println(voltage, 2);
    }
  }
}

// === IMU 2 SPI Helpers ===
uint8_t readRegister(uint8_t cs, uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SD_CS, HIGH);
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return val;
}

void writeRegister(uint8_t cs, uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SD_CS, HIGH);
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

int16_t read16(uint8_t cs, uint8_t regHigh) {
  uint8_t high = readRegister(cs, regHigh);
  uint8_t low  = readRegister(cs, regHigh + 1);
  return (int16_t)((high << 8) | low);
}
