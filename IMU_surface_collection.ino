#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect IMU!");
    while (1);
  }

  imu.enableDefault();

  // Set accelerometer to 52 Hz, ±4g (CTRL1_XL = ODR 52Hz + FS_XL = 01)
  imu.writeReg(LSM6::CTRL1_XL, 0x28); // 0b00101000

  // Set gyroscope to 104 Hz, ±245 dps (CTRL2_G = ODR 104Hz + FS_G = 00)
  imu.writeReg(LSM6::CTRL2_G, 0x48); // 0b01001000

  Serial.println("IMU initialized with ±4g accel, ±245 dps gyro.");
}

void loop() {
  imu.readAcc();
  imu.readGyro();

  // Convert to real-world units
  float ax = imu.a.x * 0.122 * 9.80665 / 1000.0; // m/s²
  float ay = imu.a.y * 0.122 * 9.80665 / 1000.0;
  float az = imu.a.z * 0.122 * 9.80665 / 1000.0;

  float gx = imu.g.x * 0.00875;   // dps
  float gy = imu.g.y * 0.00875;
  float gz = imu.g.z * 0.00875;

  // Print results
  Serial.print("Accel (m/s^2): ");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.println(az);

  Serial.print("Gyro (dps): ");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  delay(500);
}
