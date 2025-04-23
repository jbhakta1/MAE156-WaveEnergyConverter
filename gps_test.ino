#include <TinyGPS++.h>

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);        // For Serial Monitor
  Serial1.begin(9600);       // GT-U7 connected to Serial1 on Mega (TX1/RX1)

  Serial.println("GT-U7 GPS Module Starting...");
}

void loop() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());

    if (gps.location.isUpdated()) {
      Serial.println("----- GPS Data -----");
      Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
      Serial.print("Satellites: "); Serial.println(gps.satellites.value());
      Serial.println("--------------------");
      delay(1000);
    }
  }
}
