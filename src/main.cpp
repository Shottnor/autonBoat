#include <TinyGPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <LSM303.h>

#define GPS_TX_PIN 0
#define GPS_RX_PIN 2

SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);
TinyGPS gps;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      double targetLatitude = 50.224878;
      double targetLongitude = 7.480644;
      
      float currentLatitude, currentLongitude;
      gps.f_get_position(&currentLatitude, &currentLongitude);
      
      float distance = TinyGPS::distance_between(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
      float course = TinyGPS::course_to(currentLatitude, currentLongitude, targetLatitude, targetLongitude);

      Serial.print("Distance to target: ");
      Serial.print(distance);
      Serial.print(" meters\tCourse: ");
      Serial.print(course);
      Serial.println(" degrees");

      // Implement your navigation logic here, based on distance and course
    }
  }
}