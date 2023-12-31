#include <TinyGPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <LSM303.h>

#define GPS_TX_PIN 0
#define GPS_RX_PIN 2

SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);
TinyGPS gps;
LSM303 cmps;

void currentPos(float targetLat,float targetLon, float* add_lat, float* add_lon, float* add_heading, float* add_distance, int* add_sats){ //all the adresses for the variables
  while(gpsSerial.available() > 0){
   
    char c = gpsSerial.read();
      if (gps.encode(c)) {
        
        float currentLatitude, currentLongitude;
        gps.f_get_position(&currentLatitude, &currentLongitude); //get current position
        
        *add_distance = TinyGPS::distance_between(currentLatitude, currentLongitude, targetLat, targetLon); //get distance between two points
        *add_heading = TinyGPS::course_to(currentLatitude, currentLongitude, targetLat, targetLon); //get heading between two points
        *add_lat = currentLatitude;
        *add_lon = currentLongitude;
        *add_sats = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
        //Serial.print("Distance to target: ");
        //Serial.print(distance);
        //Serial.print(" meters\tCourse: ");
        //Serial.print(course);
        //Serial.println(" degrees");
        
      }
    
  }
}


void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  Wire.begin();
  cmps.init();
  cmps.enableDefault();
  cmps.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767}; //define standard calibration values for our compass
  cmps.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}

float targetChange(float current, float wanted){
  
  float targetChange;
  
  if(current + wanted > 180){ //need to go right
    if(current > wanted){
      targetChange = current - wanted;
    }else{
      targetChange = wanted - current;
    }

  }else{ //need to go left
    if(current < wanted){
      targetChange = current - wanted;
    }else{
      targetChange = wanted - current;
    }
  }

return targetChange;
  
}

void loop() {
  float targetLat = 50.351368; //target Latitude
  float targetLon = 7.565410; //target Longitude
  float latitude, longitude, wantedHeading, currentHeading, distance;
  int sats;
  currentPos(targetLat, targetLon, &latitude, &longitude, &wantedHeading, &distance, &sats); //pass data adresses to Position handler
  cmps.read(); //read compass value
  currentHeading = cmps.heading();  //store compass heading
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("    wanted: ");
  Serial.print(wantedHeading);
  Serial.print("  current:  ");
  Serial.print(currentHeading);
  Serial.print("    sats: ");
  Serial.print(sats);
  Serial.print("    Change: ");
  Serial.println(targetChange(currentHeading, wantedHeading));
  
  delay(100);
  
}