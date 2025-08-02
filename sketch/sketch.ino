#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include "spa.h"

LiquidCrystal_I2C display(0x20,20,4);

// LED and Servo pin
int LG = 3, LR = 4, LO1 = 12, LO2 = 13; 
int S1 = 10, S2 = 11;

Servo azimuthServo;
Servo elevationServo;

TinyGPSPlus gps;

double latitude = 0.0, longitude = 0.0, altitude = 0.0;
int day = 0, month = 0, year = 0;
int hour = 0, minute = 0, second = 0;
int timezone_offset = 1;  // Timezone : Morroco,

SunPosition sun = calculateSunPosition(
        year, month, day,
        hour, minute, second,
        latitude, longitude,
        altitude,
        timezone_offset
      );

void setup() {
  Serial.begin(9600);
  display.init();
  display.backlight();
  
  pinMode(LG, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(LO1, OUTPUT);
  pinMode(LO2, OUTPUT);
  
  azimuthServo.attach(S1);
  elevationServo.attach(S2);
}

void loop() {
  while (Serial.available() > 0) {
    gps.encode(Serial.read());

    if (gps.location.isUpdated()) {
      digitalWrite(LG, HIGH);
      digitalWrite(LR, LOW);
      
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      altitude = gps.altitude.meters();

      day = gps.date.day();
      month = gps.date.month();
      year = gps.date.year();

      hour = gps.time.hour();
      minute = gps.time.minute();
      second = gps.time.second();

      serial_data();
      LCD();

    } else {
       digitalWrite(LG, LOW);
       digitalWrite(LR, HIGH);
    }
  }
  
 servo_control();
  
}

void serial_data() {
  Serial.println("======== GPS DATA ========");
  Serial.print("Latitude     : "); Serial.println(latitude, 6);
  Serial.print("Longitude    : "); Serial.println(longitude, 6);
  Serial.print("Altitude     : "); Serial.print(altitude); Serial.println(" m");
  
  Serial.print("Date (UTC)   : ");
  if (day < 10) Serial.print('0'); Serial.print(day); Serial.print('/');
  if (month < 10) Serial.print('0'); Serial.print(month); Serial.print('/');
  Serial.println(year);

  Serial.print("Time (UTC)   : ");
  if (hour < 10) Serial.print('0'); Serial.print(hour); Serial.print(':');
  if (minute < 10) Serial.print('0'); Serial.print(minute); Serial.print(':');
  if (second < 10) Serial.print('0'); Serial.println(second);

  Serial.println("==========================");
  Serial.println("====== SOLAR ANGLES ======"); 
  Serial.print("Azimuth      : "); Serial.println(sun.azimuth, 2);
  Serial.print("Elevation    : "); Serial.println(sun.elevation, 2);
  Serial.println("==========================");
}

void servo_control() {
  int azimuthServoAngle = map(sun.azimuth, 90, 270, 0, 180);     // Sun moves East→South→West
  int elevationServoAngle = map(sun.elevation, 0, 90, 0, 180);   // Sun rises from horizon to overhead

  azimuthServoAngle = constrain(azimuthServoAngle, 0, 180);
  elevationServoAngle = constrain(elevationServoAngle, 0, 180);

  azimuthServo.write(azimuthServoAngle);
  elevationServo.write(elevationServoAngle);
  
  if (azimuthServoAngle != 0) {
    digitalWrite(LO1, HIGH);
  } else {
    digitalWrite(LO1, LOW);
  }

  if (elevationServoAngle != 0) {
    digitalWrite(LO2, HIGH);
  } else {
    digitalWrite(LO2, LOW);
  }
  
}

void LCD() {
  display.clear();

  display.setCursor(0, 0);
  display.print("Lat:");
  display.print(latitude, 6);

  display.setCursor(0, 1);
  display.print("Lon:");
  display.print(longitude, 6);
  
  display.setCursor(0, 2);
  display.print("UCT:");
  if (hour < 10) display.print('0'); display.print(hour); display.print(':');
  if (minute < 10) display.print('0'); display.print(minute); display.print(':');
  if (second < 10) display.print('0'); display.print(second);
    
  delay(100);

  display.setCursor(0, 0);
  display.print("Az:");
  display.print(sun.azimuth); 
  
  display.setCursor(0, 1);
  display.print("El:");
  display.print(sun.elevation); 
 
}
