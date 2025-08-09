# Solar Panel Positioning System with GPS | Arduino-Based

> The entire system is simulated in Proteus Design Suite, including GPS, servos, LCD, and Arduino.

![Simulation](https://github.com/youness-el-kabtane/Solar-Panel-Positioning-System-with-GPS-Arduino-Based/blob/b9f9d9e6f24a8534f1aded9cec29575aba3051ad/image/image1.png)

This Arduino-based Solar Panel Positioning System utilizes GPS coordinates and a streamlined Solar Position Algorithm (SPA) to automatically track the sun's movement throughout the day. The system employs two servo motors mounted on a 3D-printed mechanical frame to precisely control both azimuth and elevation angles, ensuring optimal solar panel orientation for maximum energy capture. By combining real-time GPS positioning data with efficient sun-tracking calculations, this automated dual-axis tracker significantly improves solar panel efficiency compared to fixed installations.

## Components List

![Components](#)

**Display:**

-   LCD 20x4 connected to SCL and SDA pins
-   Displays: azimuth, elevation, latitude, longitude, time

**Status Indicators:**

-   2 LEDs for GPS status (data received/not received) - Pins 4, 3
-   2 LEDs for servo motor status (moving/stopped) - Pins 13, 12

**GPS Module:**

-   Neo-6M GPS module
-   Connections: TX, GND, VCC

**Actuators:**

-   2 Servo motors for angle control - Pins 11, 10
-   Controls azimuth and elevation positioning

## NEO-6M GPS

> **NEO-6M GPS module** outputting **NMEA sentences**, which are standard GPS data formats used by GPS receivers to send information
> like position, speed, date, and time.

### What the NEO-6M GPS module does
---
-   It connects to GPS satellites and calculates:
    
    -   **Latitude & Longitude** (your position on Earth)
    -   **Altitude** (height above sea level)
    -   **Speed** (movement over ground)
    -   **Direction** (course over ground)
    -   **Time & Date** (UTC time from satellites)
        
-   It sends this data to a microcontroller or PC over a **serial interface (UART)** as text strings.

### Understanding the NMEA sentence
---

![NMEA sentence](#)

Each line starts with a **$** and is a specific **NMEA sentence**. You have two types here:

#### 1. `$GPRMC` – Recommended Minimum Data
---

```py
$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C
```
Breakdown:

-   **045103.000** → Time (04:51:03 UTC)
-   **A** → Status (A = valid fix, V = no fix) 
-   **3014.1984,N** → Latitude (30°14.1984' North)
-   **09749.2872,W** → Longitude (97°49.2872' West)
-   **0.67** → Speed over ground (knots)
-   **161.46** → Course over ground (degrees)
-   **030913** → Date (03 Sep 2013 in this example)
-   **A*7C** → Mode and checksum

#### 2. `$GPGGA` – GPS Fix Data
---

Example:
```py
$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62
```
Breakdown:

-   **045104.000** → Time (04:51:04 UTC)
-   **3014.1985,N** → Latitude
-   **09749.2873,W** → Longitude
-   **1** → Fix quality (0 = no fix, 1 = GPS fix, 2 = DGPS fix)
-   **09** → Number of satellites used
-   **1.2** → Horizontal dilution of precision (HDOP)    
-   **211.6,M** → Altitude = 211.6 meters above sea level    
-   **-22.5,M** → Height of geoid above WGS84 ellipsoid    
-   **0000*62** → Checksum

> The module is continuously outputting **GPS position, time, altitude,
> and satellite data** in NMEA format.   You can parse these sentences
> in Arduino, Python, or any language to extract **human-readable GPS
> coordinates**.

## **Code Structure**

1.  **Main Arduino sketch** (`.ino`)
    
    -   Handles **hardware setup** (GPS, LCD, LEDs, servos).
    -   Reads GPS data.
    -   Calls the SPA algorithm to compute sun angles.
    -   Moves servos based on sun angles.
    -   Displays data on LCD & Serial Monitor.
        
2.  **SPA Algorithm Implementation** (`spa.cpp`)
    
    -   Contains `calculateSunPosition()` function.
    -   Does the math to calculate **azimuth** (compass direction) and **elevation** (height above horizon) of the Sun for a given place and time.
        
3.  **Header file** (`spa.h`)
    
    -   Defines the `SunPosition` struct.
    -   Declares the `calculateSunPosition()` function so it can be used in the `.ino` file.

### SPA (Solar Position Algorithm)
---

> At the core of this system is a simplified algorithm based on the **[Solar Position Algorithm (SPA)](https://midcdmz.nrel.gov/spa/)** to compute the sun's position from time and location data.

The **SPA (Solar Position Algorithm)** calculates the Sun’s position in the sky for a given date, time, and location.  
It uses astronomy formulas to find:

-   **Declination** → Sun’s tilt relative to Earth’s equator
    
-   **Equation of Time** → small clock-time correction due to Earth’s tilt/orbit shape
    
-   **Hour Angle** → Sun’s position relative to local noon
    
-   From these, it computes **azimuth** (compass direction) and **elevation** (height above horizon).

#### 1. Setup and Constants
---

```cpp
#define DEG_TO_RAD 0.0174532925
#define RAD_TO_DEG 57.2957795
```
-   Conversion constants:
    -   **DEG_TO_RAD** = multiply degrees by this to get radians.
    -   **RAD_TO_DEG** = multiply radians by this to get degrees.
-   All trigonometric functions in C++ (`sin`, `cos`, `asin`, `acos`) work in radians, so these are necessary.

#### 2.Time in Fractional Hours
---

```cpp
double timeUTC = hour + minute / 60.0 + second / 3600.0 - timezone_offset;
```
-   Converts **hour:minute:second** into **decimal hours** (e.g., 14:30:00 → 14.5).
-   Adjusts for **timezone offset** to get UTC time.
-   Required because solar calculations work from UTC.

#### 3.Day of the Year
---

```cpp
int N1 = floor(275 * month / 9);
int N2 = floor((month + 9) / 12);
int K = 1 + floor((year - 4 * floor(year / 4) + 2) / 3);
int N = N1 - (N2 * K) + day - 30;
```
-   Calculates **N**, the day number in the year (1–365).
-   Accounts for leap years (`K` factor).
-   Needed to determine where Earth is in its orbit.

#### 4.Sun Declination Angle (δ)
---

```cpp
double decl = 23.45 * sin(DEG_TO_RAD * ((360.0 / 365.0) * (N - 81)));
```
-   **Declination** is the Sun’s angle north/south of the equator.
-   It changes through the year as Earth tilts toward/away from the Sun.
-   `23.45°` is Earth’s axial tilt.

#### 5.Equation of Time (EoT)
---

```cpp
double B = 360.0 / 365.0 * (N - 81);
double EoT = 9.87 * sin(2 * DEG_TO_RAD * B)
            - 7.53 * cos(DEG_TO_RAD * B)
            - 1.5 * sin(DEG_TO_RAD * B);
```
-   Corrects for:
    1.  **Earth’s elliptical orbit**
    2.  **Axial tilt**
-   Used to adjust solar time vs clock time.

#### 6.Solar Time and Hour Angle
---

```cpp
double solarTime = timeUTC * 60 + 4 * longitude + EoT; // in minutes
double hourAngle = (solarTime / 4.0) - 180.0;
```
-   **solarTime** → Minutes since midnight in "Sun time".
    -   `4 * longitude` converts longitude degrees to time (1° = 4 minutes).
    -   Adds **Equation of Time** correction.
-   **hourAngle** → Sun’s position east/west of solar noon:
    -   Negative in the morning, 0 at noon, positive in the afternoon.

#### 7.Elevation Angle
---

```cpp
double elevation = asin(
  sin(DEG_TO_RAD * decl) * sin(DEG_TO_RAD * latitude) +
  cos(DEG_TO_RAD * decl) * cos(DEG_TO_RAD * latitude) * cos(DEG_TO_RAD * hourAngle)
) * RAD_TO_DEG;
```
-   **Elevation** = height of Sun above horizon:
    -   **0°** = horizon
    -   **90°** = directly overhead
-   Uses **spherical trigonometry** with:
    -   Declination
    -   Latitude      
    -   Hour angle

#### 8.Azimuth Angle
---

```cpp
double azimuth = acos(
  (sin(DEG_TO_RAD * decl) * cos(DEG_TO_RAD * latitude) -
   cos(DEG_TO_RAD * decl) * sin(DEG_TO_RAD * latitude) * cos(DEG_TO_RAD * hourAngle)) /
  cos(DEG_TO_RAD * elevation)
) * RAD_TO_DEG;

if (hourAngle > 0) azimuth = 360 - azimuth;
```
-   **Azimuth** = compass direction to Sun:
    -   0° = North
    -   90° = East      
    -   180° = South
    -   270° = West
-   The last `if` fixes morning/afternoon direction.

#### 9.Return Results
---

```cpp
result.azimuth = azimuth;
result.elevation = elevation;
return result;
```
-   Packs the two results into `SunPosition` struct.
-   Returned to the main Arduino code for servo control.

### The Main Code (`.ino`)
---

#### 1. Libraries
---

```cpp
#include <LiquidCrystal_I2C.h>  // LCD display
#include <Servo.h>              // Servo motor control
#include <Wire.h>               // I2C communication
#include <TinyGPSPlus.h>        // GPS data parsing
#include "spa.h"                // Sun Position Algorithm
```
These bring in all the required hardware drivers and the SPA math function.

#### 2. Hardware Setup
---

```cpp
LiquidCrystal_I2C display(0x20,20,4);
int LG = 3, LR = 4, LO1 = 12, LO2 = 13;
int S1 = 10, S2 = 11;

Servo azimuthServo, elevationServo;
```
-   **LG, LR** → Green/Red LEDs for GPS fix status.
-   **LO1, LO2** → Outputs that turn ON when servos are active.
-   **S1, S2** → Pins for servo motors (azimuth & elevation control).

#### 3. GPS Data Handling
---

```cpp
TinyGPSPlus gps;

while (Serial.available() > 0) {
  gps.encode(Serial.read());
  if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    altitude = gps.altitude.meters();
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();
    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
  }
}
```
-   Reads NMEA sentences from GPS.
-   Extracts **latitude, longitude, altitude, date, and time** when GPS has a valid fix.

#### 4. SPA Algorithm Call
---

```cpp
SunPosition sun = calculateSunPosition(
  year, month, day,
  hour, minute, second,
  latitude, longitude,
  altitude,
  timezone_offset
);
```
-   Passes GPS data into the SPA math function.
-   Returns **`sun.azimuth`** and **`sun.elevation`**.

#### 5. Servo Control
---

```cpp
int azimuthServoAngle = map(sun.azimuth, 90, 270, 0, 180);
int elevationServoAngle = map(sun.elevation, 0, 90, 0, 180);

azimuthServo.write(constrain(azimuthServoAngle, 0, 180));
elevationServo.write(constrain(elevationServoAngle, 0, 180));
```
-   Converts sun angles to servo positions.
-   Makes the tracker follow the Sun’s position.

#### 6. LCD and Serial Output
---

![Serial Monitor](#)

-   Displays latitude, longitude, UTC time, azimuth, and elevation on the LCD.
-   Prints the same info to the serial monitor.

## 3D Models

This project includes physical components supported by 3D-printed parts for mounting and positioning the solar panel and sensors.

### 3D Models – Original Source
---

Some of the 3D models used in this project were derived from the following open-source Thingiverse project:

-   **Title:** [Solar Cell Tracking](https://www.thingiverse.com/thing:2939509/files)
    
-   **Author:** Michaelo
    
-   **License:** Creative Commons - Attribution (CC BY)
    

> These models were copied and reused in accordance with the license terms, which require proper attribution. Full credit is given to the original designer, **Michaelo**, for sharing these useful resources with the maker community.

## References

 - [NREL's Solar Position Algorithm (SPA)](https://midcdmz.nrel.gov/spa/)
 - [3D Models](https://www.thingiverse.com/thing:2939509/files)

---
**Author:** Youness El Kabtane

**Website:** [younesselkabtane](https://sites.google.com/view/younesselkabtane/home?authuser=1)

**Version:** 1.0.0

**Made with 💗**

