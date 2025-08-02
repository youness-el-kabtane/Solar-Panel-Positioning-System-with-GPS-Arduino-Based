# Solar Panel Positioning System with GPS | Arduino-Based

## Overview
> The entire system is simulated in Proteus Design Suite, including GPS, servos, LCD, and Arduino.

![Simulation](https://github.com/youness-el-kabtane/Solar-Panel-Positioning-System-with-GPS-Arduino-Based/blob/91e7d0451814cadda96cdbafb89b83d9f8f5630d/image/image1.png) 

This project is a **solar tracking system** that uses **GPS data** to calculate the real-time position of the sun (azimuth and elevation) and orients a solar panel accordingly using **servo motors**. It is built with an **Arduino Uno**, integrates a **GPS module**, **LCD display**, and uses **LEDs for status indicators**.

At the core of this system is a simplified algorithm based on the **[Solar Position Algorithm (SPA)](https://midcdmz.nrel.gov/spa/)** to compute the sun's position from time and location data.

![Simulation](https://github.com/youness-el-kabtane/Solar-Panel-Positioning-System-with-GPS-Arduino-Based/blob/35853b0ba3b8db147bef0a6d393176bcba101068/image/image2.png) 

## Components Used
| Component          | Description                               
|--------------------|------------------------------------------
| Arduino Uno        | Main microcontroller                      
| GPS Module         | NEO-6M or compatible                      
| 2x Servo Motors    | For azimuth and elevation control         
| 20x4 LCD           | To display system data                    
| LEDs               | Status indication (GPS fix, motor active) 
| External Power     | For servos and GPS                        

## Schematic 

![Schematic](https://github.com/youness-el-kabtane/Solar-Panel-Positioning-System-with-GPS-Arduino-Based/blob/35853b0ba3b8db147bef0a6d393176bcba101068/image/image5.png) 

## How It Works
This system tracks the sun using real-time location and time data from a GPS module. 

1.  ### GPS Module (NEO-6M or similar)
    
    The GPS module constantly provides the Arduino with the following real-time data:
    
    -   **Latitude & Longitude**: The geographic position of your system.
        
    -   **Altitude**: The height above sea level, used to improve accuracy.
        
    -   **UTC Date and Time**: Required for astronomical calculations.
        
    
    > This information is parsed using the `TinyGPSPlus` library and passed to the sun position algorithm.

![GPS Monitor](https://github.com/youness-el-kabtane/Solar-Panel-Positioning-System-with-GPS-Arduino-Based/blob/35853b0ba3b8db147bef0a6d393176bcba101068/image/image3.png) 

2.  ### Data Processing with Arduino
    
    Once the GPS data is received:
    
    -   The Arduino extracts relevant values like year, month, day, hour, minute, second, latitude, longitude, and altitude.
        
    -   It then passes this data to the custom **Sun Position Algorithm** written in C++ (`spa.cpp`).

![Serial Monitor](https://github.com/youness-el-kabtane/Solar-Panel-Positioning-System-with-GPS-Arduino-Based/blob/35853b0ba3b8db147bef0a6d393176bcba101068/image/image4.png) 
        
3.  ### Sun Position Algorithm (SPA)
    
    The function `calculateSunPosition()` in `spa.cpp` processes the date, time, and location to calculate:
    
    -   **Azimuth Angle**: The compass direction from which sunlight is coming (0° = North, 90° = East).
        
    -   **Elevation Angle**: The angle of the sun above the horizon (0° = sunrise/sunset, 90° = directly overhead).
       
    > This simplified SPA implementation closely follows solar geometry formulas based on Earth's orbit.
    
4.  ### Servo Motor Control
    
    The calculated **azimuth** and **elevation** angles are mapped to values suitable for controlling **two servo motors**:
    
    -   One servo rotates the panel **horizontally (azimuth)**.
        
    -   The other tilts the panel **vertically (elevation)**.
        
    These angles are converted into PWM signals that rotate the solar panel to face the sun's direction as precisely as possible.
    
5.  ### LCD Display & LED Indicators
    
    The system includes:
    
    -   A **20x4 LCD** screen that displays:
        
        -   Current GPS coordinates
            
        -   Calculated azimuth and elevation angle
            
    -   **LEDs** for visual feedback:
        
        -   Tow LED blinks to indicate the GPS status.
            
        -   Tow LED indicate servo movement .

## 3D Model 

This project includes physical components supported by 3D-printed parts for mounting and positioning the solar panel and sensors.

### 3D Models – Original Source

Some of the 3D models used in this project were derived from the following open-source Thingiverse project:

-   **Title:** [Solar Cell Tracking](https://www.thingiverse.com/thing:2939509/files)
    
-   **Author:** Michaelo
    
-   **License:** Creative Commons - Attribution (CC BY)
    

> These models were copied and reused in accordance with the license terms, which require proper attribution. Full credit is given to the original designer, **Michaelo**, for sharing these useful resources with the maker community.

----
**Author:** Youness El Kabtane
**Website:** [younesselkabtane](https://sites.google.com/view/younesselkabtane/home?authuser=1)
**Version:** 1.0.0

**Made with 💗**
