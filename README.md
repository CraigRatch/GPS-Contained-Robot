# GPS-Contained-Robot
GPS-based autonomous geofencing system with motor control for testing boundary detection and navigation.

# GPS-Based Autonomous Geofencing System

## Overview
This project is part of my Electronic Engineering degree work, focusing on **autonomous geofencing** using GPS data. The system retrieves **NMEA sentences** from a GPS module, parses them using the TinyGPS++ library, and calculates the distance from a user-defined target point (Centre point). Based on this, the system can control motors to navigate or respond when moving in or out of defined boundaries. Such as turn when boundarie is reached. 

---

## Features
- Real-time GPS data retrieval and parsing  
- Distance calculation from a reference point  
- Testing and comparison of different geofencing approaches  
- Integration with L298N motor driver for autonomous movement  
- Serial monitoring of GPS coordinates, satellites, and distance  

---

## Hardware Required
- Arduino (e.g., Uno, Mega)  
- GPS Module (compatible with NMEA sentences)  
- L298N Motor Driver  
- DC Motors and power supply  
- Jumper wires and breadboard  

---

## Software / Libraries
- Arduino IDE  
- [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)  
- [SoftwareSerial](https://www.arduino.cc/en/Reference/SoftwareSerial)  

---

## Wiring / Pin Configuration
| Component          | Arduino Pin |
|-------------------|-------------|
| GPS RX             | 6           |
| GPS TX             | 5           |
| Motor IN1 (M1A)    | 8           |
| Motor IN2 (M1B)    | 9           |
| Motor IN3 (M2A)    | 10          |
| Motor IN4 (M2B)    | 11          |
| Motor ENA          | 12          |
| Motor ENB          | 7           |

---

## How It Works
1. The GPS module sends NMEA sentences to the Arduino.  
2. TinyGPS++ parses these sentences to extract latitude, longitude, and satellite data.  
3. The system calculates the distance to a **user-defined reference point** (the center of the geofence).  
4. If the device is within the defined boundary, it moves forward.  
5. If it moves out of bounds, the motors stop and the device turns to adjust its path.  

---

## Usage
1. Clone or download this repository.  
2. Open the `.ino` file in Arduino IDE.  
3. Install the required libraries (TinyGPS++, SoftwareSerial).  
4. Connect your GPS and motor driver according to the wiring table.  
5. Upload the code to your Arduino and monitor serial output for GPS data and distance.  

---

## Testing
- Tested various geofencing radii and boundary types.  
- Serial monitor displays real-time GPS coordinates, satellites, and distance to the reference point.  
- Motor responses adjust based on the calculated distance.  

---

## Notes
- Make sure GPS module has a clear view of the sky for accurate data.  
- Adjust `maxdistance` in the code to change the geofence radius.  
- Most ordinary GPS modules are accurate to about 2 meteres. 

---

## License
This project is open-source. Feel free to use and modify for educational purposes.  

---



