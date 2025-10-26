//---------------------------Craig Ratchford-----------------------------------//
//--------------------------GPS contained robot-------------------------------//


#include <SoftwareSerial.h>  // Library for using serial communication on other pins
#include <TinyGPS++.h>       // Library for parsing GPS data
#include <Adafruit_GPS.h>    // Optional: Adafruit GPS library (not used directly here)
#include <math.h>            // Math functions (optional, not used here)

// Define pins for GPS module
#define GPS_RX_PIN 6
#define GPS_TX_PIN 5

// Initialize software serial for GPS communication
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;  // Create TinyGPS++ object


// User-defined target coordinates
// These coordinates define the "center point" of a virtual boundary.
// The program will measure the distance between the current GPS position
// and this target point to determine if the device is within the allowed range.
const float userLatitude = 53.32642745971679;   // Target latitude (center of boundary)
const float userLongitude = -6.27157163619995; // Target longitude (center of boundary)

float gpsLatitude, gpsLongitude;  // Variables to store current GPS coordinates

int maxdistance = 5; // Maximum allowed distance in meters

// Define pins for motor driver (L298N)
const int motor1A = 8;    // IN1
const int motor1B = 9;    // IN2
const int motor2A = 10;   // IN3
const int motor2B = 11;   // IN4
const int enA = 12;       // Enable pin for motor 1
const int enB = 7;        // Enable pin for motor 2

void setup() {
  Serial.begin(115200);        // Start serial monitor
  gpsSerial.begin(9600);       // Start GPS serial communication
  pinMode(4, OUTPUT);          // Unused pin (can be removed if not needed)

  // Initialize motor control pins as OUTPUT
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
}

// Function prototype for distance calculation
float getdistance(float userLatitude, float userLongitude, float gpsLatitude, float gpsLongitude);

void loop() {
  // Read GPS data if available
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {  // Parse GPS data
      // Print number of satellites seen
      Serial.print("Satellites seen: ");
      Serial.println(gps.satellites.value());

      // Print current GPS coordinates
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 14);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 14);

      // Store GPS coordinates in variables
      gpsLatitude = gps.location.lat();
      gpsLongitude = gps.location.lng();

      // Calculate distance from target coordinates
      float dd = getdistance(userLatitude, userLongitude, gpsLatitude, gpsLongitude);
      Serial.print("Distance between is: ");
      Serial.println(dd, 6);

      // Check if within boundary
      if (dd < maxdistance) {
        Serial.println("Within boundary");
        moveForward();  // Move forward if within allowed distance
      } else {
        Serial.println("Out of bounds");
        stopMotors();   // Stop motors
        delay(200);

        // Turn right and avoid obstacle
        turnRight();
        delay(1000);
        moveForward();  // Move forward after turning

        delay(200);
      }
    }
  }
}

// Function to calculate distance between two GPS points
float getdistance(float userLatitude, float userLongitude, float gpsLatitude, float gpsLongitude) {
  float distance = gps.distanceBetween(userLatitude, userLongitude, gpsLatitude, gpsLongitude);
  return distance;  // Return distance in meters
}

// Function to move both motors forward
void moveForward() {
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);

  analogWrite(enA, 200);  // Set speed of motor 1
  analogWrite(enB, 200);  // Set speed of motor 2
}

// Function to stop both motors
void stopMotors() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
}

// Function to turn the robot right
void turnRight() {
  // Reverse one motor to turn
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);

  delay(700);  // Duration of turn

  analogWrite(enA, 120);  // Set motor speed after turn
  analogWrite(enB, 120);
}
