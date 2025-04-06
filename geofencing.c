#include <Wire.h>
#include <TinyGPS++.h>
#include <math.h>

// GPS module connection via hardware serial
#define GPS_BAUD 9600
#define EARTH_RADIUS 6371000  // Earth's radius in meters
#define PROXIMITY_THRESHOLD 10.0  // 10 meters threshold for waypoint proximity
#define MAX_SPEED 2.0  // Maximum speed in m/s for safety checks

// Predefined route coordinates (latitude and longitude)
const float route[] = {
    34.0206, -118.2867, // Point 1: USC Campus
    34.0215, -118.2850, // Point 2: Another location around USC
    34.0220, -118.2870, // Point 3: Another location around USC
    // Add more points as needed
};

// Number of points in the route
const int route_size = sizeof(route) / sizeof(route[0]) / 2;

TinyGPSPlus gps;
int currentWaypoint = 0;
bool routeComplete = false;
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1000; // Update every second

// Structure to hold GPS data
struct GPSData {
    float latitude;
    float longitude;
    float speed;
    float course;
    bool isValid;
};

void setup() {
    Serial.begin(9600);
    Serial1.begin(GPS_BAUD);
    Serial.println("GPS Geofencing System Initialized");
}

void loop() {
    // Read GPS data
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    // Update at regular intervals
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = millis();
        
        GPSData currentData = getGPSData();
        
        if (currentData.isValid) {
            printGPSData(currentData);
            
            if (!routeComplete) {
                checkWaypointProximity(currentData);
                checkRouteProgress(currentData);
            }
        } else {
            Serial.println("Waiting for valid GPS signal...");
        }
    }
}

GPSData getGPSData() {
    GPSData data;
    data.isValid = false;
    
    if (gps.location.isValid() && gps.speed.isValid() && gps.course.isValid()) {
        data.latitude = gps.location.lat();
        data.longitude = gps.location.lng();
        data.speed = gps.speed.mps();
        data.course = gps.course.deg();
        data.isValid = true;
    }
    
    return data;
}

void printGPSData(GPSData data) {
    Serial.print("Location: ");
    Serial.print(data.latitude, 6);
    Serial.print(", ");
    Serial.print(data.longitude, 6);
    Serial.print(" | Speed: ");
    Serial.print(data.speed, 1);
    Serial.print(" m/s | Course: ");
    Serial.print(data.course, 1);
    Serial.println("°");
}

void checkWaypointProximity(GPSData currentData) {
    float routeLat = route[2 * currentWaypoint];
    float routeLon = route[2 * currentWaypoint + 1];
    
    float distance = calculateDistance(currentData.latitude, currentData.longitude, 
                                     routeLat, routeLon);
    
    if (distance <= PROXIMITY_THRESHOLD) {
        Serial.print("Reached waypoint ");
        Serial.print(currentWaypoint + 1);
        Serial.print("! Distance: ");
        Serial.print(distance, 1);
        Serial.println(" meters");
        
        // Move to next waypoint
        currentWaypoint++;
        if (currentWaypoint >= route_size) {
            routeComplete = true;
            Serial.println("Route completed!");
        }
    }
}

void checkRouteProgress(GPSData currentData) {
    // Calculate distance to next waypoint
    float routeLat = route[2 * currentWaypoint];
    float routeLon = route[2 * currentWaypoint + 1];
    float distance = calculateDistance(currentData.latitude, currentData.longitude, 
                                     routeLat, routeLon);
    
    // Calculate bearing to next waypoint
    float bearing = calculateBearing(currentData.latitude, currentData.longitude,
                                   routeLat, routeLon);
    
    // Calculate heading error
    float headingError = fabs(bearing - currentData.course);
    if (headingError > 180) {
        headingError = 360 - headingError;
    }
    
    Serial.print("Distance to next waypoint: ");
    Serial.print(distance, 1);
    Serial.print("m | Heading error: ");
    Serial.print(headingError, 1);
    Serial.println("°");
    
    // Safety checks
    if (currentData.speed > MAX_SPEED) {
        Serial.println("WARNING: Speed exceeds maximum limit!");
    }
}

// Calculate distance between two points using Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon/2) * sin(dLon/2);
              
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;
}

// Calculate bearing between two points
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = radians(lon2 - lon1);
    
    float y = sin(dLon) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) -
              sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
              
    float bearing = degrees(atan2(y, x));
    return fmod((bearing + 360), 360);
}

// Helper function to convert degrees to radians
float radians(float degrees) {
    return degrees * M_PI / 180.0;
}

// Helper function to convert radians to degrees
float degrees(float radians) {
    return radians * 180.0 / M_PI;
}
