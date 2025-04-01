#include <Wire.h>
#include <TinyGPS++.h>

// GPS module connection via hardware serial
#define GPS_BAUD 9600

// Predefined route coordinates (latitude and longitude)
const float route[] = {
    34.0206, -118.2867, // Point 1: USC Campus
    34.0215, -118.2850, // Point 2: Another location around USC
    34.0220, -118.2870, // Point 3: Another location around USC
    // Add more points as needed
};

// Number of points in the route
const int route_size = sizeof(route) / sizeof(route[0]) / 2; // Divide by 2 because each point has lat/lon

TinyGPSPlus gps;

void setup() {
    Serial.begin(9600); // Serial monitor for output
    Serial1.begin(GPS_BAUD); // GPS module connected to hardware serial port 1
}

void loop() {
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    if (gps.location.isUpdated()) {
        float currentLat = gps.location.lat();
        float currentLon = gps.location.lng();

        Serial.print("Current Location: ");
        Serial.print("Lat: ");
        Serial.print(currentLat, 6);
        Serial.print(", Lon: ");
        Serial.println(currentLon, 6);

        // Check if current location is near any of the predefined route points
        for (int i = 0; i < route_size; i++) {
            float routeLat = route[2 * i];     // Latitude for the i-th route point
            float routeLon = route[2 * i + 1]; // Longitude for the i-th route point

            // Check proximity with a simple threshold (e.g., within 50 meters)
            if (isWithinProximity(currentLat, currentLon, routeLat, routeLon)) {
                Serial.print("You are near route point: ");
                Serial.print(i + 1);
                Serial.print(" (Lat: ");
                Serial.print(routeLat, 6);
                Serial.print(", Lon: ");
                Serial.println(routeLon, 6);
            }
        }
    }
}

// Function to check if the current GPS coordinates are within a proximity threshold of a route point
bool isWithinProximity(float lat1, float lon1, float lat2, float lon2) {
    const float proximityThreshold = 0.00045; // ~50 meters (depending on lat/lon scale)
    
    // Simple distance check (Haversine formula or approximated)
    float latDiff = lat1 - lat2;
    float lonDiff = lon1 - lon2;

    float distance = sqrt(latDiff * latDiff + lonDiff * lonDiff);
    return distance < proximityThreshold;
}
