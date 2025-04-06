#define F_CPU 7372800UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

// GPS USART configuration
#define GPS_BAUD 9600
#define GPS_UBRR ((F_CPU / (16UL * GPS_BAUD)) - 1)

// Predefined route coordinates (latitude and longitude)
const float route[] = {
    34.0206, -118.2867, // Point 1: USC Campus
    34.0215, -118.2850, // Point 2: Another location around USC
    34.0220, -118.2870, // Point 3: Another location around USC
};

// Number of points in the route
const uint8_t route_size = sizeof(route) / sizeof(route[0]) / 2;

// GPS data structure
struct GPSData {
    float latitude;
    float longitude;
    uint8_t isValid;
};

// USART initialization
void usart_init() {
    // Set baud rate
    UBRR0H = (uint8_t)(GPS_UBRR >> 8);
    UBRR0L = (uint8_t)GPS_UBRR;
    
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// USART transmit function
void usart_transmit(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer
    UDR0 = data;
}

// USART receive function
uint8_t usart_receive() {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)));
    // Get and return received data
    return UDR0;
}

// Parse NMEA sentence
uint8_t parse_nmea(char* sentence, struct GPSData* gps) {
    // Check if it's a GGA sentence
    if (strncmp(sentence, "$GPGGA", 6) == 0) {
        char* token = strtok(sentence, ",");
        uint8_t field = 0;
        
        while (token != NULL) {
            switch (field) {
                case 2: // Latitude
                    if (strlen(token) > 0) {
                        gps->latitude = atof(token) / 100.0;
                    }
                    break;
                case 4: // Longitude
                    if (strlen(token) > 0) {
                        gps->longitude = atof(token) / 100.0;
                    }
                    break;
                case 6: // Fix quality
                    gps->isValid = (atoi(token) > 0);
                    break;
            }
            token = strtok(NULL, ",");
            field++;
        }
        return 1;
    }
    return 0;
}

// Calculate distance between two points using Haversine formula
float calculate_distance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * M_PI / 180.0;
    float dLon = (lon2 - lon1) * M_PI / 180.0;
    
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
              sin(dLon/2) * sin(dLon/2);
              
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return 6371000 * c; // Earth's radius in meters
}

// Check if current location is near any route point
uint8_t check_route_proximity(struct GPSData* current) {
    for (uint8_t i = 0; i < route_size; i++) {
        float distance = calculate_distance(
            current->latitude, current->longitude,
            route[2*i], route[2*i+1]
        );
        
        if (distance <= 15.0) { // 515 meters threshold
            return i + 1; // Return point number (1-based)
        }
    }
    return 0;
}

int main(void) {
    // Initialize USART
    usart_init();
    
    // Buffer for NMEA sentences
    char nmea_buffer[100];
    uint8_t buffer_index = 0;
    
    // GPS data structure
    struct GPSData current_gps = {0, 0, 0};
    
    while (1) {
        // Read GPS data
        if (UCSR0A & (1 << RXC0)) {
            char c = usart_receive();
            
            if (c == '$') {
                buffer_index = 0;
            }
            
            if (buffer_index < sizeof(nmea_buffer) - 1) {
                nmea_buffer[buffer_index++] = c;
                
                if (c == '\n') {
                    nmea_buffer[buffer_index] = '\0';
                    
                    if (parse_nmea(nmea_buffer, &current_gps)) {
                        if (current_gps.isValid) {
                            uint8_t near_point = check_route_proximity(&current_gps);
                            if (near_point) {
                                // Point reached, take appropriate action
                            }
                        }
                    }
                    buffer_index = 0;
                }
            }
        }
    }
    
    return 0;
}
