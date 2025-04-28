#include "ruff_route.h"

// GPS data structure
struct GPSData {
    float lat;
    float lon;
    uint8_t valid;
};

// Initialize UART for GPS communication
void init_uart(void) {
    // Set baud rate
    uint16_t ubrr = ((F_CPU/(16UL*GPS_BAUD))-1);
    UBRR0H = ubrr >> 8;
    UBRR0L = ubrr;
    
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Check if data is available to read
static inline uint8_t usart_rx_ready(void) {
    return UCSR0A & (1 << RXC0);
}

// Receive a character
static inline char usart_receive(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

// Parse GGA sentence
static uint8_t parse_gga(char *s, struct GPSData *g) {
    if (strncmp(s, "$GPGGA", 6)) return 0;
    
    char *tok = strtok(s, ",");
    uint8_t i = 0;
    
    while (tok) {
        switch (i) {
            case 2:  // Latitude
                g->lat = atof(tok) / 100.0;
                break;
            case 4:  // Longitude
                g->lon = atof(tok) / 100.0;
                break;
            case 6:  // Fix quality
                g->valid = (atoi(tok) > 0);
                break;
        }
        tok = strtok(NULL, ",");
        i++;
    }
    return 1;
}

// Calculate distance between two points using Haversine formula
float calculate_distance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * M_PI / 180;
    float dLon = (lon2 - lon1) * M_PI / 180;
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
              sin(dLon/2) * sin(dLon/2);
    return 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
}

// Calculate bearing between two points
float calculate_bearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * M_PI / 180;
    float y = sin(dLon) * cos(lat2 * M_PI / 180);
    float x = cos(lat1 * M_PI / 180) * sin(lat2 * M_PI / 180) -
              sin(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * cos(dLon);
    return atan2(y, x) * 180 / M_PI;
}

void update_gps(void) {
    static char buf[100];
    static uint8_t bi = 0;
    struct GPSData gps = {0};
    
    // Read NMEA sentence
    if (usart_rx_ready()) {
        char c = usart_receive();
        if (c == '$') bi = 0;
        if (bi < sizeof(buf) - 1) buf[bi++] = c;
        if (c == '\n') {
            buf[bi] = 0;
            if (parse_gga(buf, &gps) && gps.valid) {
                current_latitude = gps.lat;
                current_longitude = gps.lon;
            }
            bi = 0;
        }
    }
} 