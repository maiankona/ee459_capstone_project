//This allows the creation of graphics
#include <Adafruit_GFX.h>

//Helps with SPI communication
#include <Adafruit_ILI9341.h>

// Pin definitions [Update]
#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8

// Button pins (adjust these to match your circuit)
#define LEFT 2
#define RIGHT 3
#define SELECT 4

// Create an instance of the ILI9341 display
// tft --> the instance 
// TFT_CS --> chip select (im talking to you) 
// TFT_DC --> data/command (differentiates command and pixel information) 
// TFT_RST --> reset 
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Variables for speed and route
int speed = 1; // speed levels: 1=slow, 2=medium, 3=fast
String route = "None";
int currentState = 0; // 0=Menu, 1=Route Selection, 2=Speed Selection, 3=Running
int highlightedRoute = 1; // 1 for Route 1, 2 for Route 2

void setup() {
  //Initializes the pins for input
  DDRC |= (1 << LEFT)|(1 << RIGHT) | (1 << SELECT);
  tft.begin();
  tft.setRotation(3); // Adjust for your screen orientation
  tft.fillScreen(ILI9341_BLACK);

  // Initial menu display
  displayRouteSelection();
}

void loop() {
  // Route Selection: Left or Right button to select Route 1 or Route 2
  if (currentState == 1) {
    if (digitalRead(BUTTON_LEFT) == LOW || digitalRead(BUTTON_RIGHT) == LOW) {
      // Switch route based on button press
      if (digitalRead(BUTTON_LEFT) == LOW && highlightedRoute == 2) {
        highlightedRoute = 1; // Highlight Route 1
        delay(200); // debounce
        displayRouteSelection();
      }
      if (digitalRead(BUTTON_RIGHT) == LOW && highlightedRoute == 1) {
        highlightedRoute = 2; // Highlight Route 2
        delay(200); // debounce
        displayRouteSelection();
      }
    }
    // If select button is pressed, move to speed selection
    if (digitalRead(BUTTON_SELECT) == LOW) {
      currentState = 2; // Transition to speed selection
      delay(200); // debounce
      displaySpeedSelection();
    }
  }

  // Speed Selection: Left and Right buttons to adjust speed
  if (currentState == 2) {
    if (digitalRead(BUTTON_LEFT) == LOW) {
      if (speed > 1) {
        speed--;
        delay(200); // debounce
        displaySpeedSelection();
      }
    }
    if (digitalRead(BUTTON_RIGHT) == LOW) {
      if (speed < 3) {
        speed++;
        delay(200); // debounce
        displaySpeedSelection();
      }
    }
    if (digitalRead(BUTTON_SELECT) == LOW) {
      currentState = 3; // Move to running mode
      delay(200); // debounce
      displayRunningInfo();
    }
  }
}

void displayRouteSelection() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  
  tft.setCursor(50, 50);
  if (highlightedRoute == 1) {
    tft.setTextColor(ILI9341_RED); // Highlight Route 1 in red
  }
  tft.print("Route 1");
  
  tft.setCursor(50, 100);
  if (highlightedRoute == 2) {
    tft.setTextColor(ILI9341_RED); // Highlight Route 2 in red
  }
  tft.print("Route 2");

  // Reset text color to white for the next selections
  tft.setTextColor(ILI9341_WHITE);
}

void displaySpeedSelection() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(50, 50);
  tft.print("Route: ");
  tft.print(highlightedRoute == 1 ? "Route 1" : "Route 2");
  tft.setCursor(50, 100);
  tft.print("Speed: ");
  
  switch (speed) {
    case 1: tft.print("Slow"); break;
    case 2: tft.print("Medium"); break;
    case 3: tft.print("Fast"); break;
  }
}

void displayRunningInfo() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(50, 50);
  tft.print("Route: ");
  tft.print(highlightedRoute == 1 ? "Route 1" : "Route 2");
  tft.setCursor(50, 100);
  tft.print("Speed: ");
  
  switch (speed) {
    case 1: tft.print("Slow"); break;
    case 2: tft.print("Medium"); break;
    case 3: tft.print("Fast"); break;
  }
  tft.setCursor(50, 150);
  tft.print("Running...");
}
