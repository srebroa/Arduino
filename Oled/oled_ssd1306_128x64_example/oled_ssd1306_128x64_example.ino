#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

int elbow = 0;
int shoulder = 90;
int base = 180;

void setup() { 
  // Set up the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C addr 0x3D if not working use 0x3C (for the 128x64)
  display.setTextColor(WHITE);
}

void loop(){
   // draw scrolling text
  delay(1);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(25,0);
  display.println("instructables");
  display.println();
    
  display.print("E:"); // Elbow
  display.println(elbow);
  display.println();
  
  display.print("S:"); // Shoulder
  display.println(shoulder);
  display.println(); 
  
  display.print("B:"); // Base 
  display.println(base);
  display.display();
}

