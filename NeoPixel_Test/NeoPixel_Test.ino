#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define NEOPIXEL   6
#define NUMPIXELS 16

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 50
#define FADETIME 10
#define FASTFLASH 5


void setup() {
  pixels.begin();
  pixels.clear();
}

int count = 0;

void loop() {
  count++;
  if (count>23) {
    count = 1;
  }
  NeoDisplay(count);
  //pixels.setBrightness(255);
}

void NeoDisplay(int r) {
  switch (r) {
    case 1:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(0, 0, 0), DELAYVAL); // Black
      break;
    case 2:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(255, 0, 0), DELAYVAL); // Red
      break;
    case 3:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(0, 255, 0), DELAYVAL); // Green
      break;
    case 4:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(255, 255, 0), DELAYVAL); // 
      break;
    case 5:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(0, 0, 255), DELAYVAL); // Blue
      break;
    case 6:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(255, 0, 255), DELAYVAL); // 
      break;
    case 7:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(0, 255, 255), DELAYVAL); // 
      break;
    case 8:
      pixels.setBrightness(250);
      colorWipe(pixels.Color(255, 255, 255), DELAYVAL); // White
      break;
    case 10:
      fadein(pixels.Color(255, 0, 0),FADETIME);
      break;
    case 11:
      fadeout(pixels.Color(255, 0, 0),FADETIME);
      break;
    case 12:
      fadein(pixels.Color(0, 255, 0),FADETIME);
      break;
    case 13:
      fadeout(pixels.Color(0, 255, 0),FADETIME);
      break;
    case 14:
      fadein(pixels.Color(255, 255, 0),FADETIME);
      break;
    case 15:
      fadeout(pixels.Color(255, 255, 0),FADETIME);
      break;
    case 16:
      fadein(pixels.Color(0, 0, 255),FADETIME);
      break;
    case 17:
      fadeout(pixels.Color(0, 0, 255),FADETIME);
      break;
    case 18:
      fadein(pixels.Color(255, 0, 255),FADETIME);
      break;
    case 19:
      fadeout(pixels.Color(255, 0, 255),FADETIME);
      break;
    case 20:
      fadein(pixels.Color(0, 255, 255),FADETIME);
      break;
    case 21:
      fadeout(pixels.Color(0, 255, 255),FADETIME);
      break;
    case 22:
      fadein(pixels.Color(255, 255, 255),FADETIME);
      break;
    case 23:
      fadeout(pixels.Color(255, 255, 255),FADETIME);
      break;
  }
}

// 
void fadein(uint32_t c, uint8_t wait) {
  for(uint8_t x=0; x<200; x++) {
    for(uint16_t i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, c);
      pixels.setBrightness(x);
    }
    pixels.show();
    delay(wait);
  }
}

void fadeout(uint32_t c, uint8_t wait) {
  for(uint8_t x=200; x>0; --x) {
    for(uint16_t i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, c);
      pixels.setBrightness(x);
    }
    pixels.show();
    delay(wait);
  }
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c);
    pixels.show();
    delay(wait);
  }
}
