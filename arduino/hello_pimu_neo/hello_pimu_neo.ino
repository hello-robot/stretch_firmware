
/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
    
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html
  
  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/


#include <Arduino.h>
#include "Common.h"

#define ARDUINO_SAMD_ADAFRUIT
#include <Adafruit_NeoPixel_ZeroDMA.h>
#define NEOPIXEL_PIN         32
#define NUM_PIXELS 4
Adafruit_NeoPixel_ZeroDMA pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB);


void setup()        // This code runs once at startup
{     
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(BUZZER, OUTPUT);
  pixels.begin(&sercom1, SERCOM1, SERCOM1_DMAC_ID_TX, NEOPIXEL_PIN, SPI_PAD_2_SCK_3, PIO_SERCOM_ALT);
  pixels.setBrightness(32);
  pixels.show();
}

void loop() {
  uint16_t i;

  // 'Color wipe' across all pixels
  for(uint32_t c = 0xFF0000; c; c >>= 8) { // Red, green, blue
    for(i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, c);
      pixels.show();
      delay(150);
    }
  }

  /*// Rainbow cycle
  uint32_t elapsed, t, startTime = micros();
  for(;;) {
    t       = micros();
    elapsed = t - startTime;
    if(elapsed > 5000000) break; // Run for 5 seconds
    uint32_t firstPixelHue = elapsed / 32;
    for(i=0; i<pixels.numPixels(); i++) {
      uint32_t pixelHue = firstPixelHue + (i * 65536L / pixels.numPixels());
      pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(pixelHue)));
    }
    pixels.show();
  }*/
}
