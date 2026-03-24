#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define NUM_LEDS 1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

int brightness = 100; // Fixed brightness (0–255)

void setup() {
  strip.begin();
  strip.show(); // Initialize LED off
}

void loop() {
  // Set constant brightness (white light)
  strip.setPixelColor(0, strip.Color(brightness, brightness, brightness));
  strip.show();

  delay(500);
}