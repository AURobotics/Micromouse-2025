#include "Arduino.h"
#include "FastLED.h"

#define NUM_LEDS 3
#define DATA_PIN 39

CRGB leds[NUM_LEDS];

void setup() {
    // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
    // leds[0]= CRGB::Red; leds[1]=CRGB::Green; leds[2]=CRGB::Blue;
    // FastLED.show();
    pinMode(39, OUTPUT);
    digitalWrite(39, LOW);
    analogWrite(39, 255);
}

void loop() {

}