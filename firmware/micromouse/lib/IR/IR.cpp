#include <Arduino.h>

// Define LED (emitter) pins
int irLEDs[6] = {2, 3, 4, 5, 6, 7};

// Define receiver (detector) pins
int irReceivers[6] = {A0, A1, A2, A3, A4, A5};

void setup() {
    Serial.begin(9600);
    for (int i = 0; i < 6; i++) {
        pinMode(irLeds[i], OUTPUT);
        pinMode(irReceivers[i], INPUT);
        digitalWrite(irLEDs[i], LOW); // LEDs off initially
    }
}

void loop() {
    // Read each IR sensor
    for (int i = 0; i < 6; i++) {
        digitalWrite(irLEDs[i], HIGH);     // Turn on IR LED
        delayMicroseconds(200);            // small delay for to give time to led
        irValues[i] = analogRead(irReceivers[i]);  // read trigger
        digitalWrite(irLEDs[i], LOW);  //switch off led
    }
}
