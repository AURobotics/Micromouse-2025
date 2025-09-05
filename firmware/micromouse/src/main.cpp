#include <Arduino.h>


// Define LED (emitter) pins
uint8_t ir_trig_pins[] = {36, 40, 34, 35, 33, 18};

// Define receiver (detector) pins
uint8_t ir_adc_pins[] = {1};


void setup() {
    Serial.begin(115200);
    for (const auto pin : ir_trig_pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
    for (const auto pin : ir_adc_pins)
        pinMode(pin, INPUT);
    analogReadResolution(10);
}

void loop() {
    // digitalWrite(ir_trig_pins[0], HIGH);
    // delay(10);
    // int true_value = analogRead(ir_adc_pins[0]);
    // digitalWrite(ir_trig_pins[0], LOW);
    // delay(10);
    // digitalWrite(ir_trig_pins[0], HIGH);
    // u_long init_time = micros();
    // while (abs(analogRead(ir_adc_pins[0]) - true_value) >= 10) {
    //     ;
    // }
    // u_long final_time = micros();
    // Serial.println(String(final_time - init_time));
    // delay(1000);
    // static int ir_output[6] = {};
    //
    // for (uint8_t i = 0; i < 6; i++) {
    //     digitalWrite(ir_trig_pins[i], HIGH);
    //     delayMicroseconds(200);
    //     ir_output[i] = analogRead(ir_adc_pins[i]);
    //     digitalWrite(ir_trig_pins[i], LOW);
    // }
    // Serial.printf("%d, %d, %d, %d, %d, %d, %d", )
}