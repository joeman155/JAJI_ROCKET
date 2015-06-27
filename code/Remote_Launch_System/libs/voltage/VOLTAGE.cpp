#include "Arduino.h"
#include "VOLTAGE.h"

VOLTAGE::VOLTAGE() {
	active = true;
}

VOLTAGE::~VOLTAGE() {}

void VOLTAGE::setup(int pin, float r1, float r2) {
   pinMode(pin, INPUT); //make that pin an INPUT

   v_pin         = pin;
   v_resistor_r1 = r1;
   v_resistor_r2 = r2;

}

void VOLTAGE::enable() {
	active = true;
}


void VOLTAGE::disable() {
	active = false;
}


boolean VOLTAGE::isEnabled() {
	return active;
}

void VOLTAGE::read() {

   voltage =  v_arduino_voltage * ((float ) analogRead(v_pin) / 1024) * ((v_resistor_r1 + v_resistor_r2)/v_resistor_r2);  // r1, r2 as a divider

}

float VOLTAGE::value() {
	return voltage;
}