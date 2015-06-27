#ifndef VOLTAGE_H
#define VOLTAGE_H

#include <Arduino.h>

class VOLTAGE {
  public:
	VOLTAGE();
	~VOLTAGE();
    void setup(int pin, float r1, float r2);
    void enable();
    void disable();
	void read();
	float value();
	boolean isEnabled();

  private:
    boolean active;
    int   v_pin;
    float v_resistor_r1;
    float v_resistor_r2;
    const float v_arduino_voltage = 5;
    float voltage;
};

#endif