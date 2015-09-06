#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
const int analogInPin = A0;  // Analog input pin that the measures the voltage
int sensorValue = 0;        // value read from the pot
double voltage;
byte i2cdata[2];

void setup() {
pinMode(13, OUTPUT);
Serial.begin(115200); // start serial for output
// initialize i2c as slave
Wire.begin(SLAVE_ADDRESS);

// define callbacks for i2c communication
Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Ready!");
}

void loop() {
  delay(100);
  sensorValue = analogRead(analogInPin);
  voltage = (3.3 * sensorValue / 1024) * ((3.6 + 1)/1);

  // DEBUGGING
  //  Serial.print("sensor Voltage = " );
  //  Serial.println(voltage);
}

// callback for received data
void receiveData(int byteCount){

while(Wire.available()) {
number = Wire.read();
Serial.print("data received: ");
Serial.println(number);

if (number == 1){

if (state == 0){
digitalWrite(13, HIGH); // set the LED on
state = 1;
}
else{
digitalWrite(13, LOW); // set the LED off
state = 0;
}
}
}
}

// callback for sending data
void sendData(){
  sensorValue = analogRead(analogInPin);
  
  // DEBUGGING
  //voltage = (3.3 * sensorValue / 1024) * ((1.5 + 1)/1);
  // Wire.write(sensorValue>>8);  // Second value [buf[1]]
  // Wire.write(sensorValue & 0xff);            // First value [buf[0]]

  i2cdata[0] = sensorValue>>8;
  i2cdata[1] = sensorValue & 0xff;
  Wire.write(i2cdata, 2);

  // DEBUGGING
  // Serial.print("data sending: ");
  // Serial.print(sensorValue>>8); Serial.print("\t");
  //Serial.println(sensorValue & 0xff); 
}
