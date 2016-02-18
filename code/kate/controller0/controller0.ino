/*
  Controller0
  Loaded on to the main Arduino board that gets gyro data, sends to 
  other Arduino to save.
  Also controls stepper motors
  

  created 16 Feb 2016
  modified 16 Feb 2016
  by Joseph Turner
 */



// Gyroscope Variables



#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;



// STEPPER MOTOR
#define MOTOR1_DIRECTION 9
#define MOTOR1_STEP 8
#define MOTOR2_DIRECTION 7
#define MOTOR2_STEP 6

// TIME TRACKING
unsigned long time;


// DEBUGGING
boolean debugging = false;



//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;  
unsigned long last_button_time = 0; 


// the setup function runs once when you press reset or power the board
void setup() {
  // Initialise the Gyroscope
  Wire.begin();
  setupL3G4200D(250); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  attachInterrupt(0, collect_gyro_data, FALLING);  // Interrupt from Gyroscope
  delay(1500); //wait for the sensor to be ready
  
  
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  
  // INITIALIZE STEPPER MOTOR CONTROL PINS
  pinMode(MOTOR1_DIRECTION, OUTPUT);
  pinMode(MOTOR1_STEP, OUTPUT);
  pinMode(MOTOR2_DIRECTION, OUTPUT);
  pinMode(MOTOR2_STEP, OUTPUT); 
  
  
  Serial.begin(9600);
  digitalWrite(MOTOR1_DIRECTION, LOW);
  Serial.println("Wait for it!");
  delay(3000);
  Serial.println("Done!");
  digitalWrite(MOTOR1_DIRECTION, HIGH);
  digitalWrite(MOTOR1_STEP, LOW);
  
}

// the loop function runs over and over again forever
void loop() {

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(100);              // wait for a second

  

}

void collect_gyro_data() {
  print_time();
  print_debug(debugging, "Collecting Gyroscope data. ");  
  // getGyroValues();
  String str = String("X: ") + x + String(", Y: ") + y + String(", Z: ") + z;
  print_debug(debugging, str);
  print_time();
}





// GYROSCOPE RELATED ROUTINES
void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down: (4 most right bits)
  //   0b0111
  //     || \\ 
  //  200Hz   BW = 70Hz
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b01111111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}



// Time Functions
void print_time() {
  
  time = micros();
  Serial.print("Time: ");
  Serial.println(time);
}

void print_debug(boolean debug, String str) {
  if (debug) {
    Serial.println(str);
  }
}
  
