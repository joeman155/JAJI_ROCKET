/*
  Controller0 - KATE  (Kinetic Attitude Thruster Engine)
  Loaded on to the main Arduino board that gets gyro data, sends to 
  other Arduino to save.
  Also controls stepper motors
  

  created 16 Feb 2016
  modified 27 Mar 2016
  by Joseph Turner
  
  Named in memory of Aunti Kate who passed away on 17th of March 2016 at 8am EST.
 */



#include <Wire.h>
#include <SPI.h>
#include "Adafruit_FRAM_SPI.h"

// FRAM
/* Example code for the Adafruit SPI FRAM breakout */
uint8_t FRAM_CS = 10;

//Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);  // use hardware SPI

uint8_t FRAM_SCK  = 13;
uint8_t FRAM_MISO = 12;
uint8_t FRAM_MOSI = 11;
//Or use software SPI, any pins!
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);

uint16_t          addr = 0;
// int fram_size = 8192;
boolean fram_installed;




// Gyroscope Variables
boolean gyroscope_installed = true;
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define STATUS_REG 0x27
#define ZYXOR_REG 0b10000000
#define ZYXDA_REG 0b00001000
byte _buff[6];   // Used to get lots of data from Gyroscope at once!

unsigned int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;
double avg_x = 0, avg_y = 0, avg_z = 0;
double var_x = 0, var_y = 0, var_z = 0;
double angle_x = 0;
double angle_y = 0;
double angle_z = 0;
volatile boolean gotdata = false;  // Data needs to be retrieved from IMU
boolean is_processing = false;      // We are getting data RIGHT now and can't get MORE data if available
boolean dataneedsprocessing = false;  // Got some gyro data and now need to process
unsigned int dataprocessingstage = 0;
boolean dataprocessed = false;
int gyroZHigh = 0;
int gyroZLow = 0;
int gyroXHigh = 0;
int gyroXLow = 0;
boolean gyro_calibrated = false;
double factor = 0.07 * PI/180;    // Convert the raw 'digital' values to radians. We work in radians ONLY!  (0.070 is for +-2000deg/sec - got from datasheet)
int gyro_measurement_count = 0;
boolean is_first_iteration = true;  // Avoid first iteration.... tdiff is not 'right'

// Measurements from the Gyroscope
double rotation_vx, rotation_vy, rotation_vz;
double old_rotation_vx = 0;
double old_rotation_vy = 0;
double old_rotation_vz = 0;

// Calculated quantities
double rotation_ax, rotation_ay, rotation_az;

// STEPPER MOTOR
#define MOTOR1_DIRECTION 6
byte motor1_dir = B01000000;    // Used to turning bits HIGH/LOW fast!
#define MOTOR1_STEP 9
byte motor2_dir = B10000000;    // Used to turning bits HIGH/LOW fast!
#define MOTOR2_DIRECTION 7
#define MOTOR2_STEP 8


#define cw_motor HIGH
#define ccw_motor LOW

#define cw true
#define ccw false

boolean s2_motor_inverted = true;


double s1_angle = 0;   // PI * 81/180;    // PI/4;      // 0.1;
double s2_angle = PI;  // PI * 99/180;    // 3 * PI/4;  // 0.4;


double max_torque;          // Maximum torque the motor can provide at maximum speed we expect
double torque_percent;      // How much of max_torque we want to use
double moment_of_inertia;   // Moment of Inertia of arm and weight
double degrees_per_step;    // How many degrees the motor rotates per step
double mass_of_smoother;    // How much each sphererical smoother weights
double smoother_radius;     // How many meteres it is 
double mass_of_arm;         // Mass of arm
double arm_length;          // Lenght of arm. We treat arm as if  it is a Rod
double distance_to_smoother; 
double max_acceleration;    // Maximum acceleration we can attain


// BUFFER VARIABLES
int i_write = 0;          // Write position in the buffer
int i_read = 0;           // Read position in the buffer
int buffer_len = 50;
int buffer_high_waterlevel = 45; // Point at which we turn off calcs
int buffer_low_waterlevel = 20;  // Point at which we resume calcs
int buffer_level = 0;            // How far we are above the read level
boolean buffer_get_data = true;   // Variable set false when we rise above high water level and set true when we fall below
unsigned int buf[50];

// boolean pulse_firsttime;
boolean pulse_lasttime;
boolean timer_set = false;
unsigned int timer1;
int i_step = 0;           // Step we are up to
unsigned int c0;          // Initial timing, if starting from stand-still - note we calcualte this value in setup()
unsigned int c0_fast;     // Fast turn
unsigned int c0_normal;   // Normal speed
unsigned int c0_slow;     // Slow turn
unsigned int cx_min;      // Minimum wait time
//long cx_total = 0;
int cx;                   // Current cx value
int cx_last;              // Last cx value
long last_time_fired;     // Last time a step was triggered
long next_time_fired;     // The next time a step needs to be triggered
unsigned long data_time;  // Time we receive interrrupt (Data is available)
long tdiff;

// Pins
int LED_INDICATOR_PIN = 5;

// TIME TRACKING
long time;
long last_time;
long start_time;
long end_time;
long start_time1;
long end_time1;
// long ts1, ts2, ts3;



// BALANCING VARIABLES
double upper_velocity_threshold;
double lower_velocity_threshold;
unsigned int smoother_step = 0;
double corrective_angle = 0;
double mid_point_angle = 0;
double mid_point_distance = 0;
double move_to_neutral_distance = 0;
boolean s1_direction, s2_direction;
double intermediate_move = 0;
double final_angle_move = 0;
double resting_angle_move = 0;
// int step_count = 0;      // Keep Track of where the S1 stepper motor smoother is.




// DEBUGGING
boolean debugging    = false;
boolean info         = true  ;
boolean print_timing = true;


// Vectors
double thrust_vector[] = {0, 1, 0};
double x_vector[] = {1, 0, 0};
double y_vector[] = {0, 1, 0};     // Vector pointing in direction of rocket motion (up)
double vec3[]     = {0, 0, 0};     // Result of cross product



// the setup function runs once when you press reset or power the board
void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  // Initialise FRAM
 if (fram.begin()) {
    Serial.println("Found SPI FRAM");
    fram_installed = true;
  } else {
    Serial.println("No SPI FRAM found\r\n");
    fram_installed = false;
  }
  
  // Quick method to dump rotational velocities - manually
  // dumpFRAM(); 
  
  
  // Initialise Pins
  pinMode(2, INPUT);                    // Interrupts pin...for IMU
  pinMode(13, OUTPUT);                  // Debugging...but actually used by FRAM
  pinMode(LED_INDICATOR_PIN, OUTPUT);   // State indicator LED
  pinMode(A0, INPUT);                   // S1 Motor sensor (0 degrees)
  pinMode(A1, INPUT);                   // S1 Motor sensor (PI degrees)  
  pinMode(A2, INPUT);                   // S2 Motor sensor (0 degrees)
  pinMode(A3, INPUT);                   // S2 Motor sensor (PI degrees)    
  
  
  
  // Initialise Gryoscope and calibrate
  // NOTE: We don't use the values from the calibration just yet....
  if (gyroscope_installed) {
    Serial.println("Starting up L3G4200D");
    setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
    delay(2500); //wait for the sensor to be ready
  }
  
  // Enable interrupts
  if (gyroscope_installed) {
    attachInterrupt(0, gyro_data_available, RISING);  // Interrupt from Gyroscope
  }
  
  // Perform calibration
  if (gyroscope_installed) {   
    Serial.println("Start calibrating IMU...");
    calibrate_gyro();
    gyro_calibrated = true;
    Serial.println("Finished calibrating IMU.");    
    delay(1000);
  }
  
  
  // INITIALIZE STEPPER MOTOR CONTROL PINS
  pinMode(MOTOR1_DIRECTION, OUTPUT);
  pinMode(MOTOR1_STEP, OUTPUT);
  pinMode(MOTOR2_DIRECTION, OUTPUT);
  pinMode(MOTOR2_STEP, OUTPUT); 
 
  
  // Calculate Stepper timing constant c0
  // STEPPER MOTOR SY20STH30-0604A.pdf from POLOLU
/*  
  max_torque = 100;              // Gram . cm   (max torque at desired speed of 10,000pps = 3000 rpm)  // at 24 volts DC
  
  // SH2141-5541.pdf from POLOLU
  // max_torque = 40.77;            // Gram . cm   (max torque at desired speed of 10,000pps = 3000 rpm)

  // KATE System set-up  
  torque_percent = 75;           // Safety margin...don't want to exceed max_torque
  mass_of_smoother = 0.025;      // Each smoother in kg
  mass_of_arm = 0.01;            // How much mass of each arm weights
  distance_to_smoother = 0.02;   // How far from stepper motor axis to the smoother
  upper_velocity_threshold = 5 * PI/180;
  lower_velocity_threshold = 2 * PI/180;
*/  
  
  
  
  // KATE System set-up  - TESTING
  max_torque     = 100;          // Gram . cm   (max torque at desired speed of 10,000pps = 3000 rpm)  // at 24 volts DC
  torque_percent = 35;           // Safety margin...don't want to exceed max_torque...By reducing from 75 to 50..it seems to give a bit more of a safey factor...
                                 // allowing for additional time...should some other force on system be acting on the mass.
                                 // At present we are using 12 volts...we might be able to increase this if we want to use a higher voltage power source
  // SMOOTHER
  mass_of_smoother     = 0.023;  // Mass of each smoother (kg)
  smoother_radius      = 0.005;  // Bolts with two nuts... and approximation        (m)  -- IF A CYLINDER
  // smoother_radius  = 0.00905;   // Radius of mass of smoother  (calcualted assuming density = 8050kg/m^3) - IF A SPHERE
  distance_to_smoother = 0.02;   // How far from stepper motor axis to the smoother (m)
  // ARM
  mass_of_arm = 0.005;           // How much mass of each arm weights  (kg)
  arm_length  = 0.035;           // Approximate length of arm (m)
  
  upper_velocity_threshold = 25 * PI/180;
  lower_velocity_threshold = 5 * PI/180;  
  
  
  // This is quite a complicated equation. Below is a representation
  //       ||
  //       ||  
  //  axle ||             **
  //       ||           ******
  //       ||----------********    <<--- Smoother  (Assume it is a sphere....radius = 9mm)
  //            arm     ******
  //                      **
  //
  // We need to take into account:-
  //     - Moment of inertia of axle   (if known)
  //     - Moment of inertia of arm
  //     - Moment of inertia of smoother weight
  //
  //                         Moment of inertia of Arm rotation about axle                  Moment of inertia of spherical smoother                        Parallel axis thereom applied to Smoother                         
  /*
  moment_of_inertia =  (mass_of_arm * distance_to_smoother * distance_to_smoother/3)  + (2 * mass_of_smoother * smoother_radius * smoother_radius/5) +  (mass_of_smoother * distance_to_smoother * distance_to_smoother);
  moment_of_inertia = 0.005 * 0.002 * 0.002 /2;   // Moment of inertia of shaft...assuming it is steel (densisty = 8050), is a cylinder of length 45mm and radius 2mm)
  moment_of_inertia = 200 * 0.005 * 0.002 * 0.002 /2;   // Just a calc I'm doing with arm...with no mass  THIS WORKS WELL...DO NO DELETE....works best with torque_percent = 50
  moment_of_inertia = 200 * 0.005 * 0.002 * 0.002 /2   + (0.025 *0.02 * 0.02);   // Just a calc I'm doing with arm...with AND  mass  THIS WORKS WELL...DO NO DELETE....works well wth torque_percent = 100...but reduce to 50 to ensure it can deal with external forces impacting on it
  moment_of_inertia = 0  +  (1.5 * mass_of_arm * distance_to_smoother * distance_to_smoother)  + (mass_of_smoother * distance_to_smoother * distance_to_smoother);   // Just a calc I'm doing with arm...with AND  mass  THIS WORKS WELL...DO NO DELETE....works well wth torque_percent = 100...but reduce to 50 to ensure it can deal with external forces impacting on it
  */

  //                         Moment of inertia of Arm rotation about axle                  Moment of inertia of spherical smoother                        Parallel axis thereom applied to Smoother
  // NOTE HERE WE ASSUME SMOOTHER IS A CYLINDER WITH RADIUS 5 mm!!
  moment_of_inertia =  (mass_of_arm * arm_length * arm_length/3) + (0.5 * mass_of_smoother * smoother_radius * smoother_radius)   +  (mass_of_smoother * distance_to_smoother * distance_to_smoother);
   
  // Deduce maximum rotational acceleration
  max_acceleration = ((torque_percent/(double) 100)* max_torque * (double) 0.001 * (double) 0.01 * (double) 9.81)/moment_of_inertia;
  
  
  // Stepping characteristics of motor
  degrees_per_step = 0.45;   // 1/4 step
  
  
  
  // Calculate Initial timing constants - for various speeds
  c0 = 1000000 * pow(2 * degrees_per_step * PI/180/max_acceleration, 0.5);
  c0_normal = c0;
  c0_fast = c0/1.22;
  c0_slow = c0 * 1.22;
  
  
  // Min delay  (Unlikely to get this fast without motor slipping!!)
  cx_min = 250;
  
  Serial.print("gyroXLow:    ");
  Serial.println(gyroXLow);
  Serial.print("gyroXHigh:    ");
  Serial.println(gyroXHigh);
  Serial.print("gyroZLow:    ");
  Serial.println(gyroZLow);
  Serial.print("gyroZHigh:    ");
  Serial.println(gyroZHigh);
  
  Serial.print("average x: ");
  Serial.println(avg_x);
  Serial.print("average y: ");
  Serial.println(avg_y);  
  Serial.print("average z: ");
  Serial.println(avg_z);
  
  Serial.print("variance x: ");
  Serial.println(var_x);
  Serial.print("variance y: ");
  Serial.println(var_y);  
  Serial.print("variance z: ");
  Serial.println(var_z);
  
  Serial.println();  
  
  Serial.print("MoI:    ");
  Serial.print((double) moment_of_inertia, 9);
  Serial.println(" kgm^2");
  Serial.print("Max Accel: ");
  Serial.print((double) max_acceleration);
  Serial.println(" rad/s/s");
  Serial.print("C0:                   ");
  Serial.print(c0);
  Serial.println(" cycles");  
  Serial.print("CX Min:               ");
  Serial.print(cx_min);
  Serial.println(" cycles");    


  // Calibrate Smoothers (move them into position)
  Serial.println("Calibrate S1/S2");
  calibrate_smoothers();
  Serial.println("Finished");  
  

  // Speed up process  
  if (fram_installed) {
    Serial.println("Clearing FRAM");
    clearfram();
    Serial.println("FRAM cleared");
  }


  delay(1000);




  
  
  Serial.println("S1 ANGLE: " + String(s1_angle));
  Serial.println("S2 ANGLE: " + String(s2_angle));     
  
  Serial.println("System Initialised!");  
  digitalWrite(LED_INDICATOR_PIN, HIGH);
}



// the loop function runs over and over again forever
void loop() {

  long currMicros = micros();
  
  // Data available!
  get_latest_rotation_data_all();  
  
  // Simulate rotation - but ONLY if gyroscope disabled
  if (! gyroscope_installed) {
     rotation_vz = rotation_vz + 1 * PI/180;
  }

  // PRINT ORIENTATION
  /*
  double angle_x_deg = angle_x * 180/PI;
  double angle_y_deg = angle_y * 180/PI;
  double angle_z_deg = angle_z * 180/PI;  
  print_debug(info, "POS- X: " + String(angle_x_deg)     + ", Y: " + String(angle_y_deg) + ", Z: " + String(angle_z_deg)); 
  */
  
  if (smoother_step == 0 && check_system_stability(rotation_vx, rotation_vy, rotation_vz, rotation_ax, rotation_ay, rotation_az)) {
    digitalWrite(LED_INDICATOR_PIN, LOW);
    start_time1 = micros();
    
    calculate_smoother_location(rotation_vx, rotation_vy, rotation_vz);

    smoother_step = 1;
    if (info) {
      print_debug(info, "-- System needs stabilising --");    
      print_debug(info, "RS: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));
      print_debug(info, "RA: " + String(rotation_ax) + ", " + String(rotation_ay) + ", " + String(rotation_az));  
      print_time();    
      print_debug(info, "CA: " + String(corrective_angle));  
    }
  
  }
  
  if (smoother_step > 0) {
    
    smoother_step_processing();
  }
  
 
  if (print_timing) {
    if (end_time1 > 0) {
      long total_time = end_time1 - start_time1;
      Serial.print("TT: ");
      Serial.println(total_time, DEC);
      end_time1 = 0;
    }
  }
  
  
}



// Interrupt routine
void gyro_data_available() {
  gotdata = true;
  data_time = micros();
}




// GYROSCOPE RELATED ROUTINES
void getGyroValues(boolean write_gyro_to_fram)
{
  // Wait around until we know we can get data from Gyro
  byte statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  while(!(statusflag & ZYXDA_REG)) {   // || (statusflag & ZYXOR_REG)
     statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  }
  
  // Read data from gyro
  readFromGyro(L3G4200D_Address, 0x28 | 0x80, 6, _buff);

  // statusflag = readRegister(L3G4200D_Address, STATUS_REG);

  // Assemble data to get rotational speeds
  x = (((int)_buff[1]) << 8) | _buff[0];
  y = (((int)_buff[3]) << 8) | _buff[2];
  z = (((int)_buff[5]) << 8) | _buff[4];  
  
  // Zero values that are within zero value. (i.e. assume they are zero!)
  if (gyro_calibrated) {
     zeroGyro();
  }
  
  if (write_gyro_to_fram && fram_installed && addr < 8180) {
    byte val[10];
    val[0] = _buff[1];
    val[1] = _buff[0];
    val[2] = _buff[3];
    val[3] = _buff[2];
    val[4] = _buff[5];
    val[5] = _buff[4];  
    val[6] = data_time & 0xFF;
    val[7] = (data_time >>8 ) & 0xFF;
    val[8] = (data_time >>16) & 0xFF;
    val[9] = (data_time >>24) & 0xFF;     
  
    fram.writeEnable(true); 
    fram.write(addr, (uint8_t *) &val[0], 10);
    fram.writeEnable(false);   
    addr = addr + 10;
  } ;

}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down: (4 most right bits)
  //   0b0111
  //     || \\ 
  //  200Hz   BW = 70Hz
  // writeRegister(L3G4200D_Address, CTRL_REG1, 0b01111111);
  //
  //   0b0000
  //     || \\ 
  //  100Hz   BW = 12.5Hz
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);  
  

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);


  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:
  // Very most left bit 0x80 means do not overwrite
  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b10000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b10010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b10110000);
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


void readFromGyro(int deviceAddress, byte address, int num, byte _buff[])
{
  Wire.beginTransmission(deviceAddress); // start transmission to device
  Wire.write(address); // sends address to read from
  Wire.endTransmission(); // end transmission
 
  Wire.beginTransmission(deviceAddress); // start transmission to device
  Wire.requestFrom(deviceAddress, num); // request 6 bytes from device Registers: DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1
   
  int i = 0;
  while(Wire.available()) // device may send less than requested (abnormal)
  {
    _buff[i] = Wire.read(); // receive a byte
    i++;
  }
  if(i != num)
  {
 
  }
  Wire.endTransmission(); // end transmission
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
  if (! print_timing) return;
  
  long time = micros();
  double  time_display = time/(double) 1000000;
  Serial.print("Time: ");
  printDouble(time_display,10000);
}

void print_debug(boolean debug, String str) {
  if (debug) {
    Serial.println(str);
  }
}
  

  
void calculate_acceleration(double vx, double vy, double vz, boolean exclude_y)
{
  double vx_avg, vy_avg, vz_avg;
  time = micros();
  long tdiff = time - last_time;
  
  // Want to skip first data point... (tdiff is unreliable)
  if (! is_first_iteration) {
    // Calculate Average velocity over time interval
    vx_avg = (vx + old_rotation_vx)/2;
    vz_avg = (vz + old_rotation_vz)/2;  

    // Numerical integrate to get angle
    angle_x = angle_x + vx_avg * tdiff/1000000;
    angle_z = angle_z + vz_avg * tdiff/1000000;

    // Only get y value IF we want it!
    if (! exclude_y) {
       vy_avg = (vy + old_rotation_vy)/2;
       angle_y = angle_y + vy_avg * tdiff/1000000;
    }  
  } else {
    is_first_iteration = false;
  }
  
  // Calculate Acceleration
  rotation_ax = 1000000 * (vx - old_rotation_vx)/tdiff;
  old_rotation_vx = vx;
  
  if (! exclude_y) {
    rotation_ay = 1000000 * (vy - old_rotation_vy)/tdiff;
    old_rotation_vy = vy;  
  }
  
  rotation_az = 1000000 * (vz - old_rotation_vz)/tdiff;
  old_rotation_vz = vz;  
  
  last_time = time;
}




void calculate_acceleration1(double vx, double vy, double vz, boolean exclude_y)
{
  double vx_avg, vy_avg, vz_avg;

  
  // Want to skip first data point... (tdiff is unreliable)
  if (! is_first_iteration) {
    // Calculate Average velocity over time interval
    vx_avg = (vx + old_rotation_vx)/2;
    vz_avg = (vz + old_rotation_vz)/2;  

    // Numerical integrate to get angle
    angle_x = angle_x + vx_avg * tdiff/1000000;
    angle_z = angle_z + vz_avg * tdiff/1000000;

    // Only get y value IF we want it!
    if (! exclude_y) {
       vy_avg = (vy + old_rotation_vy)/2;
       angle_y = angle_y + vy_avg * tdiff/1000000;
    }  
  } else {
    is_first_iteration = false;
  }

}



void calculate_acceleration2(double vx, double vy, double vz, boolean exclude_y)
{
  
  // Calculate Acceleration
  rotation_ax = 1000000 * (vx - old_rotation_vx)/tdiff;
  old_rotation_vx = vx;
  
  if (! exclude_y) {
    rotation_ay = 1000000 * (vy - old_rotation_vy)/tdiff;
    old_rotation_vy = vy;  
  }
  
  rotation_az = 1000000 * (vz - old_rotation_vz)/tdiff;
  old_rotation_vz = vz;  
  
}




// Detect instability of the rocket  
boolean check_system_stability(double vx, double vy, double vz, double ax, double ay, double az)
{
  // Check if our velocity measurements exceed upper threshold
  if (abs(vx) > upper_velocity_threshold || abs(vz) > upper_velocity_threshold) {
    
    // Peform additional check to double check we need to make adjustments to CG
    if (
	(sgn(ax) * sgn(vx) == -1 || abs(vx) <  upper_velocity_threshold)
	&&
	(sgn(az) * sgn(vz) == -1 || abs(vz) < upper_velocity_threshold) 
	) {
	  return false;
	} 

    return true;
  }

  return false;
}



void calculate_smoother_location(double vx, double vy, double vz)
{
  	// Deduce where the smoother should be
	// theta = Math.atan(r.getAng_vz()/r.getAng_vx());
	// We Already have the Rotational velocity co-ordinate in the local system
	// RealVector corrective_torque_direction = utils.revolveVector(0, Math.PI,  0, rotation_velocity_local);
	double corrective_torque_direction[] = { -1 * vx , -1 * vy, -1 * vz};
								
				
	// Create Unit Correction'Vector'
        double corrective_rotation[] = {corrective_torque_direction[0], corrective_torque_direction[1], corrective_torque_direction[2]};
				
	// Generate the thrust vector ... not caring about magnitude...only direction...in local coordinate system
	// DECLARED AT BEGINNING
			
	// Determine the direction of the CG vector...needed to produce torque to oppose the current motion
        crossproduct(thrust_vector, corrective_rotation);
        double corrective_cg_vector[] = { vec3[0], vec3[1], vec3[2] };
						
	// Determine angle vector in X-direction in local reference frame... Use this later to find angle the CG vector makes with X-axis
	// DECLARED AT BEGINNING	
				
	// Determine the angle this CG makes 
	double corrective_cg_vector_size = pow(pow(corrective_cg_vector[0], 2) + pow(corrective_cg_vector[1], 2) + pow(corrective_cg_vector[2], 2), 0.5 );

        // Calculate dot product
        double dot_product = (x_vector[0] * corrective_cg_vector[0] + x_vector[1] * corrective_cg_vector[1] + x_vector[2] * corrective_cg_vector[2])/corrective_cg_vector_size;
        
        // Correct for any rounding issues (that make the abs(dot_product) > 1); Otherwise we get NAN errors!
        if (dot_product > 1) {
           dot_product = 1;
        } else if (dot_product < -1) {
           dot_product = -1;
        }
        
        // Deduce the angle!
	corrective_angle = acos(dot_product);				
				
	// Figure out if    0 < angle 180  OR   180 < angle < 360
	double zcross = x_vector[0] * corrective_cg_vector[2] - corrective_cg_vector[0] * x_vector[2];					
								
	// Need to get corrective_angle beteween 0 and 2 x PI. We use cross product above to sort this out.
	if (zcross > 0) {
		corrective_angle = 2 * PI - corrective_angle;
	}

	// Smoothers 180 degrees out of phase from direction of 'corrective CG vector'
	corrective_angle = corrective_angle + PI;
					
	// Make sure angle is between 0 and 6.28
	corrective_angle = angle_reorg(corrective_angle);			
  
}


void smoother_step_processing()
{
  if (smoother_step == 1) {
    smoother_step_1();
  } else if (smoother_step == 2) {
    smoother_step_2();
  } else if (smoother_step == 3) {
    smoother_step_3();
  } else if (smoother_step == 4) {
    smoother_step_4();
  } else if (smoother_step == 5) {
    smoother_step_5();
  } else if (smoother_step == 6) {
    smoother_step_6();
  } else if (smoother_step == 7) {
    smoother_step_7();
  } 
  
}


void smoother_step_1() 
{

        //   **** CALCULATE HOW FAR AND IN WHAT DIRECTIONS TO GET BACK TO NEUTRAL POSITION ****
        mid_point_angle = angle_between(s1_angle, s2_angle);  
        
        // We know that mid_point_angle MUST be less then OR Equal to 180 degrees BECAUSE this angle is got from dot-product
        move_to_neutral_distance = (PI - mid_point_angle)/2;                                                                                                                                                                                                     

        // **** CALCULATE WHERE MID POINT OF SMOOTHERS IS AND HOW FAR TO MOVE TO CORRECTIVE ANGLE ****
        //      WE DO THIS BECAUSE IT MIGHT BE QUICKER TO DO THIS INSTEAD OF GOING TO NEUTRAL POSITION 
        mid_point_distance  = (s1_angle + s2_angle)/2;	                                           // Angular distance mid-way between s1 and s2
        
        // We want the mid-point being the direction in which the acute angle (< 180) is directed
        // This calculation works because s1_angle, s2_angle are ALWAYS between 0 and 2PI
        if (abs(s2_angle - s1_angle) > PI) {
          mid_point_distance = mid_point_distance + PI;
        }
        mid_point_distance  = angle_reorg(mid_point_distance);                                     // Convert this distance to 0...2PI 
        

        intermediate_move = angle_between(corrective_angle, mid_point_distance);
	if (intermediate_move > PI/2) {                                                             // If Greater than PI/2, then we are being in-efficient
                mid_point_distance = mid_point_distance + PI;                                       // So we consider moving opposite side (180 out of phase) 
                mid_point_distance = angle_reorg(mid_point_distance);                               // towards corrective angle direction
                intermediate_move = angle_between(corrective_angle, mid_point_distance);
	} 

      	smoother_step = 2;	

/*
        Serial.println("INTERMEDIATE:    " + String(intermediate_move));
	Serial.println("S1/S2 Angle:       " + String(mid_point_angle));
	Serial.println("NEUTRAL MOV REQ'D: " + String(move_to_neutral_distance));
	Serial.println("s1_direction: "    + String(s1_direction));
*/
	
}


// Move smoothers so they are 180 degrees out of phase if deemed necessary
void smoother_step_2() 
{

	if (move_to_neutral_distance <= 0) {
                // Already 180 degrees out of phase, so no need to move
		smoother_step = 4;
                // No need to move to neutral position, already in neutral position
                // print_debug(debugging, "No need to move to neutral position, already in neutral position");
	} else if (intermediate_move < PI/4) { 
                // Only a small movement required, so we will move straight to that position...this is because the increased speed in getting to the final
                // position outweighs the imbalances that might be caused.
		smoother_step = 4;
                // Only a small movement required, so we will move straight to that position
                // print_debug(debugging, "Only a small movement required, so we will move straight to that position");                
        } else {
                // OK...so we have a large movement, and we can't afford to destabilise system, so we need to move to neutral position
                derive_direction(); 
                
                print_time();
                derive_speed(move_to_neutral_distance);
                print_debug(info, "Neutral Move");
                move_stepper_motors(s1_direction, s2_direction, move_to_neutral_distance, 0);
                smoother_step = 3;			
	}
}



  

// This is run if smoothers moved to get into neutral position. We will need to re-calc some values
void smoother_step_3() 
{
	// Find angle between the two smoothers...then halve...this is the mid-point
        // (We need to re-calculate because we may have moved in smoothers in previous step)
  	// mid_point_angle = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));  
        mid_point_angle =  angle_between(s1_angle, s2_angle); 

        mid_point_distance  = (s1_angle + s2_angle)/2;	                                           // Angular distance mid-way between s1 and s2
        
        // We want the mid-point being the direction in which the acute angle (< 180) is directed
        // This calculation works because s1_angle, s2_angle are ALWAYS between 0 and 2PI
        if (abs(s2_angle - s1_angle) > PI) {
          mid_point_distance = mid_point_distance + PI;
        }
        mid_point_distance  = angle_reorg(mid_point_distance);                                     // Convert this distance to 0...2PI 
        
        // Deduce the distance we need to move
        intermediate_move = angle_between(corrective_angle, mid_point_distance);
	if (intermediate_move > PI/2) {                                                             // If Greater than PI/2, then we are being in-efficient
                mid_point_distance = mid_point_distance + PI;                                       // So we consider moving opposite side (180 out of phase) 
                mid_point_distance = angle_reorg(mid_point_distance);                               // towards corrective angle direction
                intermediate_move = angle_between(corrective_angle, mid_point_distance);
	} 
				
						
				
	// Serial.println("step3: S1/S2 Angle:                  " + String(mid_point_angle));  
        // Serial.println("step3: mid_point_distance Angle:     " + String(mid_point_distance)); 
        // print_debug(debugging, "zcross:     " + String(zcross));
        // print_debug(debugging, "S1 DIR:     " + String(s1_direction));
        // print_debug(debugging, "S2 DIR:     " + String(s2_direction));   
        // Serial.println("step3:  IM: " + String(intermediate_move));
	
	// Signal to code to go on to 'Intermediate' move
	smoother_step = 4;  
}


// Perform immediate move (if required) and then calculate the final move.
void smoother_step_4() 
{
	// Intermediate move - moving both weights together
	   
    	
	if (intermediate_move < 0) {				
		smoother_step = 5;
	} else {
  
	  // s1_direction = 1;  // FALSE = CCW, TRUE = CW
	  // s2_direction = 1;  // FALSE = CCW, TRUE = CW
		

          // DIRECTION TO GET SMOOTHERS TO INTERMEDIATE POSITION - in FASTEST POSSIBLE WAY!
          // To assist us in finding directions to move the smoothers we need to get Cross product of the midpoint vector and the
          // Corrective direction vectore.
          // The direction this resultant vector...up (+ve y) or down (-ve y) tells us which way to rotate the smoothers
          // NOTE: The Smoothers ALWAYS move in opposite directions with respect to each other
          // If you need to get a bit of an idea as to how we came to this, see Intermediate_Move_Directions.xlsx
        

          double i_vector[] = {cos(mid_point_distance), 0, -sin(mid_point_distance)};
          double c_vector[]  = {cos(corrective_angle), 0, -sin(corrective_angle)};
        
          crossproduct(i_vector, c_vector);
        
          double zcross = y_vector[0] * vec3[0] + y_vector[1] * vec3[1] + y_vector[2] * vec3[2];
        
          // BASED ON SIGN OF DOT PRODUCT, WE KNOW WHICH DIRECTION TO MOVE SMOOTHERS
          if (zcross > 0 ) {
            s1_direction = cw;
	    s2_direction = cw;
          } else if (zcross < 0 ) {
            s1_direction = ccw;
	    s2_direction = ccw;
          } 
  
          print_time();
          derive_speed(intermediate_move);
          print_debug(info, "IM: " + String(intermediate_move));
	  move_stepper_motors(s1_direction, s2_direction, intermediate_move, 0);
	  smoother_step = 5;
        }


        // CALCULATE THE FINAL MOVE
        mid_point_angle =  angle_between(s1_angle, s2_angle);
        
        mid_point_distance  = (s1_angle + s2_angle)/2;	                                           // Angular distance mid-way between s1 and s2

        final_angle_move = angle_between(corrective_angle, s1_angle);
}



void smoother_step_5() 
{
		
	if (final_angle_move < 0) {
		smoother_step = 6;
		
	} else {
  
          // DIRECTION TO GET SMOOTHERS TO FINAL POSITION - in FASTEST POSSIBLE WAY!
          // To assist us in finding directions to move the smoothers we need to get Cross product of the s1 smoother vector and the
          // Corrective direction vectore.
          // The direction this resultant vector...up (+ve y) or down (-ve y) tells us which way to rotate the smoothers
          // NOTE: The Smoothers ALWAYS move in opposite directions with respect to each other
          // If you need to get a bit of an idea as to how we came to this, see Intermediate_Move_Directions.xlsx
        

          double s1_vector[] = {cos(s1_angle), 0, -sin(s1_angle)};
          double c_vector[]  = {cos(corrective_angle), 0, -sin(corrective_angle)};
        
          crossproduct(s1_vector, c_vector);
        
          double zcross = y_vector[0] * vec3[0] + y_vector[1] * vec3[1] + y_vector[2] * vec3[2];
        
          // BASED ON SIGN OF DOT PRODUCT, WE KNOW WHICH DIRECTION TO MOVE SMOOTHERS
          if (zcross > 0 ) {
            s1_direction = cw; 
	    s2_direction = ccw;
          } else if (zcross < 0 ) {
            s1_direction = ccw;
	    s2_direction = cw;
          } 

//          print_debug(debugging, "zcross:     " + String(zcross));
//          print_debug(debugging, "S1 DIR:     " + String(s1_direction));
//          print_debug(debugging, "S2 DIR:     " + String(s2_direction));  
          print_time();
          c0 = c0_normal;
          print_debug(info, "FM: " + String(final_angle_move));
          move_stepper_motors(s1_direction, s2_direction, final_angle_move, lower_velocity_threshold); 
          smoother_step = 6;
        }
}


void smoother_step_6() 
{
 
 	// First make sure that acceleration is in opposite direction of velocity (i.e. it is slowing down)
	if ((sgn(rotation_ax) * sgn(rotation_vx) != -1 && abs(rotation_vx) > upper_velocity_threshold) 
		|| 
	    (sgn(rotation_az) * sgn(rotation_vz) != -1 && abs(rotation_vz) > upper_velocity_threshold)) {
		
	    // print_debug(debugging, "NOT REDUCING VELOCITY. EITHER malfunction in code, or change in forces or we are now over correcting!");
	    print_debug(debugging, "Malfu or over correcting.!");		
        
// COMMENT this code in this IF blovk FOR TESTING WHEN NOT NOT EXCEPTING ACTUAL CORRECTION OF SYSTEM (because we 'simulate' a movement

	    // So. let's assume we are over-correcting
	    smoother_step = 7;

	    
            //   **** CALCULATE HOW FAR AND IN WHAT DIRECTIONS TO GET BACK TO NEUTRAL POSITION ****
            mid_point_angle = angle_between(s1_angle, s2_angle);
            resting_angle_move = (PI - mid_point_angle)/2;
			
	    derive_direction();
                			
        }
				
	// If velocity < lower_velocity_threshold, then start to reduce acceleration
	if (abs(rotation_vx) <  lower_velocity_threshold && abs(rotation_vz) <  lower_velocity_threshold) {
		print_debug(debugging, "Success. Easing back back");
	
	     smoother_step = 7;
             
             //   **** CALCULATE HOW FAR AND IN WHAT DIRECTIONS TO GET BACK TO NEUTRAL POSITION ****
             mid_point_angle = angle_between(s1_angle, s2_angle);

             // We know that mid_point_angle MUST be less then OR Equal to 180 degrees BECAUSE this angle is got from dot-product
             resting_angle_move = (PI - mid_point_angle)/2;
		
             derive_direction();
	}							
}


void smoother_step_7() 
{
  print_time();
  c0 = c0_normal;
  print_debug(info, "RM: " + String(resting_angle_move));
  move_stepper_motors(s1_direction, s2_direction, resting_angle_move, lower_velocity_threshold);
  smoother_step = 0;
  digitalWrite(LED_INDICATOR_PIN, HIGH);
  end_time1 = micros();
  
  // delay(10000000);
}



// We always want to move the stepper motors 'together'. Though we might want to move in different or SAME directions
// We will want to move the same angle.
// If threshold == 0, then we ignore this
void move_stepper_motors(boolean s1_direction, boolean s2_direction, double angle, double threshold)
{
  // CALCULATE # OF STEPS
  int steps = round((angle * 180 / PI) / degrees_per_step);
  int half_steps = steps/2;  
  boolean finished_pulse;
  double angle_moved = angle;
  int steps_moved = 0;
  
  // Old variables, used in old timing routine
  // long next_step;
  // long actual_step;
  // cx_total = 0;
  // boolean first_triggered = true;
  // int i = 0;  

  
  // BUFFER INITIALISATION
  i_write = 0;
  i_read  = 0;
  i_step  = 0;
  buffer_level = 0;
  int i_step_calc = 0;
  
  // CODE HERE TO SET SMOOTHER STEPPER MOTOR DIRECTIONS
  s1_stepper_motor_direction(s1_direction);
  s2_stepper_motor_direction(s2_direction);

  // DO THE MOVE COMMANDS HERE - SPEED UP
  finished_pulse = false;
  pulse_lasttime = false;

  

  // Serial.print("Steps to start: "); Serial.println(String(half_steps));
  // Serial.println();
  start_time = micros();
  while (! finished_pulse) { 
  
    /*
    Serial.print(i_read, DEC);
    Serial.print(" ");
    Serial.print(i_write);
    Serial.print(" ");
    Serial.println(buffer_level, DEC);
*/
    
    // Calculate timings and push on to buffer
    if (buffer_get_data && i_step_calc < half_steps) { 
        buf[i_write++] = calculate_stepper_interval_new_up(i_step_calc);
        
        if (i_write >  buffer_len - 1) i_write = 0;
        i_step_calc++; 
        buffer_level++;
    } 
    
    // Regulate calculation of next interval
    if (buffer_get_data) {
       if (buffer_level > buffer_high_waterlevel) {
          buffer_get_data = false;
       }
    } else {
       if (buffer_level < buffer_low_waterlevel) {
          buffer_get_data = true;   
       }
    }
    
    // JOE - There is probably an issue with this...because we might pulse motors BEFORE the last pulse has completed...i.e. we don't give stepper motor sufficient 
    // time to rest.
    // Do this AFTER we have had a chance to calculate the next timing...to better our chances of a smooth ride.
    if (i_step == 0) {
        pulse_motors();
        i_step++; 
    }    
    
    // Initialise next Interrupt
    if (! timer_set && i_step < half_steps) {
      timer_set = true;
      
      //cx_total = cx_total + buf[i_read];
      timer1 = 8 * buf[i_read++] - 1;
      //Serial.print("t1:"); 
      // Serial.println(buf[i_read]);
          
      cli();
      if (i_read == buffer_len) i_read = 0;
      
      i_step++; 
      buffer_level--;
//      Serial.println(buffer_level, DEC);            
      
      // TIMER1
      TCCR1A = 0;// set entire TCCR1A register to 0
      TCCR1B = 0;// same for TCCR1B
      TCNT1  = 0;//initialize counter value to 0
      // set compare match register for 1hz increments
      OCR1A = timer1;// = (8*10^6) / (164*1) - 1 (must be <65536)    ... This is just an example calc
      // turn on CTC mode
      TCCR1B |= (1 << WGM12);
      // No scaling
      TCCR1B |= (1 << CS10);

      // enable timer compare interrupt
      TIMSK1 |= (1 << OCIE1A);      
      
      sei(); 
     
      dataprocessed = false;
    }
    
    
    if (i_step >= half_steps) {
      finished_pulse = true;
    }

 
    // ONLY get data if we are 
    // 1. Wanting to test to see if we are hitting a thresold
    //  AND
    // 2. we have heaps of time (> 750microseconds)
    //  AND
    // 3. we haven't checked it in this 'step' (Restricting one check per step). Each step is only 0.45 degreees....so this is acceptable.
    if (threshold > 0 && 
        (
         buf[i_read] > 750 && ! dataprocessed
        )
       ) {

        // Because there is so much processing and we can't afford to too much CPU because we need 
        // to be ready to set the Timer Interrupt, we needed to break up the processing into 4 parts. 
        if (dataprocessingstage == 3) {
           get_latest_rotation_data4(true);
           dataprocessed = true;
        } else if (dataprocessingstage == 2) {
           get_latest_rotation_data3(true);
           dataprocessed = true;
        } else if (dataprocessingstage == 1) {
           get_latest_rotation_data2(true);
           dataprocessed = true;
        } else if (dataprocessingstage == 0) {
           get_latest_rotation_data1();
           dataprocessed = true;
        } else if (dataprocessingstage == 4) {

         
         // Threshold value > 0, this means we should do some threshold checks.
         if (abs(rotation_vx) < threshold && abs(rotation_vz) < threshold) {
            // print_debug(debugging, "Under Threshold. Slowing down.");
            // And if threshold is met, we need to calculate angle moved
          
            //         we need to slow down, hence factor of two...steps speeding up = steps slowing down
            //         degrees per step - whatever it is defined as
            //         PI/180     Convert from degress to radians
            //
            // NOTE: We moved this calculation till after we move Stepper motor. This calc is not required for
            // the successful functioning of the Stepper Motor.
            steps_moved = i_step;
            break;
         }
         dataprocessingstage = 0;
         dataprocessed = true;
       }     
    }
  
    
  }
  
  
  
  
/*  
  Serial.print("Start time: "); Serial.println(start_time);  
  Serial.print("Finish time: "); Serial.println(end_time);  
  tdiff = end_time - start_time;
  Serial.print("Time taken: "); Serial.println(tdiff);
  
  Serial.print("Steps moved: "); Serial.println(String(i_step));
  Serial.print("cx_total: "); Serial.println(cx_total);
  Serial.println();
  Serial.println();
*/


  // NOW WE WANT TO GO BACKWARDS. We want to use the timing values that we got before!
  // We use the buffer backawards!!
  buffer_get_data = true;                     // Very eagar to get data... assume we are on the way up...not way down in the buffer.
  int steps_remaining = i_step;               // steps - i_step;       // Remaining steps to do
  timer_set = false;                          // Ready to set and enable timer1
  finished_pulse = false;                     // Starting while loop all over again
  buffer_level = buffer_len - buffer_level;   // Calculating new buffer_level

  // How many steps we need to skip over...from where we start calculating values
  int i_step_calc_skip = buffer_level;
  
  // Set the read position back one (because we have advanced one in previous while loop)
  i_read--;
  if (i_read < 0) i_read = buffer_len - 1; 

  // The first delay in the slow down is the last one written....cx_last. So we need this.
  cx_last = buf[i_write];

  // i_write - Continue from where we were...but we go in opposite direction!  

  i_step_calc = 0;                            // Iterator....for getting step timing
  i_step = 0;                                 // Reset steps back to zero....this is number of steps we do
  

/*
  Serial.print("Steps to stop: "); Serial.println(steps_remaining, DEC);
  Serial.print("Buffer Level: ");  Serial.println(buffer_level);
  Serial.print("i_step_calc:  ");  Serial.println(i_step_calc);
  Serial.print("i_step_calc_skip:  ");  Serial.println(i_step_calc_skip);
  Serial.print("i_read:  ");  Serial.println(i_read);
  Serial.print("i_write:  ");  Serial.println(i_write);
  Serial.print("cx_last: "); Serial.println(cx_last);
  Serial.println();
*/
  
  while (! finished_pulse) { 
  
//    if (buffer_level < 2 && i_step >10) {
//      Serial.println("z");
//    }
        
    // Initialise next Interrupt
    if (! timer_set && i_step < steps_remaining-1) {
      timer_set = true;
      
      //cx_total = cx_total + buf[i_read];
      timer1 = 8 * buf[i_read--] - 1;
      // Serial.print("t2:"); 
      // Serial.println(buf[i_read]);

      if (i_step == steps_remaining - 2) {
        pulse_lasttime = true;
      }

      cli();
      buffer_level--;
      if (i_read < 0) i_read = buffer_len - 1;
      
      /*
      if (i_read == 0) {
        i_read = buffer_len - 1;
      } else {
        i_read--;
      }
      */
      i_step++; 
      
      // TIMER1
      TCCR1A = 0;// set entire TCCR1A register to 0
      TCCR1B = 0;// same for TCCR1B
      TCNT1  = 0;//initialize counter value to 0
      // set compare match register for 1hz increments
      OCR1A = timer1;// = (8*10^6) / (164*1) - 1 (must be <65536)    ... This is just an example calc
      // turn on CTC mode
      TCCR1B |= (1 << WGM12);
      // No scaling
      TCCR1B |= (1 << CS10);

      // enable timer compare interrupt
      TIMSK1 |= (1 << OCIE1A);      
      
      sei(); 
    }    
    
    if (i_step >= steps_remaining-1) {
      finished_pulse = true;
    }      
    
    // Calculate values
    if (buffer_get_data && (i_step_calc + i_step_calc_skip) < steps_remaining && i_step_calc_skip > 0 ) {
        // Serial.print("cx_last: "); Serial.println(cx_last);
        buf[i_write--] = calculate_stepper_interval_new_down(steps_remaining, i_step_calc, i_step_calc_skip); 
        if (i_write <  0) i_write = buffer_len - 1;
        // Serial.print(i_write); Serial.print("   -   "); Serial.print(steps_remaining); Serial.print("   -   "); Serial.print(i_step_calc); Serial.print("   -   ");Serial.print(i_step_calc_skip); Serial.print("   -   ");Serial.println(buf[i_write]);        
        i_step_calc++; 
        buffer_level++;
    } 
    
    // Regulate calculation of next interval
    if (buffer_get_data) {
       if (buffer_level > buffer_high_waterlevel) {
          buffer_get_data = false;
       }
    } else {
       if (buffer_level < buffer_low_waterlevel) {
          buffer_get_data = true;   
       }
    }

  }  
  
  
  if (info) {
    end_time = micros();
    double move_time = (end_time - start_time) / (double) 1000000;
    
    // If steps_moved > 0..then we didn't move the whole way. So now calculate steps moved.
    if (steps_moved > 0)  {
       angle_moved = 2 * steps_moved * degrees_per_step * (PI/180);   // Total angle that is moved (speed up + slow down)
    }
    // Serial.print("cx_total: "); Serial.println(cx_total);
    Serial.print("MT: ");
    printDouble(move_time, 10000);
    double move_speed = (double) (PI/3) * move_time / (double) angle_moved;
    Serial.print("MS: ");
    printDouble(move_speed, 10000);  
    Serial.print("AM: ");
    Serial.println(String(angle_moved));
  }
   
  
   // We want to keep track of where the smoothers are...no feedback..we just count steps
   if (! s1_direction) {
     s1_angle = s1_angle + angle_moved;
     s1_angle = angle_reorg(s1_angle);
   } else if (s1_direction) {
     s1_angle = s1_angle - angle_moved;
     s1_angle = angle_reorg(s1_angle);     
   }
   
   if (! s2_direction) {
     s2_angle = s2_angle + angle_moved;
     s2_angle = angle_reorg(s2_angle);
   } else if (s2_direction) {
     s2_angle = s2_angle - angle_moved;
     s2_angle = angle_reorg(s2_angle);     
   }   
   
  print_debug(info, "S1 ANGLE: " + String(s1_angle));
  print_debug(info, "S2 ANGLE: " + String(s2_angle));   
  // print_debug(debugging, "step_count: " + String(step_count));
}


static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

static double angle_reorg(double angle) {
	double new_angle;
	
	if (abs(angle) >= 2 * PI) {
		new_angle = 2 * PI * ((angle/(2 * PI)) - round(angle/(2 * PI)));
	} else {
		new_angle = angle;
	}
	
	if (new_angle < 0) {
		new_angle = new_angle + PI * 2;
	}
	return new_angle;
	
}

void crossproduct(double vec1[], double vec2[])
{        
        vec3[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
        vec3[1] = vec1[0] * vec2[2] - vec1[2] * vec2[0];
        vec3[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}



void stepper_motor_direction(byte motor, boolean direction)
{
  if (! direction) {
     PORTD = PORTD | motor;
  } else if (direction) {
     PORTD = PORTD & ~motor;
  }
}


void s1_stepper_motor_direction(boolean direction)
{
  stepper_motor_direction(motor1_dir, direction);
}

void s2_stepper_motor_direction(boolean direction)
{
  if (s2_motor_inverted) {
    stepper_motor_direction(motor2_dir, ! direction);
  } else {
    stepper_motor_direction(motor2_dir, direction);
  }
}



// Provides the time to wait
// - updown - 1 for speeding up, 0 for speeding down
// - step   - The step number we are up to
int calculate_stepper_interval_new_up(int next_step)
{
  int cx;

  if (next_step == 0) {
    cx = c0;
    cx_last = cx;
  } else if (next_step == 1) {
    cx = cx_last * 0.4142;
    cx_last = cx;
  } else {
    cx = cx_last - (2 * cx_last)/(4 * next_step + 1);
    cx_last = cx;
  }
  
  return cx;
}


// Provides the time to wait
// - updown - 1 for speeding up, 0 for speeding down
// - step   - The step number we are up to
int calculate_stepper_interval_new_down(int starting_step, int next_step, int step_skip)
{
  int cx;

  if (next_step  == 0) {
    cx = cx_last;
  } else if (next_step + step_skip - starting_step == -1) {
    cx = cx_last / 0.4142;
    cx_last = cx;
  } else {
    cx = cx_last - (2 * cx_last)/(4 * (next_step + step_skip - starting_step) + 1);
    cx_last = cx;
  }
  
  return cx;
}



/*
// Implement speed restriction...doesn't result in much slow down...but should reduce skipping of steps
int speed_limit(int speed)
{
  if (speed < cx_min) {
    return cx_min;
  }
  
  return speed;
}
*/


void pulse_motors()
{
  // First bit (bit on very right) is for D8 == step motor 2
  // Second bit (bit immediately to the left of the 'First bit') is for D9 == step motor 1  
  PORTB = PORTB | B00000011; 
  delayMicroseconds(1);       // Double what spec says we need min of 1 microsecond...
  PORTB = PORTB & B11111100;
  delayMicroseconds(1);       // Double what spec says we need min of 1 microsecond...  
}

void pulse_motor_s1()
{
  // First bit (bit on very right) is for D8 == step motor 2
  // Second bit (bit immediately to the left of the 'First bit') is for D9 == step motor 1  
  PORTB = PORTB | B00000010; 
  delayMicroseconds(1);       // Double what spec says we need min of 1 microsecond...
  PORTB = PORTB & B11111100;
  delayMicroseconds(1);       // Double what spec says we need min of 1 microsecond...   
}



void pulse_motor_s2()
{
  // First bit (bit on very right) is for D8 == step motor 2
  // Second bit (bit immediately to the left of the 'First bit') is for D9 == step motor 1  
  PORTB = PORTB | B00000001; 
  delayMicroseconds(1);       // Double what spec says we need min of 1 microsecond...
  PORTB = PORTB & B11111100;
  delayMicroseconds(1);       // Double what spec says we need min of 1 microsecond...   
}



void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
     frac = (val - int(val)) * precision;
   else
      frac = (int(val)- val ) * precision;
   int frac1 = frac;
   while( frac1 /= 10 )
       precision /= 10;
   precision /= 10;
   while(  precision /= 10)
       Serial.print("0");

   Serial.println(frac,DEC) ;
}


void calibrate_gyro()
{
  int i = 0;
  
  // Without this line, it gets 'stuck'
  getGyroValues(false);
  
  while (i < 500)
  {
     // delayMicroseconds(20000);
    
    if (gotdata && ! is_processing) {
       is_processing = true;
       getGyroValues(false);
       
       /*
       Serial.print("x = "); Serial.print(x); Serial.print("    ");
       Serial.print("y = "); Serial.print(y); Serial.print("    ");
       Serial.print("z = "); Serial.println(z);       
       */
       
       // Z
       if (z > gyroZHigh) {
          gyroZHigh = z;
       } else if(z < gyroZLow) {
          gyroZLow = z;
       }
    
       // X
       if (x > gyroXHigh) {
          gyroXHigh = x; 
       } else if (x < gyroXLow) {
          gyroXLow = x;
       }  
  
  
       // Get Average
       avg_x = (avg_x * i + x)/(i+1);
       avg_y = (avg_y * i + y)/(i+1);
       avg_z = (avg_z * i + z)/(i+1);   
   
       // Get Variance
       var_x = (var_x * i + (x * x))/(i+1);
       var_y = (var_y * i + (y * y))/(i+1);
       var_z = (var_z * i + (z * z))/(i+1);   
   
       i++;
       
      is_processing = false;
      // gotdata = false;
    }
  }
}



// Get latest IMU data (if available)
void get_latest_rotation_data_all()
{
    
  if (gotdata && ! is_processing) {
    is_processing = true;
    
    // Get rotation rates in radians per second
    rotation_vx = x * factor;
    rotation_vy = y * factor;
    rotation_vz = z * factor;
    
    gotdata = false;
    getGyroValues(true);
    
    // Calculate acceleration
    calculate_acceleration(rotation_vx, rotation_vy, rotation_vz, false);
    is_processing = false;
    
    gyro_measurement_count++;
  }
}  




// Get latest IMU data (if available)
void get_latest_rotation_data1()
{
  time = micros();
  tdiff = time - last_time;
  last_time = time;
  
  if (gotdata && ! is_processing) {
    is_processing = true;
    
    gotdata = false;
    getGyroValues(false);
    
    is_processing = false;
    dataprocessingstage++;
  }
} 



// Get latest IMU data (if available)
void get_latest_rotation_data2(boolean exclude_y)
{
  
  if (! is_processing) {
    is_processing = true;
    
    gyro_measurement_count++;
    
    // Get rotation rates in radians per second
    rotation_vx = x * factor;
    if (! exclude_y) {
      rotation_vy = y * factor;
    }
    rotation_vz = z * factor;
    
    
    is_processing = false;
    dataprocessingstage++;
  }
} 


// Get latest IMU data (if available)
void get_latest_rotation_data3(boolean exclude_y)
{
  
  if (! is_processing) {
    is_processing = true;
  
    // Calculate acceleration
    calculate_acceleration1(rotation_vx, rotation_vy, rotation_vz, exclude_y);  
  
    is_processing = false;
    dataprocessingstage++;
  }  
    
}


// Get latest IMU data (if available)
void get_latest_rotation_data4(boolean exclude_y)
{
  
  if (! is_processing) {
    is_processing = true;
    
    // Calculate acceleration
    calculate_acceleration2(rotation_vx, rotation_vy, rotation_vz, exclude_y);      
  
    is_processing = false;
    dataprocessingstage++;
  }    
}


void clearfram()
{
    for (uint16_t a = 0; a < 8192; a++) {
    fram.writeEnable(true);
    fram.write8(a, 0x00);
    fram.writeEnable(false);
  }
}




void dumpFRAM()
{
    byte xmsb, xlsb, ymsb, ylsb, zmsb, zlsb;
    byte d1, d2, d3, d4;
    // unsigned long data_time;
    for (uint16_t a = 0; a < 8180; a=a+10) {
      xmsb = fram.read8(a);
      xlsb = fram.read8(a+1);
    
      ymsb = fram.read8(a+2);
      ylsb = fram.read8(a+3);

      zmsb = fram.read8(a+4);
      zlsb = fram.read8(a+5);  
    
      d1 = fram.read8(a+6);
      d2 = fram.read8(a+7);
      d3 = fram.read8(a+8);
      d4 = fram.read8(a+9);      
      
      // data_time = (unsigned long) d1 | (unsigned long) (d2 << 8) | (unsigned long) (d3 << 16) | (unsigned long) (d4 << 24);
    
      x = ((xmsb << 8) | xlsb);
      y = ((ymsb << 8) | ylsb);
      z = ((zmsb << 8) | zlsb);
      
      rotation_vx = x * factor;
      rotation_vy = y * factor;
      rotation_vz = z * factor;
      
      print_debug(info, "RS: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));
      // Serial.println(data_time, HEX);
      
      // Print Time
      Serial.print(d4, HEX);
      Serial.print(d3, HEX);
      Serial.print(d2, HEX);
      Serial.println(d1, HEX);      
      
    }
    
    delay(10000000);
}




// DIRECTION TO GET SMOOTHERS TO REST/NEUTRAL POSITION - in FASTEST POSSIBLE WAY!
// To assist us in finding directions to move the smoothers we need to get Cross product of the two smoother angles
// and see which direction this vector is pointing...up (+ve y) or down (-ve y)
// NOTE: The Smoothers ALWAYS move in opposite directions
// If you need to get a bit of an idea as to how we came to this, see Intermediate_Move_Directions.xlsx
void derive_direction()
{      
            s1_angle = angle_reorg(s1_angle);
            s2_angle = angle_reorg(s2_angle);
    
            // Get vector equivalents that the s1/s2 smoothers make. We have negate the Z direction, because the angle goes in opposite direction
            // Remember the smoothers lie in the X-Z plane
            double s1_vector[] = {cos(s1_angle), 0, -sin(s1_angle)};
            double s2_vector[] = {cos(s2_angle), 0, -sin(s2_angle)};

            crossproduct(s1_vector, s2_vector);
        
            double zcross = y_vector[0] * vec3[0] + y_vector[1] * vec3[1] + y_vector[2] * vec3[2];
        
            // BASED ON SIGN OF DOT PRODUCT, WE KNOW WHICH DIRECTION TO MOVE SMOOTHERS
            if (zcross > 0) {
              s1_direction = ccw;
	      s2_direction = cw;
            } else if (zcross < 0) {
              s1_direction = cw;
	      s2_direction = ccw;
            }
}  


/*
void update_count()
{
  if (s1_direction == 2) {
    step_count++;
  } else {
    step_count--;
  }
}
*/


// The routine Assumes that angle1, angle2 are between 0 and 2PI
double angle_between(double angle1, double angle2)
{
  double angle;

  angle = abs(angle1 - angle2);  
  angle = angle_reorg(angle);
  
  if (angle > PI) {
   angle = 2 * PI -  angle;
  }
  
  return angle;
}



ISR(TIMER1_COMPA_vect){//timer0 interrupt - pulses motor
  timer_set = false;      // Allow next interrupt to be set.
  if (! pulse_lasttime)   pulse_motors();
  TIMSK1 &= ~_BV(OCIE1A); // Disable interrupt (only want this interrupt to occur ONCE!)
}



/*
void increment_i_write()
{
 if (i_write >=  buffer_len - 1) {
   i_write = 0;
 } else {
   i_write++;
 }
}


void decrement_i_write()
{
 if (i_write ==  0) {
   i_write = buffer_len - 1;
 } else {
   i_write--;
 }
}
*/


// Used to adjust speed....we want the smoothers to run at different speeds to try
// and ensure least amount of skipped steps.
void derive_speed(double angle)
{
  if (angle < 0.3) {
    c0 = c0_slow; 
   } else {
    c0 = c0_fast;
  }
}


// Rotate smoothers indivually, s2 first, then s1
// We rotate each smoother about axis twice. We look for ON.... then OFF
// We get a reading of the steps at each side one...then we move backwards to get into the correct position.
// Note: If the sensor starts off as high, we go for a second revolution.
void calibrate_smoothers()
{

 int i, steps_to_move_back;
 int step_on ;        // Where the sensor goes on.
 int step_off;        // Where the sensor goes off
 int middle_position; // Where we want to place the smoother
 int sensor_value;
 boolean found_position;  // Set false...until position is found.

 // S2 Motor
 i = 0;
 step_on = -1;
 step_off = -1;
 found_position = false;
 s2_stepper_motor_direction(ccw);
 while(i < 1600 && ! found_position) {
   pulse_motor_s2();
   delay(5); 
   i++;
   
   // Get value on Hall sensor.  A3 pin == S2 Sensor at PI
   sensor_value = digitalRead(A3);
   
   if (sensor_value == LOW && step_on < 0 && step_off < 0 && i > 64) { 
      step_on = i;
      // Serial.print("Step on: "); Serial.println(step_on);      
   }
   
   if (sensor_value == HIGH && step_on >= 0 && step_off < 0) {
      step_off  = i;
      // Serial.print("Step off: "); Serial.println(step_off);
   }
   
   if (step_on >= 0 && step_off >= 0) {
      middle_position = (step_on + step_off)/2;
      found_position = true;
      // Serial.print("Finished finding Step on and Step off. Middle Position: "); Serial.println(middle_position);
   }
   
 } 
  
 if (found_position) {
    steps_to_move_back = (i - middle_position);
    // Serial.print("Found position! Moving backward "); Serial.print(steps_to_move_back); Serial.println(" steps to it now");
    s2_stepper_motor_direction(cw);
    i = 0;
    while(i < steps_to_move_back) {
      pulse_motor_s2();
      delay(5);
      i++;
    }
    
    
 } else {
    Serial.println("Cal S2 Failed");
 }

  
  

 // S1 Motor
 i = 0;
 step_on = -1;
 step_off = -1;
 found_position = false;
 s1_stepper_motor_direction(ccw);
 while(i < 1600 && ! found_position) {
   pulse_motor_s1();
   delay(5); 
   i++;
   
   // Get value on Hall sensor.  A0 pin == S1 Sensor at 0 radians
   sensor_value = digitalRead(A0);
   
   if (sensor_value == LOW && step_on < 0 && step_off < 0 && i > 64) { 
      step_on = i;
      // Serial.print("Step on: "); Serial.println(step_on);      
   }
   
   if (sensor_value == HIGH && step_on >= 0 && step_off < 0) {
      step_off  = i;
      // Serial.print("Step off: "); Serial.println(step_off);
   }
   
   if (step_on >= 0 && step_off >= 0) {
      middle_position = (step_on + step_off)/2;
      found_position = true;
      // Serial.print("Finished finding Step on and Step off. Middle Position: "); Serial.println(middle_position);
   }
   
 } 
  
 if (found_position) {
    steps_to_move_back = (i - middle_position);
    // Serial.print("Found position! Moving backward "); Serial.print(steps_to_move_back); Serial.println(" steps to it now");
    s1_stepper_motor_direction(cw);
    i = 0;
    while(i < steps_to_move_back) {
      pulse_motor_s1();
      delay(5);
      i++;
    }
    
    
 } else {
    Serial.println("Cal S1 Failed");
 }


  
}



// Set to zero, if within low/high readings
void zeroGyro()
{
  
  if (x >= gyroXLow && x <= gyroXHigh) {
    x = 0;
  }
  
  if (z >= gyroZLow && z <= gyroZHigh) {
    z = 0;
  }
}
