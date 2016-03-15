/*
  Controller0
  Loaded on to the main Arduino board that gets gyro data, sends to 
  other Arduino to save.
  Also controls stepper motors
  

  created 16 Feb 2016
  modified 16 Feb 2016
  by Joseph Turner
 */



#include <Wire.h>
#include <SPI.h>
#include "Adafruit_FRAM_SPI.h"

// FRAM
/* Example code for the Adafruit SPI FRAM breakout */
uint8_t FRAM_CS = 10;

//Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);  // use hardware SPI

uint8_t FRAM_SCK= 13;
uint8_t FRAM_MISO = 12;
uint8_t FRAM_MOSI = 11;
//Or use software SPI, any pins!
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);

uint16_t          addr = 0;
int fram_size = 8192;
boolean fram_installed;




// Gyroscope Variables
boolean gyroscope_installed = true;
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;
volatile boolean gotdata = false;  // Data needs to be retrieved from IMU
boolean is_processing = false;      // We are getting data RIGHT now and can't get MORE data if available
boolean dataneedsprocessing = false;  // Got some gyro data and now need to process
int gyroZero = 0;
int gyroHigh = 0;
int gyroLow = 0;
double factor = 0.007 * PI/180;    // Convert the raw 'digital' values to radians. We work in radians ONLY!  (0.0070 is for +-2000deg/sec - got from datasheet)
int gyro_measurement_count = 0;

// Measurements from the Gyroscope
double rotation_vx, rotation_vy, rotation_vz;
double old_rotation_vx, old_rotation_vy, old_rotation_vz;

// Calculated quantities
double rotation_ax, rotation_ay, rotation_az;

// STEPPER MOTOR
#define MOTOR1_DIRECTION 6
#define MOTOR1_STEP 8
#define MOTOR2_DIRECTION 7
#define MOTOR2_STEP 9

#define cw HIGH
#define ccw LOW

double s1_angle = 0;   // PI * 81/180;    // PI/4;      // 0.1;
double s2_angle = PI;  // PI * 99/180;    // 3 * PI/4;  // 0.4;


double max_torque;          // Maximum torque the motor can provide at maximum speed we expect
double torque_percent;      // How much of max_torque we want to use
double moment_of_inertia;   // Moment of Inertia of arm and weight
double steps_per_rotation;  // # of Steps per revolution
double mass_of_smoother;    // How much each sphererical smoother weights
double smoother_radius;     // How many meteres it is 
double mass_of_arm;
double distance_to_smoother; 
double max_acceleration;    // Maximum acceleration we can attain

long c0 = 11000;  // Initial timing, if starting from standstill
long cx;          // Current cx value
long cx_last;     // Last cx value
long last_time_fired;   // Last time a step was triggered
long next_time_fired;   // The next time a step needs to be triggered
unsigned long data_time;         // Time we receive interrrupt (Data is available)



// TIME TRACKING
long time;
long last_time;


// BALANCING VARIABLES
double upper_velocity_threshold;
double lower_velocity_threshold;
unsigned int smoother_step = 0;
double corrective_angle = 0;
double mid_point_angle = 0;
double mid_point_distance = 0;
double move_to_neutral_distance = 0;
double s1_direction = 0, s2_direction = 0;
double intermediate_move = 0;
double s1_diff = 0, s2_diff = 0;
double final_angle_move = 0;
double resting_angle_move = 0;
int step_count = 0;      // Keep Track of where the S1 stepper motor smoother is.


// DEBUGGING
boolean debugging = true;


// Vectors
double y_vector[] = {0, 1, 0};     // Vector pointing in direction of rocket motion (up)
double vec3[] = {0, 0, 0};         // Result of cross product



// the setup function runs once when you press reset or power the board
void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  // Initialise FRAM
 if (fram.begin()) {
    Serial.println("Found SPI FRAM");
    fram_installed = true;
  } else {
    Serial.println("No SPI FRAM found ... check your connections\r\n");
    fram_installed = false;
  }
  
  // Quick method to dump rotational velocities - manually
  // dumpFRAM(); 
  
  
  // Initialise Pins
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT);
  
  // Initialise Gryoscope
  if (gyroscope_installed) {
    Serial.println("starting up L3G4200D");
    setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
    delay(1500); //wait for the sensor to be ready
    Serial.println("Start calibrating...");
    calibrate();
    Serial.println("Finished calibrating...");    
    delay(1000);
  }
  
  
  
  // INITIALIZE STEPPER MOTOR CONTROL PINS
  pinMode(MOTOR1_DIRECTION, OUTPUT);
  pinMode(MOTOR1_STEP, OUTPUT);
  pinMode(MOTOR2_DIRECTION, OUTPUT);
  pinMode(MOTOR2_STEP, OUTPUT); 
  
  
  
  digitalWrite(MOTOR1_DIRECTION, LOW);
  
  // Calculate Stepper timing constant c0
  // STEPPER MOTOR SY20STH30-0604A.pdf from POLOLU
  max_torque = 100;              // Gram . cm   (max torque at desired speed of 10,000pps = 3000 rpm)  // at 24 volts DC
  
  // SH2141-5541.pdf from POLOLU
  // max_torque = 40.77;            // Gram . cm   (max torque at desired speed of 10,000pps = 3000 rpm)

  // KATE System set-up  
  torque_percent = 75;           // Safety margin...don't want to exceed max_torque
  steps_per_rotation = 200;      // # of Steps per revolution
  mass_of_smoother = 0.025;      // Each smoother in kg
  smoother_radius  = 0.00905;    // Radius of mass of smoother  (calcualted assuming density = 8050kg/m^3)
  mass_of_arm = 0.01;            // How much mass of each arm weights
  distance_to_smoother = 0.02;   // How far from stepper motor axis to the smoother
  upper_velocity_threshold = 5 * PI/180;
  lower_velocity_threshold = 2 * PI/180;
  
  // KATE System set-up  - TESTING
  torque_percent = 75;           // Safety margin...don't want to exceed max_torque...By reducing from 75 to 50..it seems to give a bit more of a safey factor...
                                 // allowing for additional time...should some other force on system be acting on the mass.
                                 // At present we are using 12 volts...we might be able to increase this if we want to use a higher voltage power source
  steps_per_rotation = 200;      // # of Steps per revolution
  mass_of_smoother = 0.025;      // Each smoother in kg
  smoother_radius  = 0.00905;    // Radius of mass of smoother  (calcualted assuming density = 8050kg/m^3)
  mass_of_arm = 0.01;            // How much mass of each arm weights
  distance_to_smoother = 0.015;   // How far from stepper motor axis to the smoother
  upper_velocity_threshold = 5 * PI/180;
  lower_velocity_threshold = 2 * PI/180;  
  
  
  // This is quite a complicated equation. Below is a representation
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
  moment_of_inertia =  (mass_of_arm * distance_to_smoother * distance_to_smoother/3)  + (2 * mass_of_smoother * smoother_radius * smoother_radius/5) +  (mass_of_smoother * distance_to_smoother * distance_to_smoother);
  moment_of_inertia = 0.005 * 0.002 * 0.002 /2;   // Moment of inertia of shaft...assuming it is steel (densisty = 8050), is a cylinder of length 45mm and radius 2mm)
  moment_of_inertia = 200 * 0.005 * 0.002 * 0.002 /2;   // Just a calc I'm doing with arm...with no mass  THIS WORKS WELL...DO NO DELETE....works best with torque_percent = 50
  moment_of_inertia = 200 * 0.005 * 0.002 * 0.002 /2   + (0.025 *0.02 * 0.02);   // Just a calc I'm doing with arm...with AND  mass  THIS WORKS WELL...DO NO DELETE....works well wth torque_percent = 100...but reduce to 50 to ensure it can deal with external forces impacting on it
  
  // Deduce maximum rotational acceleration
  max_acceleration = ((torque_percent/(double) 100)* max_torque * (double) 0.001 * (double) 0.01 * (double) 9.81)/moment_of_inertia;
  
  // Calculate Initial timing constant
  c0 = 1000000 * pow(2 * 1.8 * PI/180/max_acceleration, 0.5);
  Serial.print("Moment of Inertia:    ");
  Serial.print((double) moment_of_inertia, 9);
  Serial.println(" kgm^2");
  Serial.print("Maximum Acceleration: ");
  Serial.print((double) max_acceleration);
  Serial.println(" rad/s/s");
  Serial.print("C0:                   ");
  Serial.print((double) c0);
  Serial.println(" cycles");  
  
  
  // Speed up process  
  if (fram_installed) {
    Serial.println("Clearing FRAM...");
    clearfram();
    Serial.println("FRAM cleared!");
  }


  // delay(5000);  

  if (gyroscope_installed) {
    attachInterrupt(0, gyro_data_available, RISING);  // Interrupt from Gyroscope
  }
  Serial.println("System Initialised!");  
  digitalWrite(5, HIGH);
}



// the loop function runs over and over again forever
void loop() {

  long currMicros = micros();
  
  // Data available!
  get_latest_rotation_data_all();  

  
  
  
  // Simulate rotation
  // rotation_vz = rotation_vz + 1 * PI/180;

/* 
// Do not need all this debugging 
  print_debug(debugging, "# Measurements: " + String(gyro_measurement_count));
  print_debug(debugging, "S1 ANGLE: " + String(s1_angle));
  print_debug(debugging, "S2 ANGLE: " + String(s2_angle));
*/  
  
  if (smoother_step == 0 && check_system_stability(rotation_vx, rotation_vy, rotation_vz, rotation_ax, rotation_ay, rotation_az)) {
    print_debug(debugging, "------------------------------ System needs stabilising ------------------------------");    
    print_time();
    print_debug(debugging, "Rotation speed: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));
    print_debug(debugging, "Rotation accel: " + String(rotation_ax) + ", " + String(rotation_ay) + ", " + String(rotation_az));  
  
    calculate_smoother_location(rotation_vx, rotation_vy, rotation_vz);
    smoother_step = 1;
  }
  
  if (smoother_step > 0) {
    print_debug(debugging, "Correction Angle: " + String(corrective_angle));
    smoother_step_processing();
  }
  
  /*
  // DO NOT NEED ALL THIS DEBUGGING
  print_debug(debugging, "Rotation speed: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));
  print_debug(debugging, "Rotation accel: " + String(rotation_ax) + ", " + String(rotation_ay) + ", " + String(rotation_az));  
*/
  
}




void gyro_data_available() {
  gotdata = true;
  data_time = micros();
  // digitalWrite(13, HIGH);
}




// GYROSCOPE RELATED ROUTINES
void getGyroValues(boolean exclude_y, boolean write_gyro_to_fram){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = 0x0;
  byte yLSB = 0x0;
  if (! exclude_y) {
    byte yMSB = readRegister(L3G4200D_Address, 0x2B);
    byte yLSB = readRegister(L3G4200D_Address, 0x2A);
    y = ((yMSB << 8) | yLSB);
  }   

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
  
  
  if (write_gyro_to_fram && fram_installed && addr < 8180) {
    byte val[10];
    val[0] = xMSB;
    val[1] = xLSB;
    val[2] = yMSB;
    val[3] = yLSB;
    val[4] = zMSB;
    val[5] = zLSB;  
    val[6] = data_time & 0xFF;
    val[7] = (data_time >>8 ) & 0xFF;
    val[8] = (data_time >>16) & 0xFF;
    val[9] = (data_time >>24) & 0xFF;     
  
    fram.writeEnable(true); 
    fram.write(addr, (uint8_t *) &val[0], 10);
    fram.writeEnable(false);   
    addr = addr + 10;
  }

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
  
  time = micros();
  rotation_ax = 1000000 * (vx - old_rotation_vx)/(time - last_time);
  old_rotation_vx = vx;
  
  if (! exclude_y) {
    rotation_ay = 1000000 * (vy - old_rotation_vy)/(time - last_time);
    old_rotation_vy = vy;  
  }
  
  rotation_az = 1000000 * (vz - old_rotation_vz)/(time - last_time);
  old_rotation_vz = vz;  
  
  
  last_time = time;
  
}


  
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
	double dist = pow(pow(vx, 2) + pow(vy, 2) + pow(vz, 2), 0.5);
								
				
	// Create Unit Correction'Vector'
	double corrective_rotation[] = {corrective_torque_direction[0]/dist, corrective_torque_direction[1]/dist, corrective_torque_direction[2]/dist};
				
	// Generate the thrust vector ... not caring about magnitude...only direction...in local coordinate system
	double thrust_vector[] = {0, 1, 0};
			
	// Determine the direction of the CG vector...needed to produce torque to oppose the current motion
        crossproduct(thrust_vector, corrective_rotation);
        double corrective_cg_vector[] = { vec3[0], vec3[1], vec3[2] };
						
	// Determine angle vector in X-direction in local reference frame... Use this later to find angle the CG vector makes with X-axis
	double x_vector[] = {1, 0, 0};		
				
	// Determine the angle this CG makes 
	double corrective_cg_vector_size = pow(pow(corrective_cg_vector[0], 2) + pow(corrective_cg_vector[1], 2) + pow(corrective_cg_vector[2], 2), 0.5 );
	corrective_angle = acos((x_vector[0] * corrective_cg_vector[0] + x_vector[1] * corrective_cg_vector[1] + x_vector[2] * corrective_cg_vector[2])/corrective_cg_vector_size);				
				
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
 	mid_point_angle     = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle)); // Find angle between smoothers... ALWAYS returns angle of 0..PI. This is
                                                                                                   // used to determine how far we need to move the smoothers to be back in the
                                                                                                   // neutral position.
                                                                                                   
	// We know that mid_point_angle MUST be less then OR Equal to 180 degrees BECAUSE this angle is got from dot-product				
	if (mid_point_angle < PI) {
		move_to_neutral_distance = (PI - abs(mid_point_angle))/2;
	} else {
		move_to_neutral_distance = 0;
	}                                                                                                   
                                  
                                                                                                  
        
        derive_direction();
        
        

        // **** CALCULATE WHERE MID POINT OF SMOOTHERS IS AND HOW FAR TO MOVE TO CORRECTIVE ANGLE ****
        //      WE DO THIS BECAUSE IT MIGHT BE QUICKER TO DO THIS INSTEAD OF GOING TO NEUTRAL POSITION 
        mid_point_distance  = (s1_angle + s2_angle)/2;	                                           // Angular distance mid-way between s1 and s2
        
        // We want the mid-point being the direction in which the acute angle (< 180) is directed
        // This calculation works because s1_angle, s2_angle are ALWAYS between 0 and 2PI
        if (abs(s2_angle - s1_angle) > PI) {
          mid_point_distance = mid_point_distance + PI;
        }
        mid_point_distance  = angle_reorg(mid_point_distance);                                     // Convert this distance to 0...2PI 
        

        intermediate_move = abs(corrective_angle - mid_point_distance);                             // Angular distance we must move to get mid-point at corrective angle
	if (intermediate_move > PI/2) {                                                             // If Greater than PI/2, then we are being in-efficient
                mid_point_distance = mid_point_distance + PI;                                       // So we consider moving opposite side (180 out of phase) 
                mid_point_distance = angle_reorg(mid_point_distance);                               // towards corrective angle direction
		intermediate_move = abs(corrective_angle - mid_point_distance);
	} 

      	smoother_step = 2;	

        Serial.println("INTERMEDIATE:    " + String(intermediate_move));
	Serial.println("S1/S2 Angle:       " + String(mid_point_angle));
	Serial.println("NEUTRAL MOV REQ'D: " + String(move_to_neutral_distance));
	Serial.println("s1_direction: "    + String(s1_direction));
	
	s1_angle = angle_reorg(s1_angle);
	s2_angle = angle_reorg(s2_angle);
  
}


// Move smoothers so they are 180 degrees out of phase if deemed necessary
void smoother_step_2() 
{

	if (move_to_neutral_distance <= 0) {
                // Already 180 degrees out of phase, so no need to move
		smoother_step = 3;
                print_debug(debugging, "No need to move to neutral position, already in neutral position");
	} else if (intermediate_move < PI/4) { 
                // Only a small movement required, so we will move straight to that position...this is because the increased speed in getting to the final
                // position outweighs the imbalances that might be caused.
		smoother_step = 3;
                print_debug(debugging, "Only a small movement required, so we will move straight to that position");                
        } else {
                // OK...so we have a large movement, and we can't afford to destabilise system, so we need to move to neutral position
                print_debug(debugging, "Neutral Move.");
                move_stepper_motors(s1_direction, s2_direction, move_to_neutral_distance, 0);
                smoother_step = 3;			
	}
}



void smoother_step_3() 
{
  
	// Find angle between the two smoothers...then halve...this is the mid-point
        // (We need to re-calculate because we may have moved in smoothers in previous step)
	mid_point_angle = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));  

        mid_point_distance  = (s1_angle + s2_angle)/2;	                                           // Angular distance mid-way between s1 and s2
        
        // We want the mid-point being the direction in which the acute angle (< 180) is directed
        // This calculation works because s1_angle, s2_angle are ALWAYS between 0 and 2PI
        if (abs(s2_angle - s1_angle) > PI) {
          mid_point_distance = mid_point_distance + PI;
        }
        mid_point_distance  = angle_reorg(mid_point_distance);                                     // Convert this distance to 0...2PI 
        
        // Deduce the distance we need to move
        intermediate_move = abs(corrective_angle - mid_point_distance);                             // Angular distance we must move to get mid-point at corrective angle
	if (intermediate_move > PI/2) {                                                             // If Greater than PI/2, then we are being in-efficient
                mid_point_distance = mid_point_distance + PI;                                       // So we consider moving opposite side (180 out of phase) 
                mid_point_distance = angle_reorg(mid_point_distance);                               // towards corrective angle direction
		intermediate_move = abs(corrective_angle - mid_point_distance);
	} 
				
				
	// s1_direction = 1;  // 0 - No movement, 1 = CCW, 2 = CW
	// s2_direction = 1;  // 0 - No movement, 1 = CCW, 2 = CW
				
	if (intermediate_move <= PI/2 && corrective_angle <= mid_point_distance) {
		s1_direction = 2;
		s2_direction = 2;
	} else if (intermediate_move > PI/2 && corrective_angle <= mid_point_distance) {
		s1_direction = 1;
		s2_direction = 1;
		intermediate_move = PI - intermediate_move;
	} else if (intermediate_move <= PI/2 && corrective_angle > mid_point_distance) {
		s1_direction = 1;
		s2_direction = 1;
	} else if (intermediate_move > PI/2 && corrective_angle > mid_point_distance) {
		s1_direction = 2;
		s2_direction = 2;
		intermediate_move = PI - intermediate_move;
	}
				
				
//	Serial.println("Midpoint Angle: " + mid_point_angle);
//	Serial.out.println("Intermediate move (distance): " + intermediate_move);
//	Serial.out.println("S1 Direction: " + s1_direction);
//	Serial.out.println("S2 Direction: " + s2_direction);

	
	// Signal to code to go on to 'Intermediate' move
	smoother_step = 4;  
}



void smoother_step_4() 
{
	// Intermediate move - moving both weights together
	// intermediate_move = intermediate_move - interval.doubleValue() * s1.getMax_angular_speed();
        // Determine # of steps required for intermediate_move
			
	if (intermediate_move < 0) {				
		smoother_step = 5;
	} else {
          print_debug(debugging, "Intermediate Move: " + String(intermediate_move));
	  move_stepper_motors(s1_direction, s2_direction, intermediate_move, 0);
	  smoother_step = 5;
        }


        // CALCULATE THE FINAL MOVE
	s1_diff = s1_angle - corrective_angle;
	s2_diff = s2_angle - corrective_angle;
	

        double val = cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle);
        if (abs(val) > 1) {
          // Yes, we get rounding errors that result in val being slightly > 1 or slightly less then -1. We deal with them here. Else we get NAN errors.
          if (val > 1) { 
             val = 1;
          }
          if (val < -1) {
             val = -1;
          }
        }
        
	mid_point_angle = acos(val);  
        mid_point_distance  = (s1_angle + s2_angle)/2;	                                           // Angular distance mid-way between s1 and s2

        final_angle_move = abs(abs(corrective_angle - mid_point_distance) - mid_point_angle/2);
        
        print_debug(debugging, "Final Move: " + String(final_angle_move));

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
          s1_direction = 2; // CW
	  s2_direction = 1; // CCW
        } else if (zcross < 0 ) {
          s1_direction = 1; // CCW
	  s2_direction = 2; // CW
        } else  {
          s1_direction = 0;
          s2_direction = 0;
        }


          print_debug(debugging, "zcross:     " + String(zcross));
          print_debug(debugging, "S1 DIR:     " + String(s1_direction));
          print_debug(debugging, "S2 DIR:     " + String(s2_direction));          
          print_debug(debugging, "Final Move: " + String(final_angle_move));
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
		
	    Serial.println("NOT REDUCING VELOCITY. EITHER malfunction in code, or change in forces or we are now over correcting!");
			
// COMMENT this code in this IF blovk FOR TESTING WHEN NOT NOT EXCEPTING ACTUAL CORRECTION OF SYSTEM (because we 'simulate' a movement

	    // So. let's assume we are over-correcting
	    smoother_step = 7;
	    resting_angle_move = PI/2;
			
	    derive_direction();
                
        
/*
		
	    if (s2_angle >= s1_angle) {
		s1_direction = 1; // CW
		s2_direction = 2; // CCW
	    } else if (s2_angle < s1_angle) {
		s1_direction = 2; // CW
		s2_direction = 1; // CCW
	    }		
*/				
        }
				
	// If velocity < lower_velocity_threshold, then start to reduce acceleration
	if (abs(rotation_vx) <  lower_velocity_threshold && abs(rotation_vz) <  lower_velocity_threshold) {
		Serial.println("SUCCESSFULLY REDUCING VELOCITY! Need to ease back back");
	
	     smoother_step = 7;
             resting_angle_move = PI/2;
		
             derive_direction();

            
/*            
		if (s2_angle >= s1_angle) {
			s1_direction = 1; // CW
			s2_direction = 2; // CCW
		} else if (s2_angle < s1_angle) {
			s1_direction = 2; // CW
			s2_direction = 1; // CCW
		}
*/

	}				
				
}

void smoother_step_7() 
{
  print_debug(debugging, "Rest Move: " + String(resting_angle_move));
  move_stepper_motors(s1_direction, s2_direction, resting_angle_move, lower_velocity_threshold);
  smoother_step = 0;

}


// We always want to move the stepper motors 'together'. Though we might want to move in different or SAME directions
// We will want to move the same angle.
// If threshold == 0, then we ignore this
void move_stepper_motors(short s1_direction, short s2_direction, double angle, double threshold)
{
  // CALCULATE # OF STEPS
  int steps = round((angle * 180 / PI) / 1.8);
  boolean finished_pulse;
  int i = 0;
  double angle_moved = angle;
  // boolean notfinished = true;
  boolean first_triggered = true;
  long start_time = micros();
  
  // CODE HERE TO SET SMOOTHER STEPPER MOTOR DIRECTIONS
  s1_stepper_motor_direction(s1_direction);
  s2_stepper_motor_direction(s2_direction);
  

  // DO THE MOVE COMMANDS HERE - SPEED UP
  while (i < steps/2) { // && notfinished) {


      // Do first pulse ASAP...no waiting
      if (first_triggered == true) {
          pulse_motors();
          
      if (s1_direction == 2) {
    step_count++;} 
else {
step_count--;
}

          last_time_fired = micros();
          first_triggered = false;
          
          // CODE HERE TO DO THE STEP at JUST the right time
          next_time_fired = last_time_fired + c0;
          cx_last = c0;
          
          //Serial.print("STEP: " + String(i) + String("/") + String(steps/2) + String(", "));
      } else {
          finished_pulse = false;
          while (! finished_pulse) {
              unsigned long current_time = micros();
              if (current_time > next_time_fired) {
                  long actual_step = current_time - last_time_fired;  // Comment out to speed up routine!
                  last_time_fired = current_time;
                  pulse_motors();
                  
      if (s1_direction == 2) {
    step_count++;} 
else {
step_count--;
}                  
          
                  // CODE HERE TO DO THE STEP at JUST the right time
                  long next_step = calculate_stepper_interval(0, i);   
                  next_time_fired = last_time_fired + next_step;
                  
                  //Serial.print("STEP: " + String(i) + String("/") + String(steps/2) + String(", "));   // Comment out to speed up routine!
                  Serial.println(actual_step, DEC);   // Comment out to speed up routine!
            
                  finished_pulse = true;
                
                  // If there is data available...get it now...we have some spare time (until next pulse) to get it!
                  if (dataneedsprocessing) {
                     get_latest_rotation_data2(true);
                  } else {
                     get_latest_rotation_data1(true);
                  }
              }
          }              
    }
    
    
    
    
    
    i++;
    if (threshold > 0) {
       // Threshold value > 0, this means we should do some threshold checks.
       if (abs(rotation_vx) < threshold && abs(rotation_vz) < threshold) {
          // And if threshold is met, we need to calculate angle moved
          
          //         we need to slow down, hence factor of two...steps speeding up = steps slowing down
          //         360/200   (degrees per step) - 200 steps per revolution
          //         PI/180     Convert from degress to radians
          angle_moved = 2 * i * (360 / 200) * (PI/180);   // Total angle that is moved (speed up + slow down)
          break;
       }     
    }
  }
  
  int steps_remaining = i+1;
  first_triggered = true;
  i = 0;
  // DO THE MOVE COMMANDS HERE - TO SPEED DOWN
  while (i < steps_remaining) {
        
      finished_pulse = false;
      while (! finished_pulse) {
          unsigned long current_time = micros();
          if (current_time > next_time_fired) {
              long actual_step = current_time - last_time_fired;   // Comment out to speed up routine!
              last_time_fired = current_time;
              pulse_motors();
              
      if (s1_direction == 2) {
    step_count++;} 
else {
step_count--;
}              
                  
              // CODE HERE TO DO THE STEP at JUST the right time
              long next_step = calculate_stepper_interval(steps_remaining, i);   
              next_time_fired = last_time_fired + next_step;
                  
              //Serial.print("STEP: " + String(i) + String("/") + String(steps/2) + String(", "));
              Serial.println(actual_step, DEC);     // Comment out to speed up routine!
           
              finished_pulse = true;
            
              // If there is data available...get it now...we have some spare time (until next pulse) to get it!
              if (dataneedsprocessing) {
                 get_latest_rotation_data2(true);
              } else {
                 get_latest_rotation_data1(true);
              }     
          }
      } 
    
    i++;
  }
  
  
  // Finished our pulses, but we need to wait until Step motor has completely stopped.
  finished_pulse = false;
  while (! finished_pulse) {  
    unsigned long current_time = micros();
          if (current_time > next_time_fired) {
            finished_pulse = true;
            long actual_step = current_time - last_time_fired;
            Serial.println("Final Wait: " + String(actual_step));
          }
  }
  
  
  
  long end_time = micros();
  double move_time = (end_time - start_time) / (double) 1000000;
  Serial.print("MOVE TIME: ");
  printDouble(move_time, 10000);
  double move_speed = (double) (PI/3) * move_time / (double) angle_moved;
  Serial.print("MOVE SPEED: ");
  printDouble(move_speed, 10000);  
  Serial.print("Angle Moved: ");
  Serial.println(String(angle_moved));
  
  
   // We want to keep track of where the smoothers are...no feedback..we just count steps
   if (s1_direction == 1) {
     s1_angle = s1_angle + angle_moved;
     s1_angle = angle_reorg(s1_angle);
   } else if (s1_direction == 2) {
     s1_angle = s1_angle - angle_moved;
     s1_angle = angle_reorg(s1_angle);     
   }
   
   if (s2_direction == 1) {
     s2_angle = s2_angle + angle_moved;
     s2_angle = angle_reorg(s2_angle);
   } else if (s2_direction == 2) {
     s2_angle = s2_angle - angle_moved;
     s2_angle = angle_reorg(s2_angle);     
   }   
   
  print_debug(debugging, "S1 ANGLE: " + String(s1_angle));
  print_debug(debugging, "S2 ANGLE: " + String(s2_angle));   
  print_debug(debugging, "step_count: " + String(step_count));
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



void stepper_motor_direction(int motor, int direction)
{
  
  if (direction == 1) {
     digitalWrite(motor, cw);
  } else if (direction == 2) {
     digitalWrite(motor, ccw);
  }
}


void s1_stepper_motor_direction(int direction)
{
  // Serial.println("MOTOR1: " + String(direction));
  stepper_motor_direction(MOTOR1_DIRECTION, direction);
}

void s2_stepper_motor_direction(int direction)
{
  // Serial.println("MOTOR2: " + String(direction));
  stepper_motor_direction(MOTOR2_DIRECTION, direction);
}


// Provides the time to wait
// - updown - 1 for speeding up, 0 for speeding down
// - step   - The step number we are up to
long calculate_stepper_interval(int starting_step, int next_step)
{
  long cx;

  if (next_step - starting_step == 1) {
    cx = cx_last * 0.4142;
    cx_last = cx;
  } else if (next_step - starting_step == -1 ) {
    cx = cx_last / 0.4142;
  } else {
    cx = cx_last - (2 * cx_last)/(4 * (next_step - starting_step) + 1);
    cx_last = cx;
  }
  
  return cx;
}


void pulse_motors()
{
  PORTB = PORTB | B00000011;
  delayMicroseconds(1);     // Double what spec says we need min of 1 microsecond...
  PORTB = PORTB & B11111100;
  delayMicroseconds(1);     // Double what spec says we need min of 1 microsecond...  
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


void calibrate()
{
  for(int i = 0; i < 4000; i++)
  {
    getGyroValues(false, false);

    if(z > gyroHigh)
    {
      gyroHigh = z;
    }
    else if(z < gyroLow)
    {
      gyroLow = z;
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
    getGyroValues(false, true);
    
    // Calculate acceleration
    calculate_acceleration(rotation_vx, rotation_vy, rotation_vz, false);
    is_processing = false;
    
    gyro_measurement_count++;
  }
}  




// Get latest IMU data (if available)
void get_latest_rotation_data1(boolean exclude_y)
{
  
  if (gotdata && ! is_processing) {
    is_processing = true;
    
    gotdata = false;
    getGyroValues(exclude_y, false);
    
    is_processing = false;
    dataneedsprocessing = true;
  }
} 



// Get latest IMU data (if available)
void get_latest_rotation_data2(boolean exclude_y)
{
  
  if (dataneedsprocessing && ! is_processing) {
    is_processing = true;
    
    gyro_measurement_count++;
    
    // Get rotation rates in radians per second
    rotation_vx = x * factor;
    if (! exclude_y) {
      rotation_vy = y * factor;
    }
    rotation_vz = z * factor;
    
    // Calculate acceleration
    calculate_acceleration(rotation_vx, rotation_vy, rotation_vz, exclude_y);
    is_processing = false;
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
    for (uint16_t a = 0; a < 819; a=a+10) {
      d1 = fram.read8(a);
      Serial.println(d1, HEX);
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
      
      print_debug(debugging, "Rotation speed: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));
      // Serial.println(data_time, HEX);
      Serial.print(d4, HEX);
      Serial.print(d3, HEX);
      Serial.print(d2, HEX);
      Serial.println(d1, HEX);      
      
    }
    
    delay(1000000);
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
              s1_direction = 1; // CCW
	      s2_direction = 2; // CW
            } else if (zcross < 0) {
              s1_direction = 2; // CW
	      s2_direction = 1; // CCW
            }
}  


