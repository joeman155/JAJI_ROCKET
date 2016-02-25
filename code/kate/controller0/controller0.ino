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


// Gyroscope Variables
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;

// Measurements from the Gyroscope
float rotation_vx, rotation_vy, rotation_vz;
float old_rotation_vx, old_rotation_vy, old_rotation_vz;

// Calculated quantities
float rotation_ax, rotation_ay, rotation_az;

// STEPPER MOTOR
#define MOTOR1_DIRECTION 9
#define MOTOR1_STEP 8
#define MOTOR2_DIRECTION 7
#define MOTOR2_STEP 6

#define cw HIGH
#define ccw LOW

double s1_angle = 0;
double s2_angle = PI;
int s1_step = 0;      // Correlates to s1_angle
int s2_step = 100;    // Correlates to s2_angle


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



// TIME TRACKING
long time;
long last_time;


// BALANCING VARIABLES
unsigned int upper_velocity_threshold;
unsigned int lower_velocity_threshold;
unsigned int smoother_step = 0;
double corrective_angle = 0;
double move_to_neutral_distance = 0;
double s1_direction = 0, s2_direction = 0;
double intermediate_move = 0;
double s1_diff = 0, s2_diff = 0;
double final_angle_move = 0;
double resting_angle_move = 0;
double offset = 0.000; // 1/2 angle between final resting place of smoothers



// DEBUGGING
boolean debugging = true;



//variables to keep track of the timing of recent interrupts
double vec3[] = {0, 0, 0};
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
  
  
  Serial.begin(115200);
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
  upper_velocity_threshold = 5;
  lower_velocity_threshold = 2;
  
  // KATE System set-up  - TESTING
  torque_percent = 50;           // Safety margin...don't want to exceed max_torque...By reducing from 75 to 50..it seems to give a bit more of a safey factor...
                                 // allowing for additional time...should some other force on system be acting on the mass.
                                 // At present we are using 12 volts...we might be able to increase this if we want to use a higher voltage power source
  steps_per_rotation = 200;      // # of Steps per revolution
  mass_of_smoother = 0.025;      // Each smoother in kg
  smoother_radius  = 0.00905;    // Radius of mass of smoother  (calcualted assuming density = 8050kg/m^3)
  mass_of_arm = 0.01;            // How much mass of each arm weights
  distance_to_smoother = 0.015;   // How far from stepper motor axis to the smoother
  upper_velocity_threshold = 5;
  lower_velocity_threshold = 2;  
  
  
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
  moment_of_inertia = 200 * 0.005 * 0.002 * 0.002 /2;   // Just a calc I'm doing with arm...with no mass  THIS WORKS WELL...DO NO DELETE.
  
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
  
  
/*
  move_x(200);
  digitalWrite(MOTOR1_DIRECTION, HIGH);
  delay(1);    
  move_x(200);
  delay(1000000);
*/

  delay(5000);  
  
  
  
  Serial.println("System Initialised!");
  
  /*
  delay(3000);
  Serial.println("Done!");
  digitalWrite(MOTOR1_DIRECTION, HIGH);
  digitalWrite(MOTOR1_STEP, LOW);
  */
  
  
  
}

// the loop function runs over and over again forever
void loop() {
/*
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(100);              // wait for a second
*/


  print_time();
  rotation_vz = rotation_vz + 1 * PI/180;


  print_debug(debugging, "S1 ANGLE: " + String(s1_angle));
  print_debug(debugging, "S2 ANGLE: " + String(s2_angle));

  calculate_acceleration(rotation_vx, rotation_vy, rotation_vz);
  
  
  if (smoother_step == 0 && check_system_stability(rotation_vx, rotation_vy, rotation_vz, rotation_ax, rotation_ay, rotation_az)) {
    print_debug(debugging, "System needs stabilising");
    calculate_smoother_location(rotation_vx, rotation_vy, rotation_vz);
    smoother_step = 1;
  }
  
  if (smoother_step > 0) {
    print_debug(debugging, "Correction Angle: " + String(corrective_angle));
    smoother_step_processing();
  }
  
  print_debug(debugging, "Rotation speed: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));
  print_debug(debugging, "Rotation accel: " + String(rotation_ax) + ", " + String(rotation_ay) + ", " + String(rotation_az));  
  


  

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
  double  time_display = time/(double) 1000000;
  Serial.print("Time: ");
  printDouble(time_display,10000);
}

void print_debug(boolean debug, String str) {
  if (debug) {
    Serial.println(str);
  }
}
  
  
  
void calculate_acceleration(float vx, float vy, float vz)
{
  
  time = micros();
  rotation_ax = 1000000 * (vx - old_rotation_vx)/(time - last_time);
  rotation_ay = 1000000 * (vy - old_rotation_vy)/(time - last_time);
  rotation_az = 1000000 * (vz - old_rotation_vz)/(time - last_time);
  last_time = time;
  
  old_rotation_vx = vx;
  old_rotation_vy = vy;  
  old_rotation_vz = vz;  
  
}


  
boolean check_system_stability(float vx, float vy, float vz, float ax, float ay, float az)
{

  // Check if our velocity measurements exceed upper threshold
  if (abs(180 * vx/PI) > upper_velocity_threshold || abs(180 * vz/PI) > upper_velocity_threshold) {
    
    // Peform additional check to double check we need to make adjustments to CG
    if (
	(sgn(ax) * sgn(vx) == -1 || abs(180 * vx/PI) <  upper_velocity_threshold)
	&&
	(sgn(az) * sgn(vz) == -1 || abs(180 * vz/PI) < upper_velocity_threshold) 
	) {
	  return false;
	} 

    return true;
  }

  return false;
}



void calculate_smoother_location(float vx, float vy, float vz)
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
								
	// The smoothers are put 180 degrees out from the direction the CG vectors point in.
	// corrective_angle = corrective_angle + Math.PI;
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
 	double mid_point_angle = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));				
	mid_point_angle = angle_reorg(mid_point_angle);
		

	// We know that mid_point_angle MUST be less then 180 degrees BECAUSE this angle is got from dot-product				
	if (mid_point_angle < PI) {
		move_to_neutral_distance = (PI - mid_point_angle)/2;
	} else {
		move_to_neutral_distance = 0;
	}
				
	
	smoother_step = 2;
	
	if (angle_reorg(s2_angle) >= angle_reorg(s1_angle)) {
		s1_direction = 1; // CW
		s2_direction = 2; // CCW
	} else if (angle_reorg(s2_angle) < angle_reorg(s1_angle)) {
		s1_direction = 2; // CW
		s2_direction = 1; // CCW
	}				
	

	Serial.println("NEUTRALMIDPOINT: " + String(mid_point_angle));
	Serial.println("NEUTRAL: "         + String(move_to_neutral_distance));
	Serial.println("s1_direction: "    + String(s1_direction));
	
	s1_angle = angle_reorg(s1_angle);
	s2_angle = angle_reorg(s2_angle);
  
}



void smoother_step_2() 
{

	if (move_to_neutral_distance <= 0) {
		smoother_step = 3;
                print_debug(debugging, "No need to move to neutral position, already in neutral position");
	} else {
                print_debug(debugging, "Neutral Move.");
                move_stepper_motors(s1_direction, s2_direction, move_to_neutral_distance, 0);
                smoother_step = 3;			
	}


}



void smoother_step_3() 
{
  
	/// Find angle between the two smoothers...then halve...this is the mid-point
	double mid_point_angle = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));  
	double mid_point_direction = mid_point_angle / 2 + s1_angle;
				
	// Deduce the distance we need 
	intermediate_move = abs(corrective_angle - mid_point_direction);
				
	// If Greater than Pi, then we are being in-efficient
	if (intermediate_move >= PI) {
		intermediate_move = intermediate_move - PI;
	} 
				
				
	// s1_direction = 1;  // 0 - No movement, 1 = CCW, 2 = CW
	// s2_direction = 1;  // 0 - No movement, 1 = CCW, 2 = CW
				
	if (intermediate_move <= PI/2 && corrective_angle <= mid_point_direction) {
		s1_direction = 2;
		s2_direction = 2;
	} else if (intermediate_move > PI/2 && corrective_angle <= mid_point_direction) {
		s1_direction = 1;
		s2_direction = 1;
		intermediate_move = PI - intermediate_move;
	} else if (intermediate_move <= PI/2 && corrective_angle > mid_point_direction) {
		s1_direction = 1;
		s2_direction = 1;
	} else if (intermediate_move > PI/2 && corrective_angle > mid_point_direction) {
		s1_direction = 2;
		s2_direction = 2;
		intermediate_move = PI - intermediate_move;
	}
				
				
/*				
	Serial.println("Midpoint Angle: " + mid_point_angle);
	Serial.out.println("Intermediate move (distance): " + intermediate_move);
	Serial.out.println("S1 Direction: " + s1_direction);
	Serial.out.println("S2 Direction: " + s2_direction);
*/
	
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

	s1_diff = s1_angle - corrective_angle;
	s2_diff = s2_angle - corrective_angle;
					
	final_angle_move = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));  
	final_angle_move = final_angle_move / 2 - offset;

}



void smoother_step_5() 
{
		
	if (final_angle_move < 0) {
		smoother_step = 6;
		
	} else {
          // Move S1 around
	  if (s1_diff > PI) {
                // Move Smoother1 CCW
                s1_direction = 1;
	  } else if (s1_diff <= PI && s1_diff > 0) {
                // Move Smoother1  CW
                s1_direction = 2;
	  } else if (s1_diff < 0 && s1_diff > - PI){
                // Move Smoother1 CCW
                s1_direction = 1;
	  } else if (s1_diff <= -PI) {
                // Move Smoother1  CW
                s1_direction = 2;
	  } else { 
                s1_direction = 0;
	  }

				
				
	  // Move S2 around
	  if (s2_diff > PI) {
                // Move Smoother2 CCW
                s2_direction = 1;
	  } else if (s2_diff > 0 && s2_diff <= PI) {
		// Move Smoother2  CW
                s2_direction = 2;
	  } else if (s2_diff < 0 && s2_diff > - PI){
                // Move Smoother2 CCW
                s2_direction = 1;
	  } else if (s2_diff <= -PI) {
                // Move Smoother2 CW
                s2_direction = 2;
	  } else { 
		// System.out.println("No Movement required - s2");
                s2_direction = 0;
	  }

          print_debug(debugging, "Final Move." + String(final_angle_move));
          move_stepper_motors(s1_direction, s2_direction, final_angle_move, 0);
          smoother_step = 6;
          delay(10000000);
        }
}


void smoother_step_6() 
{
 
 	// First make sure that acceleration is in opposite direction of velocity (i.e. it is slowing down)
	if ((sgn(rotation_ax) * sgn(rotation_vx) != -1 && abs(180 * rotation_vx/PI) > upper_velocity_threshold) 
		|| 
	    (sgn(rotation_az) * sgn(rotation_vz) != -1 && abs(180 * rotation_vz/PI) > upper_velocity_threshold)) {
		
	    Serial.println("NOT REDUCING VELOCITY. EITHER malfunction in code, or change in forces or we are now over correcting!");
					
	    // So. let's assume we are over-correcting
	    smoother_step = 7;
	    resting_angle_move = PI/2;
			
	    s1_angle = angle_reorg(s1_angle);
	    s2_angle = angle_reorg(s2_angle);
		
	    if (s2_angle >= s1_angle) {
		s1_direction = 1; // CW
		s2_direction = 2; // CCW
	    } else if (s2_angle < s1_angle) {
		s1_direction = 2; // CW
		s2_direction = 1; // CCW
	    }					
	
        }
				
	// If velocity < lower_velocity_threshold, then start to reduce acceleration
	if (abs(180 * rotation_vx/PI) <  lower_velocity_threshold &&
			abs(180 * rotation_vz/PI) <  lower_velocity_threshold) {
		Serial.println("SUCCESSFULLY REDUCING VELOCITY! Need to ease back back");
	
		smoother_step = 7;
		resting_angle_move = PI/2;
			
	        s1_angle = angle_reorg(s1_angle);
	        s2_angle = angle_reorg(s2_angle);
		
		if (s2_angle >= s1_angle) {
			s1_direction = 1; // CW
			s2_direction = 2; // CCW
		} else if (s2_angle < s1_angle) {
			s1_direction = 2; // CW
			s2_direction = 1; // CCW
		}
	}				
				
}

void smoother_step_7() 
{
  print_debug(debugging, "Rest Move." + String(resting_angle_move));
  move_stepper_motors(s1_direction, s2_direction, resting_angle_move, 5);
  smoother_step = 0;

}


// We always want to move the stepper motors 'together'. Though we might want to move in different or SAME directions
// We will want to move the same angle.
// If threshold == 0, then we ignore this
void move_stepper_motors(short s1_direction, short s2_direction, double angle, double threshold)
{
  // CALCULATE # OF STEPS
  int steps = round((angle * 180 / PI) / 1.8);
  int i = 0;
  double angle_moved = angle;
  boolean notfinished = true;
  boolean first_triggered = true;
  long start_time = micros();
  
  // CODE HERE TO SET SMOOTHER STEPPER MOTOR DIRECTIONS
  s1_stepper_motor_direction(s1_direction);
  s2_stepper_motor_direction(s2_direction);
  
  
  // DO THE MOVE COMMANDS HERE - SPEED UP
  while (i <= steps/2 && notfinished) {
      // Serial.println("S:" + String(i));
      
      // Do first pulse ASAP...no waiting
      if (first_triggered == true) {
          pulse_motor(MOTOR1_STEP);
          pulse_motor(MOTOR2_STEP);
          last_time_fired = micros();
          first_triggered = false;
          
      
          // CODE HERE TO DO THE STEP at JUST the right time
          next_time_fired = last_time_fired + c0;
          cx_last = c0;
          
          // Serial.println("Moving Step: " + String(i) + String(" at time: ") + String(last_time_fired));
          Serial.println(String(last_time_fired));
          Serial.println(String(c0));
      } else {
          boolean finished_pulse = false;
          while (! finished_pulse) {
              unsigned long current_time = micros();
              if (current_time > next_time_fired) {
                  long actual_step = current_time - last_time_fired;
                  last_time_fired = current_time;
                  pulse_motor(MOTOR1_STEP);
                  pulse_motor(MOTOR2_STEP);
                  
                  // last_time_fired = micros();
          
                  // CODE HERE TO DO THE STEP at JUST the right time
                  long next_step = calculate_stepper_interval(0, i);   
                  // long next_step = calc_up(i);
                  // next_step = c0; cx_last = c0;
                  next_time_fired = last_time_fired + next_step;
                  
                  // Serial.println("Moving Step: " + String(i) + String(" at time: ") + String(last_time_fired));
                  // Serial.println(String(last_time_fired));
                  // Serial.println(String(next_step)+ String(", ") + String(actual_step));
                  Serial.println(actual_step, DEC);
            
                finished_pulse = true;
              }
          }        
                
    }
    
    
      
    i++;
    if (threshold > 0) {
       // Threshold value > 0, this means we should do some threshold checks.
       if (abs(180 * rotation_ax/PI) < threshold && abs(180 * rotation_az/PI) < threshold) {
          // And if threshold is met, we need to calculate angle moved
          angle_moved = i * 180 / PI / 1.8;
          notfinished = false;
          smoother_step = 10;
       }       
    }
  }
  
  int steps_remaining = i;
  first_triggered = true;
  i = 0;
  // DO THE MOVE COMMANDS HERE - SPEED DOWN
  while (i < steps_remaining) {
      // Serial.println("S:" + String(i));
    
      // Do first pulse ASAP...no waiting
      if (first_triggered == true) {
          pulse_motor(MOTOR1_STEP);
          pulse_motor(MOTOR2_STEP);
          last_time_fired = micros();
          first_triggered = false;
          
      
          // CODE HERE TO DO THE STEP at JUST the right time
          next_time_fired = last_time_fired + cx_last;
          
          // Serial.println("Moving Step: " + String(i) + String(" at time: ") + String(last_time_fired));
          Serial.println(String(last_time_fired));
          Serial.println(String(cx_last));
      } else {
          boolean finished_pulse = false;
          while (! finished_pulse) {
              unsigned long current_time = micros();
              if (current_time > next_time_fired) {
                  long actual_step = current_time - last_time_fired;
                  last_time_fired = current_time;
                  pulse_motor(MOTOR1_STEP);
                  pulse_motor(MOTOR2_STEP);
                  
                  // last_time_fired = micros();
          
                  // CODE HERE TO DO THE STEP at JUST the right time
                  long next_step = calculate_stepper_interval(steps_remaining, i);   
                  // long next_step = calc_down(i);
                  // next_step = c0; cx_last = c0;
                  next_time_fired = last_time_fired + next_step;
                  
                  // Serial.println("Moving Step: " + String(i) + String(" at time: ") + String(last_time_fired));
                  // Serial.println(String(last_time_fired));
                  // Serial.println(String(next_step)+ String(", ") + String(actual_step));
                  Serial.println(actual_step, DEC);
            
                finished_pulse = true;
              }
          }        
                
    }
    
    
    i++;
    
    /*
    // THRESHOLD CODE NOT APPLICABLE HERE
    // STILL SLOWING DOWN. CAN't JUST ESCAPE ROUTINE...let it go to completion.
    if (threshold > 0) {
       // Threshold value > 0, this means we should do some threshold checks...i.e. are we fixing things up?
       
       // And if threshold is met, we need to calculate angle moved
       angle_moved = angle_moved + i * 180 / PI / 1.8;
       
    }
    */
      
  }
  
  
  long end_time = micros();
  double move_time = (end_time - start_time) / (double) 1000000;
  Serial.print("MOVE TIME: ");
  printDouble(move_time, 10000);
  double move_speed = (double) 60 * move_time / (double) 90;
  Serial.print("MOVE SPEED: ");
  printDouble(move_speed, 10000);  
  
  
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
  Serial.println("MOTOR1: " + String(direction));
  stepper_motor_direction(MOTOR1_DIRECTION, direction);
}

void s2_stepper_motor_direction(int direction)
{
  Serial.println("MOTOR2: " + String(direction));
  stepper_motor_direction(MOTOR2_DIRECTION, direction);
}


// Provides the time to wait
// - updown - 1 for speeding up, 0 for speeding down
// - step   - The step number we are up to
long calculate_stepper_interval(int starting_step, int next_step)
{
  long cx;
  // double was_last_cx;
  // was_last_cx = cx_last;
  if (next_step - starting_step == 1) {
    cx = cx_last * 0.4142;
    cx_last = cx;
  } else if (next_step - starting_step == -1 ) {
    cx = cx_last / 0.4142;
  } else {
    cx = cx_last - (2 * cx_last)/(4 * (next_step - starting_step) + 1);
    cx_last = cx;
  }
  // Serial.println("STEP=" + String(next_step) + String(",last_cx=") + String(was_last_cx) + String(",cx=") + String(cx));
  
  return cx;
}

long calc_up(int next_step)
{
  long cx;
  double val1 = pow(next_step, 0.5);
  
  cx = cx_last * (pow(next_step + 1, 0.5) - val1)/(val1 - pow(next_step - 1, 0.5)); 
  Serial.println("STEP=" + String(next_step) + String(",cx=") + String(cx));
  cx_last = cx;
  return cx;
}


long calc_down(int next_step)
{
  long cx;
  

  double val1 = pow(next_step, 0.5);
  
  cx = cx_last * (val1 - pow(next_step - 1, 0.5))/(pow(next_step + 1, 0.5) - val1); 
  Serial.println("STEP=" + String(next_step) + String(",cx=") + String(cx));
  
  
  cx_last = cx;
  return cx;
}


// Pulse the motor
void pulse_motor (short motor)
{
  digitalWrite(motor, HIGH);
  delayMicroseconds(2);     // Double what spec says we need min of 1 microsecond...
  digitalWrite(motor, LOW);
  delayMicroseconds(2);     // Double what spec says we need min of 1 microsecond...
  
  
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



void move_x(int x)
{
  int i = 0;
  
  while (i < x) {
  pulse_motor(MOTOR1_STEP);
  delayMicroseconds(2000);
  
  i++;
  }
}
