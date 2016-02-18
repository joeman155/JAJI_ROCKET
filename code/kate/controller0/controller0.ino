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

double s1_angle = 0;
double s2_angle = PI;


// TIME TRACKING
unsigned long time;
unsigned long last_time;


// BALANCING VARIABLES
unsigned int upper_velocity_threshold = 5;
unsigned int lower_velocity_threshold = 2;
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

  print_time();
  rotation_vz = rotation_vz - 1 * PI/180;


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
  print_time();
  


  

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
	
/*
	System.out.println("NEUTRALMIDPOINT: " + mid_point_angle);
	System.out.println("NEUTRAL: " + move_to_neutral_distance);
	System.out.println("s1_direction: " + s1_direction);
*/	
	s1_angle = angle_reorg(s1_angle);
	s2_angle = angle_reorg(s2_angle);
  
}



void smoother_step_2() 
{

	if (move_to_neutral_distance <= 0) {
		smoother_step = 3;
	} else {
                print_debug(debugging, "Neutral Move.");
                move_stepper_motors(s1_direction, s2_direction, move_to_neutral_distance, 0);
                smoother_step = 3;
                
                
  
  /*
		move_to_neutral_distance = move_to_neutral_distance - interval.doubleValue() * s2.getMax_angular_speed();
		
		if (s1_direction == 1) {
			s1.setAng_y(s1_angle - interval.doubleValue() * s1.getMax_angular_speed());
			s2.setAng_y(s2_angle + interval.doubleValue() * s2.getMax_angular_speed());
		} else if (s1_direction == 2) {
			s1.setAng_y(s1_angle + interval.doubleValue() * s1.getMax_angular_speed());
			s2.setAng_y(s2_angle - interval.doubleValue() * s2.getMax_angular_speed());
		} else {
			System.out.println("Unusual state. Not able to move back to resting state");
		} 
*/
			
	}


	// System.out.println("BACK TO NEUTRAL S1 ANGLE: " + s1_angle);
	// System.out.println("BACK TO NEUTRAL S2 ANGLE: " + s2_angle);

}



void smoother_step_3() 
{
  
	/// Find angle between the two smoothers...then halve...this is the mid-point
	double mid_point_angle = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));  
	mid_point_angle = mid_point_angle / 2;
				
	// Deduce the distance we need 
	intermediate_move = abs(corrective_angle - mid_point_angle);
				
	// If Greater than Pi, then we are being in-efficient
	if (intermediate_move >= PI) {
		intermediate_move = intermediate_move - PI;
	} 
				
				
	// s1_direction = 1;  // 0 - No movement, 1 = CCW, 2 = CW
	// s2_direction = 1;  // 0 - No movement, 1 = CCW, 2 = CW
				
	if (intermediate_move <= PI/2 && corrective_angle < mid_point_angle) {
		s1_direction = 2;
		s2_direction = 2;
	} else if (intermediate_move > PI/2 && corrective_angle < mid_point_angle) {
		s1_direction = 1;
		s2_direction = 1;
		intermediate_move = PI - intermediate_move;
	} else if (intermediate_move <= PI/2 && corrective_angle > mid_point_angle) {
		s1_direction = 1;
		s2_direction = 1;
	} else if (intermediate_move > PI/2 && corrective_angle > mid_point_angle) {
		s1_direction = 2;
		s2_direction = 2;
		intermediate_move = PI - intermediate_move;
	}
				
				
/*				
	System.out.println("Midpoint Angle: " + mid_point_angle);
	System.out.println("Intermediate move (distance): " + intermediate_move);
	System.out.println("S1 Direction: " + s1_direction);
	System.out.println("S2 Direction: " + s2_direction);
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
          print_debug(debugging, "Intermediate Move." + String(intermediate_move));
	  move_stepper_motors(s1_direction, s2_direction, intermediate_move, 0);
	  smoother_step = 5;
        }

	s1_diff = s1_angle - corrective_angle;
	s2_diff = s2_angle - corrective_angle;
					
	final_angle_move = acos(cos(s1_angle) * cos(s2_angle) + sin(s1_angle) * sin(s2_angle));  
	final_angle_move = final_angle_move / 2 - offset;


	// System.out.println("S1 ANGLE: " + s1_angle);
	// System.out.println("S2 ANGLE: " + s2_angle);  
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
        }


					
	// System.out.println("FINAL S1 ANGLE: " + s1_angle);
	// System.out.println("FINAL S2 ANGLE: " + s2_angle);

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
	    resting_angle_move = PI;
			
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
  
  /*
	resting_angle_move = resting_angle_move - interval.doubleValue() * s1.getMax_angular_speed();
		
	System.out.println("S1:" + utils.angle_reorg(s1.getAng_y()));
	System.out.println("S2:" + utils.angle_reorg(s2.getAng_y()));
	if (s1_direction == 1) {
		s1.setAng_y(s1.getAng_y() - interval.doubleValue() * s1.getMax_angular_speed());
		s2.setAng_y(s2.getAng_y() + interval.doubleValue() * s2.getMax_angular_speed());
	} else if (s1_direction == 2) {
		s1.setAng_y(s1.getAng_y() + interval.doubleValue() * s1.getMax_angular_speed());
		s2.setAng_y(s2.getAng_y() - interval.doubleValue() * s2.getMax_angular_speed());
	} else {
		System.out.println("Unusual state. Not able to move back to resting state");
	}
			
	// System.out.println("RESTING S1 ANGLE: " + s1.getAng_y());
	// System.out.println("RESTING S2 ANGLE: " + s2.getAng_y());
		
	if (resting_angle_move <= 0 || (Math.abs(180 * rotation_acceleration_local.getEntry(0)/Math.PI) < 5 && Math.abs(180 * rotation_acceleration_local.getEntry(2)/Math.PI) < 5)) {
		set_course = 10;
		System.out.println("Back to a semi-stable state, " + resting_angle_move + ", " + Math.abs(rotation_acceleration_local.getEntry(0)) + ", " + Math.abs(rotation_acceleration_local.getEntry(2)));
	}  
*/
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
  
  // CODE HERE TO SET SMOOTHER STEPPER MOTOR DIRECTIONS
  
  
  // DO THE MOVE COMMANDS HERE
  while (i < steps) {
    i++;
    Serial.println("Moving Step: " + String(i));
    
    // CODE HERE TO DO THE STEP at JUST the right time
    
    if (threshold > 0) {
       // Threshold value > 0, this means we should do some threshold checks.
       
       // And if threshold is met, we need to calculate angle moved
       angle_moved = i * 180 / PI / 1.8;
    }
  }
  
  
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
