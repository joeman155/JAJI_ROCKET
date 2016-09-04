/*
  Controller1 - KATE  (Kinetic Attitude Thruster Engine)
  Loaded on to the main Arduino board that gets gyro data, sends to 
  other Arduino to save.
  Also controls servos

  // Installation Notes:
  - Use the modified version of Adafruit-BMP085-Library-master which has some functions to compute pressure and temperature

  Created 14 Aug 2016
  Modified 14 Aug 2016
  by Joseph Turner
  
  Named in memory of Aunti Kate who passed away on 17th of March 2016 at 8am EST.
 */


#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_FRAM_I2C.h"
#include <Adafruit_BMP085.h>

// Define functions
void fastBlinkLed(long);
void fastBlinkLed();
void setupIMU();
void imu_data_available();
void calibrate_imu();
void calibrate_weights();
void initialise_ap_timer(int timer1);



// VARIABLES - CONFIGURATION
// Overall control
boolean active_control = false;                 // Enable/Disable Stability sense
boolean air_pressure_sensor_enabled = false;    // Enable/disable the Air Pressure Sensor
boolean air_pressure_sensor_available = false;  // Indicates if Air Pressure sensor found
boolean imu_enabled = true;                     // Indicates that we want to use IMU (if present)
boolean imu_available = false;                  // Indicates if IMU available to use
boolean launch_begun  = false;                  // Indicates if launch has begun yet.


// FRAM
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t framAddr = 0;
boolean  fram_installed;
boolean  fram_start_recording = false;      // Is set false initially, to hold off any data recording.
                                            // We only start recording data when launch is detected.



// Air Pressure Sensor
Adafruit_BMP085 bmp;
uint32_t pressure = 0.;
float    temperature = 0.;
byte     rawApTData[11];
int      ap_measurement_count = 0;
volatile boolean gotAPdata = false;   // Data needs to be retrieved from Air Pressure Sensor
unsigned long ap_data_time;  // Time we receive interrrupt (Data is available)
#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3




// IMU Variables
MPU6050 accelgyro;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
byte    _buff[17];      // Used to get lots of data from IMU in one i2c call!





// MOTION SENSING VARIABLES
// Gyroscope Statistics
double avg_gx = 0, avg_gy = 0, avg_gz = 0;  // Deduced Average rotational rates     (STATS)
double var_gx = 0, var_gy = 0, var_gz = 0;  // Deduced Variance rotational rates    (STATS)
double avg_ax = 0, avg_ay = 0, avg_az = 0;  // Deduced Average acceleration rates   (STATS)
double var_ax = 0, var_ay = 0, var_az = 0;  // Deduced Variance acceleration rates  (STATS)

// Gyroscope
int    gx, gy, gz;                          // Raw values from gyroscope
double angle_x = 0;                         // Deduced X Attitude
double angle_y = 0;                         // Deduced Y Attitude
double angle_z = 0;                         // Deduced Z Attitude
double rotation_vx, rotation_vy, rotation_vz;
double old_rotation_vx = 0;
double old_rotation_vy = 0;
double old_rotation_vz = 0;
double rotation_ax, rotation_ay, rotation_az;

// Data Acquisition
volatile boolean gotIMUdata = false;        // Data needs to be retrieved from IMU
boolean is_processing = false;              // We are getting data RIGHT now and can't get MORE data if available
unsigned long imu_data_time;                // Time we receive interrrupt (Data is available)
int imu_measurement_count = 0;              // # of Measurements we make

// IMU Calibration
boolean imu_calibrated = false;
int gyroZHigh = 0, gyroZLow = 0, gyroYHigh = 0, gyroYLow = 0, gyroXHigh = 0, gyroXLow = 0;
int accelZHigh = 0, accelZLow = 0, accelYHigh = 0, accelYLow = 0, accelXHigh = 0, accelXLow = 0;

// Accelerometer
int16_t ax, ay, az;
int16_t ay_history[20];                   // Keep track of latest 20 readings
int acceleration_threshold_count = 10;    // Number of continuous readings above which we assume rocket is in flight
int acceleration_threshold = 4096;        // At +/- 16g, 1g = 1024. So 4g = 4096


// Other IMU Variables
double factor = 0.0305 * PI/180;    // Convert the raw 'digital' values to radians. We work in radians ONLY!  (0.070 is for +-2000deg/sec - got from datasheet)
boolean is_first_iteration = true;  // Avoid first iteration.... tdiff is not 'right'




// SERVO CONFIGURATION
// Because the bottom servo is inverted, it's direction is in the opposite direction to the top servo.
// We acknowledge this difference here.
boolean s2_motor_inverted = true;

// Initial weight positions
double s1_angle = 0;
double s2_angle = PI;
#define cw true
#define ccw false





// PINS
#define INTERRUPT_PIN 2      // use pin 2 on Arduino Uno & most boards
int LED_INDICATOR_PIN = 4;   // Indicates state of the system
int SERVO1_PIN        = 3;
int SERVO2_PIN        = 6;
int LED_DEBUGGING     = 13;


// TIME TRACKING VARIABLES
long time;
long last_time;
long start_time;
long end_time;
long start_time1;
long end_time1;



// BALANCING VARIABLES
double    upper_velocity_threshold;
double    lower_velocity_threshold;
unsigned int smoother_step = 0;
double    corrective_angle = 0;
double    mid_point_angle = 0;
double    mid_point_distance = 0;
double    move_to_neutral_distance = 0;
boolean   s1_direction, s2_direction;
double    intermediate_move = 0;
double    final_angle_move = 0;
double    resting_angle_move = 0;


// DEBUGGING
boolean debugging    = true;
boolean info         = true  ;
boolean print_timing = true;


// Vectors
double thrust_vector[] = {0, 1, 0};
double x_vector[]      = {1, 0, 0};
double y_vector[]      = {0, 1, 0};     // Vector pointing in direction of rocket motion (up)
double vec3[]          = {0, 0, 0};     // Result of cross product



// the setup function runs once when you press reset or power the board
void setup() {
 // Initialise Pins
 pinMode(INTERRUPT_PIN, INPUT);                    // Interrupts pin...for IMU
 pinMode(LED_DEBUGGING, OUTPUT);                  // Debugging.. 
 pinMode(SERVO1_PIN, OUTPUT);          // Detect when launch is done.
 pinMode(SERVO2_PIN, OUTPUT);          // Detect when launch is done.
 pinMode(LED_INDICATOR_PIN, OUTPUT);   // State indicator LED
 pinMode(A2, INPUT_PULLUP);            // Voltage Sensor

   
 Wire.begin();
 Serial.begin(38400);

 fastBlinkLed(5000L);  // Give person 5 seconds warning...that system is coming online....
  
  // Initialise FRAM
 if (fram.begin()) {
    Serial.println("Found i2C FRAM");
    fram_installed = true;
  } else {
    Serial.println("No FRAM");
    fram_installed = false;
  } 


  // Initialise Air Pressure Sensor  
  if (air_pressure_sensor_enabled) {
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    } else {
      Serial.println("Found BMP085");
      air_pressure_sensor_available = true;
      initialise_ap_timer(3124); // Initialise time to get readings every 0.1 seconds
    }
  }

    

  // TODO - WILL REPLACE WITH SOME ROUTINE THAT TAKES SERIAL INPUT 
  // If no connection to launch detect...then show 
//  if (digitalRead(DATA_DETECT_PIN) == LOW) {
//     dumpFRAM(); 
//  } 


  // Clear fRAM  
  if (fram_installed) {
    Serial.println("Clear FRAM");
    clearfram();
    Serial.println("FRAM cleared");
  }

    
  // Initialise Gryoscope and calibrate
  // NOTE: We don't use the values from the calibration just yet....
  if (imu_enabled) {
    Serial.println("Start IMU");
    setupIMU();     // Configure MPU-6050
    delay(2500);    //wait for the sensor to be ready
    imu_available = true;
  }
  
  // Enable interrupts
  if (imu_available) {
    Serial.println("Attaching routine for IMU Interrupts");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), imu_data_available, RISING);  // Interrupt from IMU
    mpuIntStatus = accelgyro.getIntStatus();
  
    // Perform calibration 
    Serial.println("Cal IMU");
    calibrate_imu();
    imu_calibrated = true;

    if (info) {
       Serial.println("GYROSCOPE");
       Serial.print("AVG: "); Serial.print(avg_gx); Serial.print("\t");  Serial.print(avg_gy); Serial.print("\t");Serial.println(avg_gz); 
       Serial.print("VAR: "); Serial.print(var_gx); Serial.print("\t");  Serial.print(var_gy); Serial.print("\t");Serial.println(var_gz); 

       Serial.println("ACCELEROMETER");
       Serial.print("AVG: "); Serial.print(avg_ax); Serial.print("\t");  Serial.print(avg_ay); Serial.print("\t");Serial.println(avg_az); 
       Serial.print("VAR: "); Serial.print(var_ax); Serial.print("\t");  Serial.print(var_ay); Serial.print("\t");Serial.println(var_az); 
    }
    Serial.println("End Cal IMU");    
    delay(1000);
  }
  


  // Calibrate Smoothers (move them into position)
  Serial.println("Cal S1/S2");
  calibrate_weights();
  Serial.println("Finished");  
  




  delay(1000);

  
  Serial.println("S1 ANG: " + String(s1_angle));
  Serial.println("S2 ANG: " + String(s2_angle));     
  
  Serial.println("System Init");
  digitalWrite(LED_INDICATOR_PIN, HIGH);   // Indicates to user we are ready! Just waiting for launch to disconnect launch detect wires.

}




// the loop function runs over and over again forever
void loop() {

  long currMicros = micros();

  Serial.print("Time: "); Serial.println(currMicros);


  // Data available!
  if (imu_available) {
     check_for_imu_data();  
  }
  if (air_pressure_sensor_available) {
     check_for_ap_data();
  }
  
  
  // Simulate rotation - but ONLY if gyroscope disabled
  if (! imu_available) {
     rotation_vz = rotation_vz + 1 * PI/180;
  }


  // Our trigger point for starting recording is 'acceleration_threshold_count' continuous acceleration readings 
  // have an absolute value that is above 'acceleration_threshold'
  // The IMU is positioned so that Y-axis is up/down, so it is this axis that will experience the acceleration
  if (! launch_begun) {
     launch_begun = detect_trigger_condition(acceleration_threshold, acceleration_threshold_count);
     fram_start_recording = true; // We now want to start recording.
  }

  
  // Show IMU data
  if (debugging) {
      Serial.print("ax: "); Serial.print(ax); Serial.print("\t");  Serial.print("ay: "); Serial.print(ay); Serial.print("\t"); Serial.print("az: ");Serial.println(az); 
      Serial.print("gx: "); Serial.print(gx); Serial.print("\t");  Serial.print("gy: "); Serial.print(gy); Serial.print("\t"); Serial.print("gz: ");Serial.println(gz); 
  }

  // PRINT ORIENTATION
  double angle_x_deg = angle_x * 180/PI;
  double angle_y_deg = angle_y * 180/PI;
  double angle_z_deg = angle_z * 180/PI;  
  print_debug(info, "POS- X: " + String(angle_x_deg)     + ", Y: " + String(angle_y_deg) + ", Z: " + String(angle_z_deg)); 
  

    
  if (active_control && smoother_step == 0 && check_system_stability(rotation_vx, rotation_vy, rotation_vz, rotation_ax, rotation_ay, rotation_az)) {
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
void imu_data_available() {
  gotIMUdata = true;
  imu_data_time = micros();
}


// AirPressure sensor
void getAPValues(boolean write_imu_to_fram)
{
  // Get Air Pressure
  int32_t airpressure = bmp.readRawPressure();

  rawApTData[0] = 10;
  rawApTData[1] = (airpressure>>16) & 0xFF;
  rawApTData[2] = (airpressure>>8) & 0xFF;
  rawApTData[3] = airpressure & 0xFF;    

  // Get Temperature
  int16_t temperature = bmp.readRawTemperature();
       
  rawApTData[4] = (temperature>>8) & 0xFF;
  rawApTData[5] = temperature & 0xFF;
       

  // Time
  rawApTData[6]  = ap_data_time & 0xFF;
  rawApTData[7]  = (ap_data_time >>8 ) & 0xFF;
  rawApTData[8]  = (ap_data_time >>16) & 0xFF;
  rawApTData[9] = (ap_data_time >>24) & 0xFF;              


  // -20 is to allow enough space for data
  if (write_imu_to_fram && fram_installed && framAddr < (32768 - 20)) {  
    fram.write(framAddr, (uint8_t *) &rawApTData[0], 10);
    framAddr = framAddr + 10;
  } ;
    
}



// GYROSCOPE RELATED ROUTINES
void getIMUValues(boolean write_imu_to_fram)
{
  // Read data from IMU
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Assemble Gyro and Acceleration data in Buffer
  _buff[0] = 17;
  _buff[1] =   gx >> 8;
  _buff[2] =  (gx & 0xFF);
  _buff[3] =   gy >> 8;
  _buff[4] =  (gy & 0xFF);
  _buff[5] =   gz >> 8;
  _buff[6]  = (gz & 0xFF);
  _buff[7] =   ax >> 8;
  _buff[8] =  (ax & 0xFF);
  _buff[9] =   ay >> 8;
  _buff[10] =  (ay & 0xFF);
  _buff[11] =  az >> 8;
  _buff[12] = (az & 0xFF);    


  // Time 
  _buff[13] = imu_data_time & 0xFF;
  _buff[14] = (imu_data_time >>8 ) & 0xFF;
  _buff[15] = (imu_data_time >>16) & 0xFF;
  _buff[16] = (imu_data_time >>24) & 0xFF;

  // Zero values that are within zero value. (i.e. assume they are zero!)
  if (imu_calibrated) {
     zeroAccelGyro();
  }

  
  // -20 is to allow enough space for data
  if (write_imu_to_fram && fram_installed && framAddr < (32768 - 20)) {  
    fram.write(framAddr, (uint8_t *) &_buff[0], 17);
    framAddr = framAddr  + 17;
  } ;

}

void setupIMU(){
  // Initialise IMU
  accelgyro.initialize();

  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);    // Set Gyroscope to +-1000degrees/second
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);   // Set Acceleration to +-16g....1g = 1024
  accelgyro.setRate(20);                                    // 1 reading every 0.1 seconds
  accelgyro.setIntEnabled(0x1);                            // Enable Interrupts
  
  
  // Verify Connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
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
  

  
void calculate_ang_acceleration(double vx, double vy, double vz)
{
  double vx_avg, vy_avg, vz_avg;
  time = micros();
  long tdiff = time - last_time;
  
  // Want to skip first data point... (tdiff is unreliable)
  if (! is_first_iteration) {
    // Calculate Average velocity over time interval
    vx_avg = (vx + old_rotation_vx)/2;
    vy_avg = (vy + old_rotation_vy)/2;
    vz_avg = (vz + old_rotation_vz)/2;  

    // Numerical integrate to get angle
    angle_x = angle_x + vx_avg * tdiff/1000000;
    angle_y = angle_y + vy_avg * tdiff/1000000;
    angle_z = angle_z + vz_avg * tdiff/1000000;

  } else {
    is_first_iteration = false;
  }
  
  // Calculate Acceleration
  rotation_ax = 1000000 * (vx - old_rotation_vx)/tdiff;
  old_rotation_vx = vx;
  rotation_ay = 1000000 * (vy - old_rotation_vy)/tdiff;
  old_rotation_vy = vy;  
  rotation_az = 1000000 * (vz - old_rotation_vz)/tdiff;
  old_rotation_vz = vz;  
  
  last_time = time;
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
        mid_point_distance  = (s1_angle + s2_angle)/2;                                             // Angular distance mid-way between s1 and s2
        
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
                print_debug(info, "Neutral Move");
                move_servos(s1_direction, s2_direction, move_to_neutral_distance, 0);
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

        mid_point_distance  = (s1_angle + s2_angle)/2;                                             // Angular distance mid-way between s1 and s2
        
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
          print_debug(info, "IM: " + String(intermediate_move));
          move_servos(s1_direction, s2_direction, intermediate_move, 0);
          smoother_step = 5;
        }


        // CALCULATE THE FINAL MOVE
        mid_point_angle =  angle_between(s1_angle, s2_angle);
        
        mid_point_distance  = (s1_angle + s2_angle)/2;                                             // Angular distance mid-way between s1 and s2

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
          print_debug(info, "FM: " + String(final_angle_move));
          move_servos(s1_direction, s2_direction, final_angle_move, lower_velocity_threshold); 
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
  print_debug(info, "RM: " + String(resting_angle_move));
  move_servos(s1_direction, s2_direction, resting_angle_move, lower_velocity_threshold);
  smoother_step = 0;
  digitalWrite(LED_INDICATOR_PIN, HIGH);
  end_time1 = micros();
  
}


void move_servos(boolean s1_direction, boolean s2_direction, double angle, double threshold)
{
 // TODO


  print_debug(info, "S1 ANG: " + String(s1_angle));
  print_debug(info, "S2 ANG: " + String(s2_angle));   
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


void calibrate_imu()
{
  int i = 0;
  
  // Without this line, it gets 'stuck'
  getIMUValues(false);
  
  while (i < 25)
  {
    // delayMicroseconds(20000);
    if (debugging) 

    
    if (gotIMUdata && ! is_processing) {
       is_processing = true;
       
       mpuIntStatus = accelgyro.getIntStatus();
       getIMUValues(false);

       // GYROSCOPE
       
       // Z
       if (gz > gyroZHigh) {
          gyroZHigh = gz;
       } else if(gz < gyroZLow) {
          gyroZLow = gz;
       }

       // Y
       if (gy > gyroYHigh) {
          gyroYHigh = gy;
       } else if(gy < gyroYLow) {
          gyroYLow = gy;
       }
       
       // X
       if (gx > gyroXHigh) {
          gyroXHigh = gx; 
       } else if (gx < gyroXLow) {
          gyroXLow = gx;
       }  
  
  
       // Get Average
       avg_gx = (avg_gx * i + gx)/(i+1);
       avg_gy = (avg_gy * i + gy)/(i+1);
       avg_gz = (avg_gz * i + gz)/(i+1);   
   
       // Get Variance
       var_gx = (var_gx * i + (gx * gx))/(i+1);
       var_gy = (var_gy * i + (gy * gy))/(i+1);
       var_gz = (var_gz * i + (gz * gz))/(i+1);   



       // ACCELERATION
       // Z
       if (az > accelZHigh) {
          accelZHigh = az;
       } else if(az < accelZLow) {
          accelZLow = az;
       }

       // Y
       if (ay > accelYHigh) {
          accelYHigh = ay;
       } else if(ay < accelYHigh) {
          accelYHigh = ay;
       }
       
       // X
       if (ax > accelXHigh) {
          accelXHigh = ax; 
       } else if (ax < accelXLow) {
          accelXLow = ax;
       }  
  
  
       // Get Average
       avg_ax = (avg_ax * i + ax)/(i+1);
       avg_ay = (avg_ay * i + ay)/(i+1);
       avg_az = (avg_az * i + az)/(i+1);   
   
       // Get Variance
       var_ax = (var_ax * i + (ax * ax))/(i+1);
       var_ay = (var_ay * i + (ay * ay))/(i+1);
       var_az = (var_az * i + (az * az))/(i+1);   

       i++;
       
      is_processing = false;
      gotIMUdata = false;
    }
  }
}



// Get latest IMU data (if available)
void check_for_imu_data()
{
    
  if (gotIMUdata && ! is_processing) {
    is_processing = true;
    
    mpuIntStatus = accelgyro.getIntStatus();
    Serial.print("mpuIntStatus: "); Serial.println(mpuIntStatus);
    
    getIMUValues(true);
    
    // Get rotation rates in radians per second
    rotation_vx = gx * factor;
    rotation_vy = gy * factor;
    rotation_vz = gz * factor;    
    
    // Calculate acceleration
    calculate_ang_acceleration(rotation_vx, rotation_vy, rotation_vz);
    is_processing = false;
    
    imu_measurement_count++;
    gotIMUdata = false;
  }
}  



// Check for Air Pressure data
void check_for_ap_data()
{

  if (gotAPdata && ! is_processing) {
    is_processing = true;
    
    gotAPdata = false;
    getAPValues(true);
    
    ap_measurement_count++;
  }

}






// Zero out fRAM
void clearfram()
{
    for (uint16_t a = 0; a < 32768; a++) {
    fram.write8(a, 0x00);
  }
}




void dumpFRAM()
{
    byte gxmsb, gxlsb, gymsb, gylsb, gzmsb, gzlsb;  // GYRO
    byte axmsb, axlsb, aymsb, aylsb, azmsb, azlsb;  // ACCELERATION
    byte pmsb, pcsb, plsb;                          // AIR PRESSURE
    byte tmsb, tlsb;                                // TEMP
    byte d1, d2, d3, d4;
    int a;
    byte pklen;
    
    int32_t  UT, UP, B3, B5, B6, X1, X2, X3;  // Computation of the pressure/temperature
    uint32_t B4, B7;   // Computation of the pressure/temperature
    
    for (a = 0; a < (32768); a=a+1) {
      pklen = fram.read8(a);
      if (pklen == 10) {

         // PRESSURE + TEMPERATURE
         pmsb = fram.read8(a+1);
         pcsb = fram.read8(a+2);
         plsb = fram.read8(a+3);
         tmsb = fram.read8(a+4);
         tlsb = fram.read8(a+5);  
      
         // TIME
         d1 = fram.read8(a+6);
         d2 = fram.read8(a+7);
         d3 = fram.read8(a+8);
         d4 = fram.read8(a+9);       

         a = a + 10;
        
      } else if (pklen == 17) {
         // GYRO
         gxmsb = fram.read8(a+1);
         gxlsb = fram.read8(a+2);
         gymsb = fram.read8(a+3);
         gylsb = fram.read8(a+4);
         gzmsb = fram.read8(a+5);
         gzlsb = fram.read8(a+6);  

         // ACCEL
         axmsb = fram.read8(a+7);
         axlsb = fram.read8(a+8);
         aymsb = fram.read8(a+9);
         aylsb = fram.read8(a+10);
         azmsb = fram.read8(a+11);
         azlsb = fram.read8(a+12);     

         // TIME
         d1 = fram.read8(a+13);
         d2 = fram.read8(a+14);
         d3 = fram.read8(a+15);
         d4 = fram.read8(a+16);      

         a = a + 17;
      } else {
        Serial.println("Unrecognized packet length - possibly at end");
      }
      


    
            
   

      
// MANIPULATE DATA FOR DISPLAYING
      if (pklen == 17) {
         // Gyroscope
         gx = ((gxmsb << 8) | gxlsb);
         gy = ((gymsb << 8) | gylsb);
         gz = ((gzmsb << 8) | gzlsb);
      
         rotation_vx = gx * factor;
         rotation_vy = gy * factor;
         rotation_vz = gz * factor;


         // Accelerometer
         ax = ((axmsb << 8) | axlsb);
         ay = ((aymsb << 8) | aylsb);
         az = ((azmsb << 8) | azlsb);
         
      } else if (pklen == 10) {
         // Air-Pressure
         UP = (pmsb<<16) | (pcsb<<8) | plsb;
         
         // Temperature
         UT = (tmsb<<8) | tlsb;


         // Time

         // NOTE: THIS USES A MODIFIED VERSION OF ADAFRUIT BMP085 LIBRARY
         // Compute Temperature and pressure from raw values UP and UT above
         pressure = bmp.computePressure(UP, UT);
         temperature = bmp.computeTemperature(UT);    
      }
      
    /*  
     Serial.print(xmsb); Serial.print(" ");
     Serial.print(xlsb); Serial.print(" ");
     Serial.print(ymsb); Serial.print(" ");    
     Serial.print(ylsb); Serial.print(" ");
     Serial.print(zmsb); Serial.print(" ");    
     Serial.print(zlsb); Serial.print(" ");
     Serial.print(d1); Serial.print(" ");
     Serial.print(d2); Serial.print(" ");
     Serial.print(d3); Serial.print(" ");    
     Serial.println(d4); 
    */


      // ROTATION
      print_debug(info, "RS: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));  // RS = Rotational Speed
      print_debug(info, "LA: " + String(rotation_vx) + ", " + String(rotation_vy) + ", " + String(rotation_vz));  // LA = Linear Acceleration

      
      // Print Time
      Serial.print(d4);
      Serial.print(" ");
      Serial.print(d3);
      Serial.print(" ");
      Serial.print(d2);
      Serial.print(" ");
      Serial.println(d1 );   
 
    }

/*
    // Pressure/Temperature readings
    for (a = (8180 - 160); a < 8180 ; a=a+5) {

      // Pressure/Altitude bytes
      byte msb = fram.read8(a);
      byte csb = fram.read8(a+1);
      byte lsb = fram.read8(a+2);
      // Temperature bytes
      byte msbT = fram.read8(a+3);
      byte lsbT = fram.read8(a+4); 


//      Serial.print(msb);
//      Serial.print(" ");
//      Serial.print(csb);
//      Serial.print(" ");
//      Serial.println(lsb);

      
      long pressure_whole =  ((long)msb << 16 | (long)csb << 8 | (long)lsb) ; // Construct whole number pressure
      pressure_whole >>= 6;
 
      lsb &= 0x30; 
      lsb >>= 4;
      float pressure_frac = (float) lsb/4.0;

      pressure = (float) (pressure_whole) + pressure_frac; 

   
      // Calculate temperature, check for negative sign
      long foo = 0;
      if(msbT > 0x7F) {
         foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
         temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
         temperature *= -1.;
      } else {
         temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
      }
  
      // Output data array to serial printer; comma delimits useful for importing into excel spreadsheet
      // Serial.print("Time ,"); Serial.print((j/5)*(1<<0)); Serial.print(", seconds");
      Serial.print(", Temperature = ,"); Serial.print(temperature, 1); Serial.print(", C,");
      Serial.print(" Pressure = ,"); Serial.print(pressure/1000., 2); Serial.println(", kPa");
    
    }
*/

    
    
    // Blink led slowly...so we know we are at the end.
    while(1) {
      slowBlinkLED();
    }
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




// Set to zero, if within low/high readings
void zeroAccelGyro()
{
  
  if (gx >= gyroXLow && gx <= gyroXHigh) {
    gx = 0;
  }

  if (gy >= gyroYLow && gy <= gyroYHigh) {
    gy = 0;
  }
    
  if (gz >= gyroZLow && gz <= gyroZHigh) {
    gz = 0;
  }

  if (ax >= gyroXLow && ax <= gyroXHigh) {
    ax = 0;
  }

  if (ay >= gyroYLow && ay <= gyroYHigh) {
    ay = 0;
  }
    
  if (az >= gyroZLow && az <= gyroZHigh) {
    az = 0;
  }  
}


void veryfastBlinkLED()
{
  blinkLED(100);
}

void fastBlinkLED()
{
  blinkLED(200);
}


void slowBlinkLED()
{
  blinkLED(500);
}

void blinkLED(int led_delay)
{
      digitalWrite(LED_INDICATOR_PIN, HIGH);
      delay(led_delay);
      digitalWrite(LED_INDICATOR_PIN, LOW);
      delay(led_delay);  
}


// Perform fast Blink LED for 'duration' seconds
void fastBlinkLed(long duration)
{
  long start_time = millis();
  
  while (millis() < start_time + duration) {
    veryfastBlinkLED();
  }

}




// Move Weights into required 'start' position
void calibrate_weights()
{


}



// Initialise timer for 
void initialise_ap_timer(int timer1)
{
      // int timer1 = 3124; // interrupt freq = 8,000,000 / (256 * (3124 + 1)) = 10hz
          
      cli();
      
      // TIMER1
      TCCR1A  = 0;// set entire TCCR1A register to 0
      TCCR1B  = 0;// same for TCCR1B
      TCNT1   = 0;//initialize counter value to 0
      // set compare match register for 1hz increments
      OCR1A   = timer1;// = (8*10^6) / (164*1) - 1 (must be <65536)    ... This is just an example calc
      // turn on CTC mode
      TCCR1B |= (1 << WGM12);
      // No scaling
      TCCR1B |= (1 << CS12);

      // enable timer compare interrupt
      TIMSK1 |= (1 << OCIE1A);      
      
      sei(); 
     
}


ISR(TIMER1_COMPA_vect){//timer0 interrupt - pulses motor
  gotAPdata = true;      // Allow next interrupt to be set.
}


// Determine if last 10 readings have breached the Acceleration Threshold settings.
// If so, return boolean TRUE
// Otherwise, return FALSE
boolean detect_trigger_condition(int acceleration_threshold, int acceleration_threshold_count)
{


  return false;
}

