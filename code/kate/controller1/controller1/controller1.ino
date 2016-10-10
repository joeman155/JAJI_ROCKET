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
#include <Servo.h>
#include "ApplicationMonitor.h"

// See http://www.megunolink.com/articles/how-to-detect-lockups-using-the-arduino-watchdog/  for information on how to use information
// Produced by this to debug issue.
Watchdog::CApplicationMonitor ApplicationMonitor;

// Uncomment out line below to enable stability correction code
// #define STABILITY_CORRECTION

// Allows us to fiddle with some values, to ensure we can test properly
#define TESTING_MODE

// Additional debugging beyond what is normally required.
// #define INFO
#define DEBUG


// Air Pressure Sensor Code
// #define AIRSENSOR



#ifdef STABILITY_CORRECTION
#include "Stabilisation.h"
#endif


// Define functions
void fastBlinkLed(long);
void fastBlinkLed();
void setupIMU();
void imu_data_available();
void calibrate_imu();
void weights_starting_pos();
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
boolean  no_more_space = false;                 // Set TRUE when no more space left in FRAM
boolean  fram_available = true;                 // Specifies if we use FRAM - even if installed.
boolean  fram_installed = false;
uint16_t fram1AddrStart = 0;                    // Start address of FRAM Bank 1
uint16_t fram1AddrEnd   = 32599;                // End address of FRAM Bank 1
uint16_t fram1Addr = fram1AddrStart;            // Ptr, start at beginning - Bank 1
int      fram1MaxPacketSize = 17;               // Maximum size of packet
int      Fram1StartPos = 0;                     // Where the Start Address is
int      historicalBytes = 512;                 // Maximum # of historical Bytes that we will keep (and not overwrite)

uint16_t framPtr = 32760;                     // The is where we keep the oldest pointer
                                                // of data in Block 2

#ifdef DEBUG                                                
long     memory_start;                          // Used to track time of start of launch
long     memory_end;                            // Used to track point where we fill up all the memory bank. 
#endif





#ifdef AIRSENSOR
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

#endif



// IMU Variables
MPU6050 accelgyro;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
byte    _buff[17];      // Used to get lots of data from IMU in one i2c call!
unsigned int cycles_without_data = 0;
#ifdef DEBUG
int imu_data_rate = 49;
#else
int imu_data_rate = 49;
#endif




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
volatile boolean gotIMUdata = false;         // Data needs to be retrieved from IMU
boolean          is_processing = false;      // We are getting data RIGHT now and can't get MORE data if available
unsigned long    imu_data_time;              // Time we receive interrrupt (Data is available)
int              imu_measurement_count = 0;  // # of Measurements we make

// IMU Calibration
boolean imu_calibrated = false;
int     gyroZHigh = 0, gyroZLow = 0, gyroYHigh = 0, gyroYLow = 0, gyroXHigh = 0, gyroXLow = 0;
int     typicalGyroCal = 150;


// Accelerometer
int16_t ax, ay, az;
int acceleration_threshold_count_required = 3;  // Number of continuous readings above which we assume rocket is in flight
int acceleration_threshold_count = 0;     // How many measurements have been over the acceleration_threhold
int acceleration_threshold = 2048;        // At +/- 16g, 1g = 1024. So 4g = 4096  (So here we are looking at 1g accel + gravity = 2g)   




// Other IMU Variables
double factor = 0.0305 * PI / 180;  // Convert the raw 'digital' values to radians. We work in radians ONLY!  (0.070 is for +-2000deg/sec - got from datasheet)
boolean is_first_iteration = true;  // Avoid first iteration.... tdiff is not 'right'




// SERVO CONFIGURATION
// Because the bottom servo is inverted, it's direction is in the opposite direction to the top servo.
// We acknowledge this difference here.
#define top_servo 1
#define bottom_servo 2
Servo      topservo;
Servo      bottomservo;
double     gear_ratio = 0.3333;
double     topservo_angle;            // Angle of top servo
double     bottomservo_angle;         // Angle of bottom servo
boolean    s2_motor_inverted = true;
boolean    move_servo = false;
int        timer2_count = 0;          // Keeps a track of how many times interrupt 2 has fired off.
int        timer2_max_count = 100;    // Maximum # of times we want it to fire off
int        reference_angle;               // The measurement of the 
//int        req_track_angle;           // The angle we want to track. This is set at lift-off
//int        act_track_angle;           // Actual angle we want to be pointing in.
boolean   track_mode = false;        // TRUE if we try to compenstate for Z-axis rotation


// Initial weight positions
double s1_angle = 0;
double s2_angle = PI;
#define cw true
#define ccw false





// PINS
#define INTERRUPT_PIN       2   // use pin 2 on Arduino Uno & most boards
#define LED_INDICATOR_PIN   4   // Indicates state of the system
#define SERVOBOTTOM_PIN     6
#define SERVOTOP_PIN        3
#define LED_DEBUGGING       13
#define READ_MODE_ENABLE_DETECT_PIN   11   // Need to set this HIGH, so that we can use jumper between 11 and 12.
#define READ_MODE_PIN       12               // Has a pulldown resistor to Ground. 


// TIME TRACKING VARIABLES
long time;
long last_time;
long start_time;
long end_time;
long start_time1;
long end_time1;



// DEBUGGING
boolean print_timing = false;



// the setup function runs once when you press reset or power the board
void setup() {

#ifdef TESTING_MODE
// LOW ACCERATION THRESHOLD TESTING VALUES
acceleration_threshold_count_required = 1;
acceleration_threshold = 1024 + 500;   // i.e. Gravity + some acceleration
#endif

  
  // Initialise Pins
  pinMode(INTERRUPT_PIN,     INPUT);                   // Interrupts pin...for IMU
  pinMode(LED_DEBUGGING,     OUTPUT);                  // Debugging..
  pinMode(SERVOTOP_PIN,      OUTPUT);                  // Detect when launch is done.
  pinMode(SERVOBOTTOM_PIN,   OUTPUT);                  // Detect when launch is done.
  pinMode(LED_INDICATOR_PIN, OUTPUT);                  // State indicator LED
  pinMode(READ_MODE_ENABLE_DETECT_PIN, OUTPUT);        // Set Output, so we can put a HIGH on it - permanently
  pinMode(READ_MODE_PIN,     INPUT);                   // Wish to detect if HIGH is on this...then we are wanting to go into framDump routine
  pinMode(A2,                INPUT_PULLUP);            // Voltage Sensor

  // Quickly initialise this pin - We use this has +3.3v for jumper to pint 12 (READ_MODE_PIN)
  digitalWrite(READ_MODE_ENABLE_DETECT_PIN, HIGH);

  // Starting up the Serial Port
  Serial.begin(115200);
  Serial.println("Powering up...");

  // Initialise the Servos
  topservo.attach(SERVOTOP_PIN);
  bottomservo.attach(SERVOBOTTOM_PIN);

  // Dump WatchDog data for debugging...
  ApplicationMonitor.Dump(Serial);

  // Give person 5 seconds warning...that system is coming online....
  fastBlinkLed(5000L);  

  // Initialise FRAM
  if (fram_available) {
     if (fram.begin()) {
       Serial.println("Found FRAM");
       fram_installed = true;
     } else {
       Serial.println("No FRAM");
       fram_installed = false;
     }
  }


#ifdef AIRSENSOR
  // Initialise Air Pressure Sensor
  if (air_pressure_sensor_enabled) {
    if (!bmp.begin()) {
      Serial.println("NotFound BMP180.");
    } else {
      Serial.println("Found BMP180");
      air_pressure_sensor_available = true;
      initialise_ap_timer(3124);      // Initialise time to get readings every 0.1 seconds
    }
  }
#endif


  // User puts a jumper on this to detect data extraction
  if (digitalRead(READ_MODE_PIN) == HIGH) {
      Serial.println("Going into DumpFRAM routine.");
      dumpFRAM();
  }
  

  if (fram_installed) {
     Serial.println("Waiting 20 seconds before cleaing FRAM");
     delay(20000);
  }


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
    Serial.print("imu_data_rate: "); Serial.println(imu_data_rate);
    Serial.println("Attaching routine for IMU Interrupts");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), imu_data_available, RISING);  // Interrupt from IMU
    mpuIntStatus = accelgyro.getIntStatus();

    // Perform calibration - IMU must not be rotation 
    Serial.println("Cal IMU");
    calibrate_imu();

    Serial.println("LOWS: ");
    Serial.print(gyroXHigh); Serial.print("\t"); Serial.print(gyroYHigh); Serial.print("\t"); Serial.println(gyroZHigh); 
    Serial.println("HIGHS: "); 
    Serial.print(gyroXLow); Serial.print("\t"); Serial.print(gyroYLow); Serial.print("\t"); Serial.println(gyroZLow); 

    imu_calibrated = true;

    // Confirm that the Calication values look 'sane'
    if (check_calibration_values()) {
        Serial.println("Cal issues. Rocket not stationary?");
        errorCondition();
    }

    Serial.println("GYRO");
    Serial.print("AVG: "); Serial.print(avg_gx); Serial.print("\t");  Serial.print(avg_gy); Serial.print("\t"); Serial.println(avg_gz);
    Serial.print("VAR: "); Serial.print(var_gx); Serial.print("\t");  Serial.print(var_gy); Serial.print("\t"); Serial.println(var_gz);
    Serial.println("ACCEL");
    Serial.print("AVG: "); Serial.print(avg_ax); Serial.print("\t");  Serial.print(avg_ay); Serial.print("\t"); Serial.println(avg_az);
    Serial.print("VAR: "); Serial.print(var_ax); Serial.print("\t");  Serial.print(var_ay); Serial.print("\t"); Serial.println(var_az);

    Serial.println("End Cal IMU");
    delay(1000);
  }



  // Move weights to their starting position
  // Serial.println("Cal S1/S2");
  weights_starting_pos();
  Serial.println("Finished");



  Serial.println("System Init");
  ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_4s);
  digitalWrite(LED_INDICATOR_PIN, HIGH);   // Indicates to user we are ready! Just waiting for launch to disconnect launch detect wires.

}




// the loop function runs over and over again forever
void loop() {
 ApplicationMonitor.IAmAlive();


#ifdef DEBUG  
  long currMicros = micros();
  Serial.print("Time: "); Serial.println(currMicros);
#endif  



  
  // IMU SENSOR DATA
  if (imu_available) {
    check_for_imu_data();
  }


#ifdef AIRSENSOR
  // AIR-PRESSURE SENSOR DATA
  if (air_pressure_sensor_available) {
    check_for_ap_data();
  }
#endif
  

  // Compensate for Z-axis rotation movement, when enabled
  if (track_mode) {
     // Find out how much the Sensor has rotated since last time.
     int angle_x_degrees = angle_x * 180 / PI;
     int angle_diff = angle_x_degrees - reference_angle;

     // Determine the new Direction the rocket is pointed in
     reference_angle = angle_x_degrees;
     
     // Move the Top Servo back around
     topservo_angle    = topservo_angle - angle_diff;
     bottomservo_angle = bottomservo_angle - angle_diff;

#ifdef INFO     
     Serial.print("X Angle: "); Serial.print(angle_x_degrees);
     Serial.print("  reference_angle: "); Serial.print(reference_angle);
     Serial.print("  TOPSERVO_ANGLE: "); Serial.print(topservo_angle);
     Serial.print("  DIFF: "); Serial.println(angle_diff);
#endif     

     // If angle has changed, then move it.
     if (abs(angle_diff) > 1 ) {
      // JOE
        // set_top_servo_position(topservo_angle);
        // set_bottom_servo_position(bottomservo_angle);
        moveTopMass (topservo_angle);
        moveBottomMass (bottomservo_angle);
     }
  }
  

  // Servo Move
  if (move_servo) {
      move_servo = false;
      topservo_angle = 180;
      bottomservo_angle = 180;
      // set_top_servo_position(topservo_angle);
      // set_bottom_servo_position(bottomservo_angle);
      moveTopMass (180);
      moveBottomMass (180);
      reference_angle = angle_x * 180 / PI;
      track_mode = true;
  }

 

  // Simulate rotation - but ONLY if gyroscope disabled
  if (! imu_available) {
    rotation_vz = rotation_vz + 1 * PI / 180;
  }


  // Perfom launch detection code
  launch_detection();



  // Show IMU data
#ifdef INFO
    Serial.print("ax: "); Serial.print(ax); Serial.print("\t");  Serial.print("ay: "); Serial.print(ay); Serial.print("\t"); Serial.print("az: "); Serial.println(az);
    Serial.print("gx: "); Serial.print(gx); Serial.print("\t");  Serial.print("gy: "); Serial.print(gy); Serial.print("\t"); Serial.print("gz: "); Serial.println(gz);
#endif



#ifdef INFO
  // PRINT ORIENTATION
  double angle_x_deg = angle_x * 180 / PI;
  double angle_y_deg = angle_y * 180 / PI;
  double angle_z_deg = angle_z * 180 / PI;
  
  Serial.print("ANGULAR POS- X: ");
  Serial.print(angle_x_deg);
  Serial.print("Y: ");
  Serial.print(angle_y_deg);
  Serial.print("Z: ");
  Serial.println(angle_z_deg);
#endif  



#ifdef STABILITY_CORRECTION
  // ACTIVE CONTROL
  if (active_control && smoother_step == 0 && check_system_stability(rotation_vx, rotation_vy, rotation_vz, rotation_ax, rotation_ay, rotation_az)) {
    digitalWrite(LED_INDICATOR_PIN, LOW);
    start_time1 = micros();

    calculate_smoother_location(rotation_vx, rotation_vy, rotation_vz);

    smoother_step = 1;
#ifdef DEBUG
      Serial.println("-- System needs stabilising --");
      Serial.print("RS: ");
      Serial.print(rotation_vx);
      Serial.print(", ");
      Serial.print(rotation_vy);
      Serial.print(", ");
      Serial.println(rotation_vz);

      Serial.print("RA: ");
      Serial.print(rotation_ax);
      Serial.print(", ");
      Serial.print(rotation_ay);
      Serial.print(", ");
      Serial.println(rotation_az);      
      print_time();

      Serial.print("CA: ");
      Serial.println(corrective_angle);
#endif

  }

  if (smoother_step > 0) {
    smoother_step_processing();
  }
#endif


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


#ifdef AIRSENSOR
// AirPressure sensor
void getAPValues(boolean write_imu_to_fram)
{
  // Get Air Pressure
  int32_t airpressure = bmp.readRawPressure();

  rawApTData[0] = 10;
  rawApTData[1] = (airpressure >> 16) & 0xFF;
  rawApTData[2] = (airpressure >> 8) & 0xFF;
  rawApTData[3] = airpressure & 0xFF;

  // Get Temperature
  int16_t temperature = bmp.readRawTemperature();

  rawApTData[4] = (temperature >> 8) & 0xFF;
  rawApTData[5] = temperature & 0xFF;


  // Time
  rawApTData[6] =  ap_data_time & 0xFF;
  rawApTData[7] = (ap_data_time >> 8 ) & 0xFF;
  rawApTData[8] = (ap_data_time >> 16) & 0xFF;
  rawApTData[9] = (ap_data_time >> 24) & 0xFF;


  if (write_imu_to_fram && fram_installed && ! no_more_space) {
     writeFramPacket (&rawApTData[0], 10);
  }


}
#endif


// GYROSCOPE RELATED ROUTINES
void getIMUValues(boolean write_imu_to_fram, boolean launch_detection)
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
  _buff[14] = (imu_data_time >> 8 ) & 0xFF;
  _buff[15] = (imu_data_time >> 16) & 0xFF;
  _buff[16] = (imu_data_time >> 24) & 0xFF;

  // Zero values that are within zero value. (i.e. assume they are zero!)
  if (imu_calibrated) {
    zeroGyroReadings();
  }


  // Perform logic to detect if launch is taking place
  if (launch_detection) {
    if (abs(ax) > acceleration_threshold) {
       acceleration_threshold_count++;
    } else {
       acceleration_threshold_count = 0;
    }

  }

  if (write_imu_to_fram && fram_installed && ! no_more_space) {
     writeFramPacket(&_buff[0], 17);
  }

}



void setupIMU() {
  // Initialise IMU
  accelgyro.initialize();

  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);    // Set Gyroscope to +-1000degrees/second
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);    // Set Acceleration to +-16g....1g = 1024
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_10);                // Low Pass filter  
  accelgyro.setRate(imu_data_rate);                         // Freq = 1000 / (imu_data_rate + 1)  
  accelgyro.setIntEnabled(0x1);                             // Enable Interrupts


  // Verify Connection
  Serial.println("Testing MPU6050 connection");
  Serial.println(accelgyro.testConnection() ? "MPU6050 successful" : "MPU6050 failed");

}




// Time Functions
void print_time() {
  if (! print_timing) return;

  long time = micros();
  double  time_display = time / (double) 1000000;
  Serial.print("Time: ");
  printDouble(time_display, 10000);
}



void calculate_ang_acceleration(double vx, double vy, double vz)
{
  double vx_avg, vy_avg, vz_avg;
  time = micros();
  long tdiff = time - last_time;

  // Want to skip first data point... (tdiff is unreliable)
  if (! is_first_iteration) {
    // Calculate Average velocity over time interval
    vx_avg = (vx + old_rotation_vx) / 2;
    vy_avg = (vy + old_rotation_vy) / 2;
    vz_avg = (vz + old_rotation_vz) / 2;

    // Numerical integrate to get angle
    angle_x = angle_x + vx_avg * tdiff / 1000000;
    angle_y = angle_y + vy_avg * tdiff / 1000000;
    angle_z = angle_z + vz_avg * tdiff / 1000000;

  } else {
    is_first_iteration = false;
  }

  // Calculate Acceleration
  rotation_ax = 1000000 * (vx - old_rotation_vx) / tdiff;
  old_rotation_vx = vx;
  rotation_ay = 1000000 * (vy - old_rotation_vy) / tdiff;
  old_rotation_vy = vy;
  rotation_az = 1000000 * (vz - old_rotation_vz) / tdiff;
  old_rotation_vz = vz;

  last_time = time;
}


// ------------------------------


void printDouble( double val, unsigned int precision) {
  // prints val with number of decimal places determine by precision
  // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
  // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  Serial.print("."); // print the decimal point
  unsigned int frac;
  if (val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val) - val ) * precision;
  int frac1 = frac;
  while ( frac1 /= 10 )
    precision /= 10;
  precision /= 10;
  while (  precision /= 10)
    Serial.print("0");

  Serial.println(frac, DEC) ;
}




void calibrate_imu()
{
  int i = 0;
  double deltagx, deltagy, deltagz;
  double deltaax, deltaay, deltaaz;
  
  // Without this line, it gets 'stuck'
  getIMUValues(false, false);

  while (i < 25)
  {
      if (gotIMUdata && ! is_processing) {
        is_processing = true;

        mpuIntStatus = accelgyro.getIntStatus();
        getIMUValues(false, false);

        // GYROSCOPE

        // Z
        if (gz > gyroZHigh) {
          gyroZHigh = gz;
        } else if (gz < gyroZLow) {
          gyroZLow = gz;
        }

        // Y
        if (gy > gyroYHigh) {
          gyroYHigh = gy;
        } else if (gy < gyroYLow) {
          gyroYLow = gy;
        }

        // X
        if (gx > gyroXHigh) {
          gyroXHigh = gx;
        } else if (gx < gyroXLow) {
          gyroXLow = gx;
        }


        deltagx = (gx - avg_gx);
        deltagy = (gy - avg_gy);
        deltagz = (gz - avg_gz);
        // Get Average - GYROSCOPE
        avg_gx = (avg_gx * i + gx) / (i + 1);
        avg_gy = (avg_gy * i + gy) / (i + 1);
        avg_gz = (avg_gz * i + gz) / (i + 1);

        // Get Variance - GYROSCOPE
        var_gx = var_gx + deltagx * (gx - avg_gx);
        var_gy = var_gy + deltagy * (gy - avg_gy);
        var_gz = var_gz + deltagz * (gz - avg_gz);


        deltaax = (ax - avg_ax);
        deltaay = (ay - avg_ay);
        deltaaz = (az - avg_az);
        // Get Average - ACCELEROMETER
        avg_ax = (avg_ax * i + ax) / (i + 1);
        avg_ay = (avg_ay * i + ay) / (i + 1);
        avg_az = (avg_az * i + az) / (i + 1);

        // Get Variance - ACCELEROMETER
        var_ax = var_ax + deltaax * (ax - avg_ax);
        var_ay = var_ay + deltaay * (ay - avg_ay);
        var_az = var_az + deltaaz * (az - avg_az);        

        i++;

        is_processing = false;
        gotIMUdata = false;
      }
  }
  var_ax = var_ax / (i - 1);
  var_ay = var_ay / (i - 1);
  var_az = var_az / (i - 1);

  var_gx = var_gx / (i - 1);
  var_gy = var_gy / (i - 1);
  var_gz = var_gz / (i - 1);  
}



// Get latest IMU data (if available)
void check_for_imu_data()
{

  if (gotIMUdata && ! is_processing) {
    is_processing = true;
    Serial.print("cwd: "); Serial.println(cycles_without_data);
    cycles_without_data = 0;

    mpuIntStatus = accelgyro.getIntStatus();
    // Serial.print("mpuIntStatus: "); Serial.println(mpuIntStatus);

    getIMUValues(true, true);

    // Get rotation rates in radians per second
    rotation_vx = gx * factor;
    rotation_vy = gy * factor;
    rotation_vz = gz * factor;

    // Calculate acceleration
    calculate_ang_acceleration(rotation_vx, rotation_vy, rotation_vz);
    is_processing = false;

    imu_measurement_count++;
    gotIMUdata = false;
  } else {
     cycles_without_data++;
  }
}




#ifdef AIRSENSOR
// Check for Air Pressure data
void check_for_ap_data()
{

  if (gotAPdata && ! is_processing) {
    is_processing = true;

    getAPValues(true);
    gotAPdata = false;

    ap_measurement_count++;
  }

  is_processing = false;

}
#endif





// Zero out fRAM
void clearfram()
{
  // Clear it 16 bytes at a time. Speeds things up considerably
  byte fz[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  for (uint16_t a = 0; a < 32768; a=a+16) {
    // fram.write8(a, 0x00);
    fram.write(a, fz, 16);
  }
}




void dumpFRAM()
{
  byte gxmsb, gxlsb, gymsb, gylsb, gzmsb, gzlsb;  // GYRO
  byte axmsb, axlsb, aymsb, aylsb, azmsb, azlsb;  // ACCELERATION
  byte pmsb, pcsb, plsb;                          // AIR PRESSURE
  byte tmsb, tlsb;                                // TEMP
  byte d1, d2, d3, d4;
  int a = 0;
  byte pklen;
  int addr;

  byte servo;
  byte angle;
 

  int32_t  UT, UP, B3, B5, B6, X1, X2, X3;  // Computation of the pressure/temperature
  uint32_t B4, B7;   // Computation of the pressure/temperature

  // Find Starting Address
  fram1Addr = getFram1Start();

#ifdef DEBUG
  Serial.print("STARTING POSITION: "); Serial.println(fram1Addr);
#endif
  
  // Make sure we read less then size of FRAM Bank 1
  while (a < fram1AddrEnd) {

    // fram1Addr = start_addr + a;
    pklen = fram.read8(fram1Addr);
    if (pklen == 10) {

      // PRESSURE + TEMPERATURE
      pmsb = fram.read8(fram1Addr + 1);
      pcsb = fram.read8(fram1Addr + 2);
      plsb = fram.read8(fram1Addr + 3);
      tmsb = fram.read8(fram1Addr + 4);
      tlsb = fram.read8(fram1Addr + 5);

      // TIME
      d1 = fram.read8(fram1Addr + 6);
      d2 = fram.read8(fram1Addr + 7);
      d3 = fram.read8(fram1Addr + 8);
      d4 = fram.read8(fram1Addr + 9);

      fram1Addr = advanceFram1Addr (fram1Addr, 10);
      a = a + 10;

    } else if (pklen == 17) {
      // GYRO
      gxmsb = fram.read8(fram1Addr + 1);
      gxlsb = fram.read8(fram1Addr + 2);
      gymsb = fram.read8(fram1Addr + 3);
      gylsb = fram.read8(fram1Addr + 4);
      gzmsb = fram.read8(fram1Addr + 5);
      gzlsb = fram.read8(fram1Addr + 6);

      // ACCEL
      axmsb = fram.read8(fram1Addr + 7);
      axlsb = fram.read8(fram1Addr + 8);
      aymsb = fram.read8(fram1Addr + 9);
      aylsb = fram.read8(fram1Addr + 10);
      azmsb = fram.read8(fram1Addr + 11);
      azlsb = fram.read8(fram1Addr + 12);

      // TIME
      d1 = fram.read8(fram1Addr + 13);
      d2 = fram.read8(fram1Addr + 14);
      d3 = fram.read8(fram1Addr + 15);
      d4 = fram.read8(fram1Addr + 16);

      fram1Addr = advanceFram1Addr (fram1Addr, 17);
      a = a + 17;
    } else if (pklen == 7) {
      // Servo
      servo = fram.read8(fram1Addr + 1);
      angle = fram.read8(fram1Addr + 2);

      // TIME
      d1 = fram.read8(fram1Addr + 3);
      d2 = fram.read8(fram1Addr + 4);
      d3 = fram.read8(fram1Addr + 5);
      d4 = fram.read8(fram1Addr + 6);

      fram1Addr = advanceFram1Addr (fram1Addr, 7);
      a = a + 7;
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


      // ROTATION
      Serial.print("RS: ");
      Serial.print(rotation_vx);
      Serial.print(", ");    
      Serial.print(rotation_vy);
      Serial.print(", ");
      Serial.println(rotation_vz);  // RS = Rotational Speed


      Serial.print("LA: ");
      Serial.print(ax);
      Serial.print(", ");    
      Serial.print(ay);
      Serial.print(", ");
      Serial.println(az);  // LA = Linear Acceleration
    
#ifdef AIRSENSOR
    } else if (pklen == 10) {
      // Air-Pressure
      UP = (pmsb << 16) | (pcsb << 8) | plsb;

      // Temperature
      UT = (tmsb << 8) | tlsb;


      // Time

      // NOTE: THIS USES A MODIFIED VERSION OF ADAFRUIT BMP085 LIBRARY
      // Compute Temperature and pressure from raw values UP and UT above
      pressure = bmp.computePressure(UP, UT);
      temperature = bmp.computeTemperature(UT);
#endif      
    } else if (pklen == 7) {
      // Servo Move
      if (servo == top_servo) {
         Serial.print("Top Servo: ");
         Serial.println(angle);
      } else if (servo == bottom_servo) {
         Serial.print("Bottom Servo: ");
         Serial.println(angle);
      }
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



               

    // Print Time
    Serial.println("For Time - MSB First: ");
    Serial.print(d4);
    Serial.print(" ");
    Serial.print(d3);
    Serial.print(" ");
    Serial.print(d2);
    Serial.print(" ");
    Serial.println(d1 );

  }

  // Blink led slowly...so we know we are at the end.
  while (1) {
    slowBlinkLED();
  }
}









// Set to zero, if within low/high readings
void zeroGyroReadings()
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
  ApplicationMonitor.IAmAlive();
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
    ApplicationMonitor.IAmAlive();
    veryfastBlinkLED();
  }
}





// Initialise timer for Air Pressure Sensor. This is so we get a measurement every 0.1 seconds
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


ISR(TIMER2_COMPA_vect) { //timer1 interrupt -  move servos
  if (timer2_count > timer2_max_count) {
      move_servo = true;
      timer2_count = 0;
      TIMSK2 &= ~_BV(OCIE2A); // Disable interrupt (only want this interrupt to occur time2_max_count times!)    
  } else {
      timer2_count++;
  }

  // gotAPdata = true; 
  // ap_data_time = micros();
}



// Determine if last XX readings have breached the Acceleration Threshold settings.
// If so, return boolean TRUE
// Otherwise, return FALSE
boolean detect_trigger_condition()
{
  
  if (acceleration_threshold_count >= acceleration_threshold_count_required) {
     return true;
  }
  return false;
}



// This is a wrapper function that:-
// * Writes the data to the fRAM - BANK 1
// * Writes address to which we wrote the data into, into BANK 2
// * Updates BANK3 with oldest data in BANK 2.
//
// The first two bytes are reserved for pointer to where we need to find the appropriate ADDR
//
void writeFramPacket(byte * data, int packlen)
{
  int difference;
  
  // Write to Bank 1, but only while:-
  // No launch has been detected
  // or
  // When a launch is detected and we haven't written more then XX number of butes
  if (
      (! launch_begun )
      ||
      (
        (launch_begun)
        &&
        (fram1Addr > Fram1StartPos)
      )
      ||
      (
        (launch_begun)
        &&
        (advanceFram1Addr(fram1Addr, packlen) < Fram1StartPos)
      )      
     ) {
     fram.write(fram1Addr, (uint8_t *) data, packlen);
     fram1Addr = advanceFram1Addr(fram1Addr, packlen);
     } else {

#ifdef DEBUG
        memory_end = micros();
        long total_time = (memory_end - memory_start)/1000000;
        Serial.print("Saved "); Serial.print(total_time); Serial.println(" seconds of data.");
#endif
        
        // Don't try and get any more data, no point as it will not be written to fRAM.
        no_more_space = true;
        while (1) {
          slowFlash(); // Slow flash indicates no more space to write data.
        }
     }


  // Write the framAddr to Bank 2 - ONLY when no launch has been detected.
  if (! launch_begun) {
    if (fram1Addr > Fram1StartPos) {
       difference = fram1Addr - Fram1StartPos;
    } else {
       difference = (fram1AddrEnd - fram1AddrStart) - (Fram1StartPos - fram1Addr);
    }
    
    if (difference < historicalBytes) {
       Fram1StartPos = 0;
    } else {
       Fram1StartPos = advanceFram1Addr(Fram1StartPos, packlen);
    }

     // Write the Start Position to the 3rd bank
     fram.write8(framPtr, (Fram1StartPos >> 8) & 0xff);
     fram.write8(framPtr+1, Fram1StartPos & 0xff);

  }

// #ifdef DEBUG
//    Serial.print("Current pos: "); Serial.println(fram1Addr);
//    Serial.print("Start   pos: "); Serial.println(Fram1StartPos);  
// #endif
}


int advanceFram1Addr(int addr, int len)
{
   addr = addr + len;
   if (addr >= (fram1AddrEnd - fram1MaxPacketSize)) {
     addr = fram1AddrStart;
   }
  return addr;
}



int getFram1Start()
{
  int startFram1Addr;
  byte bank1_smsb, bank1_slsb;
  
  bank1_smsb = fram.read8(framPtr);
  bank1_slsb = fram.read8(framPtr+1);

  startFram1Addr = ((bank1_smsb << 8) | bank1_slsb);  

  return startFram1Addr;   
}




boolean check_calibration_values()
{
  boolean calibration_issue = false;

  // Gyroscope Calibration issues detection
  if (abs(gyroXHigh) > typicalGyroCal || abs(gyroXLow) > typicalGyroCal) {
     calibration_issue = true;
  }

  if (abs(gyroYHigh) > typicalGyroCal || abs(gyroYLow) > typicalGyroCal) {
     calibration_issue = true;
  }

  if (abs(gyroZHigh) > typicalGyroCal || abs(gyroZLow) > typicalGyroCal) {
     calibration_issue = true;
  }
    
  
 return calibration_issue;
}


void errorCondition()
{
  while (1)
  {
    digitalWrite(LED_INDICATOR_PIN, HIGH);
    delay(200);
    digitalWrite(LED_INDICATOR_PIN, LOW);
    delay(100);
  }
}

void slowFlash()
{
  int i = 0;
  while (i < 5)
  {
    digitalWrite(LED_INDICATOR_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_INDICATOR_PIN, LOW);
    delay(200);
    i++;
  }
}

// 
// We write data to fram continuously, however, when launch condition is detected, we need to determine where
// we are up to in the fram (the address) and based on the historical record we keep, figure out where we 
// want to stop. This is so we don't write over historical data
//
// The IMU is positioned so that Y-axis is up/down, so it is this axis that will experience the acceleration
// Once the threshold has been reached (certain number of reads above a certain value) on the y-Axis, we know
// that the rocket must be moving!
//
void launch_detection()
{
  if (! launch_begun) {
    launch_begun = detect_trigger_condition();

    if (launch_begun) {
#ifdef DEBUG        
      Serial.print("Launch Time: ");
      Serial.println(micros());
#endif      
      // Initialise Timer, so that a servo move begins in 0.1 seconds
      initialise_servo_move(63);
      // 63   = 0.002048 seconds .... So if we do this 100 times...equates to 0.2 seconds
      // 127  = 0.004096 seconds
      // 254  = 0.00816 seconds
      
      
#ifdef DEBUG      
       memory_start = micros();
       Serial.println();
       Serial.println();
       Serial.println();
       Serial.println("*******  LAUNCH DETECTED *******");
       Serial.println();
       Serial.println();
       Serial.println();
#endif       
       digitalWrite(LED_INDICATOR_PIN, LOW);
    }
  }
}  




// Initialise timer2 for movement of the servo after launch detection
void initialise_servo_move(int timer2)
{
  // int timer2 = 63; // interrupt freq = 8,000,000 / (256 * (63 + 1)) = 488hz == 0.002seconds
  // VALUE = (8000000/FREQ)/256 - 1
  timer2_count = 0;

  cli();
 
  // Clear Timer2 - If we don't do this, it fires immediately!
  TIFR2 = _BV(OCF2A);

  // TIMER2
  TCCR2A  = 0;// set entire TCCR1A register to 0
  TCCR2B  = 0;// same for TCCR1B
  TCNT2   = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR2A   = timer2;    // = (8*10^6) / (164*1) - 1 (must be <65536)    ... This is just an example calc
  // turn on CTC mode
  TCCR2B |= (1 << WGM22);
  // Scaling - 256
  TCCR2B |= (1 << CS22);

  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();

}


// Move Top servo     
void set_top_servo_position(double degrees)
{
  byte byte_degrees = (byte) degrees;
#ifdef DEBUG  
  Serial.print("Moving Top Servos ");
  Serial.println(byte_degrees);
#endif    
  topservo.write(byte_degrees);

  recordservomove (top_servo, byte_degrees);
}



void moveTopMass(double angle)
{
  set_top_servo_position (20 + (angle  * gear_ratio));
}



void moveBottomMass(double angle)
{
  set_bottom_servo_position (20 + ((360 - angle)  * gear_ratio));
}



void rotateBottomMass()
{
  int i = 360;
  moveBottomMass(i);
  delay(1000);
  while (i > 0) {
    moveTopMass(i);
    moveBottomMass(i);
    delay(50);
    i = i -5;
  }
}


// Move Bottom servo     
void set_bottom_servo_position(double degrees)
{
  byte byte_degrees = (byte) degrees;
#ifdef DEBUG  
  Serial.print("Moving Bottom Servo: ");
  Serial.println(byte_degrees);
#endif    
  bottomservo.write(byte_degrees);

  recordservomove (bottom_servo, byte_degrees);
}


void recordservomove(byte servo, byte angle)
{
  unsigned long    servomove_time;
  servomove_time = micros();

  // Length of packet, servo moved and angle it is moved to
  _buff[0] = 7;
  _buff[1] = servo;
  _buff[2] = angle;

  // Time
  _buff[3] = servomove_time & 0xFF;
  _buff[4] = (servomove_time >> 8 ) & 0xFF;
  _buff[5] = (servomove_time >> 16) & 0xFF;
  _buff[6] = (servomove_time >> 24) & 0xFF;

  if (fram_installed && ! no_more_space) {
     writeFramPacket(&_buff[0], 7);
  }  
}


// Move Weights into required 'start' position
void weights_starting_pos()
{
   rotateBottomMass ();
   moveTopMass (90);
   moveBottomMass (270);   
}   
