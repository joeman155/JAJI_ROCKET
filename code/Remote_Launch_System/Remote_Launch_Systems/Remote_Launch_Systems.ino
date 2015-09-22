/* REMOTE ROCKET LAUNCH SYSTEM - RRLS */

#include <voltage.h>
#include <Kalman.h>
#include <SPI.h>
#include <SD.h>
#include <SoftI2C.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <SFE_LSM9DS0.h>
#include <SFE_BMP180.h>
 


// delay between measurements
#define LOOP_DELAY 50

// Pins
const int  continuitySensePin = 8;  // This is the pin number...not direct access
const int  powerPin           = 6;  // PORTH, 6   -- Digital pin 9
const int  armPin             = 3;  // PORTH, 3   -- Digital pin 6
const int  continuityTestPin  = 5;  // This is the pin number...not direct access
const int  launchPin          = 3;  // This is the pin number...not direct access
const int  igniterPsuPin      = A3;
const int  arduinoPsuPin      = A2;
const int  igniterBurnDelay   = 2000;
const int  launch_countdown_delay = 6000;  // The 5 second countdown.
int state;

// SD Card and file declarations
const int chipSelect = 4;
File myFile;
int sdCardState = 0;  // 0 = uninitialised , -1 = error, 1 = initialised

// Generic declarations
char inData[20]; // Allocate some space for the string
char noCommand[1];
char inChar=-1; // Where to store the character read
char nextChar=-1; // Where we store the peeked character
byte index = 0; // Index into array; where to store the character

#define M_PI 3.14159265
#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI)) 
#define DEGREES_TO_RADIANS(degrees) ((degrees) * (M_PI / 180.0)) 

static char outstr[15];
String str;

// Other config
long heartbeat_count;
int  continuityState = 1;         // current state of the button

// Timing
uint32_t ulCur;


// Menu
unsigned long menutime = 200;
int EndFlag = 0;
char param[10];  // Parameter for functions called when requests sent.
const boolean menu_enabled = true;

// States
short int cutdown = 0; // Start up disabled

// Voltage Sensors
VOLTAGE ignpsu;
VOLTAGE ardupsu;

// IMU
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
const byte INT1XM = 2; // INT1XM tells us when accel data is ready
const byte INT2XM = 3; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready

// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

// KALMAN
uint32_t timer;
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Yaw
Kalman kalmanY; // Pitch
Kalman kalmanZ; // Role
float yaw, roll, pitch;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter


// GPS
TinyGPS gps;
bool newData;
unsigned int read_time = 100;


// RTC
#define DS3232_I2C_ADDRESS 0x68

// Air Pressure - BMP180
SFE_BMP180 pressure;
double bmp180_pressure, bmp180_temperature;

// Debugging
unsigned short int DEBUGGING = 1;


void setup() {
  // Serial Port we program with
  Serial.begin(115200);
  
  // GPS
  Serial1.begin(4800);
  
  // Radio Modem
  Serial2.begin(57600);
  sendPacket ("S");  
  
  // i2C
  Wire.begin();
 
  // Initialise IMU
  init_imu();

  // Initialise BMP180
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    sendPacket("E06");
  }
  
  // Initialisations
  heartbeat_count = 1;

  // Initialise the voltage measurements
  ignpsu.setup(igniterPsuPin, 10, 4.7); 
  ardupsu.setup(arduinoPsuPin, 15, 4.7); 
 
  // Initialise the Launch Systems
  initLaunchSystem();
  resetLaunchSystem();

  // Initialise microSD card.
  if (!SD.begin(chipSelect)) {
    sdCardState = -1;
    sendPacket("E00");
  } else {
    sdCardState = 1;
  }
}

void loop() {

 // Get Serial Input (menu) 
 if (menu_enabled) {
    pollSerial();
    
    // Just allow enough time for responses, etc to make their way through.
    delay(50);    
 }
 

  
 // Heartbeat
 heartbeat();
 

 // Air Pressure, Temperature, voltages
 // Prefix: D00
 // Format of string is:-
 // D00:InternalTemp,ExternalTemp,AirPressure,CPUVoltage,IGNVoltage
 //  - Temperatures are in Kelvin
 //  - Pressures are in Pascals
 extractPressureTemperature();
 bmp180_temperature += 273;
 sendPacket(String("D00:") + String(bmp180_pressure), false);
 sendPacket(String(",")    + String(bmp180_temperature), false);  // NOTE: Will need to put external temp here....for now, we are duplicating internal temp
 sendPacket(String(",")    + String(bmp180_temperature),false); 
 
 // also wish to package voltage readings along with other measurements immediately above.
 ardupsu.read();
 ignpsu.read();
 dtostrf(ardupsu.value(),4, 2, outstr);   
 sendPacket (String(",") + String(outstr), false); 
 dtostrf(ignpsu.value(),5, 2, outstr);  
 sendPacket (String(",") + String(outstr), true);   



 // GPS Tracking
 // Prefix: D01
 // Format of string is:-
 // D01:latitude,longitude,altitude,date,time,heading,speed,#satellites
 //
 // date is in format dd/mm/yyyy
 // time is in format hh.mintes.seconds.hundreths
 extractGPSInfo();



 // Local Time (Time since startup, or reboot)
 // Prefix: D02
 ulCur = micros();
 sendPacket(String("D02:") + String(ulCur));
 
  
  
 // IMU Code
 // Prefix: D06
 // Format of string is:-
 // D06:Roll,Pitch,Yaw,gyroX,gyroY,gyroZ,accX,accY,accZ,timer
 extractIMUInfo();

   
   
 // Launch System status
 // Prefix: D07, D08
 sendPacket(String("D07:") + String(isLaunchSystemPowered()));
 sendPacket(String("D08:") + String(isLaunchSystemArmed()));  
 
  
 delay(LOOP_DELAY);
}


int processRxSerial(char *rxString)
{
 int theend = 0;  // Indicates to calling routine if we are ending our listening on the Serial Port
 
 param[0] = '\0';
 // Determine action to take
 if (rxString[0] != '\0') {
    // sendPacket(String("Received: ") + String(rxString));
 
    // Decode rxString....to get parameter (if provided)
    if (strlen(rxString) > 4) {
       // Make sure next character is a colon.
       if (strncmp(rxString+3, ":",1) == 0) {
          strncpy(param, rxString+4, strlen(rxString) -4);
       }
    }


// DEBUGGING
//   Serial2.print("Param:");
//   Serial2.println(String(param));
//   Serial2.println(strlen(param));   

      
   /* Command Checks */
   if (strcmp(rxString, "R00") == 0) 
   {
      sendPacket(String("A00"));
      delay(100); // Delay exiting.... so to give ground station enough time to process.
      theend = 1;     
   }     
   else if (strncmp(rxString, "R01", 3) == 0) 
   {
      state = powerRequest(param);
      sendPacket(String("A01:") + String(state));      
   } 
   else if (strcmp(rxString, "R08") == 0) 
   {
      sendPacket(String("A08:") + String(isLaunchSystemPowered()));            
   }   
   else if (strcmp(rxString, "R03") == 0) 
   {
      state = checkContinuity();
      sendPacket(String("A03:") + String(state));
   } 
   else if (strcmp(rxString, "ST") == 0) 
   {
      ardupsu.read();
      dtostrf(ardupsu.value(),5, 2, outstr);   
      sendPacket (String(outstr)); 
   } else if (strcmp(rxString, "VT") == 0) 
   {
      ignpsu.read();
      dtostrf(ignpsu.value(),5, 2, outstr);  
      sendPacket (String(outstr));
   } 
   else if (strncmp(rxString, "R02", 3) == 0) 
   {
      state = armRequest(param);
      sendPacket(String("A02:") + String(state));
   } 
   else if (strcmp(rxString, "R05") == 0) 
   {
      // Download Photo - TODO
      sendPacket(String("A05"));        
   }    
   else if (strcmp(rxString, "R06") == 0) 
   {
      // Skip Download of photo - TODO
      sendPacket(String("A06"));        
   }       
   else if (strcmp(rxString, "R07") == 0) 
   {
      cutdown = 1;
      sendPacket(String("A07"));        
   }    
   else if (strcmp(rxString, "R09") == 0) 
   {
      sendPacket(String("A09:") + String(isLaunchSystemArmed()));        
   } 
   else if (strcmp(rxString, "R04") == 0) 
   {
      state = initiatePreLaunch();
      sendPacket(String("A04:") + String(state));
      
      // If all preLaunch steps worked, then do the launch!
      if (state == 1) {
         delay(launch_countdown_delay);
         initiateLaunch(); 
      }
   }    
   else if (strcmp(rxString, "R05") == 0) 
   {
      // Initiate File Transfer
      sendPacket(String("A05"));
   } else 
   {
      sendPacket("-9999");
   }
 }  
  
  return theend;
}



void heartbeat() 
{
  str = String("H:") + String(heartbeat_count);
  sendPacket(str);
  heartbeat_count++;
}


void sendPacket(String str) {
  Serial2.println(str);
  logString(str);
  
  // Debug to console, if debugging is enabled
  if (DEBUGGING == 1) {
    Serial.println(str);
  }
  
}


void sendPacket(String str, boolean eol) {
  if (eol) {
    Serial2.println(str);
    logString(str);
  } else {
    Serial2.print(str);
    logString(str, eol);
  }    
  
  // Debug to console, if debugging is enabled
  if (DEBUGGING == 1) {
    if (eol) {
       Serial.println(str);
    } else {
       Serial.print(str);
    }
  }
  
}


void pollSerial() 
{  
 // Make sure all prior data sent is REALLY sent and wait a little for it to be processed by groundstation.  
 Serial2.flush();
 delay(200);
 
 sendPacket("M"); // Menu  (to tell the other end we are ready to receive commands)
  
 unsigned long startTime = millis();
 inData[0]    = '\0';
 EndFlag = 0;
 index = 0;
 

 
 while(!EndFlag) {
    if (millis() - startTime > menutime) {   
      EndFlag = 1;
    }
      
    while(Serial2.available() > 0 ) {
      
      // Accept nothing longer than 20 characters 
       if(index > 19) {
          break;   // To long...refusing to process.
       }
       inChar = Serial2.read(); // Read a character
       
       inData[index] = inChar; // Store it
       index++; // Increment where to write next
       inData[index] = '\0'; // Null terminate the string
       
       // If we detect carriage return, this means end of command (person/program) has hit enter
       if (inData[index-1] == '\n' && inData[index-2] == '\r') {
          inData[index-2] = '\0';                // Do not wish to include crlf in the text string
          EndFlag = processRxSerial(inData);     // Exit the whole "menu", IF processing asks us to do it  
          
          // If EndFlag == 0...then this means we still want to be in the menu.
          // So we prompt the groundstation with the menu again.
          if (EndFlag == 0) {
             sendPacket("M");
          }
          
          // Resetting everything back ready for next string
          index = 0;          
          inData[0] = '\0';
          
          // Get out of the first while loop...back to menu
          break;          
       }
       

    }
  }
  
}


void relayOn(int v_pin) {
  digitalWrite(v_pin, HIGH);
}


void relayOff(int v_pin) {
  digitalWrite(v_pin, LOW);
}



// Reset system back to default position...which is everything off
void resetLaunchSystem() {
  relayOff(launchPin);   
  relayOff(continuityTestPin); 
  poweroffLaunchSystem();
  disArmLaunchSystem();
}


// Check Continuity of circuit
// Returns :-
// 2 if no power is applied.
// 1 if there is continuity
// 0 if there is no continuity
int checkContinuity(){

  // Check if Power is on
  if (! isLaunchSystemPowered()) return 2;
  
  relayOn(continuityTestPin);   // turn the LED on (HIGH is the voltage level)
  delay(100);                              // wait briefly... incase of contact bounce
    
  continuityState = digitalRead(continuitySensePin);

  delay(100);                              // wait briefly... incase of contact bounce
  relayOff(continuityTestPin);   // turn the LED on (HIGH is the voltage level) 
  delay(50);                              // wait briefly... incase of contact bounce  
  
  // A high on pin...means no continuity...
  // a low on pin means there IS continuity
  if (continuityState == LOW) {
     return 1;
  }  else {
     return 0;
  }
}


// initialise the 'Launcher' pins and system
void initLaunchSystem() {
  pinMode(powerPin, OUTPUT);
  pinMode(armPin, OUTPUT);
  pinMode(continuityTestPin, OUTPUT);
  pinMode(launchPin, OUTPUT);  
  pinMode(continuitySensePin, INPUT);
 
}

// Attempt to initiate the launch
// Returns:-
// 2 if system is not powered
// 3 if system is not armed
// 4 if continuity test failed
// 1 if the launch sequence completes successfully
//
// NOTE: Regardless of the  outcome, the system is dis-armed
//
int initiatePreLaunch() {
  int result;
  
  // Check if Power is on
  if (! isLaunchSystemPowered()) return 2;
  
  // Check if System is armed
  if (! isLaunchSystemArmed()) return 3;
  
  // Perform continuity test
  if (checkContinuity() != 1) {
     // Continuity failed...but first disarm system...as part of safety process.
     result = disArmLaunchSystem();    
     return 4;
  }
  // Got to here, so must be okay.
  return 1;
}

// Initiate the launch!    
int initiateLaunch() {  
  int result;
  
  relayOn(launchPin);
  delay(igniterBurnDelay);
  relayOff(launchPin);
  
  // Then disarm system
  result = disArmLaunchSystem();
  
  return 1;
}


// Return true or false...to indicate if system is powered or not
int isLaunchSystemPowered() {
  return bitRead(PORTH,powerPin);
  
}

// Return true or false...to indicate if system is armed or not
boolean isLaunchSystemArmed() {
  return bitRead(PORTH,armPin); 
}


// Perform requested operation on Launch Power and returns result:-
// 0 if power is off
// 1 if power is on
// 2 if there was an error...not understanding input parameter)
int powerRequest(char *param) {
   if (strlen(param) == 0) {
      togglepowerLaunchSystem();  
   } else if (strcmp(param, "1") == 0) {
      poweronLaunchSystem();
   } else if (strcmp(param, "0") == 0) {
     poweroffLaunchSystem();
   } else {
     return 2;
   }
   
   return isLaunchSystemPowered();
}

// Toggle power on/off (opposite of what it is now)
void togglepowerLaunchSystem() {
  if (isLaunchSystemPowered() == 0) {
    poweronLaunchSystem();
  } else {
    poweroffLaunchSystem();
  }
}


// Turn Power on
void poweronLaunchSystem() {
   bitWrite(PORTH, powerPin, HIGH);
}

// Turn Power off (We dis-arm for safety purposes as well)
void poweroffLaunchSystem() {
   int result = disArmLaunchSystem();   // Disarm system as well (if it is armed)
   bitWrite(PORTH, powerPin, LOW);
}


// Handle request to Arm
// Returns value:-
// 0 if system is disarmed
// 1 if system is armed
// 9 if there is an error
int armRequest(char *param) {
   int result;
   if (strlen(param) == 0) {
      result = togglearmLaunchSystem();  
   } else if (strcmp(param, "1") == 0) {
      result = armLaunchSystem();
   } else if (strcmp(param, "0") == 0) {
      result = disArmLaunchSystem();
   } else {
     result = 9;
   }
 
   // If we were ableto successfully perform the operation, we return the arm/dis-arm status
   // Otherwise, we return the result (error)
   if (result == 1) {
      return isLaunchSystemArmed();
   } else {
      return result;
   }
}

// Toggle Arm (i.e. do opposite of current setting)
int togglearmLaunchSystem() {
  int result;
  if (isLaunchSystemArmed() == 0) {
    result = armLaunchSystem();
  } else {
    result = disArmLaunchSystem();
  }
  return result;
}

// Arm system
// Note: will only arm system if power is on. Errors if no power on.
int armLaunchSystem() {
  
  // Check if Power is on
  if (! isLaunchSystemPowered()) return 2;
  
  // Perform continuity test
  if (checkContinuity() != 1) return 3;
  
  bitWrite(PORTH, armPin, HIGH);
  return 1;
}

// Dis-arm system
int disArmLaunchSystem() {
  bitWrite(PORTH, armPin, LOW);
  return 1;
}


// Log string to log file
void logString(String str) {
  // ONLY proceed if no sdcard errors
  if (sdCardState == 1) {
     myFile = SD.open("log.txt", FILE_WRITE);

     // if the file opened okay, write to it:
     if (myFile) {
       myFile.println(str);
       myFile.close();
     } else {
      sdCardState = -1; // Error occured
     }
  } 
}



// Log string to log file
void logString(String str, boolean eol) {
  // ONLY proceed if no sdcard errors
  if (sdCardState == 1) {
     myFile = SD.open("log.txt", FILE_WRITE);

     // if the file opened okay, write to it:
     if (myFile) {
       if (eol) {
          myFile.println(str);
       } else {
          myFile.print(str);
       }
       myFile.close();
     } else {
      sdCardState = -1; // Error occured
     }
  } 
}



// Here's a fun function to calculate your heading, using Earth's
// magnetic field.
// It only works if the sensor is flat (z-axis normal to Earth).
// Additionally, you may need to add or subtract a declination
// angle to get the heading normalized to your location.
// See: http://www.ngdc.noaa.gov/geomag/declination.shtml
void printHeading(float hx, float hy)
{
  float heading;
  
  if (hy > 0)
  {
    heading = 90 - (atan(hx / hy) * (180 / PI));
  }
  else if (hy < 0)
  {
    heading = - (atan(hx / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hx < 0) heading = 180;
    else heading = 0;
  }
  
  Serial.print("Heading: ");
  Serial.println(heading, 2);
}

void getHeading(float hz, float hy, double *heading)
{
  
  if (hy > 0)
  {
    *heading = 90 - (atan(hz / hy) * (180 / PI));
  }
  else if (hy < 0)
  {
    *heading = - (atan(hz / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hz < 0) *heading = 180;
    else *heading = 0;
  }  
  
}






void readDS3232time(byte *second, 
byte *minute, 
byte *hour, 
byte *dayOfWeek, 
byte *dayOfMonth, 
byte *month, 
byte *year)
{
  Wire.beginTransmission(DS3232_I2C_ADDRESS);
  Wire.write(0); // set DS3232 register pointer to 00h
  Wire.endTransmission();  
  Wire.requestFrom(DS3232_I2C_ADDRESS, 7); // request 7 bytes of data from DS3232 starting from register 00h

  // A few of these need masks because certain bits are control bits
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
}





void extractPressureTemperature()
{
  char status;
  double p0,a;
  
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    status = pressure.getTemperature(bmp180_temperature);
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        status = pressure.getPressure(bmp180_pressure,bmp180_temperature);
        if (status != 0)
        {
          // Print out the measurement:
          // SUCCESS
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}



// Convert vector xb,yb,zb back to Global Reference Frame
// a,b,c are the angles (yaw, pitch and roll) - all in degrees.
// The results are put into the xg, yg and zg
void rotateMatrix(double xb, double yb, double zb, double a, double b, double c,
                  double *xg, double *yg, double *zg)
{
  float M[3][3];     // Rotation Matrix
  // float xg, yg, zg;  // Vector in the global reference frame
  
  // Convert to Radians as cosine/sine expect angles in Radians
  a = DEGREES_TO_RADIANS(a);
  b = DEGREES_TO_RADIANS(b);
  c = DEGREES_TO_RADIANS(c);
  
  // Initialise the Matrix based on the angle of the Body co-ordinate system.
  M[0][0] = cos(a) * cos(b);
  M[0][1] = cos(c) * sin(a) + cos(a) * sin(c) * sin(b);
  M[0][2] = sin(c) * sin(a) - cos(c) * cos(a) * sin(b);
  
  M[1][0] = -cos(b) * sin(a);
  M[1][1] = cos(c) * cos(a) - sin(c) * sin(b) * sin(a);
  M[1][2] = cos(a) * sin(c) + cos(c) * sin(b) * sin(a);
  
  M[2][0] = sin(b);
  M[2][1] = -cos(b) * sin(c);
  M[2][2] = cos(b) * cos(c);
  
  
  // Compute the vector x,y,z in the Global Reference system
  *xg = M[0][0] * xb + M[0][1] * yb + M[0][2] * zb;
  *yg = M[1][0] * xb + M[1][1] * yb + M[1][2] * zb;
  *zg = M[2][0] * xb + M[2][1] * yb + M[2][2] * zb;
  
  
}


// Extract and calculate information for IMU.
void extractIMUInfo()
{
    // Read Accelerometer
  dof.readAccel();
  accX = dof.calcAccel(dof.ax);
  accY = dof.calcAccel(dof.ay);
  accZ = dof.calcAccel(dof.az);  


  // Heading from Magnetometer.
  double heading;  
  dof.readMag();  
  getHeading((float) dof.mz, (float) dof.my, &heading);  


  // Read Gyro 
  dof.readGyro();
  gyroX = dof.calcGyro(dof.gx);
  gyroY = dof.calcGyro(dof.gy);
  gyroZ = dof.calcGyro(dof.gz);  


  // Time Steps
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();  
  
   
  // Derive Roll and Pitch
  double roll, pitch;
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accX) * RAD_TO_DEG;
  pitch = atan(-accZ / sqrt(accY * accY + accX * accX)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accZ * accZ + accX * accX)) * RAD_TO_DEG;
  pitch = atan2(-accZ, accX) * RAD_TO_DEG;
#endif

  
  double gyroXrate = gyroX; 
  double gyroYrate = gyroY; 
  double gyroZrate = gyroZ; 
  yaw = heading;
  
  // Use Kalman filter to get new Pitch and Role
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleZ > 90) || (roll > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(roll);
    kalAngleZ = roll;
  } else
    kalAngleZ = kalmanZ.getAngle(roll, gyroZrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroZrate = -gyroZrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleZ = kalmanZ.getAngle(roll, gyroZrate, dt); // Calculate the angle using a Kalman filter
#endif


  // Use Kalman to derive Yaw
  kalAngleX = kalmanX.getAngle(yaw, -gyroXrate, dt); // Calculate the angle using a Kalman filter
  yaw = kalAngleX;
 
  // Reverse Angle of Pitch
  pitch = -kalAngleY;

  // Roll == kalAngleZ
  roll = kalAngleZ;

 // Correct Acceration - (with board with components up, up (z) is positive), but board is other way around
 // with components down. Acceration should be positive. (Though it reads negative)
 accZ = accZ * -1;

  
  /* Print Useful Data */
#if  0            // Set to 1 to activate
  Serial.print("RAW: "); 
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
  Serial.print("\t");

  Serial.print(String("Roll: ") + roll); Serial.print("\t");
  Serial.print(String("Pitch: ") + pitch); Serial.print("\t");
  Serial.print(String("Yaw: ") + yaw); Serial.println("\t");
#endif

 sendPacket(String("D06:") + String(roll), false);
 sendPacket(String(",") + String(pitch), false);  
 sendPacket(String(",") + String(yaw), false);  
 sendPacket(String(",") + String(gyroX), false);  
 sendPacket(String(",") + String(gyroY), false);  
 sendPacket(String(",") + String(gyroZ), false);  
 sendPacket(String(",") + String(accX), false);  
 sendPacket(String(",") + String(accY), false);  
 sendPacket(String(",") + String(accZ), false);   
 sendPacket(String(",") + String(timer));    

}


// Initialise the IMU
void init_imu()
{
  uint16_t status = dof.begin();
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
  
  delay(100); // Wait for sensor to stabilize
  dof.readAccel();
  accX = dof.calcAccel(dof.ax);
  accY = dof.calcAccel(dof.ay);
  accZ = dof.calcAccel(dof.az);  
  
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accX) * RAD_TO_DEG;
  pitch = atan(-accZ / sqrt(accY * accY + accX * accX)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accZ * accZ + accX * accX)) * RAD_TO_DEG;
  pitch = atan2(-accZ, accX) * RAD_TO_DEG;
#endif

  // Assuming the system is level, we use Magnetometer to get initial bearing.
  dof.readMag();
  double heading;
  getHeading(dof.mz, dof.my, &heading);

  // Get initial angels
  kalmanX.setAngle(heading); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(roll);
  yaw = heading;
  /*
  Serial.println(String("yaw: ") + yaw);    
  Serial.println(String("roll: ") + roll);
  Serial.println(String("pitch: ") + pitch);
  */
  
  timer = micros();
  delay(3000);
}  



// Extract and print GPS Info
void extractGPSInfo()
{
  
 newData = false;
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < read_time;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
         newData = true;
    }
  }  
  
  // If we have new data, we send it.
  if (newData)
  {
    float flat, flon;
    float flat_processed, flon_processed;
    short int sat_count;
    unsigned long age;
    unsigned long speed, course, altitude;
    int year;
    byte month, day, hour, minute, second, hundredths;    
    
    gps.f_get_position(&flat, &flon, &age);
    flat_processed = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    flon_processed = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    sat_count      = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
    
    // latitude
    dtostrf(flat_processed, 12, 8, outstr);
    sendPacket(String("D01:") + String(outstr), false); 
    
    // longitude
    dtostrf(flon_processed, 12, 8, outstr);
    sendPacket(String(",") + String(outstr), false);;
    
    // altitude
    altitude = gps.altitude()/100;
    
    // date/time
    gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
    sendPacket(String(",") + String(day) + String("/") + String(month) + String("/") + String(year), false);
    sendPacket(String(",") + String(hour) + String(".") + String(minute) + String(".") + String(second) + String(".") + String(hundredths), false);
    
    // heading
    course = gps.f_course();    
    sendPacket(String(",") + String(course), false);
    
    // speed
    speed = gps.f_speed_kmph();
    sendPacket(String(",") + String(speed), false);
    
    // # of Satellites
    sendPacket(String(",") + String(sat_count));
  }  
}  


// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ((val/10*16) + (val%10));
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}


