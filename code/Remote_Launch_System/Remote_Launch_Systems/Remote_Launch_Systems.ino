#include <voltage.h>
#include <Kalman.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftI2C.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <SFE_BMP180.h>



// Pins
const int  continuitySensePin = 8;  // This is the pin number...not direct access
const int  powerPin = 6;            // PORTH, 6   -- Digital pin 9
const int  armPin   = 3;            // PORTH, 3   -- Digital pin 6
const int  continuityTestPin = 5;   // This is the pin number...not direct access
const int  launchPin = 3;           // This is the pin number...not direct access
const int  igniterPsuPin = A3;
const int  arduinoPsuPin = A2;
const int  igniterBurnDelay = 2000;
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

// Menu
unsigned long menutime = 5000;
int EndFlag = 0;
char param[10];  // Parameter for functions called when requests sent.
const boolean menu_enabled = false;

// States
short int cutdown = 0; // Start up disabled

// Sensors
VOLTAGE ignpsu;
VOLTAGE ardupsu;

// IMU
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 500 // 500 ms between prints


float gx_avg = 0;
float gy_avg = 0;
float gz_avg = 0;

float gx_bias = 0.66;
float gy_bias = -0.31;
float gz_bias = -6.39;

float gx_val;
float gy_val;
float gz_val;
unsigned long n = 0;
unsigned long m = 0;
unsigned long o = 0;

float x,y,z;       // location
float vx,vy,vz;    // velocity


// KALMAN
uint32_t timer;
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;
float yaw, roll, pitch;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
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
#define ALTITUDE 45.0 // Altitude of Cairns address

// Debugging
unsigned short int DEBUGGING = 1;
void setup() {
  // Serial Port we program with
  Serial.begin(57600);
  
  // GPS
  Serial1.begin(4800);
  
  // Radio Modem
  Serial2.begin(57600);
  sendPacket ("S");  
  
  // i2C
  Wire.begin();
 
  // Initialise IMU
  uint16_t status = dof.begin();
  delay(100); // Wait for sensor to stabilize
  dof.readAccel();
//  Serial.println("LSM9DS0 WHO_AM_I's returned: 0x");
//  Serial.println(status, HEX);
//  Serial.println("Should be 0x49D4");
  accX = dof.calcAccel(dof.ax);
  accY = dof.calcAccel(dof.ay);
  accZ = dof.calcAccel(dof.az);  
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  // Assuming the system is level, we use Magnetometer to get initial bearing.
  dof.readMag();
  double heading;
  getHeading(dof.mx, dof.my, &heading);


  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(heading);
  gyroXangle = roll;
  gyroYangle = pitch;
  yaw = heading;
  Serial.println(String("yaw: ") + yaw);    
  Serial.println(String("roll: ") + roll);
  Serial.println(String("pitch: ") + pitch);
  delay(3000);
  
  // INITIALISE INITIAL CONDITIONS
  x = y = z = 0;
  vx = vy = vz = 0;

  // Initialise BMP180
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
  
  // Initialisations
  heartbeat_count = 1;

  ignpsu.setup(igniterPsuPin, 10, 4.7); 
  ardupsu.setup(arduinoPsuPin, 15, 4.7); 
 
  initLaunchSystem();
  resetLaunchSystem();

  // Initialise microSD card.
  if (!SD.begin(chipSelect)) {
    sdCardState = -1;
    sendPacket("E00");
  } else {
    sdCardState = 1;
  }


timer = micros();
}

void loop() {

 // Get Serial Input (menu) 
 if (menu_enabled) {
    pollSerial();
 }
  
 // Heartbeat
 // heartbeat();



 // Air Pressure, Temperature
 // Prefix: D00
 // DISABLE PRESSURE FOR NOW
//  displayPressure();
  

 // GPS Tracking
 // Prefix: D01
 // DISABLED GPS FOR NOW WHILE WE DEVELOP IMU CODE
 /*
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
    gps.f_get_position(&flat, &flon, &age);
    flat_processed = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    flon_processed = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    sat_count      = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
    
    dtostrf(flat_processed, 12, 8, outstr);
    sendPacket(String("LAT=") + String(outstr));
    
    dtostrf(flon_processed, 12, 8, outstr);
    sendPacket(String("LON=") + String(outstr));;
    
    sendPacket(String("SAT=") + String(sat_count));
    // sendPacket(" PREC=");
    //sendPacket(String(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop()));
  }  
  */


 // Local Time 
 // Prefix: D02
 // DISABLE TIME FOR NOW
//  displayTime();
 


 // Voltages
 // Prefix: D04, D05
 // DISABLED VOLTAGES WHILE WE DEVELOP OTHER CODE
 /*
 ardupsu.read();
 dtostrf(ardupsu.value(),5, 2, outstr);   
 sendPacket (String("D04:") + String(outstr)); 

 ignpsu.read();
 dtostrf(ignpsu.value(),5, 2, outstr);  
 sendPacket (String("D05:") + String(outstr));   
 */
  
 // IMU Code
 // Prefix: D06
  // printGyro();  // Print "G: gx, gy, gz"
  // printAccel(); // Print "A: ax, ay, az"
  //printMag();   // Print "M: mx, my, mz"
  
  // Heading from Magnetometer.
  double heading, heading_tmp;
  heading = 0;

  
  dof.readAccel();
  dof.readMag();
  accX = dof.calcAccel(dof.ax);
  accY = dof.calcAccel(dof.ay);
  accZ = dof.calcAccel(dof.az);  
  getHeading((float) dof.mx, (float) dof.my, &heading_tmp);  
  heading += heading_tmp;  
  dof.readAccel();
  dof.readMag();
  accX = dof.calcAccel(dof.ax) + accX;
  accY = dof.calcAccel(dof.ay) + accY;
  accZ = dof.calcAccel(dof.az) + accZ;  
  getHeading((float) dof.mx, (float) dof.my, &heading_tmp);  
  heading += heading_tmp;  
  dof.readAccel();
  dof.readMag();
  accX = (dof.calcAccel(dof.ax) + accX)/3;
  accY = (dof.calcAccel(dof.ay) + accY)/3;
  accZ = (dof.calcAccel(dof.az) + accZ)/3;  
  getHeading((float) dof.mx, (float) dof.my, &heading_tmp);  
  heading += heading_tmp;  
  
  heading = heading / 3;
  
  dof.readGyro();
  gyroX = dof.calcGyro(dof.gx);
  gyroY = dof.calcGyro(dof.gy);
  gyroZ = dof.calcGyro(dof.gz);  
  
 
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();  
  
  // Serial.println(String("DT: " ) + String(dt));
  
  // Derive Roll and Pitch
  double roll, pitch;
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif




  
  double gyroXrate = gyroX; 
  double gyroYrate = gyroY; 
  double gyroZrate = gyroZ; 
  yaw = heading;
  
  // Use Kalman filter to get new Pitch and Role
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;


  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  // Use Kalman to derive Yaw
  // yaw = yaw + (gyroZ * dt)+.07;   // Number of 0.07 was deduced from experiment...seems to keep Yaw about the same value (0) when we first turn on
  // Serial.print("yaw :"); Serial.print(yaw); Serial.print("\t"); Serial.print("Gyrorate: "); Serial.println(gyroZrate);
  kalAngleZ = kalmanZ.getAngle(yaw, -gyroZrate, dt); // Calculate the angle using a Kalman filter
  yaw = kalAngleZ;
  // Serial.print("NEW yaw :"); Serial.println(yaw); 
  
  
  
  // Reverse Angle of Pitch
  pitch = -kalAngleY;


// Adjust Roll (because we have the board upside down)
roll = -(180 - kalAngleX);
if (roll < -180 & roll > -360) {
  roll  = roll + 360;
}

// Correct Acceration - (with board with components up, up (z) is positive), but board is other way around
// with components down. Acceration should be positive. (Though it reads negative)
accZ = accZ * -1;



 


// Convert acceleration components to Global Co-Ordinate system...which is assumed to be the starting position
// Which has z straight up, x and y in the plane and perpendicular
float accXg, accYg, accZg;
rotateMatrix(accX, accY, accZ, yaw, pitch, roll, &accXg, &accYg, &accZg);


  /* Print Data */
#if 0             // Set to 1 to activate
  Serial.print("RAW: "); Serial.print(accX); Serial.print("\t");
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

  if (o > 180) {
     Serial.print(heading); Serial.print("/");Serial.println(yaw);
     o = 1;
  } else {
    o++;
  }
     

// Remove gravity
accZg = accZg - 1.05;

if (abs(accXg) > 0.1 || abs(accYg) > 0.1 || abs(accZg) > 0.3) {
  if (n > 2) {
     // Serial.print("GRF:   "); Serial.print(accXg); Serial.print("\t"); Serial.print(accYg); Serial.print("\t"); Serial.print(accZg); Serial.print("\t"); Serial.println("Heading/Yaw: "); 
     n = 1;
  } else {
    n++;
  }
}

// Compute velocity updates
vx = vx + accXg * dt;
vy = vy + accYg * dt;
vz = vz + accZg * dt;





// Computer position
x = x + vx * dt;
y = y + vy * dt;
z = z + vz * dt;

  if (m > 30) {
     Serial.print("VX: "); Serial.print(vx); Serial.print("\t"); Serial.print("VY: "); Serial.print(vy); Serial.print("\t"); Serial.print("VZ: "); Serial.println(vz);
     // Serial.print("X: "); Serial.print(x); Serial.print("\t"); Serial.print("Y: "); Serial.print(y); Serial.print("\t"); Serial.print("Z: "); Serial.println(z);
     m = 1;
  } else {
    m++;
  }


accZg = accZg + 1.05;

/*
 float xbias = kalmanX.getBias();
 float ybias = kalmanY.getBias();
 Serial.print("XBias: "); Serial.println(xbias);
 Serial.print("YBias: "); Serial.println(ybias);  

  
//  Serial.print(roll); Serial.print("\t");
//  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

//  Serial.print(pitch); Serial.print("\t");
//  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
 */

  
  /*
  // Used to calculate bias
  if (n < 32767) {
    gx_avg = (n * gx_avg + dof.calcGyro(dof.gx))/(n + 1);
    gy_avg = (n * gy_avg + dof.calcGyro(dof.gy))/(n + 1);
    gz_avg = (n * gz_avg + dof.calcGyro(dof.gz))/(n + 1);
    n++;
  }
  
   Serial.println(String("CALC BIAS: ") + String(gx_avg) + String(",") + String(gy_avg) + "," + String(gz_avg));
  gx_bias = gx_avg;
  gy_bias = gy_avg;
  gz_bias = gz_avg;
  */
  
  // DO NOT WANT ALL IMU DATA FOR NOW
  /*
  // Print the heading and orientation for fun!
  printHeading((float) dof.mx, (float) dof.my);
  printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay), 
                   dof.calcAccel(dof.az));
 */
   
   
 // Launch System status
 // Prefix: D07, D08
 // DISABLED LAUNCH STATUS STUFF FOR NOW
 /*
 sendPacket(String("D07:") + String(isLaunchSystemPowered()));
 sendPacket(String("D08:") + String(isLaunchSystemArmed()));  
 */
  
  
  
  
  
   
//  delay(1000);
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
      state = initiateLaunch();
      sendPacket(String("A04:") + String(state));
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



void pollSerial() 
{  
 sendPacket("M"); // Menu  (to tell the other end we are ready to receive commands)
  
 unsigned long startTime = millis();
 inData[0]    = '\0';
 EndFlag = 0;
 index = 0;
 
 Serial2.flush();
 
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
  delay(400);                              // wait briefly... incase of contact bounce
    
  continuityState = digitalRead(continuitySensePin);

  delay(400);                              // wait briefly... incase of contact bounce
  relayOff(continuityTestPin);   // turn the LED on (HIGH is the voltage level) 
  delay(200);                              // wait briefly... incase of contact bounce  
  
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
int initiateLaunch() {
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





void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  dof.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  gx_val = dof.calcGyro(dof.gx) - gx_bias;
  dtostrf(gx_val, 4, 2, outstr);
  Serial.print(outstr);
  Serial.print(", ");
  
  gy_val = dof.calcGyro(dof.gy) - gy_bias;
  dtostrf(gy_val, 4, 2, outstr);
  Serial.print(outstr);
  Serial.print(", ");
  
  gz_val = dof.calcGyro(dof.gz) - gz_bias;
  dtostrf(gz_val, 4, 2, outstr);  
  Serial.println(outstr);
#elif defined PRINT_RAW
  Serial.print(dof.gx);
  Serial.print(", ");
  Serial.print(dof.gy);
  Serial.print(", ");
  Serial.println(dof.gz);
#endif
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  dof.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(dof.calcAccel(dof.ax), 2);
  Serial.print(", ");
  Serial.print(dof.calcAccel(dof.ay), 2);
  Serial.print(", ");
  Serial.println(dof.calcAccel(dof.az), 2);
#elif defined PRINT_RAW 
  Serial.print(dof.ax);
  Serial.print(", ");
  Serial.print(dof.ay);
  Serial.print(", ");
  Serial.println(dof.az);
#endif

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  dof.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(dof.calcMag(dof.mx), 2);
  Serial.print(", ");
  Serial.print(dof.calcMag(dof.my), 2);
  Serial.print(", ");
  Serial.println(dof.calcMag(dof.mz), 2);
#elif defined PRINT_RAW
  Serial.print(dof.mx);
  Serial.print(", ");
  Serial.print(dof.my);
  Serial.print(", ");
  Serial.println(dof.mz);
#endif
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

void getHeading(float hx, float hy, double *heading)
{
  
  if (hy > 0)
  {
    *heading = 90 - (atan(hx / hy) * (180 / PI));
  }
  else if (hy < 0)
  {
    *heading = - (atan(hx / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hx < 0) *heading = 180;
    else *heading = 0;
  }  
  
}

// Another fun function that does calculations based on the
// acclerometer data. This function will print your LSM9DS0's
// orientation -- it's roll and pitch angles.
void printOrientation(float x, float y, float z)
{
  float pitch, roll;
  
  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
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


void displayTime()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  
  // retrieve data from DS3232  
  readDS3232time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  
  // send it to the serial monitor
  Serial.print(hour, DEC);// convert the byte variable to a decimal number when being displayed
  Serial.print(":");
  if (minute<10)
  {
      Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
      Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print("  ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print("  Day of week:");
  switch(dayOfWeek-1){
  case 1:
    Serial.println("Sunday");
    break;
  case 2:
    Serial.println("Monday");
    break;
  case 3:
    Serial.println("Tuesday");
    break;
  case 4:
    Serial.println("Wednesday");
    break;
  case 5:
    Serial.println("Thursday");
    break;
  case 6:
    Serial.println("Friday");
    break;
  case 7:
    Serial.println("Saturday");
    break;
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


void displayPressure()
{
  char status;
  double T,P,p0,a;
  
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb
/*
          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          Serial.println(" feet");
*/         
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
void rotateMatrix(float xb, float yb, float zb, float a, float b, float c,
                  float *xg, float *yg, float *zg)
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
