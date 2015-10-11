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
#include "ssdv.h"
 

// Debugging
unsigned short int DEBUGGING = 1;
unsigned short int RECEIVEPORT = 2;  // 0 = Serial, 1 for Serial1, 2 for Serial2

// delay between measurements
#define LOOP_DELAY 10
short int profile = 1;              // Tells what 'profile' we follow ...e.g. just imu...or launch ,etc 
                                    // 1 = Normal   (no IMU mass data)
                                    // 2 = IMU      (lots of IMU data for orientation tab)


// Profiles configuration
unsigned long profile2_timer = 0;   // Track how long we are in profile2 state.
unsigned long profile_timer;        // Used to ensure we only show profile setting every few seconds.


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
unsigned long launch_timer   = 0;


// SD Card and file declarations
const int chipSelect = 4;
File imgFile;   // Just for images
File ssdvFile;  // Holds SSDV data
File myFile;    // For other files
int sdCardState = 0;  // 0 = uninitialised , -1 = error, 1 = initialised

// Generic declarations
char inData[20]; // Allocate some space for the string
char noCommand[1];
char inChar=-1; // Where to store the character read
char nextChar=-1; // Where we store the peeked character
byte index = 0; // Index into array; where to store the character
char hex_code[3];
char temp_string[15];
char temp_string2[3];
boolean error;


#define M_PI 3.14159265
#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI)) 
#define DEGREES_TO_RADIANS(degrees) ((degrees) * (M_PI / 180.0)) 

static char outstr[15];
String str;

// Other config
long heartbeat_count;
int  continuityState = 1;         // current state of the button
unsigned long heartbeat_timer   = 0;

// Timing
uint32_t ulCur;
unsigned long timing_timer   = 0;

// Menu
unsigned int  menutime;                   // Menu time.
unsigned int  menutime_initial = 250;    // Initial time we wait
unsigned int  menutime_final   = 1000;   // The time we wait if we suddenly find we have commands being sent.
unsigned long menu_timer       = 0;      
int EndFlag = 0;                         // Means end of menu
char param[10];  // Parameter for functions called when requests sent.
const boolean menu_enabled = true;
boolean getting_data;            // Indicates if command was processed on menu.

// States
short int cutdown = 0; // Start up disabled


// Linksprite Camera - on Serialport 3
byte ZERO = 0x00;
byte incomingbyte;
long int j=0,k=0,count=0,i=0x0000;
uint8_t MH,ML;
boolean camera_EndFlag=0;
char current_pic_name[15];
unsigned long picture_timer = 0;   // (note, for pictures we want a long period.....so specified in seconds...not msec)
boolean picture_ready = false;     // Used to indicate if file is present to send to groundstation
boolean create_ssdv_successful = false; // Used to determine if SSDV file created or not.

// SSDV
char type = SSDV_TYPE_NORMAL;
char callsign[7];
uint8_t image_id = 0;
ssdv_t ssdv;
uint8_t pkt[SSDV_PKT_SIZE], *jpeg;
unsigned long ssdv_timer   = 0;



// IMU
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
unsigned long imu_timer   = 0;

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
unsigned long gps_timer = 0;


// RTC
#define DS3232_I2C_ADDRESS 0x68

// Air Pressure - BMP180
SFE_BMP180 pressure;
double bmp180_pressure, bmp180_temperature;
VOLTAGE ignpsu;
VOLTAGE ardupsu;
unsigned long sensors_timer = 0;


void setup() {
  // Serial Port we program with
  Serial.begin(115200);
  
  // GPS
  Serial1.begin(4800);
  
  // Linksprite Camera
  Serial3.begin(115200);
  delay(200);
  SendResetCmd();
  delay(4000);
  
//  SetBaudRateCmd(0x2A);     // 2A = 38400, 1C = 57600, 0x0d = 115200
//  delay(100);
//  Serial3.end();
//  
//  Serial3.begin(38400);
  delay(100);
  SetImageSizeCmd(0x00);    // 0x1b - 1280*960,  0x1c - 1024X768,    0x00 - 640/480
                            // NOTE: Because we are using SSDV, we can't transmit images where the dimensions
                            //       are whole multiple of 32.
  delay(100);  
  
  
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



// LOOP AROUND GETTING DATA AND SENDING BACK
// We have different profiles....data sets that we can send back.
void loop() {

  if (profile == 1) {
     profile1();
  } else if (profile == 2) {
     if (profile2_timer == 0) {
        profile2_timer = millis();
     }    
     profile2();
  }

}


// Standard profile
// The parameters passed to each function are:-
// - The Period - in Milliseconds.
void profile1()
{
  show_menu(1000);

  show_profile(3000);
  
  send_heartbeat(5000);

  physical_measurements(5000);

  show_gps(10000);

  timing(500);
  
  imu(1000);
  
  launch_systems(500);
  
  // Only allow automatic taking pictures IF the launch system is NOT powered.
  if (! isLaunchSystemPowered()) {
     if(!picture_ready) {  
        picture_ready = takePicture(300); // This figure 60 is in seconds...not milliseconds like other ones.
     }
  }
  
  // If a picture was taken, send it down.
  if (picture_ready && !create_ssdv_successful) {
     create_ssdv_successful = create_image_ssdv_file();
     
     if (!create_ssdv_successful) {
        picture_ready = false; // Something bad happened...let's not try sending...
     }
  }
  
  // If we were able to create the ssdv file successfully, then we try sending
  if (picture_ready && create_ssdv_successful) {
     boolean send_ssdv_successful = send_ssdv_file();
        
     // If we were able to send file successfully, then we send finish packet (D12)        
     if (send_ssdv_successful) {
         sendPacket("D12");  // Indicates to ground station that image transfer has finished. 
         delay(100);
         picture_ready = false;           // Allow taking of photos again
         create_ssdv_successful = false;  // Reset back....ALL READY to take new pictures
     } 
  }
    
  delay(LOOP_DELAY);   
}


// IMU profile
// The parameters passed to each function are:-
// - The Period - in Milliseconds.
void profile2()
{
  show_menu(30000); // Don't really need this, but keeping in here for safety measure.
  
  show_profile(3000);
  
  // We only want to sit in profile2 profile for 30 seconds.
  if (millis() - profile2_timer > 30000) {
    profile = 1;           // Back to profile1
    profile2_timer = 0;    // Reset timer
  }

  send_heartbeat(5000);

  imu(100);

  delay(LOOP_DELAY);   
}


// Process what we receive from the GroundStation
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
   else if (strcmp(rxString, "R10") == 0) 
   {
      // Toggle the profile value.
      if (profile == 1) {
         profile = 2;
      } else if (profile == 2) {
         profile = 1;
      } 
      
      sendPacket(String("A10:") + String(profile));
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
      sendPacket("E10");
   }
 }  
  
  return theend;
}



// Send out Heartbeart
void heartbeat() 
{
  str = String("H:") + String(heartbeat_count);
  sendPacket(str);
  heartbeat_count++;
}


// Send Packets to GroundStation and to log file
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

void sendPacket(String str, boolean eol, boolean logging) {
  if (eol) {
    Serial2.println(str);
    if (logging) logString(str);
  } else {
    Serial2.print(str);
    if (logging) logString(str, eol);
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


// Send Menu to groundstation and wait for a response.
void pollSerial(HardwareSerial &serial_device) 
{  
 // Make sure all prior data sent is REALLY sent and wait a little for it to be processed by groundstation.  
 serial_device.flush();
 
 // Wait for data that hasn't made it to GS to get there.... BEFORE issuing menu command
 delay(200);
 
 sendPacket("M"); // Menu  (to tell the other end we are ready to receive commands)
 unsigned long startTime = millis();
 inData[0]    = '\0';
 EndFlag = 0;
 index = 0;
 menutime = menutime_initial; // Initial menu time length
 getting_data = false;

 
 while(!EndFlag) {
    if (millis() - startTime > menutime) {   
      EndFlag = 1;
    }
      
    while(serial_device.available() > 0 ) {
      getting_data = true;
      // If we get a character, then there must be something coming our way...so we extend our listen period a bit longer.
      if (menutime < menutime_final) {
         menutime = menutime_final;
      }
      
      
      // Accept nothing longer than 20 characters 
       if(index > 19) {
          break;   // To long...refusing to process.
       }
       inChar = serial_device.read(); // Read a character
              
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
      sendPacket("E09", true, false);
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
// Get the heading based on the magnetic readings
void getHeading(float hz, float hy, double *heading)
{
  /*
  if (hy > 0 )
  {
    *heading = 90 - (atan(hz / hy) * (180 / PI));
  }
  else if (hy < 0)
  {
    *heading =  - (atan(hz / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hz < 0) *heading = 180;
    else *heading = 0;
  } 
 */ 
 
  if (hy > 0 && hz > 0)
  {
    *heading = 360 - (atan(hy / hz) * (180 / PI));
  }
  else if (hy > 0 && hz < 0)
  {
    *heading =  180 - (atan(hy / hz) * (180 / PI));
  }
  else if (hy < 0 && hz < 0)
  {
    *heading =  90 + (atan(hz / hy) * (180 / PI));
  }
  else if (hy < 0 && hz > 0)
  {
    *heading =  - (atan(hy / hz) * (180 / PI));
  }  
  else // hy = 0
  {
    if (hz < 0) *heading = 180;
    else *heading = 0;
  }  
  
  *heading = 360 - *heading;
  
}





// Read the time on the local clock
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




// Get the Pressure and temperature
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

  // Get readings from Magnetometer.
  double heading;  
  dof.readMag();  



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
 
  // Reverse Angle of Pitch
  pitch = -kalAngleY;

  // Roll == kalAngleZ
  roll = kalAngleZ;

  // Correct Acceration - (with board with components up, up (z) is positive), but board is other way around
  // with components down. Acceration should be positive. (Though it reads negative)
  accZ = accZ * -1;

  // Derive heading
  float cmx, cmy;
  cmx = dof.mz * cos(DEGREES_TO_RADIANS(pitch)) + dof.my * sin(DEGREES_TO_RADIANS(roll)) * sin(DEGREES_TO_RADIANS(pitch)) + dof.mx * cos(DEGREES_TO_RADIANS(roll)) * sin(DEGREES_TO_RADIANS(pitch));
  cmy = dof.my * cos(DEGREES_TO_RADIANS(roll)) - dof.mx * sin(DEGREES_TO_RADIANS(roll));
  getHeading((float) cmx, (float) cmy, &heading);  
  yaw = heading;
  
  // Attempting to correct around the transistion point 0--360
  if (yaw > 360) {
     yaw = yaw - 360;
  }
  if (yaw < 0) {
    yaw = 360 + yaw;
  }
  
  // Use Kalman to derive Yaw
  if (yaw > 180 && yaw < 360 && kalAngleX > 0 && kalAngleX < 180 )  { 
     // Going from EAST to WEST
     kalmanX.setAngle(yaw);  
     kalAngleX = yaw;
  } else if (kalAngleX > 180 && kalAngleX < 360 && yaw > 0 && yaw < 180 )  { 
     // Going from WEST to EAST
     kalmanX.setAngle(yaw);  
     kalAngleX = yaw;
  } else {
     kalAngleX = kalmanX.getAngle(yaw, -gyroXrate, dt); // Calculate the angle using a Kalman filter
     yaw = kalAngleX;
  }
  
  
     


  
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
  
  delay(300);
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
    sendPacket(String(",") + String(outstr), false);
    
    // altitude
    altitude = gps.altitude()/100;
    sendPacket(String(",") + String(altitude), false);    
    
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




void show_menu(int menu_period)
{
 // Get Serial Input (menu) 
 if (millis() - menu_timer > menu_period) {
    if (menu_enabled) {
      if (RECEIVEPORT == 0) {
         pollSerial(Serial);
      } else if (RECEIVEPORT == 2) {
         pollSerial(Serial2);
      }
    }
    menu_timer = millis();
 }
}


void send_heartbeat(int heartbeat_period)
{
 // Heartbeat
 if (millis() - heartbeat_timer > heartbeat_period) {
    heartbeat();
    heartbeat_timer = millis();
 }
} 

void physical_measurements(int sensors_period)
{
 // Air Pressure, Temperature, voltages
 // Prefix: D00
 // Format of string is:-
 // D00:InternalTemp,ExternalTemp,AirPressure,CPUVoltage,IGNVoltage
 //  - Temperatures are in Kelvin
 //  - Pressures are in Pascals
 if (millis() - sensors_timer > sensors_period) {
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
  
   sensors_timer = millis();
 }
}


void show_gps(int gps_period)
{
 // GPS Tracking
 // Prefix: D01
 // Format of string is:-
 // D01:latitude,longitude,altitude,date,time,heading,speed,#satellites
 //
 // date is in format dd/mm/yyyy
 // time is in format hh.mintes.seconds.hundreths
 // Only collect GPS info every 'gps_period' seconds
 if (millis() - gps_timer > gps_period) {
    extractGPSInfo();
    gps_timer = millis();
 }
}

void timing(int timing_period)
{
 // Local Time (Time since startup, or reboot)
 // Prefix: D02
 if (millis() - timing_timer > timing_period) { 
    ulCur = micros();
    sendPacket(String("D02:") + String(ulCur));
    timing_timer = millis();
 }
}


void show_profile(int profile_period)
{
  // Prefix: D04
  if (millis() - profile_timer > profile_period) { 
     sendPacket(String("D04:") + String(profile));
     profile_timer = millis();
  }
} 

void imu(int imu_period)
{
 // IMU Code
 // Prefix: D06
 // Format of string is:-
 // D06:Roll,Pitch,Yaw,gyroX,gyroY,gyroZ,accX,accY,accZ,timer
 if (millis() - imu_timer > imu_period) { 
    extractIMUInfo();
    imu_timer = millis();
 }
}


void launch_systems(int launch_period)
{
 // Launch System status
 // Prefix: D07, D08
 if (millis() - launch_timer > launch_period) { 
    sendPacket(String("D07:") + String(isLaunchSystemPowered()));
    sendPacket(String("D08:") + String(isLaunchSystemArmed()));  
    launch_timer = millis();
 }
}



void SendResetCmd()
{
    Serial3.write(0x56);
    Serial3.write(ZERO);
    Serial3.write(0x26);
    Serial3.write(ZERO);
}
 
/*************************************/
/* Set ImageSize :
/* <1> 0x22 : 160*120
/* <2> 0x11 : 320*240
/* <3> 0x00 : 640*480
/* <4> 0x1D : 800*600
/* <5> 0x1C : 1024*768
/* <6> 0x1B : 1280*960
/* <7> 0x21 : 1600*1200
/************************************/
void SetImageSizeCmd(byte Size)
{
    Serial3.write(0x56);
    Serial3.write(ZERO);  
    Serial3.write(0x54);
    Serial3.write(0x01);
    Serial3.write(Size);
}
 
/*************************************/
/* Set BaudRate :
/* <1>¡¡0xAE  :   9600
/* <2>¡¡0x2A  :   38400
/* <3>¡¡0x1C  :   57600
/* <4>¡¡0x0D  :   115200
/* <5>¡¡0xAE  :   128000
/* <6>¡¡0x56  :   256000
/*************************************/
void SetBaudRateCmd(byte baudrate)
{
    Serial3.write(0x56);
    Serial3.write(ZERO);
    Serial3.write(0x24);
    Serial3.write(0x03);
    Serial3.write(0x01);
    Serial3.write(baudrate);
}
 
void SendTakePhotoCmd()
{
    Serial3.write(0x56);
    Serial3.write(ZERO);
    Serial3.write(0x36);
    Serial3.write(0x01);
    Serial3.write(ZERO); 
}
 
void SendReadDataCmd()
{
    MH=i/0x100;
    ML=i%0x100;
    Serial3.write(0x56);
    Serial3.write(ZERO);
    Serial3.write(0x32);
    Serial3.write(0x0c);
    Serial3.write(ZERO);
    Serial3.write(0x0a);
    Serial3.write(ZERO);
    Serial3.write(ZERO);
    Serial3.write(MH);
    Serial3.write(ML);
    Serial3.write(ZERO);
    Serial3.write(ZERO);
    Serial3.write(ZERO);
    Serial3.write(0x20);
    Serial3.write(ZERO);
    Serial3.write(0x0a);
    i+=0x20;
}
 
void StopTakePhotoCmd()
{
    Serial3.write(0x56);
    Serial3.write(ZERO);
    Serial3.write(0x36);
    Serial3.write(0x01);
    Serial3.write(0x03);
}


// Take a picture every picture_period milliseconds
boolean takePicture(long picture_period)
{
  boolean result = false;
  
  if (millis() - picture_timer > (picture_period * 1000)) { 
     result = takePicture_internal();
     picture_timer = millis();
  }
  
  return result;
}

// Take a picture
boolean takePicture_internal()
{
    byte a[32];
    int ii, jj;
    camera_EndFlag = 0;
    i = 0;
    
    // Generate file name
    char *fname = create_pic_fname();
    strcpy(&current_pic_name[0], fname);
       
    // Init handle for file
    imgFile = SD.open(&current_pic_name[0], FILE_WRITE); //The file name should not be too long
 
    error = false;
    if (! imgFile) {
       sendPacket("E03");
       error = true;
       return false;
    }
    
    
    // Only continue on if we can save the picture somewhere...the microSD card.
    SendTakePhotoCmd();
    delay(1000);
       
    // Just a bit of helpful information
    // DEBUGGING
    sendPacket(String("Taking picture and saving to ") + String(&current_pic_name[0]), true, false);
    
    // Consume ack bytes.
    while(Serial3.available()>0)
    {
        incomingbyte=Serial3.read();
    }       
    
    int nextPacket = 1;
    int packetCount = 0;
    ulCur = millis();
    while(!camera_EndFlag)
    {
        if (nextPacket == 1) 
        {
           nextPacket = 0;
           j=0;
           k=0;
           count=0;
           SendReadDataCmd();
           delay(5);
           a[0] =  a[1] = a[2] = a[3] = a[4] = 0x0;
        }
        
        // Exit if we can't get pic within 90 seconds...this occurs if a problem with camera
        // or camera not plugged in properly
        if((millis() - ulCur) > (unsigned long)(90000)) // 90,000 milli seconds == 90 seconds
        {
            Serial.println("E04");
            return false;
        }
        
        while(Serial3.available()>0)
        {
            incomingbyte=Serial3.read();
            k++;
            // delay(1); //250 for regular
            delayMicroseconds(333); 
            if((k>5)&&(j<32)&&(!camera_EndFlag))
            {
                a[j]=incomingbyte;
                if((a[j-1]==0xFF)&&(a[j]==0xD9))     //tell if the picture is finished
                {
                    camera_EndFlag=1;
                }
                
                // Not sure why, but we sometimes get TWO ack at beginning.
                if ((a[0] == 0x76) && (a[1] == 0x00) && (a[2] == 0x32) && (a[3] == 0x00) && (a[4] == 0x00)) {
                   j = j - 5;
                   a[0] =  a[1] = a[2] = a[3] = a[4] = 0x0;
                   count = 0;
                }                
                j++;
                count++;
            }
        }
 

 // DEBUGGING
 /*
        for(jj=0;jj<count;jj++)
        {
            if(a[jj]<0x10)  Serial.print("0");
            Serial.print(a[jj],HEX);           // observe the image through serial port
            Serial.print(" ");
        }
*/
 
        for(ii=0; ii<count; ii++) {
           imgFile.write(a[ii]);
        }
        
        
        if (j == 32 ) {
           packetCount++;
           nextPacket = 1;
        
           // DEBUGGING
           // Serial.println();
              
           Serial.print(".");
        
           if (packetCount % 80 == 0) {
              Serial.println("");
           }
             
        }
    }
 
    Serial.println("");
    imgFile.close();
    
    Serial.println("Sending Stop command");
    StopTakePhotoCmd();
    
    /*
    // Consume anything that might be in the buffer.
    while(Serial3.available()>0)
    {
        incomingbyte=Serial3.read();
        Serial.print(incomingbyte, HEX);
        Serial.print(" ");
    }    
    Serial.println("");
    */
    
    // DEBUGGING
    Serial.println("Finished writing data to file");
       

 return true;
}


// Take a picture every picture_period milliseconds
//
// NOTE: This routine not used, but kept here for future reference.
//
boolean send_image(long picture_period)
{
  boolean result = false;
  
  if (millis() - picture_timer > picture_period) { 
     result = send_image_internal();
     picture_timer = millis();
  }
  
  return result;
}

// Send image using SSDV. 
//
// NOTE: This routine not used, but kept here for future reference.
//
// Returns boolean - SUCCESS/FAILURE.
//
boolean send_image_internal()
{  
  int c, i;
  uint8_t  b[128];
  i = 0;
  int r;
  boolean finished = false;
  
  // size_t jpeg_length;	
  
  // If Image file not opened, this means we need to initialise things
  if (!imgFile) {
     callsign[0] = '\0';
     // imgFile = SD.open(&current_pic_name[0], FILE_READ); //The file name should not be too long
     imgFile = SD.open("10100656", FILE_READ); //The file name should not be too long
     ssdv_enc_init(&ssdv, type, callsign, image_id);
     ssdv_enc_set_buffer(&ssdv, pkt);
  }
  
  while(1)
  {
     while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
     {
        int byte_count = 0;
        while(byte_count < 128 && imgFile.available())
        {
           b[byte_count] = imgFile.read();
           byte_count++;   
        }
        // size_t r = byte_count;
        r = byte_count;

	if(r <= 0) {
           sendPacket("Premature end of file\n");
	   break;
	}

	ssdv_enc_feed(&ssdv, b, r);

     }
      
     if (ssdv.error) {
         sendPacket(String("E02: ") + String(ssdv.error));     
         return false;
     }	
	
     if(c == SSDV_EOI) {
        //Serial.println("ssdv_enc_get_packet said EOI");
        break;
     }	else if(c != SSDV_OK) {
        sendPacket(String("ssdv_enc_get_packet failed: ") + String(c));
        finished = false;
        imgFile.close();
        return finished;
     }
			
     sendPacket("D13:", false, false);
     for(j=0;j<SSDV_PKT_SIZE;j++)
     {  
         sprintf(&hex_code[0], "%02X", pkt[j]);
         sendPacket(&hex_code[0], false, false);
     }
     sendPacket("", true, false);
     // delay(400); // Else the receiving end gets packets too fast and can't keep up. 
     // Delay incorporated in routines that ultimately call this.
     i++;
     break;  // Going through file file o 128 bytes at a time.
   }
   
  // If we got here, we have finished successfully.
  imgFile.close();
  finished = true;

        
  // Return success/failure
  return finished;
}


// Create a uniqut filename
char *create_pic_fname()
{
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    
    // Derive full path from date/time  - format is MMDDHHMM
    // This means we comply with nothing longer than 8 characters... DOS restriction 
    readDS3232time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
    
    // Start of file (date)
    if (month < 10) {
      strcpy(temp_string, "0");
      strcat(temp_string, itoa(month, temp_string2, 10));
    } else {
      strcpy(temp_string, itoa(month, temp_string2, 10));
    }
    
    if (dayOfMonth < 10) {
      strcat(temp_string, "0");
    }
    strcat(temp_string, itoa(dayOfMonth, temp_string2, 10));
   
    
    // Second part of filename (time)
    if (hour < 10) {
      strcat(temp_string, "0");
    }
    strcat(temp_string, itoa(hour, temp_string2, 10)); 
    
    if (minute < 10) {
      strcat(temp_string, "0");
    } 
    strcat(temp_string, itoa(minute, temp_string2, 10));
    
    return &temp_string[0];
}  



// Send image using SSDV - Does not exit UNTIL it is finished.
//
// NOTE: This routine not used, but kept here for future reference.
//
// Returns number of bytes processed.
//
int send_image_orig()
{  
  int c, i;
  uint8_t  b[128];
  i = 0;
  int r;
    
  // If Image file not opened, this means we need to initialise things
  if (!imgFile) {
     callsign[0] = '\0';
     imgFile = SD.open(&current_pic_name[0], FILE_READ); //The file name should not be too long
     
     ssdv_enc_init(&ssdv, type, callsign, image_id);
     ssdv_enc_set_buffer(&ssdv, pkt);
  }
  
  while(1)
  {
     while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
     {
        int byte_count = 0;
        while(byte_count < 128 && imgFile.available())
        {
           b[byte_count] = imgFile.read();
           byte_count++;   
        }
        r = byte_count;

	if(r <= 0) {
           sendPacket("Premature end of file\n");
	   break;
	}

	ssdv_enc_feed(&ssdv, b, r);
     }
      
     if (ssdv.error) {
         sendPacket(String("E02: ") + String(ssdv.error));     
     }	
	
     if(c == SSDV_EOI) {
        //Serial.println("ssdv_enc_get_packet said EOI");
        break;
     }	else if(c != SSDV_OK) {
        sendPacket(String("ssdv_enc_get_packet failed: ") + String(c));
        return(-1);
     }
			
     sendPacket("D13:", false, false);
     for(j=0;j<SSDV_PKT_SIZE;j++)
     {  
         sprintf(&hex_code[0], "%02X", pkt[j]);
         sendPacket(&hex_code[0], false, false);
     }
     sendPacket("", true, false);
     delay(400); // Else the receiving end gets packets too fast and can't keep up. 
     i++;
   }
   
  // See if we are at end of file
  imgFile.close();
        
  // Return # of bytes we are into the file
  return 1;
}



// Create file with SSDV packets. Doing this because difficult to create and send packets as required.
//
// Returns boolean - SUCCESS/FAILURE
//
boolean create_image_ssdv_file()
{  
  int c, i;
  uint8_t  b[128];
  i = 0;
  int r;
  
  // If Image file not opened, this means we need to initialise things
  if (!imgFile) {
     // Remove ssdv file if already there
     if (SD.exists("ssdv")) {
        SD.remove("ssdv");  
     }
     
     callsign[0] = '\0';
     imgFile = SD.open(&current_pic_name[0], FILE_READ); //The file name should not be too long
     // imgFile = SD.open("10100656", FILE_READ); //The file name should not be too long
     ssdvFile = SD.open("ssdv", FILE_WRITE);
     
     ssdv_enc_init(&ssdv, type, callsign, image_id);
     ssdv_enc_set_buffer(&ssdv, pkt);
  }

  while(1)
  {
     while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
     {
        int byte_count = 0;
        while(byte_count < 128 && imgFile.available())
        {
           b[byte_count] = imgFile.read();
           byte_count++;   
        }
        // size_t r = byte_count;
        r = byte_count;

	if(r <= 0) {
           sendPacket("Premature end of file\n");
	   break;
	}

	ssdv_enc_feed(&ssdv, b, r);
     }
      
     if (ssdv.error) {
         sendPacket(String("E02: ") + String(ssdv.error));    
         return false; 
     }	
	
     if(c == SSDV_EOI) {
        //Serial.println("ssdv_enc_get_packet said EOI");
        break;
     }	else if(c != SSDV_OK) {
        sendPacket(String("ssdv_enc_get_packet failed: ") + String(c));
        return false;
     }
	
     for(j=0;j<SSDV_PKT_SIZE;j++)
     {
        ssdvFile.write(pkt[j]);
     }	
     //sendPacket("D13:", false, false);
     //for(j=0;j<SSDV_PKT_SIZE;j++)
     //{  
     //    sprintf(&hex_code[0], "%02X", pkt[j]);
     //    sendPacket(&hex_code[0], false, false);
     //}
     // sendPacket("", true, false);
     // delay(400); // Else the receiving end gets packets too fast and can't keep up. 
     // Delay incorporated in routines that ultimately call this.
     i++;
   }
   
  // See if we are at end of file
  imgFile.close();
  ssdvFile.close();
        
  // Return true (i.e. successful!)
  return true;
}


// Sends file containing all the SSDV packets
// Sends at 256bytes at a time (each of these is a packet)
// 
// Returns boolean - TRUE if finished, FALSE if not finished
boolean send_ssdv_file()
{  
  byte b[256];
  int byte_count, j, i;
  byte_count = 0;
  boolean finish = false;
  
  if (!ssdvFile) {
     ssdvFile = SD.open("ssdv", FILE_READ);
  }

  byte_count = 0;     
  while(byte_count < 256 && ssdvFile.available()) {
      b[byte_count] = ssdvFile.read();
      byte_count++;
  }
          
  sendPacket("D13:", false, false);
  for(j=0;j<256;j++)
  {  
      sprintf(&hex_code[0], "%02X", b[j]);
      sendPacket(&hex_code[0], false, false);
  }  
  sendPacket("", true, false);   
  delay(50);
  
  // If nothing else available, then close
  if (! ssdvFile.available()) {
     ssdvFile.close();
     finish = true;
  }

  return finish;
}



// Sends file containing all the SSDV packets
// Sends at 256bytes at a time (each of these is a packet)
// 
// NOTE: This routine not used, but kept here for future reference.
//
// Returns number of packets sent.
int send_ssdv_file_orig()
{  
  byte b[256];
  int byte_count, j, i;
  byte_count = 0;
  
  ssdvFile = SD.open("ssdv", FILE_READ);
  
  // Serial.println("STARTING TO SEND SSDV FILE");
  while (ssdvFile.available()) {
     byte_count = 0;     
     while(byte_count < 256) {
         b[byte_count] = ssdvFile.read();
         byte_count++;
     }
     
     
     sendPacket("D13:", false, false);
     for(j=0;j<256;j++)
     {  
         sprintf(&hex_code[0], "%02X", b[j]);
         sendPacket(&hex_code[0], false, false);
     }  
     sendPacket("", true, false);   
     delay(500);
     i++;
  }
  
  ssdvFile.close();
  // Serial.println("FINISHED SENDING SSDV FILE");

  return i;
}
