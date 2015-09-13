#include <voltage.h>
#include <SPI.h>
#include <SD.h>
#include <SoftI2C.h>
#include <Wire.h>



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

static char outstr[15];
String str;

// Other config
long heartbeat_count;
int  continuityState = 1;         // current state of the button

// Menu
unsigned long menutime = 500;
int EndFlag = 0;
char param[10];  // Parameter for functions called when requests sent.
const boolean menu_enabled = true;

// States
short int cutdown = 0; // Start up disabled

// Sensors
VOLTAGE ignpsu;
VOLTAGE ardupsu;

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

}

void loop() {

 // Get Serial Input (menu) 
 if (menu_enabled) {
    pollSerial();
    
    // Just allow enough time for responses, etc to make their way through.
    delay(200);    
 }
 

  
 // Heartbeat
 heartbeat();



 // Air Pressure, Temperature, voltages
 // Prefix: D00
 // Format of string is D00:InternalTemp,ExternalTemp,AirPressure,CPUVoltage,IGNVoltage
 sendPacket (String("D00:") + String("1000000"), false);   // DUMMY AIR PRESSURE 
 sendPacket (String(",") + String("293"), false);      // DUMMY INTERNAL TEMP
 sendPacket (String(",") + String("303"), false);    // DUMMY EXTERNAL PRESSURE
 
 ardupsu.read();
 ignpsu.read();
 dtostrf(ardupsu.value(),4, 2, outstr);   
 sendPacket (String(",") + String(outstr), false); 
 dtostrf(ignpsu.value(),5, 2, outstr);  
 sendPacket (String(",") + String(outstr), true);   
  

 // Local Time 
 // Prefix: D02
 // DISABLE TIME FOR NOW
//  displayTime();
   
   
 // Launch System status
 // Prefix: D07, D08
 sendPacket(String("D07:") + String(isLaunchSystemPowered()));
 sendPacket(String("D08:") + String(isLaunchSystemArmed()));  
 
  
 // Really don't think this delay is doing us any good. The more data the better.
 // delay(2000);
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





