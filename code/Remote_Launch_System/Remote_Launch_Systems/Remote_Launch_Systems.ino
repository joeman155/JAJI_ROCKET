#include <voltage.h>

// Pins
const int  continuitySensePin = 8;  // PORTH, 5 
const int  powerPin = 6;            // PORTH, 6   -- Digital pin 9
const int  armPin   = 3;            // PORTH, 3   -- Digital pin 6
const int  continuityTestPin = 5;   // PORTE, 3
const int  launchPin = 4;           // PORTG, 5
const int  igniterPsuPin = A3;
const int  arduinoPsuPin = A2;
const int  igniterBurnDelay = 1000;
int state;

// Generic declarations
char inData[20]; // Allocate some space for the string
char noCommand[1];
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character

static char outstr[15];
String str;

// Other config
long heartbeat_count;
int   continuityState = 1;         // current state of the button

// Menu
unsigned long menutime = 5000;
char *param;  // Parameter for functions called when requests sent.
char *result; // Data to include back with Acknowledgement
int resultint; // numeric result


// Sensors
VOLTAGE ignpsu;
VOLTAGE ardupsu;


void setup() {
  Serial.begin(9600);
  sendPacket ("S");  
  
  // Initialisations
  heartbeat_count = 1;

  ignpsu.setup(igniterPsuPin, 10, 4.7); 
  ardupsu.setup(arduinoPsuPin, 15, 4.7); 
 
  initLaunchSystem();
  resetLaunchSystem();
 
 
}

void loop() {


 // Get Serial Input (menu) 
 char *menuopt = pollSerial();
 
 param = '\0';
 // Determine action to take
 if (menuopt[0] != '\0') {
    sendPacket(String("Received: ") + String(menuopt));
 
    // Decode menuopt....to get parameter (if provided)
    if (strlen(menuopt) > 4) {
      // Make sure next character is a colon.
      if (strncmp(menuopt+3, ":",1) == 0) {
        param = menuopt+4;
      }
   }
      
   /* Command Checks */
   if (strncmp(menuopt, "R01", 3) == 0) 
   {
      state = powerRequest(param);
      sendPacket(String("A01:") + String(state));      
   } 
   else if (strcmp(menuopt, "R08") == 0) 
   {
      sendPacket(String("A08:") + String(isLaunchSystemPowered()));            
   } 
   else if (strcmp(menuopt, "R03") == 0) 
   {
      state = checkContinuity();
      sendPacket(String("A03:") + String(state));
   } 
   else if (strcmp(menuopt, "ST") == 0) 
   {
      ardupsu.read();
      dtostrf(ardupsu.value(),5, 2, outstr);   
      sendPacket (String(outstr)); 
   } else if (strcmp(menuopt, "VT") == 0) 
   {
      ignpsu.read();
      dtostrf(ignpsu.value(),5, 2, outstr);  
      sendPacket (String(outstr));
   } 
   else if (strncmp(menuopt, "R02", 3) == 0) 
   {
      state = armRequest(param);
      sendPacket(String("A02:") + String(state));
   } 
   else if (strcmp(menuopt, "R09") == 0) 
   {
      sendPacket(String("A09:") + String(isLaunchSystemArmed()));        
   } 
   else if (strcmp(menuopt, "R04") == 0) 
   {
      state = initiateLaunch();
      sendPacket(String("A04:") + String(state));
   } else 
   {
      sendPacket("-9999");
   }
}     
  
  
  // Voltages
  // Prefix: VX   (where X is a number)
  
  
  
  // Local Time 
  // Prefix: T
  
  
  
  // GPS Tracking
  // Prefix: G
  
  
  
  // IMU Code
  // Prefix: I
  
  
  
  // Air Pressure
  // Prefix: P
  
  
  
  // Temperature
  // Prefix: TX   (where X is a number)
   
  
  
  
  // Heartbeat
  heartbeat();
  
   
 delay(1000);
 
}


void heartbeat() {
  str = String("H:") + String(heartbeat_count);
  sendPacket(str);
  heartbeat_count++;
}


void sendPacket(String str) {
  Serial.println(str);
}



char *pollSerial() {  
 sendPacket("M"); // Menu  (to tell the other end we are ready to receive commands)
  
 unsigned long startTime = millis();
 inData[0]    = '\0';
 noCommand[0] = '\0';
 int EndFlag = 0;
 int gotCommand = 0;
 
 Serial.flush();
 
 while(!EndFlag) {
    if (millis() - startTime > menutime) {   
      EndFlag = 1;
    }
      
    while(Serial.available() > 0 ) {
      
      // Accept nothing longer than 20 characters 
       if(index > 19) {
          break;
       }
       inChar = Serial.read(); // Read a character
       
       // If we detect carriage return, this means end of command (person/program) has hit enter
       if (inChar == '\r') {
          gotCommand = 1; // New Line indicates the command was 'commited'         
          EndFlag = 1;    // Exit the whole "menu"
          break;          // Get out of the first while loop
       }
       
       
       inData[index] = inChar; // Store it
       index++; // Increment where to write next
       inData[index] = '\0'; // Null terminate the string
    }
  }
  
  index = 0;
  
  // Return command or noCommand based on whether the user hit carriage return or not.
  if (gotCommand == 1) {
     return   &inData[0];
  } else {
    return &noCommand[0];
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
  delay(200);                              // wait briefly... incase of contact bounce
    
  continuityState = digitalRead(continuitySensePin);

  relayOff(continuityTestPin);   // turn the LED on (HIGH is the voltage level) 
  delay(200);                              // wait briefly... incase of contact bounce  
  
  // A high on pin...means no continuity...
  // a low on pin means there IS continuity
  if (continuityState == HIGH) {
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
   char *result;
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

