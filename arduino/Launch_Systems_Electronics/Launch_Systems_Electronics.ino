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
 
 // Determine action to take
 if (menuopt[0] != '\0') {
   sendPacket(String("Received: ") + String(menuopt));
   
   /* Command Checks */
   if (strcmp(menuopt, "P1") == 0) {
      poweronLaunchSystem();
      sendPacket(String("1"));
   } else if (strcmp(menuopt, "P0") == 0) {
      poweroffLaunchSystem();
      sendPacket(String("1"));      
   } else if (strcmp(menuopt, "P?") == 0) {
      sendPacket(String(isLaunchSystemPowered()));            
   } else if (strcmp(menuopt, "CT") == 0) {
      state = checkContinuity();
      if (state == 1) {
         sendPacket(String("1"));   // There is continuity
      } else if (state == -1) {
         sendPacket(String("-1"));  // Power is not on
      } else if (state == 0) {
         sendPacket(String("0"));   // There is no continuity
      } else {
         sendPacket(String("-9"));  // An unexpected response received
      }         
   } else if (strcmp(menuopt, "ST") == 0) {
      ardupsu.read();
      dtostrf(ardupsu.value(),5, 2, outstr);   
      sendPacket (String(outstr)); 
   } else if (strcmp(menuopt, "VT") == 0) {
      ignpsu.read();
      dtostrf(ignpsu.value(),5, 2, outstr);  
      sendPacket (String(outstr));
   } else if (strcmp(menuopt, "A1") == 0) {
      state = armLaunchSystem();
      if (state == 0) {
         sendPacket(String("1"));   // Successfully armed system
      } else if (state == -1) {
         sendPacket(String("-1"));  // Power not on, refusing to arm
      } else if (state == -2) {
         sendPacket(String("-2"));  // Continuity test failed, refusing to arm
      } else {
         sendPacket(String("-9"));  // An unexpected response received
      }
   } else if (strcmp(menuopt, "A0") == 0) {
      disArmLaunchSystem();
      sendPacket(String("1"));  
   } else if (strcmp(menuopt, "A?") == 0) {
      sendPacket(String(isLaunchSystemArmed()));        
   } else if (strcmp(menuopt, "L") == 0) {
      state = initiateLaunch();
      if (state == 0) {
         sendPacket(String("1"));
      } else if (state == -1) {
         str = String("-1");
         sendPacket(str);
      } else if (state == -2) {
         str = String("-2");
         sendPacket(str);
      } else if (state == -3) {
         str = String("-3");
         sendPacket(str);         
      } else {
         sendPacket(String("-9"));
      }
   } else {
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



void resetLaunchSystem() {
  relayOff(launchPin);   
  relayOff(continuityTestPin); 
  poweroffLaunchSystem();
  disArmLaunchSystem();

}


// Returns 1 if there is continuity
//         0 if there is no continuity
int checkContinuity(){

  // Check if Power is on
  if (! isLaunchSystemPowered()) return -1;
  
  relayOn(continuityTestPin);   // turn the LED on (HIGH is the voltage level)
  delay(200);                              // wait briefly... incase of contact bounce
    
  continuityState = digitalRead(continuitySensePin);

  relayOff(continuityTestPin);   // turn the LED on (HIGH is the voltage level) 
  delay(200);                              // wait briefly... incase of contact bounce  
  
  // A high on pin...means no continuity...
  // a low on pin means there IS continuity
  if (continuityState == HIGH) {
     return 0;
  }  else {
     return 1;
  }
}


void initLaunchSystem() {
  pinMode(powerPin, OUTPUT);
  pinMode(armPin, OUTPUT);
  pinMode(continuityTestPin, OUTPUT);
  pinMode(launchPin, OUTPUT);  
  pinMode(continuitySensePin, INPUT);
 
}


int initiateLaunch() {
  
  // Check if Power is on
  if (! isLaunchSystemPowered()) return -1;
  
  // Check if System is armed
  if (! isLaunchSystemArmed()) return -2;
  
  // Perform continuity test
  if (checkContinuity() != 1) {
     // Continuity failed...but first disarm system...as part of safety process.
     disArmLaunchSystem();    
     return -3;
  }
  
  
  relayOn(launchPin);
  delay(igniterBurnDelay);
  relayOff(launchPin);
  
  // Then disarm system
  disArmLaunchSystem();
  
  return 0;
}



int isLaunchSystemPowered() {
  return bitRead(PORTH,powerPin);
  
}

boolean isLaunchSystemArmed() {
  return bitRead(PORTH,armPin); 
}


void poweronLaunchSystem() {
   bitWrite(PORTH, powerPin, HIGH);
}

void poweroffLaunchSystem() {
   disArmLaunchSystem();   // Disarm system as well.
   bitWrite(PORTH, powerPin, LOW);
}

int armLaunchSystem() {
  
  // Check if Power is on
  if (! isLaunchSystemPowered()) return -1;
  
  // Perform continuity test
  if (checkContinuity() != 1) return -2;
  
  bitWrite(PORTH, armPin, HIGH);
  return 0;
}

void disArmLaunchSystem() {
  bitWrite(PORTH, armPin, LOW);
}

