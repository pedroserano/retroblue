/********************************************************
 * Arduino code used to turn a vintage rotary phone
 * into a 21st century Bluetooth accessory.
 * 
 * Project inspired by the original retroblue and Sparkfun's tutorial at
 * https://www.sparkfun.com/products/retired/8929.
 * Updates:
 * -Changed the dialing to not stop the loop and use actual clicks of the rotary dial
 * -Upgraded the bluetooth module from rn-52 to BC-127 (because the 52 has a 
 * number of problems including non-working mic bias, and BC-127 solved all those 
 * problems, is bluetooth 4, and supported more features like Siri)
 * -Added support for Siri/Google Now through flashing the hook or dialing zero
 * -Added rudimentary dialtone
 * 
 * Still needs to be addressed:
 * -better dialtone
 * -ringtone based on Sparkfun's high voltage ringer
 ********************************************************/

#include "bc127.h"
#include <SoftwareSerial.h>

const boolean DEBUG = false;

/* ringing state */
const int NOT_RINGING = 0;
const int RINGING = 1;
int ringState = NOT_RINGING;

/* used to track if we're in a call so we can tell the Bluetooth
 * module to end the call when the user hangs up the handset */
const int NOT_IN_CALL = 0;
const int IN_CALL = 1;
int callState = NOT_IN_CALL;
int Siri=0;
const int NUM_LOOP_ITERS_PER_RING_STATE = 3;

/* The number of times we've passed in the loop in the current solenoidOut
 * state. This is modulus NUM_LOOP_ITERS_PER_RING_STATE. */
int loopItersInRingState = 0;

/* used to manage ring pulses */
const int RING_INTERVAL_LENGTH = 100;
const float PERCENT_RINGING = 0.6;
const float RINGING_TIME = RING_INTERVAL_LENGTH * PERCENT_RINGING;
int ringIntervalCount = 0;

// Used as a way to delay how many times the dialtone command is sent without delaying the rest of the system, otherwise it's too fast for the bluetooth
int dialtonecount = 0;

/* state of pulse switch
 * 1 during dialing pulse
 * 0 otherwise
 */
int pulse;
const int PULSE = 1;
const int NO_PULSE = 0;

/* state of dial
 * 0 if a number is being dialed (i.e. any time dial is not in rest position)
 * 1 if dial is in rest position
 */
int dialing;
const int IS_DIALING = 0;
const int NOT_DIALING = 1;

/* state of hook
 * 0 if phone off hook
 * 1 if phone on hook
 */
int hook;
const int OFF_HOOK = 0;
const int ON_HOOK = 1;
const int HOOK_FLASH = 2;

/* pins used to detect the signals from the phone */
const int hookPin = 4;
const int dialingPin = 3;
const int pulsePin = 2;

/* constants returned */
int NO_NUMBER = -1;
bool hasCompletedNumber;
int number;

enum State {
	WAITING,
	LISTENING_NOPULSE,
	LISTENING_PULSE
};

enum State state;
unsigned long lastStateChangeMillis;

		/**
		 * Change state, but only if enough time has elapsed since
		 * the last state change (to protect from noise).
		 */
bool changeStateIfDebounced(enum State newState);

		/**
		 * To be called when ready returns HIGH (when the rotor returns
		 * to its rest position); save the number, if valid.
		 */
void completeDial();
        
// Require DEBOUNCE_DELAY milliseconds between state changes. This value is
// noted in the class documentation. Default is 15.
int DEBOUNCE_DELAY = 10;

/* the string the Bluetooth module sends when a command is successful */
const String SUCCESS = "OK";

String buffer;
  
/* Arduino Uno only has one hardware serial interface, so set up a software
 * serial interface for communicating with the Bluetooth module.
 *
 * RX = digital pin 10, TX = digital pin 11
 */
SoftwareSerial bluetoothSerial(10, 11);

BC127 bc127(&bluetoothSerial);

void setup() {
        if (DEBUG) Serial.begin(115200);
        bluetoothSerial.begin(115200);
	pinMode(dialingPin, INPUT);
	pinMode(pulsePin, INPUT);
        pinMode(hookPin, INPUT);
        digitalWrite(hookPin, HIGH);
	digitalWrite(dialingPin, HIGH);
	digitalWrite(pulsePin, HIGH);
	lastStateChangeMillis = millis();
        bluetoothSerial.listen();
}

void print(String s) {
  if (DEBUG) Serial.print(s);
}

void println(String s) {
  if (DEBUG) Serial.println(s);
}

//======== BLUETOOTH STATUS ==================================================================
/* updates ringing/call status */
void updateStatuses()
{
      // We need to buffer our incoming serial data...
    //println("getting bluetooth status");
    if (bluetoothSerial.available() > 0) buffer.concat((char)bluetoothSerial.read());
    
    // ...then, we need to check if it's a full line from the serial port, and
    //  check its contents if it is.
    if (buffer.endsWith("\r"))
    {
      // If the buffer has a serial port connection message, we can break out of
      //  the while loop after entering data mode.
      if (buffer.startsWith("RING")) 
      {
        startRinging();
        println("Incoming Call");
      }
      if (buffer.startsWith("CALL_ACTIVE")) 
      {
        println("Call in Progress");
        if (bc127.connectionState() == BC127::SUCCESS) callState = IN_CALL;
      }
      buffer = "";  // Otherwise, clear the buffer and go back to waiting.

    }
   
}

//========== RINGTONE ======================================================================

void startRinging() {
  ringState = RINGING;
}

void stopRinging() {
  ringState = NOT_RINGING;
  loopItersInRingState = 0;
}

void doRingTick() {
  if (ringIntervalCount < RINGING_TIME && loopItersInRingState == 0) {
//     do ring command
  }
  

  ringIntervalCount = (ringIntervalCount + 1) % RING_INTERVAL_LENGTH;
  loopItersInRingState = (loopItersInRingState + 1) % NUM_LOOP_ITERS_PER_RING_STATE;
  delay(20);
}

//======== CHECK FOR FALSE DIAL SIGNAL AKA DEBOUNCE ==========================================

bool changeStateIfDebounced(enum State newState) {
	unsigned long currentMillis = millis();
	if (currentMillis < lastStateChangeMillis) {
		// clock wrapped; ignore (but could figure it out in this case)
		lastStateChangeMillis = currentMillis;
		return false;
	} else if (currentMillis - lastStateChangeMillis > DEBOUNCE_DELAY) {
		state = newState;
		lastStateChangeMillis = currentMillis;
		return true;
	} else {
		return false;
	}
}


//======= CHECK FOR VALID DIALED NUMBER ===================================================
void completeDial() {
	if (!changeStateIfDebounced(WAITING)) {
		return;
	}
	if (number > 0 && number <= 10) {
		if (number == 10) {
			number = 0;
		}
		hasCompletedNumber = true;
	}
}

//=================================================
//  MULTI-HOOK:  One hook, Multiple Events

// hook timing variables
int debounce = 20;          // ms debounce period to prevent flickering when pressing or releasing the hook
int DCgap = 500;            // max ms between clicks for a double click event

// hook variables
boolean hookVal;   // value read from hook
boolean hookLast;  // buffered value of the hook's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the hook was pressed down
long upTime = -1;           // time the hook was released
boolean ignoreUp = false;   // whether to ignore the hook release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event

int checkHook() {
   hook = digitalRead(hookPin);
   hookVal = digitalRead(hookPin);
   // hook pressed down
   if (hookVal == ON_HOOK && hookLast == OFF_HOOK && (millis() - upTime) > debounce)
   {
       downTime = millis();
       ignoreUp = false;
       waitForUp = false;
       singleOK = true;
       if ((millis()-upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
       else  DConUp = false;
       DCwaiting = false;
       hook = 0;
       println("ON HOOK");
   }
   // hook released
   else if (hookVal == OFF_HOOK && hookLast == ON_HOOK && (millis() - downTime) > debounce)
   {        
       if (not ignoreUp)
       {
           upTime = millis();
           if (DConUp == false) DCwaiting = true;
           else
           {
               hook = 2;
               println("FLASH HOOK");
               DConUp = false;
               DCwaiting = false;
               singleOK = false;
           }
       }
   }
   // Test for normal click event: DCgap expired
   if ( hookVal == OFF_HOOK && (millis()-upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && hook != 2)
   {
       hook = 1;
       println("OFF HOOK");
       DCwaiting = false;
   }
 
   
   hookLast = hookVal;
   return hook;
}
//======== DIALTONE =======================================================================

void dialtone()
{
    if (dialtonecount == 1) { dialtonecount=0;}
     while (dialing == NOT_DIALING && hook == OFF_HOOK && callState == NOT_IN_CALL) {
       readState();
      if (dialtonecount == 0) {
       bluetoothSerial.print("TONE V 255 TI 0 D FF TN AF5 L 4");
       bluetoothSerial.print("\r");
       bluetoothSerial.flush();
       println("DIALTONE");
       
      }
      dialtonecount++;
  }
       
       
}

//========= DIALING =======================================================================
void readState() {
  	dialing = digitalRead(dialingPin); //dialing HIGH means no dialing, LOW means dialing
	pulse = digitalRead(pulsePin); //pulse HIGH means gap in pulse, LOW means pulse
        hook = checkHook();
}

bool update() {
          	switch(state) {
		case WAITING:
			if (dialing == IS_DIALING && changeStateIfDebounced(LISTENING_NOPULSE))
			{
				hasCompletedNumber = false;
				number = 0;
			}
			break;
		case LISTENING_NOPULSE:
			if (dialing == NOT_DIALING) {
				completeDial();
			} else if (pulse == PULSE) {
				changeStateIfDebounced(LISTENING_PULSE);
			}
			break;
		case LISTENING_PULSE:
			if (dialing == NOT_DIALING) {
				completeDial();
			} else if (pulse == NO_PULSE && changeStateIfDebounced(LISTENING_NOPULSE))
			{
				number++;
			}
			break;
	}
	return hasCompletedNumber;
}


bool hasNextNumber() {
	return hasCompletedNumber;
}

int getNextNumber() {
	if (hasCompletedNumber) {
		hasCompletedNumber = false;
		return number;
	} else {
		return NO_NUMBER;
	}
}
//============================================================================================

void loop() {
  readState();
  updateStatuses();

  if (ringState == RINGING) {
    doRingTick();
  }
  
  if (hook == HOOK_FLASH) {
     bc127.telephoneCommands(BC127::TOGGLE_VR);
     Siri = 1;
     println("Getting Siri...");
     callState = IN_CALL;
     hook = OFF_HOOK;
  }
  
/* if we're hanging up during an active call, end the call 
This needs to look for CALL_ACTIVE */

if (hook == ON_HOOK && callState == IN_CALL) {
  println("Ending call...");
  bc127.telephoneCommands(BC127::END);
  if (Siri == 1) {
  bc127.telephoneCommands(BC127::TOGGLE_VR);
  println("Canceling Siri...");
     Siri = 0;
  }
  callState = NOT_IN_CALL;
}


        
  if (hook == OFF_HOOK) {
    updateStatuses();
    readState();
    
    if (ringState == RINGING) {
      println("Accepting call...");
      bc127.telephoneCommands(BC127::ANSWER);
      stopRinging();
      if (bc127.connectionState() == BC127::SUCCESS) callState = IN_CALL;
    }
    
    String numberString = "";
    boolean doDialNumber = false;
    int numCount = 0;
    int dialedNumber = 0;
    
    while (numCount < 10) {
      /* wait for the user to start dialing a number */

      while (dialing == NOT_DIALING && hook == OFF_HOOK && numCount == 0){
        readState();
        updateStatuses();
        dialtone();
      }
      while (dialing == NOT_DIALING && hook == OFF_HOOK && numCount > 0) {
        readState();
        updateStatuses();
      }
      
      if (hook == ON_HOOK) {
        doDialNumber = false;
        break;
      }
      
      readState();
      
      /* only record the latest number dialed if it's valid */
      if (update()) {
        dialedNumber = getNextNumber();
        numberString = numberString + dialedNumber;
        numCount++;
        print(String(dialedNumber));
        if (numCount == 3 || numCount == 6) print("-");
        if (numCount == 1 && dialedNumber == 0){
               bc127.telephoneCommands(BC127::TOGGLE_VR);
               Siri = 1;
               println("Getting Siri...");
               callState = IN_CALL;
               hook = OFF_HOOK;
        }
            
        if (numCount == 10) {
          doDialNumber = true;
        }
      }
    }
    
    print("\n");
    if (doDialNumber) {
      println("Dialing " + numberString + "...");
      bc127.telephoneCallCommands(BC127::CALL, numberString);
      if (bc127.connectionState() == BC127::SUCCESS) callState = IN_CALL;
    }
  }
}
