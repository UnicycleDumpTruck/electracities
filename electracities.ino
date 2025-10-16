// Electricities Linemen Play Wall 

//#include <Adafruit_SleepyDog.h>


// Radio Includes and variables ----------------------------------

//#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define RF69_FREQ 915.0

// Where to send packets to!
//#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     90

// Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4

// RFM69 Featherwing
//#define RFM69_CS      10
//#define RFM69_INT     11
//#define RFM69_RST     6

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission

uint32_t localCounter = 0;

struct EvenDataPacket{
  uint32_t counter;
  float batteryVoltage;
  uint8_t cubeID;
  uint8_t side;
} eventData;

// Dont put this on the stack:
uint8_t eventBuffer[sizeof(eventData)];
uint8_t from;
uint8_t len = sizeof(eventData);


void selectRadio() {
//  digitalWrite(LED,HIGH);
//  digitalWrite(WIZ_CS, HIGH);
  //delay(100);
  digitalWrite(RFM69_CS, LOW);
  //delay(100);
}


void sendPlayEvent(int side)
{
	eventData.side = side;
	eventData.cubeID = 90;
	eventData.batteryVoltage = 0;
	eventData.counter++;
	Serial.print("About to send transmission number: ");
	Serial.println(eventData.counter);	  	
	//sendEventData(); // jeff 
	//eventData.side = 0;
}

//RF communication
void sendEventData()
{  
	rf69.send((uint8_t*)&eventData, sizeof(eventData));
	rf69.waitPacketSent();
  delay(200);
}

/*
#include <Adafruit_NeoPixel.h>
//#define PIXELPIN 11
#define NUMPIXELS 1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);
*/

// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>

#define SERIALDEBUG 0 // Make everything wait to start until Serial connection established

#define LEFT_SENSOR_THRESHOLD 750
#define MID_SENSOR_THRESHOLD 750 // was 575
#define RIGHT_SENSOR_THRESHOLD 575 // was 475
int prevLeftValue = 0;
int prevMidValue = 0;
int prevRightValue = 0;

// Relays
#define RIGHTRELAY        9
#define MIDRELAY         12
#define LEFTRELAY  	     13

// Sensors on restore switches
#define LEFTSENSOR       14 // A0
#define MIDSENSOR        15 // A1
#define RIGHTSENSOR      16 // A2

// Buttons to start outages
#define LEFTBUTTON    17 // A3
#define MIDBUTTON     18 // A4
#define RIGHTBUTTON   19 // A5

#define NUM_BUTTONS 3
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {LEFTBUTTON, MIDBUTTON, RIGHTBUTTON};
Bounce * buttons = new Bounce[NUM_BUTTONS];

// States of city areas, HIGH = powered
bool leftPowerState = false;
bool midPowerState = false;
bool rightPowerState = false;

long lastPressMillis = 0;
#define INACTIVETIMEOUT 300000

void setup() {  

	Serial.begin(9600);
//	Wire.begin();
	
	#if (SERIALDEBUG)
    	// Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
    	while(!Serial);
	#endif

/*
	// First a normal example of using the watchdog timer.
	// Enable the watchdog by calling Watchdog.enable() as below.
	// This will turn on the watchdog timer with a ~4 second timeout
	// before reseting the Arduino. The estimated actual milliseconds
	// before reset (in milliseconds) is returned.
	// Make sure to reset the watchdog before the countdown expires or
	// the Arduino will reset!
	int countdownMS = Watchdog.enable(4000);
	Serial.print("Enabled the watchdog with max countdown of ");
	Serial.print(countdownMS, DEC);
	Serial.println(" milliseconds!");
	Serial.println();
*/


/*
	pinMode(PIXELPIN, OUTPUT);
	strip.begin(); // This initializes the NeoPixel library.
*/

	pinMode(LEFTRELAY, OUTPUT);
	pinMode(MIDRELAY, OUTPUT);
	pinMode(RIGHTRELAY, OUTPUT);
	
	pinMode(LEFTSENSOR, INPUT);
	pinMode(MIDSENSOR, INPUT);
	pinMode(RIGHTSENSOR, INPUT);
 
	pinMode(LEFTBUTTON, INPUT_PULLUP);
	pinMode(MIDBUTTON, INPUT_PULLUP);
	pinMode(RIGHTBUTTON, INPUT_PULLUP);
	
	
	for (int i = 0; i < NUM_BUTTONS; i++) {
    	buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    	buttons[i].interval(25);              // interval in ms
  }

	//--Radio Setup--//
	selectRadio();
	//pinMode(LED, OUTPUT);     
	pinMode(RFM69_RST, OUTPUT);
	digitalWrite(RFM69_RST, LOW);

	Serial.println("Feather Addressed RFM69 TX Test!");
	Serial.println();

	// manual reset
	digitalWrite(RFM69_RST, HIGH);
	delay(50);
	digitalWrite(RFM69_RST, LOW);
	delay(50);
  
	if (!rf69_manager.init()) {
		Serial.println("RFM69 radio init failed");
		while (1);
	}
	Serial.println("RFM69 radio init OK!");
	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
	// No encryption
	if (!rf69.setFrequency(RF69_FREQ)) {
		Serial.println("setFrequency failed");
	}

	// If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
	// ishighpowermodule flag set like this:
	rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

	// The encryption key has to be the same as the one in the server
	uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
					0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	rf69.setEncryptionKey(key);  

	eventData.cubeID = 90; // 90 is Electricities
	eventData.side = 0;
	eventData.batteryVoltage = 0;
	eventData.counter = 0;

	sendPlayEvent(0);
  restoreCityPower();
	Serial.println("Setup Complete.");
}



void loop() {
	//Watchdog.reset();
	
	readButtons();
  readSwitches();

	long currentMillis = millis();
	
	if ((currentMillis - lastPressMillis) > INACTIVETIMEOUT) {
		restoreCityPower();
		lastPressMillis = millis();
	}	

  //delay(100);
}


void restoreCityPower() {
	digitalWrite(LEFTRELAY, HIGH);
	digitalWrite(MIDRELAY, HIGH);
	digitalWrite(RIGHTRELAY, HIGH);
  leftPowerState = true;
  midPowerState = true;
  rightPowerState = true;
}

void readSwitches()
{
  int leftValue = analogRead(LEFTSENSOR);
  leftValue = analogRead(LEFTSENSOR);
  int midValue = analogRead(MIDSENSOR);
  midValue = analogRead(MIDSENSOR);
  int rightValue = analogRead(RIGHTSENSOR);
  rightValue = analogRead(RIGHTSENSOR);
  
  Serial.print(leftValue);
  Serial.print(" ");
  Serial.print(midValue);
  Serial.print(" ");
  Serial.println(rightValue);

  if (prevLeftValue > LEFT_SENSOR_THRESHOLD) {
    if (leftValue < LEFT_SENSOR_THRESHOLD) {
      Serial.println("Left Sensor Voltage dropped");
      prevLeftValue = leftValue;
      if (!leftPowerState) {
        digitalWrite(LEFTRELAY,HIGH);
        delay(200);
        leftPowerState = true;
        sendPlayEvent(4);
      }
    }
  } else if (prevLeftValue < LEFT_SENSOR_THRESHOLD) {
    if (leftValue > LEFT_SENSOR_THRESHOLD) {
      Serial.println("Left Sensor Voltage raised");
      prevLeftValue = leftValue;
      if (!leftPowerState) {
        digitalWrite(LEFTRELAY,HIGH);
        delay(200);
        leftPowerState = true;
        sendPlayEvent(4);
      }
    }
  }

  if (prevMidValue > MID_SENSOR_THRESHOLD) {
    if (midValue < MID_SENSOR_THRESHOLD) {
      Serial.println("Mid Sensor Voltage dropped");
      prevMidValue = midValue;
      if (!midPowerState) {
        digitalWrite(MIDRELAY,HIGH);
        delay(200);
        midPowerState = true;
        sendPlayEvent(5);
      }
    }
  } else if (prevMidValue < MID_SENSOR_THRESHOLD) {
    if (midValue > MID_SENSOR_THRESHOLD) {
      Serial.println("Mid Sensor Voltage raised");
      prevMidValue = midValue;
      if (!midPowerState) {
        digitalWrite(MIDRELAY,HIGH);
        delay(200);
        midPowerState = true;
        sendPlayEvent(5);
      }
    }
  }

  if (prevRightValue > RIGHT_SENSOR_THRESHOLD) {
    if (rightValue < RIGHT_SENSOR_THRESHOLD) {
      Serial.println("Right Sensor Voltage dropped");
      prevRightValue = rightValue;
      if (!rightPowerState) {
        digitalWrite(RIGHTRELAY,HIGH);
        delay(200);
        rightPowerState = true;
        sendPlayEvent(6);
      }
    }
  } else if (prevRightValue < RIGHT_SENSOR_THRESHOLD) {
    if (rightValue > RIGHT_SENSOR_THRESHOLD) {
      Serial.println("Right Sensor Voltage raised");
      prevRightValue = rightValue;
      if (!rightPowerState) {
        digitalWrite(RIGHTRELAY,HIGH);
        delay(200);
        rightPowerState = true;
        sendPlayEvent(6);
      }
    }
  }

  
}

void readButtons()
{
	bool needToToggle = false;

  for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    // If it fell, flag the need to toggle the LED
    if ( buttons[i].fell() ) {  // || buttons[i].rose() 
      needToToggle = true;
      switch (i) {
      	case 0: 
      		Serial.println("Left Button Pressed");
          if (leftPowerState) {
            digitalWrite(LEFTRELAY,LOW);
            delay(200);
            leftPowerState = false;
            sendPlayEvent(1);
          }
      		break;
      	case 1: 
      		Serial.println("Middle Button Pressed");
          if (midPowerState) {
            digitalWrite(MIDRELAY,LOW);
            delay(200);
            midPowerState = false;
            sendPlayEvent(2);
          }
      		break;
      	case 2: 
      		Serial.println("Right Button Pressed");
          if (rightPowerState) {
            digitalWrite(RIGHTRELAY,LOW);
            delay(200);
            rightPowerState = false;
            sendPlayEvent(3);
          }
      	 break;
        /*
      	case 3: 
      		Serial.println("Left Sensor Changed");
      		if (!leftPowerState) {
            digitalWrite(LEFTRELAY,HIGH);
            leftPowerState = true;
            sendPlayEvent(4);
      		}
      		break;
      	case 4: 
      		Serial.println("Middle Sensor Changed");
          if (!midPowerState) {
            digitalWrite(MIDRELAY,HIGH);
            midPowerState = true;
            sendPlayEvent(5);
          }
      		break;
      	case 5: 
      	  Serial.println("Right Sensor Changed");
          if (!rightPowerState) {
            digitalWrite(RIGHTRELAY,HIGH);
            rightPowerState = true;
            sendPlayEvent(6);
          }
      		break;
          */
		default:
			Serial.println("ERROR: Unknown Button Pressed");
			break;      	
      }
    }
  }

  // if a LED toggle has been flagged :
  if ( needToToggle ) {
    // Toggle LED state :
    //ledState = !ledState;
    //digitalWrite(LEDPIN, ledState);
    lastPressMillis = millis();
  }

}
