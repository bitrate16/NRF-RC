// Flash for self-diy NRF24lo* receiver controller.
// 6 buttons, 4-way sticks.

#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

 #define DEBUG_PRINT
#ifdef DEBUG_PRINT
#include <printf.h>
#endif

#define MOTOR   3
#define SERVO_A 5
#define SERVO_B 6

// Mappings for sticks
#define SERVO_MAP_STICK0 0, 180
#define SERVO_MAP_STICK1 0, 180
#define SERVO_MAP_STICK2 0, 180
#define SERVO_MAP_STICK3 60, 120

#define ANALOG_MAP_STICK0 0, 255
#define ANALOG_MAP_STICK1 0, 255
#define ANALOG_MAP_STICK2 0, 255
#define ANALOG_MAP_STICK3 0, 255

// Optional moddings


// Package types
#define PACKAGE_STICKS 3
#define PACKAGE_BUTTON 5
#define PACKAGE_PING   7

// Used to detect disconnect from the controller
#define CONNECTION_TIMEOUT 100

// Mpdes for output of the motor
// #define MODE_2_DIGITAL_1_ANALOG
#define MODE_2_ANALOG

#define INVERT_STICK0 0
#define INVERT_STICK1 0
#define INVERT_STICK2 0
#define INVERT_STICK3 0


// Receiver package struct.
// Contains type and values for button or values for sticks
struct Package {
	int type;
	union {
		struct {
			int number;
			int lpress;
			// Used to complete size of the package.
			// Without it, payload is not sending for sticks.
			int dummy[2];
		} button;
		struct {
			int sticks[4];
		} sticks;
	} data;
};

// Describes state of a single buttons on controller
// Single press -> 1
// Second press -> 0
struct Button {
	bool state;
	bool lstate;
};

// Use NRF24lo1 transmitter on pins 7, 8
RF24 radio(9, 10);

byte addresses[][6] = { "OLEGE_", "PIDOR_" };

// Used to detect timed disconnection from the controller
unsigned long last_receive_time;
bool disconnected;

// Servos mapped to sticks
Servo servo[2];

// Buttons states 
Button buttons[6];

// Called on connection restored after drop
void on_connection();

// Called on connection dropped
void on_disconnection();

// Called on button state received
void button_action(int button, int lpress);

// Called on sticks state received
void sticks_action(int sticks[4]);

void setup() {
#ifdef DEBUG_PRINT
	Serial.begin(115200);
#endif
	
	// Set up receiver
	radio.begin();

	radio.powerDown();
	delay(200);
	radio.powerUp();
	
	radio.setChannel(77);
	//radio.setAutoAck(1);
	//radio.setRetries(0, 8);
	radio.enableAckPayload();
	radio.setPALevel(RF24_PA_MAX);
	//radio.setCRCLength(RF24_CRC_8);
	radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
	
#ifdef DEBUG_PRINT
	// Debugger output
	printf_begin();
	radio.printDetails();
#endif
	
	radio.startListening();
	
	// Init buttons with 0
	buttons[0] = { 0, 0 };
	buttons[1] = { 0, 0 };
	buttons[2] = { 0, 0 };
	buttons[3] = { 0, 0 };
	buttons[4] = { 0, 0 };
	buttons[5] = { 0, 0 };
	
	// Set up pinout
	pinMode(SERVO_A, OUTPUT);
	pinMode(SERVO_B, OUTPUT);
	pinMode(MOTOR, OUTPUT);
 
	// setPWMNanofrequency(0x02);
	
	// Set up servos
	servo[0].attach(SERVO_A); // Remapped servos to leave three PWM pins
	servo[1].attach(SERVO_B);
	
	// Clear disconnect trigger
	last_receive_time = millis();
	disconnected = 0;
	
	// Reset outputs
	digitalWrite(MOTOR, LOW);
}

void loop() {
	byte payload = 13;
	radio.writeAckPayload(1, &payload, sizeof(byte));
	if (radio.available()) {
		Package pack;
		radio.read((byte*) &pack, sizeof(Package));
		
		// Update trigger
		last_receive_time = millis();
		if (disconnected)
			on_connection();
		disconnected = 0;
		
		switch(pack.type) {
			case PACKAGE_STICKS: {
#ifdef DEBUG_PRINT
				Serial.print(pack.data.sticks.sticks[0]); Serial.print(' ');
				Serial.print(pack.data.sticks.sticks[1]); Serial.print(' ');
				Serial.print(pack.data.sticks.sticks[2]); Serial.print(' ');
				Serial.print(pack.data.sticks.sticks[3]); Serial.println();
#endif

				if (INVERT_STICK0) pack.data.sticks.sticks[0] = 1024 - pack.data.sticks.sticks[0];
				if (INVERT_STICK1) pack.data.sticks.sticks[1] = 1024 - pack.data.sticks.sticks[1];
				if (INVERT_STICK2) pack.data.sticks.sticks[2] = 1024 - pack.data.sticks.sticks[2];
				if (INVERT_STICK3) pack.data.sticks.sticks[3] = 1024 - pack.data.sticks.sticks[3];
				
				int ssticks[4];
				ssticks[0] = map(pack.data.sticks.sticks[0], 0, 1023, SERVO_MAP_STICK0);
				ssticks[1] = map(pack.data.sticks.sticks[1], 0, 1023, SERVO_MAP_STICK1);
				ssticks[2] = map(pack.data.sticks.sticks[2], 0, 1023, SERVO_MAP_STICK2);
				ssticks[3] = map(pack.data.sticks.sticks[3], 0, 1023, SERVO_MAP_STICK3);
				
				int asticks[4];
				asticks[0] = map(pack.data.sticks.sticks[0], 0, 1023, ANALOG_MAP_STICK0);
				asticks[1] = map(pack.data.sticks.sticks[1], 0, 1023, ANALOG_MAP_STICK1);
				asticks[2] = map(pack.data.sticks.sticks[2], 0, 1023, ANALOG_MAP_STICK2);
				asticks[3] = map(pack.data.sticks.sticks[3], 0, 1023, ANALOG_MAP_STICK3);
				
#ifdef DEBUG_PRINT
				Serial.print("Servo data: ");
				Serial.print(ssticks[0]); Serial.print(' ');
				Serial.print(ssticks[1]); Serial.print(' ');
				Serial.print(ssticks[2]); Serial.print(' ');
				Serial.print(ssticks[3]); Serial.println();
				
				Serial.print("Analog data: ");
				Serial.print(asticks[0]); Serial.print(' ');
				Serial.print(asticks[1]); Serial.print(' ');
				Serial.print(asticks[2]); Serial.print(' ');
				Serial.print(asticks[3]); Serial.println();
#endif

				sticks_action(pack.data.sticks.sticks, ssticks, asticks);
				break;
			}
			
			case PACKAGE_BUTTON: {
#ifdef DEBUG_PRINT
				Serial.print(pack.data.button.number); Serial.print(' ');
				Serial.print(pack.data.button.lpress); Serial.println();
#endif				
				
				// Update states of the buttons
				if (pack.data.button.lpress)
					buttons[pack.data.button.number].lstate = !buttons[pack.data.button.number].lstate;
				else
					buttons[pack.data.button.number].state = !buttons[pack.data.button.number].state;
				
				button_action(pack.data.button.number, pack.data.button.lpress);
				break;
			}
		}
	} else if (!disconnected && millis() - last_receive_time > CONNECTION_TIMEOUT) {
		disconnected = 1;
		on_disconnection();
	}
}

void on_connection() {
	
};

void on_disconnection() {
	// servo[0].write(0);
	// servo[1].write(0);
	// servo[2].write(0);
	// servo[3].write(0);
	
  digitalWrite(SERVO_A, LOW);
  digitalWrite(SERVO_B, LOW);
  digitalWrite(MOTOR, LOW);
  
	// analogWrite(CONNECT_A0, 0);
	// analogWrite(CONNECT_A1, 0);
	// analogWrite(CONNECT_A2, 0);
	// analogWrite(CONNECT_A3, 0);
#ifdef MODE_2_DIGITAL_1_ANALOG
	digitalWrite(SERVO_A, LOW);
	digitalWrite(SERVO_B, LOW);
	digitalWrite(MOTOR, LOW);
#endif
#ifdef MODE_2_ANALOG
  digitalWrite(SERVO_A, LOW);
  digitalWrite(SERVO_B, LOW);
  digitalWrite(MOTOR, LOW);
#endif
};

void button_action(int button, int lpress) {
	switch (button) {
		case 0:
		case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      break;
	}
};

void sticks_action(int sticks[4], int ssticks[4], int asticks[4]) {	
  analogWrite(MOTOR, sticks[0] / 4);
	servo[0].write(ssticks[2]);
	servo[1].write(ssticks[3]);
};
