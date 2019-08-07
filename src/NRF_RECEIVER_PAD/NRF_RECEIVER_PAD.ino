// Flash for self-diy NRF24lo* receiver controller.
// 6 buttons, 4-way sticks.

#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

// #define DEBUG_PRINT
#ifdef DEBUG_PRINT
#include <printf.h>
#endif

// IO mappings

#define CONNECT_0 2
#define CONNECT_1 3
#define CONNECT_2 4
#define CONNECT_3 5
#define CONNECT_4 6
#define CONNECT_5 7
#define CONNECT_6 8

#define CONNECT_A0 A0
#define CONNECT_A1 A1
#define CONNECT_A2 A2
#define CONNECT_A3 A3
#define CONNECT_A4 A4
#define CONNECT_A5 A5
// A6, A7 on nano are only analog input
#define CONNECT_A6 A6
#define CONNECT_A7 A7

// Inverters for sticks
#define INVERT_STICK0 0
#define INVERT_STICK1 0
#define INVERT_STICK2 0
#define INVERT_STICK3 0

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
Servo servo[4];

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
	pinMode(CONNECT_0, OUTPUT);
	pinMode(CONNECT_2, OUTPUT);
	pinMode(CONNECT_3, OUTPUT);
	pinMode(CONNECT_4, OUTPUT);
	pinMode(CONNECT_5, OUTPUT);
	pinMode(CONNECT_6, OUTPUT);
	
	pinMode(CONNECT_A0, OUTPUT);
	pinMode(CONNECT_A1, OUTPUT);
	pinMode(CONNECT_A2, OUTPUT);
	pinMode(CONNECT_A3, OUTPUT);
	pinMode(CONNECT_A4, OUTPUT);
	pinMode(CONNECT_A5, OUTPUT);
	pinMode(CONNECT_A6, OUTPUT);
	pinMode(CONNECT_A7, OUTPUT);
	
	// Set up servos
	servo[0].attach(CONNECT_0); // Remapped servos to leave three PWM pins
	servo[1].attach(CONNECT_1);
	servo[2].attach(CONNECT_5);
	servo[3].attach(CONNECT_6);
	
	// Clear disconnect trigger
	last_receive_time = millis();
	disconnected = 0;
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
	
	// analogWrite(CONNECT_A0, 0);
	// analogWrite(CONNECT_A1, 0);
	// analogWrite(CONNECT_A2, 0);
	// analogWrite(CONNECT_A3, 0);
	analogWrite(CONNECT_2, 0);
	analogWrite(CONNECT_3, 0);
};

void button_action(int button, int lpress) {
	switch (button) {
		case 0:
		case 1:
			break;
			
		case 2: {
			digitalWrite(CONNECT_A0, buttons[button].state);
			break;
		}
		
		case 3: {
			digitalWrite(CONNECT_A1, buttons[button].state);
			break;
		}
		
		case 4: {
			digitalWrite(CONNECT_A2, buttons[button].state);
			break;
		}
		
		case 5: {
			digitalWrite(CONNECT_A3, buttons[button].state);
			break;
		}
	}
};

void sticks_action(int sticks[4], int ssticks[4], int asticks[4]) {	
	servo[0].write(ssticks[0]);
	servo[1].write(ssticks[1]);
	servo[2].write(ssticks[2]);
	servo[3].write(ssticks[3]);
	
	// analogWrite(CONNECT_A0, asticks[0]);
	// analogWrite(CONNECT_A1, asticks[1]);
	// analogWrite(CONNECT_A2, asticks[2]);
	// analogWrite(CONNECT_A3, asticks[3]);
	
	// Control H-Bridge over 3, 4 outputs with right stick
	int value = (sticks[2] > 500) ? (sticks[2] - 500) : (sticks[2] < 480) ? (480 - sticks[2]) : (0);
	
	if (value < 500)
		value = map(value, 0, 480, 0, 255);
	else
		value = map(value, 0, 1023 - 500, 0, 255);
	
	value = (value > 255) ? 255 : (value < 0) ? 0 : value;
	analogWrite(CONNECT_3, sticks[2] > 500 ? value : 0);
	analogWrite(CONNECT_4, sticks[2] < 480 ? value : 0);
};
