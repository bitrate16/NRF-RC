// Flash for self-diy NRF24lo* transmiter controller.
// 6 buttons, 4-way sticks, based on CX-10c controller.

#include <EEPROM.h>
#include <SPI.h>
#include "RF24.h"

 #define DEBUG_PRINT
// #define DEBUG_PRINT_RAW
#ifdef DEBUG_PRINT
#include <printf.h>
#endif

// IO mappings

#define STICK0 A3
#define STICK1 A2
#define STICK2 A0
#define STICK3 A1

#define BTN0 3
#define BTN1 6
#define BTN2 5
#define BTN3 4
#define BTN4 9
#define BTN5 A4

#define STLED 2

// Optional moddings

#define BTN_LONG_PRESS 1000

#define LED_ST_OFF        0
#define LED_ST_CONST      1
#define LED_ST_FLASH      2
#define LED_ST_FAST_FLASH 3

#define LED_ST_FLASH_TIME      250
#define LED_ST_FAST_FLASH_TIME 100

#define LED_POWER 150

// Package types
#define PACKAGE_STICKS 3
#define PACKAGE_BUTTON 5
#define PACKAGE_PING   7

#define PACKAGE_TIMEOUT 100

// Amount of packages should be dropped sequently to detect disconnect
#define TX_DROP_EDGE 16


// Long press stick 0 to start calibration, long press to stop
// Struct with min/max calibration values
struct STICK_CALIBRATION {
  int stmx[4];
  int stmn[4];
};

// Struct with info of button presses
struct BTN_STATE {
  int press[6];
  unsigned long time[6];
  // Used in long press to avoid multiple pressing
  int acted[6];
};

// Struct with state of flashing/idle/working LED
// FLASH --> CONST/OFF
// FAST_FLASH --> CONST/OFF
struct LED_STATE {
  // Previous type (for FLASH return to OFF/CONST state)
  int ptype;
  // Type of operation
  int type;
  // ON/OFF
  int state;
  // Time since action
  unsigned long time;
  // Count of flashes
  int count;
};

// Sender package type.
// Contains type and values for button or values for sticks
struct Package {
  int type;
  union {
    struct {
      int number;
      int lpress;
      int dummy[2];
    } button;
    struct {
      int sticks[4];
    } sticks;
  } data;
};

// Info of sticks calibration
STICK_CALIBRATION calibration;
bool calibration_mode = 0;

int sticks[4];

// When entering lock mode, stick data is not updating from input
bool lock_mode = 0;

// info of buttons states
BTN_STATE buttons;
LED_STATE led_state;

// Use NRF24lo1 transmitter on pins 7, 8
RF24 radio(7, 8);

byte addresses[][6] = { "OLEGE", "PIDOR" };

// Amount of sequently dropped packages. Used to detect disconnect
int tx_dropped = 0;

// Used to prevent longpress repeat
// On long press button could be used as sticky press function, 
//  to avoid button sticking and long press act once during while push down, 
//  use this function to mark button as acted
void btn_act(int nubmer);

// Called on button press/longpress
void btn_action(int number, int lpress);

// Set led ON/OFF with change of state flag
void led_set_state(int state);
// Set led action type (FLAST, CONST, e.t.c.)
void led_set(int type, int count);

// Sends single package with given amount of attempts.
// Allows waiting for responce of receiver. The response should be 0.
// Returns 1 on success, 0 on failture.
int send_package(byte* pack, int size);

void setup() {
#ifdef DEBUG_PRINT
  Serial.begin(115200);
#endif
  
  // Set up transmitter
  radio.begin();
  
  //radio.enableAckPayload();
  //radio.setPayloadSize(1);
  //radio.setCRCLength(RF24_CRC_8);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.setChannel(77);
  
#ifdef DEBUG_PRINT
  if (!radio.isChipConnected())
    Serial.println("Radio not connected");
    
  printf_begin();
  radio.printDetails();
#endif

  // Read calibration values
  byte* cal_struct = (byte*) &calibration;
  for (int i = 0; i < sizeof(STICK_CALIBRATION); ++i)
    cal_struct[i] = EEPROM.read(i);
  
#ifdef DEBUG_PRINT
  Serial.println("CALIBRATION: ");
  for (int i = 0; i < 4; ++i) {
    Serial.print('(');
    Serial.print(calibration.stmn[i]);
    Serial.print(", ");
    Serial.print(calibration.stmx[i]);
    Serial.print(") ");
  }
  Serial.println();
#endif
  
  buttons = {{0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}};
  
  // Clear LED state
  led_state.ptype = 0;
  led_state.type  = 0;
  led_state.state = 0;
  led_state.time  = 0;
  led_state.count = 0;
  
  // Prepare pinout
  pinMode(STLED, OUTPUT);
  
  pinMode(STICK0, INPUT_PULLUP);
  pinMode(STICK1, INPUT_PULLUP);
  pinMode(STICK2, INPUT_PULLUP);
  pinMode(STICK3, INPUT_PULLUP);
  
  pinMode(BTN0, INPUT_PULLUP);
  pinMode(BTN1, INPUT_PULLUP);
  
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);
  pinMode(BTN5, INPUT_PULLUP);
}

void loop() {
  // Read sticks & map to calibration
  if (!lock_mode) {
    sticks[0] = analogRead(STICK0);       
    sticks[1] = analogRead(STICK1);       
    sticks[2] = 1024 - analogRead(STICK2);
    sticks[3] = 1024 - analogRead(STICK3);
  }
  
  // Read stick buttons
  int rbtn[6];
  rbtn[0] = !digitalRead(BTN0);
  rbtn[1] = !digitalRead(BTN1);
  
  // read optional buttons
  rbtn[2] = !digitalRead(BTN2);
  rbtn[3] = !digitalRead(BTN3);
  rbtn[4] = !digitalRead(BTN4);
  rbtn[5] = !digitalRead(BTN5);
  
#ifdef DEBUG_PRINT
#ifdef DEBUG_PRINT_RAW
  // Debug out
  Serial.print(sticks[0]); Serial.print(' ');
  Serial.print(sticks[1]); Serial.print(' ');
  Serial.print(sticks[2]); Serial.print(' ');
  Serial.print(sticks[3]); Serial.print(' ');
  
  Serial.print(rbtn[0]); Serial.print(' ');
  Serial.print(rbtn[1]); Serial.print(' ');
  
  Serial.print(rbtn[2]); Serial.print(' ');
  Serial.print(rbtn[3]); Serial.print(' ');
  Serial.print(rbtn[4]); Serial.print(' ');
  Serial.print(rbtn[5]); Serial.print(' ');
  Serial.println();
#endif
#endif

  // Map to calibration
  if (!calibration_mode && !lock_mode) {    
    sticks[0] = map(sticks[0], calibration.stmn[0], calibration.stmx[0], 0, 1023);
    sticks[1] = map(sticks[1], calibration.stmn[1], calibration.stmx[1], 0, 1023);
    sticks[2] = map(sticks[2], calibration.stmn[2], calibration.stmx[2], 0, 1023);
    sticks[3] = map(sticks[3], calibration.stmn[3], calibration.stmx[3], 0, 1023);
  }
  
  // Check buttons states and update timings
  for (int i = 0; i < 6; ++i) {
    if (buttons.press[i] && !rbtn[i]) { // Button released
      if (!buttons.acted[i])
        btn_action(i, (millis() - buttons.time[i]) > BTN_LONG_PRESS);
      buttons.press[i] = 0;
      buttons.time[i]  = 0;
    } else if (buttons.press[i]) { // Button keeps down
      if ((millis() - buttons.time[i]) > BTN_LONG_PRESS && !buttons.acted[i]) { // Toggle long press
        btn_action(i, 1);
        // buttons.press[i] = 0;
        buttons.time[i]  = 0;
      }
    } else if (rbtn[i]) { // Button pressed
      buttons.press[i] = 1;
      buttons.acted[i] = 0;
      buttons.time[i]  = millis();
    }
  }

  // Update LED
  if (led_state.type == LED_ST_FLASH && (led_state.time + LED_ST_FLASH_TIME) < millis()
    ||
    led_state.type == LED_ST_FAST_FLASH && (led_state.time + LED_ST_FAST_FLASH_TIME) < millis()) { // Flash period done
    
    if (!led_state.state) { // Count cycle as finished, try to begin another one
      --led_state.count;
      if (led_state.count <= 0) { // Flashing cycles is done
        led_state.type = led_state.ptype;
        led_set_state(led_state.type == LED_ST_CONST);
      } else { // Turn led ON again, begin next flash cycle
        led_state.time = millis();
        led_set_state(1);
      }
    } else { // Just turn the led OFF
      led_state.time = millis();
      led_set_state(0);
    }
  }
  
  // Update led flashing
  if (led_state.type != LED_ST_FLASH && led_state.type != LED_ST_FAST_FLASH) {
     if (calibration_mode)
      led_set(LED_ST_FAST_FLASH, 4);
    else {
      // Update led lighting
      if (lock_mode)
        led_set(LED_ST_CONST, 0);
      else
        led_set(LED_ST_OFF, 0);
    }
  }
    
  
  // If !paired
  if (calibration_mode) {
    for (int i = 0; i < 4; ++i) {
      if (calibration.stmn[i] > sticks[i])
        calibration.stmn[i] = sticks[i];
      
      if (calibration.stmx[i] < sticks[i])
        calibration.stmx[i] = sticks[i];
    }
  } else {
    // Sending package with sticks
    
    Package pack;
    pack.type = PACKAGE_STICKS;
    pack.data.sticks.sticks[0] = sticks[0];
    pack.data.sticks.sticks[1] = sticks[1];
    pack.data.sticks.sticks[2] = sticks[2];
    pack.data.sticks.sticks[3] = sticks[3];
    
    bool result = send_package((byte*) &pack, sizeof(Package));
    
    if (!result) {
      if (tx_dropped > TX_DROP_EDGE && led_state.type != LED_ST_FLASH && led_state.type != LED_ST_FAST_FLASH) 
        led_set(LED_ST_FLASH, 4);
      
#ifdef DEBUG_PRINT
      Serial.println("TX failed");
#endif
    }
  }
}

int send_package(byte* pack, int size) {
  radio.stopListening();
  if (!radio.write(pack, size)) {
    radio.startListening();
    ++tx_dropped;
    return 0;
  } else {
    radio.startListening();

    // Receive ACK
    int payload;
    if (radio.available()) {
      radio.read(&payload, sizeof(int));
      tx_dropped = 0;
      return 1;
    }
    
    ++tx_dropped;
    return 0;
  }
};

void led_set_state(int state) {
  if (led_state.state && !state) {
    digitalWrite(STLED, 0);
    led_state.state = !led_state.state;
  } else if (!led_state.state && state) {
    analogWrite(STLED, LED_POWER);
    led_state.state = !led_state.state;
  }
};

void led_set(int type, int count) { 
  if (type == LED_ST_CONST || type == LED_ST_OFF) {
    led_set_state(type == LED_ST_CONST);
    led_state.type = type;
  } else {
    // if was flashing --> rewrite
    // if was constant --> move type to ptype & do flashing
    if (led_state.type == LED_ST_CONST || led_state.type == LED_ST_OFF) 
      led_state.ptype = led_state.type;
    
    led_state.type  = type;
    led_state.count = count;
    led_state.time  = millis();
    led_set_state(1);
  }
};

void btn_act(int number) {
  buttons.acted[number] = 1;
};

// Override calls for button presses
void btn_action(int number, int lpress) {
#ifdef DEBUG_PRINT
  Serial.print("Press for "); Serial.println(number);
#endif
  
  switch(number) {
    // calibration
    case 0: { 
      // press  = ?
      // Lpress = calibration mode
      
      if (lpress) {
        if (calibration_mode && !lock_mode) {
          // Write calibration values
          byte* cal_struct = (byte*) &calibration;
          for (int i = 0; i < sizeof(STICK_CALIBRATION); ++i)
            EEPROM.write(i, cal_struct[i]);
          
          calibration_mode = 0;
          
#ifdef DEBUG_PRINT
          Serial.println("NEW CALIBRATION: ");
          for (int i = 0; i < 4; ++i) {
            Serial.print('(');
            Serial.print(calibration.stmn[i]);
            Serial.print(", ");
            Serial.print(calibration.stmx[i]);
            Serial.print(") ");
          }
          Serial.println();
#endif
        } else {
          for (int i = 0; i < 4; ++i) {
            calibration.stmn[i] = 1024;
            calibration.stmx[i] = 0;
          }
          
          calibration_mode = 1;
        }
      }
      break;
    }
    
    case 1: { 
      // press  = ?
      // Lpress = lock mode
      
      if (lpress) {
        lock_mode = !lock_mode;
      }
      
      break;
    }
  }
  led_set(LED_ST_FAST_FLASH, 2);
  btn_act(number);
  
  Package pack;
  pack.type = PACKAGE_BUTTON;
  pack.data.button.number = number;
  pack.data.button.lpress = lpress;
  
  bool result = send_package((byte*) &pack, sizeof(Package));
  
#ifdef DEBUG_PRINT
  if (!result)
    Serial.println("Button TX failed");
#endif
};
