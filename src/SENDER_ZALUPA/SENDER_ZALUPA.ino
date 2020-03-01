
#include <SPI.h>
#include "RF24.h"
#include "printf.h"

RF24 radio(7, 8);

byte address[6] = "PIDOR";


void setup() {
  Serial.begin(115200);
  Serial.println("Sender.begin()");

  radio.begin();
  
  radio.enableAckPayload();
  //radio.setAutoAck(0);
  //radio.setPayloadSize(sizeof(int));
  radio.setCRCLength(RF24_CRC_8);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setDataRate(RF24_250KBPS);
  radio.setChannel(77);
  radio.openWritingPipe(address);
  //radio.setRetries(3, 5);
  radio.stopListening();

  printf_begin();
  radio.printDetails();
}

int data = 0;
void loop() {
  data = 14;
  if (radio.write(&data, 2)) {
    Serial.print("Send data ");
    Serial.print(data);
    delay(100);
    if (radio.isAckPayloadAvailable()) {
      radio.read(&data, 2);
      Serial.print(", received ACK: ");
      Serial.println(data);
    } else {
      Serial.println(", no ACK received");
    }
  } else {
    Serial.println("Failed send data");
  }
}
  
  
