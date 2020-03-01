
#include <SPI.h>
#include "RF24.h"
#include "printf.h"

RF24 radio(9, 10);

byte address[6] = "PIDOR";


void setup() {
  Serial.begin(115200);
  Serial.println("Receiver.begin()");

  radio.begin();
  
  radio.enableAckPayload();
  //radio.setAutoAck(0);
  //radio.setPayloadSize(sizeof(int));
  radio.setCRCLength(RF24_CRC_8);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setDataRate(RF24_250KBPS);
  radio.setChannel(77);
  radio.openReadingPipe(1, address);
  //radio.setRetries(3, 5);
  radio.startListening();

  printf_begin();
  radio.printDetails();
}

int data = 0;
void loop() {
  ++data;
data = 13;
  radio.writeAckPayload(1, &data, 2);
  if (radio.available()) {
    radio.read(&data, 2);
    Serial.print("Received: ");
    Serial.println(data);
  Serial.println("Sending ACK");
  } else {
    
  }
}
  
  
