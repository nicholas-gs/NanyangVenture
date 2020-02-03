#include <FlexCAN.h>

void setup() {
	delay(2000);
	Serial.println(F("Hello Teensy 3.5 CAN Test."));
	Can0.begin(1000000);
}

void loop() {
	CAN_message_t inMsg;
	inMsg.id = 0x100;
  if(Can0.available()){
    Serial.println("Message available");
  } else {
    Serial.println("Not available");
  }
  Can0.read(inMsg); 
  Serial.println(inMsg.id);
	while (Can0.available()) {
		Serial.println("Reading in message...");
		Can0.read(inMsg);
		printMsg(inMsg);
	}
	Serial.println("End of loop...");
}

void printMsg(CAN_message_t& inMsg) {
	Serial.print("ID: "); Serial.println(inMsg.id);
	Serial.print("Length: "); Serial.println(inMsg.len);
	Serial.print("Data: "); 
	for (int i = 0; i < inMsg.len; i++) {
		Serial.print(inMsg.buf[i]);
	}
	Serial.println("\n");
}
