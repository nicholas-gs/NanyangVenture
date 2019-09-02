/*
 Name:		NV11_wiper_UNO.ino
 Created:	4/12/2019 8:49:29 PM
 Author:	MX
*/

#include <CANSerializer.h>
#include <NV11AccesoriesStatus.h>
#include <avr\wdt.h>

#define CAN_CS 10
#define CAN_INT 2
#define WIPERRELAY_OUTPUT2 3
#define WIPERRELAY_OUTPUT1 4
CANSerializer serializer;
NV11AccesoriesStatus dataAcc;
// the setup function runs once when you press reset or power the board
void setup() {
	serializer.init(CAN_CS);
	pinMode(WIPERRELAY_OUTPUT2, OUTPUT);
	pinMode(WIPERRELAY_OUTPUT1, OUTPUT);
	digitalWrite(WIPERRELAY_OUTPUT2, HIGH);
	digitalWrite(WIPERRELAY_OUTPUT1, HIGH);

	pinMode(CAN_INT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CAN_INT), CAN_ISR, FALLING);

	wdt_enable(WDTO_2S);
}

// the loop function runs over and over again until power down or reset
void loop() {
	CANFrame f;
	if (!digitalRead(CAN_INT))
	{
		serializer.receiveCanFrame(&f);
		if (dataAcc.checkMatchCAN(&f))
		{
			wdt_reset(); // kick watchdog to avoid resetting arduino
			dataAcc.unpackCAN(&f);
			// H = logic high relay light on
			// L = logic low relay light off
			// --------------------------------------------------
			// relay lights are read from left to right
			// HH: rest (unused, only appears when: arduino not present OR invalid wiper status)
			// HL: rest
			// LH: slow
			// LL: fast
			switch (dataAcc.getWiper())
			{
			case NV11AccesoriesStatus::wiperOff:
				digitalWrite(WIPERRELAY_OUTPUT2, HIGH);
				digitalWrite(WIPERRELAY_OUTPUT1, LOW);
				break;
			case NV11AccesoriesStatus::wiperSlow:
				digitalWrite(WIPERRELAY_OUTPUT2, LOW);
				digitalWrite(WIPERRELAY_OUTPUT1, HIGH);
				break;
			case NV11AccesoriesStatus::wiperFast:
				digitalWrite(WIPERRELAY_OUTPUT2, LOW);
				digitalWrite(WIPERRELAY_OUTPUT1, LOW);
				break;
			default:
				digitalWrite(WIPERRELAY_OUTPUT2, HIGH);
				digitalWrite(WIPERRELAY_OUTPUT1, HIGH);
				break;
			}
		}
	}
	// if no CAN message comes in 2 seconds, UNO will self-reset


	//Serial.println("Wiper OFF");
	//digitalWrite(WIPERRELAY_OUTPUT2, HIGH);
	//digitalWrite(WIPERRELAY_OUTPUT1, HIGH);
	//delay(2000);
	//Serial.println("Wiper Slow");
	//digitalWrite(WIPERRELAY_OUTPUT2, LOW);
	//digitalWrite(WIPERRELAY_OUTPUT1, HIGH);
	//delay(2000);
	//Serial.println("Wiper Fast");
	//digitalWrite(WIPERRELAY_OUTPUT2, LOW);
	//digitalWrite(WIPERRELAY_OUTPUT1, LOW);
	//delay(2000);
	//delay(100);
}
void CAN_ISR()
{
}