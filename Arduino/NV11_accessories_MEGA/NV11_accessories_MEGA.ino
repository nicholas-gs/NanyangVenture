/*
 Name:		NV11_accessories_MEGA.ino
 Created:	3/8/2019 3:57:21 PM
 Author:	MX
*/

#include <SPI.h>
#include <ros.h>
#include <SdFat.h>
#include <CANSerializer.h>
#include <NV11AccesoriesStatus.h>
#include <RelayModule.h>
#include "Pins_lights.h"
//For loggin data to SD Card
#include <NV11BMS.h>
//For ultrasonic sensor
#include <NV11DataUltrasonic.h>
// For calculating distance in cm from signal pin for ultrasonic sensor
#define DISTANCE_FACTOR 58.138

CANSerializer serializer;
NV11AccesoriesStatus dataAcc;
NV11DataUltrasonic dataUltra;
NV11BMS dataBMS;
RelayModule brakeRelay(BRAKELIGHT_OUTPUT, RelayModule::NO);
RelayModule runninglightRelay(RUNNINGLIGHT_OUTPUT, RelayModule::NC);
RelayModule lsigRelay(LSIG_OUTPUT, RelayModule::NO);
RelayModule rsigRelay(RSIG_OUTPUT, RelayModule::NO);
bool sigOn;
unsigned long sigNextProc = 0;
const unsigned long sigInterval = 500; // 500ms delay between each signal light flash

//Pins for ultrasonic sensors
const int sigPin1 = 2;
const int sigPin2 = 4;
const int sigPin3 = 6;
const int sigPin4 = 8;
const int sigPin5 = 10;
const int sigPin6 = 12;
const int sigPin7 = 24;

uint16_t distance, RightFront, RightSide, RightBack, LeftFront, LeftSide, LeftBack, Front;

HardwareSerial& debugSerialPort = Serial;
char dataBMSString[100];
bool SD_avail = false;
SdFat card;

void setup() {
	brakeRelay.init();
	runninglightRelay.init();
	lsigRelay.init();
	rsigRelay.init();

	runninglightRelay.activate();

	// Maybe wiring is wrong for CAN?
	serializer.init(CAN_SPI_CS);
	pinMode(CAN_INTERRUPT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT), CAN_ISR, FALLING);

	//Init the ultrasonic sensor pins
	pinMode(sigPin1, OUTPUT); // Right front
	digitalWrite(sigPin1, LOW);

	pinMode(sigPin2, OUTPUT); // Right side
	digitalWrite(sigPin2, LOW);

	pinMode(sigPin3, OUTPUT); // Right back
	digitalWrite(sigPin3, LOW);

	pinMode(sigPin4, OUTPUT); // Left front
	digitalWrite(sigPin4, LOW);

	pinMode(sigPin5, OUTPUT); // Left side
	digitalWrite(sigPin5, LOW);

	pinMode(sigPin6, OUTPUT); // Left back
	digitalWrite(sigPin6, LOW);

	pinMode(sigPin7, OUTPUT); // Front
	digitalWrite(sigPin7, LOW);

	Serial.begin(9600);

	SD_avail = initSD(card);
	debugSerialPort.print("SD avail: ");
	debugSerialPort.println(SD_avail);
}

// loop function manages signal lights blinking
void loop() {
	CANFrame f;
	// relay is PULLUP by default
	// writing HIGH enables NC, disables NO
	// writing LOW enables NO, disables NC
	if (millis() > sigNextProc)
	{
		sigNextProc += sigInterval;
		if (sigOn)
		{
			// turn off signal lights
			sigOn = false;
			lsigRelay.deactivate();
			rsigRelay.deactivate();
		}
		else
		{
			// turn on signal lights if commanded to do so
			if (dataAcc.getHazard() == STATE_EN || dataAcc.getLsig() == STATE_EN)
			{
				sigOn = true;
				lsigRelay.activate();
			}
			if (dataAcc.getHazard() == STATE_EN || dataAcc.getRsig() == STATE_EN)
			{
				sigOn = true;
				rsigRelay.activate();
			}
		}

		// Get distance from all ultrasonic sensors
		RightFront = read_ultrasonic(sigPin1);
		RightSide = read_ultrasonic(sigPin2);
		RightBack = read_ultrasonic(sigPin3);
		LeftFront = read_ultrasonic(sigPin4);
		LeftSide = read_ultrasonic(sigPin5);
		LeftBack = read_ultrasonic(sigPin6);
		Front = read_ultrasonic(sigPin7);
		// Send distance readings through the CAN bus
		CANFrame f;
		dataUltra.insertData(RightFront, RightSide, RightBack, LeftFront, LeftSide, LeftBack, Front);
		dataUltra.packCAN(&f);
		bool sent = serializer.sendCanFrame(&f);

		// Debug purposes
		if (sent) {
			Serial.println("CAN Frame sent");
		}
		else {
			Serial.println("Cannot send CANframe");
		}
	}

	delay(10);
}

/*
	Triggers a single ultrasonic sensor and returns the calculated distance in cm
*/
uint16_t read_ultrasonic(int sig_pin) {
	pinMode(sig_pin, OUTPUT);
	digitalWrite(sig_pin, LOW);
	delayMicroseconds(2);
	// Send logic high to trigger the ultrasonic sensor
	digitalWrite(sig_pin, HIGH);
	delayMicroseconds(5);
	// Turn off trigger signal
	digitalWrite(sig_pin, LOW);
	// Config signal pin to input
	pinMode(sig_pin, INPUT);
	// Read in time the sig_pin remains high
	distance = pulseIn(sig_pin, HIGH);

	distance = distance / DISTANCE_FACTOR;

	return distance;
}

void CAN_ISR()
{
	CANFrame f;
	serializer.receiveCanFrame(&f);
	if (dataAcc.checkMatchCAN(&f))
	{
		dataAcc.unpackCAN(&f);
		if (dataAcc.getBrake() == STATE_EN)
			brakeRelay.activate();
		else
			brakeRelay.deactivate();
		if (dataAcc.getHeadlights() == STATE_EN)
			runninglightRelay.activate();
		else
			runninglightRelay.deactivate();
	}
	if (dataBMS.checkMatchCAN(&f))
	{
		dataBMS.packString(dataBMSString);
		if (SD_avail) {
			File writtenFile = card.open("BMS_data.txt", FILE_WRITE);
			writtenFile.println(dataBMSString);
			writtenFile.close();
		}
	}
}

/* test code

	brakeRelay.activate();
	Serial.println("brake ON");
	delay(1000);
	brakeRelay.deactivate();
	Serial.println("brake OFF");
	delay(1000);

	delay(1000);

	runninglightRelay.activate();
	Serial.println("Runninglight ON");
	delay(1000);
	runninglightRelay.deactivate();
	Serial.println("Runninglight OFF");
	delay(1000);

	delay(1000);

	digitalWrite(LSIG_OUTPUT, LOW); // lsig ON
	Serial.println("Lsig ON");
	delay(1000);
	digitalWrite(LSIG_OUTPUT, HIGH); // lsig OFF
	Serial.println("Lsig OFF");
	delay(1000);

	delay(1000);

	digitalWrite(RSIG_OUTPUT, LOW); // rsig ON
	Serial.println("Rsig ON");
	delay(1000);
	digitalWrite(RSIG_OUTPUT, HIGH); // rsig OFF
	Serial.println("Rsig OFF");
	delay(1000);

	delay(2000);
*/
