/*
 Name:		NV11_accessories_MEGA.ino
 Created:	3/8/2019 3:57:21 PM
 Author:	MX
*/

#include <ros.h>
#include <SdFat.h>
#include <CANSerializer.h>
#include <NV11AccesoriesStatus.h>
#include <RelayModule.h>
#include "Pins_lights.h"
//For loggin data to SD Card
#include <NV11BMS.h>
//For ultrasonic sensor
#include "NV11DataUltrasonic.h"

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
const int trigPin1 = 2;
const int trigPin2 = 4;
const int trigPin3 = 6;
const int trigPin4 = 8;
const int trigPin5 = 10;
const int trigPin6 = 12;
const int trigPin7 = 24;
const int echoPin1 = 3;
const int echoPin2 = 5;
const int echoPin3 = 7;
const int echoPin4 = 9;
const int echoPin5 = 11;
const int echoPin6 = 13;
const int echoPin7 = 25;
int duration, distance, RightFront, RightSide, RightBack, LeftFront, LeftSide, LeftBack, Front;

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

	serializer.init(CAN_SPI_CS);
	pinMode(CAN_INTERRUPT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT), CAN_ISR, FALLING);

	//Init the ultrasonic sensor pins
	pinMode(trigPin1, OUTPUT); // Right front
	pinMode(echoPin1, INPUT); 
	pinMode(trigPin2, OUTPUT); // Right side
	pinMode(echoPin2, INPUT); 
	pinMode(trigPin3, OUTPUT); // Right back
	pinMode(echoPin3, INPUT); 
	pinMode(trigPin4, OUTPUT); // Left front
	pinMode(echoPin4, INPUT);
	pinMode(trigPin5, OUTPUT); // Left side
	pinMode(echoPin5, INPUT);
	pinMode(trigPin6, OUTPUT); // Left back
	pinMode(echoPin6, INPUT);
	pinMode(trigPin7, OUTPUT); // Front
	pinMode(echoPin7, INPUT);

	Serial.begin(9600);

	SD_avail = initSD(card);
	debugSerialPort.print("SD avail: ");
	debugSerialPort.println(SD_avail);
}

// loop function manages signal lights blinking
void loop() {
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
	}

	read_ultrasonic(trigPin1, echoPin1);
	RightFront = distance;
	read_ultrasonic(trigPin2, echoPin2);
	RightSide = distance;
	read_ultrasonic(trigPin3, echoPin3);
	RightBack = distance;
	read_ultrasonic(trigPin4, echoPin4);
	LeftFront = distance;
	read_ultrasonic(trigPin5, echoPin5);
	LeftSide = distance;
	read_ultrasonic(trigPin6, echoPin6);
	LeftBack = distance;
	read_ultrasonic(trigPin7, echoPin7);
	Front = distance;

	CANFrame f;
	dataUltra.insertData(RightFront, RightSide, RightBack, LeftFront, LeftSide, LeftBack, Front);
	dataUltra.packCAN(&f);
	serializer.sendCanFrame(&f);

	delay(10);
}

void read_ultrasonic(int trigPin, int echoPin) {
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH);
	distance = duration * 0.0340/2;
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