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

CANSerializer serializer;
NV11AccesoriesStatus dataAcc;
NV11BMS dataBMS;
RelayModule brakeRelay(BRAKELIGHT_OUTPUT, RelayModule::NO);
RelayModule runninglightRelay(RUNNINGLIGHT_OUTPUT, RelayModule::NC);
RelayModule lsigRelay(LSIG_OUTPUT, RelayModule::NO);
RelayModule rsigRelay(RSIG_OUTPUT, RelayModule::NO);
bool sigOn;
unsigned long sigNextProc = 0;
const unsigned long sigInterval = 500; // 500ms delay between each signal light flash

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

	}

	delay(10);
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
