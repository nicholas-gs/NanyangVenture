/*
 Name:		NV10_dashboard2_DUE.ino
 Created:	3/21/2019 8:01:28 PM
 Author:	MX
*/

/*
WARNING

	DO NOT ENABLE debug(...) in this script. Verified to malfunction.
*/

/*
2-Screen Dashboard Plan

Screen 1:
	1. Speed---
	2. L-Sig. and R-Sig.---
	3. Current lap time and lap time needed
	4. Current lap number
	5. Headlight On? 
	6. FC overheating?---
	7. FC status---
Screen 2:
	1. Motor voltage(From FC)---
	3. Motor current(V1 - NA,    V2 - From shunt)
	4. FC energy usage(V1 - NA,  V2 - From Shunt+Voltage divider or Watt Meter)
	5. FC power usage(From FC)---
	
*/
#include "DashboardScreens.h"
#include "ArrowWidget.h"
#include "BarWidget.h"
#include "TextWidget.h"

#include "Pins_dashboard.h"
#include <FreeRTOS_ARM.h>

#include <NV10FuelCell.h>
#include <NV10CurrentSensor.h>
#include <NV10CurrentSensorStats.h>
#include <NV10AccesoriesStatus.h>
#include <NV11DataSpeedo.h>
#include <NV11Commands.h>

NV10FuelCell dataFC;
NV10CurrentSensor dataCS;
NV10CurrentSensorStats dataCSStats;
NV11DataSpeedo dataSpeedo;
NV10AccesoriesStatus dataAcc;
NV11Commands dataCommands;

DashboardScreens d;
HardwareSerial& CANSerialPort = Serial1;
HardwareSerial& debugSerialPort = Serial;

void setDebounce(const unsigned int pins[], uint8_t numPins, uint16_t waitTimeMultiplier = 500);
void setup()
{
	debugSerialPort.begin(9600);
	CANSerialPort.begin(9600);
	CANSerialPort.setTimeout(500);

	pinMode(CAN_OUTPUT_RST, OUTPUT);
	digitalWrite(CAN_OUTPUT_RST, LOW);
	delay(100);
	digitalWrite(CAN_OUTPUT_RST, HIGH);

	d.dashboardInit();
	// I tried putting attachinterrupt in the for loop above but failed. Lambda functions complain.
	// So here, have some wall text.
	pinMode(BTN_HAZARD, INPUT_PULLUP);
	pinMode(BTN_WIPER, INPUT_PULLUP);
	pinMode(BTN_HORN, INPUT_PULLUP);
	pinMode(BTN_HEADLIGHT, INPUT_PULLUP);
	pinMode(BTN_LSIG, INPUT_PULLUP);
	pinMode(BTN_RSIG, INPUT_PULLUP);
	pinMode(BTN_STOPWATCH, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(BTN_HAZARD), [] {
		dataAcc.toggleHazard();
	}, FALLING);
	attachInterrupt(digitalPinToInterrupt(BTN_WIPER), [] {
		dataAcc.toggleWiper();
	}, FALLING);
	attachInterrupt(digitalPinToInterrupt(BTN_HORN), [] {
		dataCommands.triggerHorn();
	}, FALLING);
	attachInterrupt(digitalPinToInterrupt(BTN_HEADLIGHT), [] {
		dataAcc.toggleHeadlights();
	}, FALLING);
	attachInterrupt(digitalPinToInterrupt(BTN_LSIG), [] {
		dataAcc.toggleLsig();
	}, FALLING);
	attachInterrupt(digitalPinToInterrupt(BTN_RSIG), [] {
		dataAcc.toggleRsig();
	}, FALLING);
	attachInterrupt(digitalPinToInterrupt(BTN_STOPWATCH), [] {
		dataCommands.triggerLaps();
	}, FALLING);

	const unsigned int pins[] = { BTN_HAZARD, BTN_WIPER, BTN_HORN, BTN_HEADLIGHT, BTN_LSIG, BTN_RSIG, BTN_STOPWATCH };
	setDebounce(pins, sizeof(pins) / sizeof(pins[0]));

	dataAcc.insertData(0, 0, 0, 0, 0, 0);
}
void loop()
{
	char s[100];
	// output dashboard display based on incoming CAN strings
	uint8_t bytesRead = CANSerialPort.readBytesUntil('\n', s, 100);
	if (bytesRead > 0)
	{
		s[bytesRead - 1] = '\0';
		debugSerialPort.print("<R> ");
		debugSerialPort.println(s);
		if (dataFC.checkMatchString(s))
		{
			dataFC.unpackString(s);
			d.dashboardNextValuesFC(dataFC.getVolts(), dataFC.getAmps(), dataFC.getPressure(), dataFC.getTemperature(), dataFC.getStatus());
		}
		else if (dataCS.checkMatchString(s))
		{
			dataCS.unpackString(s);
			d.dashboardNextValuesCS(dataCS.getVolt(), dataCS.getAmpCapIn(), dataCS.getAmpCapOut(), dataCS.getAmpMotor());
		}
		else if (dataSpeedo.checkMatchString(s))
		{
			dataSpeedo.unpackString(s);
			d.dashboardNextValuesSpeed(dataSpeedo.getSpeed());
		}
		else if (dataAcc.checkMatchString(s))
		{
			dataAcc.unpackString(s);
		}
		else if (dataCommands.checkMatchString(s))
		{
			dataCommands.unpackString(s);
			d.dashboardNextValueTime(dataCommands.getLapTime());
		}
	}
	// output CAN strings based on buttons inputs (already handled by interrupts)

	if (dataAcc.dataHasChanged())
	{
		d.dashboardToggleSig(dataAcc.getLsig(), dataAcc.getRsig());
		dataAcc.packString(s);
		CANSerialPort.println(s);
		debugSerialPort.print("<S> ");
		debugSerialPort.println(s);
	}
	if (dataCommands.dataHasChanged())
	{
		dataCommands.packString(s);
		CANSerialPort.println(s);
		debugSerialPort.print("<S> ");
		debugSerialPort.println(s);

		dataCommands.clearActivationHistory();
	}

	d.dashboardNextFrame();

	static uint8_t canRstCounter = 0;
	if (canRstCounter++ > 30)
	{
		canRstCounter = 0;
		digitalWrite(CAN_OUTPUT_RST, LOW);
		delay(100);
		digitalWrite(CAN_OUTPUT_RST, HIGH);
	}
	// wait until 500ms elapsed
	static unsigned long lastTime;
	while (millis() - lastTime < 200);
	lastTime = millis();
}