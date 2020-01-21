/*
    Name:       NV11_CANtoSerial_UNO.ino
    Created:	8/3/2019 12:06:26 AM
    Author:     DESKTOP-GV1MS6E\MX
*/
#define CAN_CS 10
#define CAN_INT 2
#include <CANSerializer.h>
#include <NV11AccesoriesStatus.h>
#include <NV11BMS.h>
#include <NV11DataSpeedo.h>
#include <NV11Commands.h>
#include <NV11DataUltrasonic.h>
CANSerializer serializer;
NV11DataSpeedo dataSpeedo;
NV11AccesoriesStatus dataAccesories;
NV11BMS dataBMS;
NV11Commands dataCommands;
NV11DataUltrasonic dataUltrasonic;

// DataPoint* canListenList[] = { &dataSpeedo, &dataBMS, &dataCommands, &dataAccesories, &dataUltrasonic }; // incoming messages: speedo BMS
// DataPoint* serialListenList[] = { &dataAccesories }; // outgoing messages: accessories
void setup()
{
	Serial.begin(9600);
	// Serial.setTimeout(500);
	// while (!Serial);
	bool setup = serializer.init(CAN_CS);
 if(setup){
  Serial.println("Setup successful");
 } else {
  Serial.println("Cannot setup!");
 }
	pinMode(CAN_INT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(CAN_INT), CAN_ISR, FALLING);
}

void loop()
{
	/*
	CANFrame f;
	char toSerialPort[100];M
	char fromSerialPort[100];
	if (serializer.receiveCanFrame(&f))
	{
		Serial.println("Receiving CAN frame"); // Debug purpose
		for (int i = 0; i < sizeof(canListenList) / sizeof(canListenList[0]); i++)
		{
			DataPoint& dataPoint = *canListenList[i];
			if (dataPoint.checkMatchCAN(&f))
			{
				Serial.println("Match Can Frame"); // Debug purpose
				dataPoint.unpackCAN(&f);
				dataPoint.packString(toSerialPort);
				Serial.println(toSerialPort);
				break;
			}
		}
	}
	if (Serial.available())
	{
		int bytesRead = Serial.readBytesUntil('\n', fromSerialPort, 100); 
		fromSerialPort[bytesRead-1] = '\0'; // "\r\n" should be the terminator, replace '\r' with '\0'
		for (int i = 0; i < sizeof(serialListenList) / sizeof(serialListenList[0]); i++)
		{
			DataPoint& dataPoint = *serialListenList[i];
			if (dataPoint.checkMatchString(fromSerialPort))
			{
				dataPoint.unpackString(fromSerialPort);
				dataPoint.packCAN(&f);
				serializer.sendCanFrame(&f);
				break;
			}
		}
	} */
}

void CAN_ISR()
{
	Serial.println("CAN Frame received!");
	CANFrame canFrame;
	serializer.receiveCanFrame(&canFrame);

	if (dataUltrasonic.checkMatchCAN(&canFrame)) {
		dataUltrasonic.unpackCAN(&canFrame);
		printUltrasonicData(dataUltrasonic);
	}
	else if (dataSpeedo.checkMatchCAN(&canFrame)) {
		dataSpeedo.unpackCAN(&canFrame);
		printSpeedoData(dataSpeedo);
	}
	else if (dataAccesories.checkMatchCAN(&canFrame)) {
		dataAccesories.unpackCAN(&canFrame);
		printAccessoriesData(dataAccesories);
	}
	else if (dataBMS.checkMatchCAN(&canFrame)) {
		dataBMS.unpackCAN(&canFrame);
		printBMSData(dataBMS);
	}
	else if (dataCommands.checkMatchCAN(&canFrame)) {
		dataCommands.unpackCAN(&canFrame);
		printDataCommands(dataCommands);
	}
}

void printUltrasonicData(NV11DataUltrasonic& dataUltrasonic) {
	Serial.println(F("****** Data Ultrasonic ******"));
	Serial.print(F("Front: ")); Serial.println(dataUltrasonic.getFront());
	Serial.print(F("Right Front: ")); Serial.println(dataUltrasonic.getRightFront());
	Serial.print(F("Right Side: ")); Serial.println(dataUltrasonic.getRightSide());
	Serial.print(F("Right back: ")); Serial.println(dataUltrasonic.getRightBack());
	Serial.print(F("Left Front: ")); Serial.println(dataUltrasonic.getLeftFront());
	Serial.print(F("Left Side: ")); Serial.println(dataUltrasonic.getLeftSide());
	Serial.print(F("Left back: ")); Serial.println(dataUltrasonic.getLeftBack());
	Serial.println("\n");
}

void printSpeedoData(NV11DataSpeedo& dataSpeedo) {
	Serial.println(F("****** Data Speedo ******"));
	Serial.print(F("Speed: ")); Serial.println(dataSpeedo.getSpeed());
	Serial.println("\n");
}

void printAccessoriesData(NV11AccesoriesStatus& dataAccessories) {
	Serial.println(F("****** Data Accessories ******"));
	Serial.print(F("lsig: ")); Serial.println(dataAccesories.getLsig());
	Serial.print(F("rsig: ")); Serial.println(dataAccesories.getRsig());
	Serial.print(F("Hazard: ")); Serial.println(dataAccesories.getHazard());
	Serial.print(F("Headlights: ")); Serial.println(dataAccesories.getHeadlights());
	Serial.print(F("Brake: ")); Serial.println(dataAccesories.getBrake());
	Serial.print(F("Wiper: ")); Serial.println(dataAccesories.getWiper());
	Serial.print(F("FourWS: ")); Serial.println(dataAccesories.getFourWS());
	Serial.print(F("Regen: ")); Serial.println(dataAccesories.getRegen());
	Serial.println("\n");
}

void printBMSData(NV11BMS& bmsData) {
	Serial.println(F("****** Data BMS ******"));
	Serial.print(F("Voltage: ")); Serial.println(bmsData.getVolt());
	Serial.print(F("Amp: ")); Serial.println(bmsData.getAmp());
	Serial.print(F("Temp: ")); Serial.println(bmsData.getTemperature());
	Serial.println("\n");
}

void printDataCommands(NV11Commands& dataCommands) {
	Serial.println(F("****** Data Commands ******"));
	Serial.print(F("Horn: ")); Serial.println(dataCommands.getHorn());
	Serial.print(F("Lap Trig: ")); Serial.println(dataCommands.getLapTrig());
	Serial.print(F("Lap Time: ")); Serial.println(dataCommands.getLapTime());
	Serial.print(F("Lap Count: ")); Serial.println(dataCommands.getLapCount());
	Serial.println("\n");
}
