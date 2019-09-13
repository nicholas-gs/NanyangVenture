/*
 Name:		NV11_steeringwheel_UNO.ino
 Created:	4/3/2019 1:28:11 AM
 Author:	MX
*/
#include <CANSerializer.h>
#include <NV11AccesoriesStatus.h>
#include <NV11Commands.h>
#include "Pins_steeringwheel.h"
CANSerializer serializer;
NV11AccesoriesStatus dataAcc;
NV11Commands dataCommands;

bool canAvail;
uint8_t prevRead[20] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 }; // store previous digitalReads to artificially detect "Rising edge"
// the setup function runs once when you press reset or power the board
//Since we want to check if button was pressed and then released.
void setup() {
	Serial.begin(9600);

	pinMode(LSIG_INPUT, INPUT_PULLUP);
	pinMode(RSIG_INPUT, INPUT_PULLUP);
	pinMode(HAZARD_INPUT, INPUT_PULLUP);
	pinMode(HEADLIGHT_INPUT, INPUT_PULLUP);
	pinMode(BRAKE_INPUT, INPUT_PULLUP);
	pinMode(WIPER_INPUT1, INPUT_PULLUP);
	pinMode(WIPER_INPUT2, INPUT_PULLUP);
	pinMode(FOURWSS_INPUT1, INPUT_PULLUP);
	pinMode(FOURWSS_INPUT2, INPUT_PULLUP);
	pinMode(REGEN_INPUT, INPUT_PULLUP);

	pinMode(SHUTDOWNPI_INPUT, INPUT_PULLUP);
	pinMode(LAPCOUNTER_INPUT, INPUT_PULLUP);
	pinMode(STATUSLED_OUTPUT, OUTPUT);

	delay(1000); // wait for all other Arduinos to startup
	canAvail = serializer.init(CAN_SPI_CS);
	if (!canAvail)
	{
		Serial.println("CAN init fail");
		digitalWrite(STATUSLED_OUTPUT, HIGH);
	}
	else
		Serial.println("CAN init success");
	attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), CAN_ISR, FALLING);
}

// the loop function runs over and over again until power down or reset
void loop() {
	CANFrame f;

	// ... code to populate dataCommands (eg: dataCommands.activateHorn())
	dataCommands.clearActivationHistory();
	if (!digitalRead(SHUTDOWNPI_INPUT) && prevRead[SHUTDOWNPI_INPUT])
	{
		dataCommands.triggerShutdownRPi();
		prevRead[SHUTDOWNPI_INPUT] = 0;
	}
	else
	{
		prevRead[SHUTDOWNPI_INPUT] = digitalRead(SHUTDOWNPI_INPUT);
	}
	if (!digitalRead(LAPCOUNTER_INPUT) && prevRead[LAPCOUNTER_INPUT])
	{
		dataCommands.triggerLaps();
		prevRead[LAPCOUNTER_INPUT] = 0;
	}
	else
	{
		prevRead[LAPCOUNTER_INPUT] = digitalRead(LAPCOUNTER_INPUT);
	}
	
	if (dataCommands.dataRequiresBroadcast())
	{
		dataCommands.packCAN(&f);
		serializer.sendCanFrame(&f);
	}
	// ... code to populate dataAcc (eg: dataAcc.setLsig(STATE_EN))

	if (!digitalRead(LSIG_INPUT) && prevRead[LSIG_INPUT])
	{
		if (dataAcc.getLsig())
		{
			dataAcc.setLsig(NV11AccesoriesStatus::disable);
		}
		else
		{
			dataAcc.setLsig(NV11AccesoriesStatus::enable);
		}
		prevRead[LSIG_INPUT] = 0;
	}
	else
	{
		prevRead[LSIG_INPUT] = digitalRead(LSIG_INPUT);
	}

	if (!digitalRead(RSIG_INPUT) && prevRead[RSIG_INPUT])
	{
		if (dataAcc.getRsig())
		{
			dataAcc.setRsig(NV11AccesoriesStatus::disable);
		}
		else
		{
			dataAcc.setRsig(NV11AccesoriesStatus::enable);
		}
		prevRead[RSIG_INPUT] = 0;
	}
	else
	{
		prevRead[RSIG_INPUT] = digitalRead(RSIG_INPUT);
	}

	if (!digitalRead(HAZARD_INPUT) && prevRead[HAZARD_INPUT])
	{
		if (dataAcc.getHazard())
		{
			dataAcc.setHazard(NV11AccesoriesStatus::disable);
		}
		else
		{
			dataAcc.setHazard(NV11AccesoriesStatus::enable);
		}
		prevRead[HAZARD_INPUT] = 0;
	}
	else
	{
		prevRead[HAZARD_INPUT] = digitalRead(HAZARD_INPUT);
	}

	if (!digitalRead(HEADLIGHT_INPUT) && prevRead[HEADLIGHT_INPUT])
	{
		if (dataAcc.getHeadlights())
		{
			dataAcc.setHeadlights(NV11AccesoriesStatus::disable);
		}
		else
		{
			dataAcc.setHeadlights(NV11AccesoriesStatus::enable);
		}
		prevRead[HEADLIGHT_INPUT] = 0;
	}
	else
	{
		prevRead[HEADLIGHT_INPUT] = digitalRead(HEADLIGHT_INPUT);
	}

	if (!digitalRead(BRAKE_INPUT))
		dataAcc.setBrake(NV11AccesoriesStatus::enable);
	else
		dataAcc.setBrake(NV11AccesoriesStatus::disable);

	if (!digitalRead(REGEN_INPUT))
		dataAcc.setRegen(NV11AccesoriesStatus::enable);
	else
		dataAcc.setRegen(NV11AccesoriesStatus::disable);

	if (!digitalRead(FOURWSS_INPUT1))
		dataAcc.setFourWS(NV11AccesoriesStatus::fourWScounterPhase); // left
	else if (!digitalRead(FOURWSS_INPUT2))
		dataAcc.setFourWS(NV11AccesoriesStatus::fourWSinPhase); // right
	else
		dataAcc.setFourWS(NV11AccesoriesStatus::fourWSfrontOnly); // center

	if (!digitalRead(WIPER_INPUT1))
		dataAcc.setWiper(NV11AccesoriesStatus::wiperOff); // left
	else if (!digitalRead(WIPER_INPUT2))
		dataAcc.setWiper(NV11AccesoriesStatus::wiperFast); // right
	else
		dataAcc.setWiper(NV11AccesoriesStatus::wiperSlow); // center

	if (dataAcc.dataRequiresBroadcast())
	{
		digitalWrite(STATUSLED_OUTPUT, HIGH^canAvail);
		dataAcc.packCAN(&f);
		serializer.sendCanFrame(&f);
		canAvail = serializer.checkNoError();
	}
	delay(100);
	digitalWrite(STATUSLED_OUTPUT, LOW^canAvail); // light up default when CAN is available
}
void CAN_ISR()
{

}
//void printButtons()
//{
//
//	Serial.println("--------ACTIVE LOW BUTTONS--------");
//	for (int i = 2; i <= 9; i++)
//	{
//		if (!digitalRead(i))
//		{
//			Serial.print(i);
//			Serial.print('\t');
//		}
//	}
//	for (int i = A0; i <= A5; i++)
//	{
//		if (!digitalRead(i))
//		{
//			Serial.print('A');
//			Serial.print(i-A0);
//			Serial.print('\t');
//		}
//	}
//	Serial.println();
//}


	//NV11Commands dataCommands(0x11);
	//// ... code to populate dataCommands (eg: dataCommands.activateHorn())
	//dataCommands.clearActivationHistory();
	//if (!digitalRead(BRAKE_INPUT))
	//{
	//	dataCommands.activateHorn();
	//}
	//
	//if (dataCommands.getHorn() == NV11AccesoriesStatus::enable)
	//{
	//	dataCommands.packCAN(&f);
	//	serializer.sendCanFrame(&f);
	//}