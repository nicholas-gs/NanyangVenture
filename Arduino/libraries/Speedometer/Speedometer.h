/*
 Name:		Speedometer.h
 Created:	5/31/2019 6:19:13 PM
 Author:	MX
 Editor:	http://www.visualmicro.com
*/

#ifndef _SPEEDOMETER_h
#define _SPEEDOMETER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class Speedometer
{
public:
	// directional speedometer, pinA should be an interrupt pin
	Speedometer(uint16_t pinA, uint16_t pinB, uint16_t diameterInMM, uint16_t ticksPerRot, bool positivePhaseshiftWhenForward);
	// non-directional speedometer, pin should be an interrupt pin
	Speedometer(uint16_t pin, uint16_t diameterInMM, uint16_t ticksPerRot);
	void trip();
	uint32_t getTotalDistTravelled();
	float getSpeedKmh();
	uint32_t getTicks();

private:
	uint32_t counter = 0, lastCounter = 0, lastTime = 0;
	uint16_t diameter, ticksPerRot;
	float ticksToMmMultiplier;
	uint8_t pinA, pinB;

	bool directional;
	int8_t positiveIndicator;
	bool phaseShift;
};

#endif

