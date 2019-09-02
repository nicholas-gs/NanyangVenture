/*
 Name:		Speedometer.cpp
 Created:	5/31/2019 6:19:13 PM
 Author:	MX
 Editor:	http://www.visualmicro.com
*/

#include "Speedometer.h"

Speedometer::Speedometer(uint16_t pinA, uint16_t pinB, uint16_t diameterInMM, uint16_t ticksPerRot, bool positivePhaseshiftWhenForward):pinA(pinA), pinB(pinB), diameter(diameterInMM), ticksPerRot(ticksPerRot), phaseShift(positivePhaseshiftWhenForward), directional(true)
{
	ticksToMmMultiplier = diameter * PI / ticksPerRot;
}

Speedometer::Speedometer(uint16_t pin, uint16_t diameterInMM, uint16_t ticksPerRot) :pinA(pin), pinB(NOT_A_PIN), diameter(diameterInMM), ticksPerRot(ticksPerRot), directional(false)
{
	ticksToMmMultiplier = diameter * PI / ticksPerRot;
}

void Speedometer::trip()
{
	counter++;
	if (!directional || (digitalRead(pinB) ^ phaseShift))
		positiveIndicator = 1;
	else
		positiveIndicator = -1;
}

uint32_t Speedometer::getTotalDistTravelled()
{
	uint32_t mmTravelled = ticksToMmMultiplier * counter;
	return mmTravelled;
}

float Speedometer::getSpeedKmh()
{
	// get deltas
	uint32_t counterDiff = counter - lastCounter;
	lastCounter = counter;
	uint32_t timeDiff = millis() - lastTime;
	lastTime = millis();
	// calc speed
	uint32_t mmTravelled = ticksToMmMultiplier * counterDiff;
	float speedKmh = 3.6 * mmTravelled / timeDiff;
	return speedKmh * positiveIndicator;
}

uint32_t Speedometer::getTicks()
{
	return counter;
}
