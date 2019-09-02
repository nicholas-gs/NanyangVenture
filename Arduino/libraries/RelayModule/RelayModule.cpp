/*
 Name:		RelayModule.cpp
 Created:	4/14/2019 4:34:07 PM
 Author:	MX
 Editor:	http://www.visualmicro.com
*/

#include "RelayModule.h"

RelayModule::RelayModule(uint8_t pin, eRelayHookup hookup, eActive active):pin(pin)
{
	invert = hookup ^ active;
}
void RelayModule::init()
{
	pinMode(pin, OUTPUT);
	deactivate();
}
void RelayModule::activate()
{
	digitalWrite(pin, !invert);
	active = true;
}
void RelayModule::deactivate()
{
	digitalWrite(pin, invert);
	active = false;
}

bool RelayModule::activated()
{
	return active;
}
