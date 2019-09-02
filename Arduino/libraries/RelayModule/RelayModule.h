/*
 Name:		RelayModule.h
 Created:	4/14/2019 4:34:07 PM
 Author:	MX
 Editor:	http://www.visualmicro.com
*/

#ifndef _RelayModule_h
#define _RelayModule_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class RelayModule
{
public:
	enum eRelayHookup { NO, NC };
	enum eActive { activeHigh, activeLow };
	RelayModule(uint8_t pin, eRelayHookup hookup, eActive active = activeLow);
	void init();
	void activate();
	void deactivate();
	bool activated();
private:
	uint8_t pin, invert;
	bool active;
};

#endif

