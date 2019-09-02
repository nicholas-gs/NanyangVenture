// DataWidget.h

#ifndef _DATAWIDGET_h
#define _DATAWIDGET_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Widget.h"
class DataWidget :public Widget
{
protected:
	DataWidget(ILI9488 * screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Alignment xAlign, Alignment yAlign);
public:
	virtual void updateFloat(float value) = 0;
};

#endif

