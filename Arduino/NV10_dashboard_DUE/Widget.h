#pragma once

#include <Arduino.h>
#include <ILI9488.h>
extern "C" void __cxa_pure_virtual(void);
typedef enum {
	alignLeft,
	alignCenter,
	alignRight,
	alignTop = alignLeft,
	alignBtm = alignRight
}Alignment;
typedef enum {
	LEFT_TO_RIGHT, RIGHT_TO_LEFT,
	TOP_TO_BOTTOM, BOTTOM_TO_TOP
}Orientation;

class Widget
{
protected:
	ILI9488* screen;
	// the ID this widget is assigned to, remains constant once assigned
	//PacketID targetID;
	// the DataPoint this widget sees whenever update() is called 
	//float* trackedValue;
	// --------- drawing purposes ---------
	const uint16_t SCREENHEIGHT = 320, SCREENWIDTH = 480;
	int16_t xPos, yPos;
	uint16_t width, height;
	uint16_t margin = 8;
	uint8_t borderStroke = 3;
	uint16_t borderFill = ILI9488_WHITE;
	uint16_t foreground = ILI9488_WHITE;
	uint16_t background = ILI9488_BLACK;
	bool stuck = false;
	virtual void refreshSettings() = 0;
	virtual void draw() = 0;
public:
	Widget(ILI9488 * screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Alignment xAlign, Alignment yAlign);
	~Widget();
	void init();
	virtual void setMargin(uint8_t margin);
	void drawBorder(uint8_t stroke, uint16_t color);
	virtual void setColors(uint16_t foreground, uint16_t background);
	virtual void updateFrame() = 0;
	virtual void updateNull() = 0;
	void wipe();
};

