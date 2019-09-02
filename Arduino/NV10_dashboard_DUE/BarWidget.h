// BarWidget.h

#ifndef _BARWIDGET_h
#define _BARWIDGET_h

#include "DataWidget.h"
class BarWidget :
	public DataWidget
{
public:
	BarWidget(ILI9488 * screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Orientation orientation);
	BarWidget(ILI9488 * screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Alignment xAlign, Alignment yAlign);
	void updateFrame();
	void updateNull();
	void updateFloat(float value);
	void setRange(uint16_t minVal, uint16_t maxVal);
	~BarWidget();
private:
	uint16_t minVal = 0, maxVal = 100;
	uint16_t actualWidth, actualHeight;
	float prevValue = 0;
	float thisValue = 0;
	int8_t facing = 1;
	void refreshSettings();
	float toWidthScale(float value);
	void draw();
	void setOrientation(Orientation o);
};


#endif

