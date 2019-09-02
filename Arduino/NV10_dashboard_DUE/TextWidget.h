// TextWidget.h

#ifndef _TEXTWIDGET_h
#define _TEXTWIDGET_h

#include "DataWidget.h"
class TextWidget:public DataWidget
{
 protected:

	 uint8_t textSize = 2;
	 int16_t cursorX = 0;
	 int16_t cursorY = 0;
	 const char* text;
	 uint8_t textWidth = 1;
	 uint8_t textHeight = 1;
	 uint8_t prevTextWidth = 0;
	 uint8_t prevTextHeight = 0;
	 void draw();
	 void refreshSettings();

 public:
	TextWidget(ILI9488 * screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Alignment xAlign, Alignment yAlign);
	void updateFrame();
	void updateNull();
	void updateFloat(float value);
	void updateText(const char* text);
};

#endif

