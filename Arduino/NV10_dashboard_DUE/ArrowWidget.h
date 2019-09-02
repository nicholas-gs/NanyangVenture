// ArrowWidget.h

#ifndef _ARROWWIDGET_h
#define _ARROWWIDGET_h

#include "Widget.h"
class ArrowWidget:public Widget
{
 protected:
	 Orientation orientation;
	 void refreshSettings();
	 void draw();

 public:
	 ArrowWidget(ILI9488 * screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Orientation o);
	 void updateFrame();
	 void updateNull();
	 void activate();
	 void deactivate();
};

#endif

