// 
// 
// 

#include "BarWidget.h"

BarWidget::BarWidget(ILI9488* screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Orientation orientation) :\
DataWidget(screen, xPos, yPos, width, height, orientation == LEFT_TO_RIGHT ? alignLeft : alignRight, alignCenter)
{
	setOrientation(orientation);
}
BarWidget::BarWidget(ILI9488* screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Alignment xAlign, Alignment yAlign) :\
DataWidget(screen, xPos, yPos, width, height, xAlign, yAlign)
{
	setOrientation(LEFT_TO_RIGHT);
}
void BarWidget::updateFrame()
{
	// TODO: slide the bar anim?
}
void BarWidget::updateFloat(float value)
{
	if (stuck)
	{
		// re-paint rectangles
		if (facing == -1)
		{
			screen->fillRect(xPos + width - margin - ceil(toWidthScale(thisValue)), yPos + margin, \
				ceil(toWidthScale(thisValue)), actualHeight, \
				foreground);
		}
		else if (facing == 1)
		{
			screen->fillRect(xPos + margin, yPos + margin, \
				ceil(toWidthScale(thisValue)), actualHeight, \
				foreground);
		}
		stuck = false;
	}
	prevValue = thisValue;
	thisValue = constrain(value, minVal, maxVal);
	draw();
}
void BarWidget::updateNull()
{
	if (!stuck)
	{
		uint16_t color = ILI9488_GREEN;
		if (facing == 1)
		{
			screen->drawFastHLine(xPos + margin, yPos + (height / 4), toWidthScale(thisValue), color);
			screen->drawFastHLine(xPos + margin, yPos + (height / 2), toWidthScale(thisValue), color);
			screen->drawFastHLine(xPos + margin, yPos + (height * 3 / 4), toWidthScale(thisValue), color);
		}
		else if (facing == -1)
		{
			screen->drawFastHLine(xPos + width - margin - toWidthScale(thisValue), yPos + (height / 4), toWidthScale(thisValue), color);
			screen->drawFastHLine(xPos + width - margin - toWidthScale(thisValue), yPos + (height / 2), toWidthScale(thisValue), color);
			screen->drawFastHLine(xPos + width - margin - toWidthScale(thisValue), yPos + (height * 3 / 4), toWidthScale(thisValue), color);
		}
		stuck = true;
	}
}
void BarWidget::draw()
{
	float thisWidth = toWidthScale(thisValue);
	float prevWidth = toWidthScale(prevValue);
	float diff = abs(thisWidth - prevWidth);
	if (facing == -1)
	{
		if (thisValue > prevValue)
		{
			// value has increased: draw foreground color, from old value to new value
			screen->fillRect(xPos + width - margin - ceil(thisWidth), yPos + margin, \
				ceil(diff), actualHeight, \
				foreground);
		}
		else if (prevValue > thisValue)
		{
			// value has decreased: draw background color, from new value to old value
			screen->fillRect(xPos + width - margin - ceil(prevWidth), yPos + margin, \
				ceil(diff), actualHeight, \
				background);
		}
	}
	else if (facing == 1)
	{
		if (thisValue > prevValue)
		{
			// value has increased: draw foreground color, from old value to new value
			screen->fillRect(xPos + margin + floor(prevWidth), yPos + margin, \
				ceil(diff), actualHeight, \
				foreground);
		}
		else if (prevValue > thisValue)
		{
			// value has decreased: draw background color, from new value to old value
			screen->fillRect(xPos + margin + floor(thisWidth), yPos + margin, \
				ceil(diff), actualHeight, \
				background);
		}

	}
}
BarWidget::~BarWidget()
{
}
void BarWidget::setRange(uint16_t minVal, uint16_t maxVal)
{
	this->minVal = minVal;
	this->maxVal = maxVal;
	// synchronize history to avoid jumps 
	thisValue = prevValue = minVal;
}
void BarWidget::setOrientation(Orientation o)
{
	switch (o)
	{
	case LEFT_TO_RIGHT:
		facing = 1;
		break;
	case RIGHT_TO_LEFT:
		facing = -1;
		break;
	}
}
void BarWidget::refreshSettings()
{
	actualWidth = width - 2 * margin;
	actualHeight = height - 2 * margin;
	if (facing == -1)
	{
		screen->fillRect(xPos + width - margin - ceil(toWidthScale(thisValue)), yPos + margin, \
			ceil(toWidthScale(thisValue)), actualHeight, foreground);
	}
	else if (facing == 1)
	{
		screen->fillRect(xPos + margin, yPos + margin, \
			ceil(toWidthScale(thisValue)), actualHeight, foreground);
	}
}
float BarWidget::toWidthScale(float value)
{
	return map(value, minVal, maxVal, 0, actualWidth);// (value - minVal) / (maxVal - minVal) * actualWidth + minVal;
}