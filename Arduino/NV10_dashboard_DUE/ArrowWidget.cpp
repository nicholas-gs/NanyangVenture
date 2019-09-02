// 
// 
// 

#include "ArrowWidget.h"

ArrowWidget::ArrowWidget(ILI9488* screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Orientation o) :
Widget(screen, xPos, yPos, width, height, o==LEFT_TO_RIGHT?alignLeft:alignRight, alignCenter)
{
	this->orientation = o;
}

void ArrowWidget::refreshSettings()
{
	wipe();
}

void ArrowWidget::draw()
{
	const uint8_t triangleHeight = height, triangleWidth = width / 3, rectangleHeight = height / 3 * 2, rectangleWidth = width / 3 * 2;
	uint16_t y = yPos + height / 2;
	switch (orientation)
	{
	case RIGHT_TO_LEFT:
		screen->fillTriangle(xPos, y, xPos + triangleWidth, y - (triangleHeight / 2), xPos + triangleWidth, y + (triangleHeight / 2), foreground);
		screen->fillRect(xPos + triangleWidth, y - rectangleHeight / 2, rectangleWidth, rectangleHeight, foreground);
		break;
	case LEFT_TO_RIGHT:
		screen->fillRect(xPos, y - rectangleHeight / 2, rectangleWidth, rectangleHeight, foreground);
		screen->fillTriangle(xPos + rectangleWidth + triangleWidth, y, xPos + rectangleWidth, y - (triangleHeight / 2), xPos + rectangleWidth, y + (triangleHeight / 2), foreground);
		break;
	}
}

void ArrowWidget::updateFrame()
{
	// TODO: timing flash on / off
}

void ArrowWidget::updateNull()
{
}

void ArrowWidget::activate()
{
	draw();
}

void ArrowWidget::deactivate()
{
	wipe();
}


