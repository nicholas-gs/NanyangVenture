// 
// 
// 

#include "TextWidget.h"

const int textWidthPerSize = 6;
const int textHeightPerSize = 8;

void TextWidget::draw()
{
	screen->setTextSize(textSize);
	screen->setCursor(cursorX, cursorY);
	screen->setTextColor(foreground, background); // set BG color so that BG can wipe out the old text
	screen->println(text);
}

void TextWidget::refreshSettings()
{
	// clear text box
	screen->setTextColor(foreground, background);
	screen->setTextSize(textSize);
	screen->setCursor(cursorX, cursorY);
	for (int i = 0; i < prevTextWidth; i++)
		screen->print(" ");

	// a char of text size 1 is 6px * 8px (H * V), which is ratio 3 / 4
	if (width > height * 3 / 4)
	{
		// take text size using height, margin excluded 
		textSize = (height - margin) / textHeightPerSize / max(textWidth, textHeight);
		//Serial.print(textSize);
		//Serial.print(" ");
		//Serial.print(textWidth);
		//Serial.print(" ");
		//Serial.print(textHeight);
		//Serial.print(" ");
		//Serial.println("Height < Width, take height");
	}
	else
	{
		// take text size using width, margin excluded 
		textSize = (width - margin) / textWidthPerSize / max(textWidth, textHeight);
		//Serial.print(textSize);
		//Serial.print(" ");
		//Serial.print(textWidth);
		//Serial.print(" ");
		//Serial.print(textHeight);
		//Serial.print(" ");
		//Serial.println("Width < Height, take width.");
	}
	//Serial.print("Text size: ");
	//Serial.println(textSize);
	if (textSize < 4)
		textSize *= 2;
	else if (textSize < 6)
		textSize *= 1.5;
	cursorX = constrain(xPos + width / 2 - textWidth * textSize * textWidthPerSize / 2, 0, SCREENWIDTH);
	cursorY = constrain(yPos + height / 2 - textHeight * textSize * textHeightPerSize / 2, 0, SCREENHEIGHT);
}

TextWidget::TextWidget(ILI9488 * screen, uint16_t xPos, uint16_t yPos, uint16_t width, uint16_t height, Alignment xAlign, Alignment yAlign) :DataWidget(screen, xPos, yPos, width, height, xAlign, yAlign)
{
	refreshSettings();
}

void TextWidget::updateFrame()
{
}

void TextWidget::updateNull()
{
	if (!stuck)
	{
		updateText("---");
		stuck = true;
	}
}

void TextWidget::updateFloat(float value)
{
	char tmp[5];
	sprintf(tmp, "%3.0f", value);
	updateText(tmp);
}

void TextWidget::updateText(const char* text)
{
	this->text = text;
	textWidth = strlen(text);
	if (prevTextWidth != textWidth)
	{
		refreshSettings();
	}
	prevTextWidth = textWidth;
	draw();
}
