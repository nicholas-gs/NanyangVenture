// DashboardScreens.h

#ifndef _DASHBOARDSCREENS_h
#define _DASHBOARDSCREENS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <ILI9488.h>
#include "TextWidget.h"
#include "BarWidget.h"
#include "ArrowWidget.h"
#include "Pins_dashboard.h"

class DashboardScreens
{

	const uint16_t SCREENWIDTH = 480;
	const uint16_t SCREENHEIGHT = 360;

	ILI9488 centerScreen = ILI9488(LCDCENTER_SPI_CS, LCD_OUTPUT_DC, LCD_OUTPUT_RST);
	//ILI9488 leftScreen = ILI9488(LCDLEFT_SPI_CS, LCD_OUTPUT_DC);
	//ILI9488 rightScreen = ILI9488(LCDRIGHT_SPI_CS, LCD_OUTPUT_DC);

	// center screen x offset: +25, -5
	TextWidget speedTxt = TextWidget(&centerScreen, SCREENWIDTH / 2, SCREENHEIGHT / 2, 150, 150, alignCenter, alignBtm);
	ArrowWidget lSigArrow = ArrowWidget(&centerScreen, SCREENWIDTH/2 - 120, SCREENHEIGHT / 2 - 30, 100, 80, RIGHT_TO_LEFT);
	ArrowWidget rSigArrow = ArrowWidget(&centerScreen, SCREENWIDTH/2 + 120, SCREENHEIGHT / 2 - 30, 100, 80, LEFT_TO_RIGHT);
	TextWidget status_txt = TextWidget(&centerScreen, SCREENWIDTH / 2, SCREENHEIGHT / 2 + 0, 150, 75, alignCenter, alignTop);
	//// left screen
	uint8_t btmOffset = 70;
	uint8_t centerSpacing = 80;
	//TextWidget capOutAmp_txt = TextWidget(&leftScreen, 0, 0, 90, 75, alignLeft, alignTop);
	//TextWidget capInAmp_txt = TextWidget(&leftScreen, SCREENWIDTH, 0, 90, 75, alignRight, alignTop);
	TextWidget motorAmp_txt = TextWidget(&centerScreen, SCREENWIDTH/2, SCREENHEIGHT - btmOffset, 80, 60, alignRight, alignCenter);
	TextWidget motorVolt_txt = TextWidget(&centerScreen, SCREENWIDTH/2, SCREENHEIGHT - btmOffset, 80, 60, alignLeft, alignCenter);
	//BarWidget capOutAmp_bar = BarWidget(&leftScreen, 480 / 2 - 10, 100, 220, 35, RIGHT_TO_LEFT);
	//BarWidget capInAmp_bar = BarWidget(&leftScreen, 480 / 2 + 10, 100, 220, 35, LEFT_TO_RIGHT);
	BarWidget motorAmp_bar = BarWidget(&centerScreen, SCREENWIDTH / 2 - centerSpacing, SCREENHEIGHT - btmOffset, 150, 60, RIGHT_TO_LEFT);
	BarWidget motorVolt_bar = BarWidget(&centerScreen, SCREENWIDTH / 2 + centerSpacing, SCREENHEIGHT - btmOffset, 150, 60, LEFT_TO_RIGHT); // original y=180, h=70
	//// right screen
	//TextWidget stackTemperature_txt = TextWidget(&rightScreen, 0, 0, 140, 80, alignLeft, alignTop);
	//TextWidget status_txt = TextWidget(&rightScreen, SCREENWIDTH / 2, 0, 200, 80, alignCenter, alignTop);
	//TextWidget pressure_txt = TextWidget(&rightScreen, SCREENWIDTH, 0, 140, 80, alignRight, alignTop);
	//TextWidget energy_txt = TextWidget(&rightScreen, SCREENWIDTH / 2, SCREENHEIGHT / 2 - 40 + 20, 400, 80, alignCenter, alignCenter);
	//BarWidget energy_bar = BarWidget(&rightScreen, SCREENWIDTH / 2, SCREENHEIGHT / 2 + 40 + 20, 400, 80, alignCenter, alignCenter);
	TextWidget time_txt = TextWidget(&centerScreen, SCREENWIDTH / 2, SCREENHEIGHT - btmOffset, 80, 60, alignLeft, alignCenter);
	TextWidget time_required_txt = TextWidget(&centerScreen, SCREENWIDTH / 2, SCREENHEIGHT - btmOffset, 80, 60, alignLeft, alignCenter);

	const uint8_t fcTimeout = 20, csTimeout = 10, smTimeout = 10;
	uint8_t fcTimeoutCounter, csTimeoutCounter, smTimeoutCounter, requiredLapTime, timeLeft;
public:
	DashboardScreens();
	void dashboardInit();
	void dashboardNextFrame();
	void dashboardNextValuesFC(int volts, int amps, float pressure, int temperature, const char* status);
	void dashboardNextValuesCS(int volts, int ampCapIn, int ampCapOut, int ampMotor);
	void dashboardNextValuesCS(int volts, int ampMotor);
	void dashboardNextValueTime(int time, int lapCount);
	void dashboardNextValueTime(int time);
	void dashboardNextValuesSpeed(int speedKmh);
	void dashboardToggleSig(int lsig, int rsig);
};

#endif

