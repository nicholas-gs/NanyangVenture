/*
 Name:		MultitaskLights.ino
 Created:	11/1/2018 1:31:02 AM
 Author:	MX
*/

#include <CANSerializer.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "Pins_front.h"
#include <MemoryFree.h>

#include <NV10AccesoriesStatus.h>
#include <NV11Commands.h>

NV10AccesoriesStatus dataAcc;
NV11Commands dataCommands;
/*
hl
sig (blink)
horn
brake in & CAN send
CAN recv
*/
#define PIXELS 6
Adafruit_NeoPixel lstrip = Adafruit_NeoPixel(PIXELS, LSIG_OUTPUT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rstrip = Adafruit_NeoPixel(PIXELS, RSIG_OUTPUT, NEO_GRB + NEO_KHZ800);
const uint32_t SIG_COLOR = Adafruit_NeoPixel::Color(255, 165, 0);
const uint32_t NO_COLOR = Adafruit_NeoPixel::Color(0, 0, 0);

// define globals
CANSerializer serializer;

/*
peripheralStates:
	Headlights
	Horn
	Lsig
	Rsig
	Wiper
	Hazard
*/
/* -------------------------------------------------------
 SERIAL PLOTTER WITH BUTTONS TO CHECK STABILITY

 NOTE:
	This is a slave device. Its state is purely set to match the dashboard's commands.

 ------------------------------------------------------- */
// toggles brake, headlights, horn
void TaskToggle(void* pvParameters);
void TaskBlink(void* pvParameters);
void TaskMoveWiper(void* pvParameters);
void TaskCAN(void* pvParameters);
QueueHandle_t queueForCAN = xQueueCreate(1, sizeof(CANFrame));
TaskHandle_t taskBlink, taskMoveWiper, taskToggle;

HardwareSerial& debugSerialPort = Serial;
void setup() {
	debugSerialPort.begin(9600);

	pinMode(PEDALBRAKE_INTERRUPT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PEDALBRAKE_INTERRUPT), BRAKE_ISR, FALLING);

	if (!serializer.init(CAN_SPI_CS, 1000))
		debugSerialPort.println("CAN FAIL!");
	xTaskCreate(
		TaskToggle
		, (const portCHAR *)"HEAD"
		, 200
		, NULL
		, 1
		, &taskToggle);
	xTaskCreate(
		TaskCAN
		, (const portCHAR *)"CAN la"
		, 300
		, NULL
		, 2
		, NULL);
	xTaskCreate(
		TaskBlink
		, (const portCHAR *)"SIG"
		, 150
		, NULL
		, 3
		, &taskBlink);
	xTaskCreate(
		TaskMoveWiper
		, (const portCHAR *)"WIPE"
		, 150
		, NULL
		, 3
		, &taskMoveWiper);
	debugSerialPort.print("Free memory in bytes: ");
	debugSerialPort.println(freeMemory());
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
void TaskToggle(void* pvParameters)
{
	CANFrame f;
	pinMode(HORN_OUTPUT, OUTPUT);
	pinMode(HEADLIGHTS_OUTPUT, OUTPUT);
	bool brakeOff = true;
	while (1)
	{
		// poll to disable brake if brake is on
		if (dataAcc.getBrake() == STATE_EN)
		{
			if (digitalRead(PEDALBRAKE_INTERRUPT))
			{
				dataAcc.setBrake(STATE_DS);
				dataAcc.packCAN(&f);
				xQueueSend(queueForCAN, &f, 100);
			}
		}

		if (dataCommands.getHorn())
		{
			debugSerialPort.println("BEEEP!");
			digitalWrite(HORN_OUTPUT, LOW);
			vTaskDelay(pdMS_TO_TICKS(500));
			digitalWrite(HORN_OUTPUT, HIGH);

			debugSerialPort.println("beep off.");
		}
		else
		{
			digitalWrite(HORN_OUTPUT, HIGH);
		}
		if (dataCommands.getLapTrig())
		{

		}
		else
		{

		}
		dataCommands.clearActivationHistory();

		if (dataAcc.getHeadlights() == STATE_EN)
		{
			digitalWrite(HEADLIGHTS_OUTPUT, HIGH);
		}
		else if (dataAcc.getHeadlights() == STATE_DS)
		{
			digitalWrite(HEADLIGHTS_OUTPUT, LOW);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
void TaskBlink(void* pvParameters)
{
	bool sigOn = false;

	lstrip.begin();
	rstrip.begin();
	lstrip.setBrightness(255);
	rstrip.setBrightness(255);
	pinMode(LED_BUILTIN, OUTPUT);
	while (1)
	{
		if (sigOn)
		{
			sigOn = false;
			
			setRGB(lstrip, PIXELS, NO_COLOR);
			setRGB(rstrip, PIXELS, NO_COLOR);
			digitalWrite(LED_BUILTIN, LOW);
			debugSerialPort.println("SIG OFF");
		}
		else
		{
			if (dataAcc.getHazard() == STATE_EN || dataAcc.getLsig() == STATE_EN)
			{
				sigOn = true;
				setRGB(lstrip, PIXELS, SIG_COLOR);
				digitalWrite(LED_BUILTIN, HIGH);
				debugSerialPort.println("LSIG ON");
			}
			if (dataAcc.getHazard() == STATE_EN || dataAcc.getRsig() == STATE_EN)
			{
				sigOn = true;
				setRGB(rstrip, PIXELS, SIG_COLOR);
				digitalWrite(LED_BUILTIN, HIGH);
				debugSerialPort.println("RSIG ON");
			}
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
void TaskMoveWiper(void* pvParameters)
{
	Servo wiper;
	int wiperPos = 0;

	uint8_t prevWiperOn = STATE_DS;
	while (1)
	{
		if (wiperPos == 0)
		{
			if (dataAcc.getWiper() == STATE_EN)
			{
				if(!wiper.attached())
					wiper.attach(WIPER_PWM_SERVO, 900, 2000);
				wiperPos = 180;
				wiper.write(wiperPos);
			}
			else if (dataAcc.getWiper() == STATE_DS)
			{
				if(wiper.attached())
					wiper.detach();
			}
		}
		else
		{
			wiperPos = 0;
			wiper.write(wiperPos);
		}
		debugSerialPort.print("WIPER ");
		debugSerialPort.println(wiperPos);
		vTaskDelay(pdMS_TO_TICKS(800));
	}
}
void TaskCAN(void *pvParameters) {

	CANFrame f;
	while (1)
	{
		// anything to send
		BaseType_t recvQueue = xQueueReceive(queueForCAN, &f, 0);
		if (recvQueue == pdTRUE)
		{
			serializer.sendCanFrame(&f);
			char str[100];
			dataAcc.packString(str);
			debugSerialPort.print(F("<S> "));
			debugSerialPort.println(str);
		}
		// anything to recv
		if (serializer.receiveCanFrame(&f))
		{
			char str[100];
			if (dataAcc.checkMatchCAN(&f))
			{
				dataAcc.unpackCAN(&f);
				debugSerialPort.print("<R> ");
				dataAcc.packString(str);
				debugSerialPort.println(str);
				xTaskAbortDelay(taskBlink);
				xTaskAbortDelay(taskMoveWiper);
			}
			else if (dataCommands.checkMatchCAN(&f))
			{
				dataCommands.unpackCAN(&f);
				debugSerialPort.print("<R> ");
				dataCommands.packString(str);
				debugSerialPort.println(str);
			}
		}
		else
		{
			if(!serializer.checkNoError())
				debugSerialPort.println("CAN error");
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
void setRGB(Adafruit_NeoPixel& strip, uint8_t numLights, uint32_t color)
{
	for (int i = 0; i < numLights; i++)
		strip.setPixelColor(i, color);
	strip.show();
}
void BRAKE_ISR()
{
	CANFrame f;
	dataAcc.setBrake(STATE_EN);
	dataAcc.packCAN(&f);
	xQueueSendFromISR(queueForCAN, &f, pdFALSE);
}