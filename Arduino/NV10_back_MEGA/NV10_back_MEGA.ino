/*
 Name:		NV10_back.ino
 Created:	6/19/2018 1:42:24 PM
 Author:	MX
*/

#include <MemoryFree.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <SdFat.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ADS1015.h>
//#include <FreeRTOS_AVR.h>
#include <CANSerializer.h>
#include <NV10FuelCell.h>
#include <NV10CurrentSensor.h>
#include <NV10CurrentSensorStats.h>
#include <NV10AccesoriesStatus.h>
#include <NV11DataSpeedo.h>

//#include "JoulemeterDisplay.h"
#include "Speedometer.h"

#include "Pins_back.h"

NV10FuelCell dataFC;
NV10CurrentSensor dataCS;
NV10CurrentSensorStats dataCSStats;
NV11DataSpeedo dataSpeedo;
NV10AccesoriesStatus dataAcc;

// struct for log send exists to wrap strings properly with path names
typedef struct  {
private:
public:
	char data[100];
	bool logThis;
	bool sendThis;
	char path[10];
	void setLogSend(bool log, bool send, const char* path);
}StructForLogSend;
QueueHandle_t queueForLogSend = xQueueCreate(1, sizeof(StructForLogSend));
QueueHandle_t queueForCAN = xQueueCreate(1, sizeof(CANFrame));
//QueueHandle_t queueForDisplay = xQueueCreate(1, sizeof(Packet));
TaskHandle_t taskBlink;

// RGB light related variables
const uint8_t PIXELS = 7;
const uint32_t LIGHT_COLOR = Adafruit_NeoPixel::Color(128, 0, 0);
const uint32_t BRAKE_COLOR = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t SIG_COLOR = Adafruit_NeoPixel::Color(255, 165, 0);
const uint32_t NO_COLOR = Adafruit_NeoPixel::Color(0, 0, 0);
Adafruit_NeoPixel lstrip = Adafruit_NeoPixel(7, LSIG_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rstrip = Adafruit_NeoPixel(7, RSIG_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lightstrip = Adafruit_NeoPixel(7, RUNNINGLIGHT_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel brakestrip = Adafruit_NeoPixel(7, BRAKE_PIN, NEO_GRB + NEO_KHZ800);

// define globals
bool SD_avail = false, CAN_avail = false;
SdFat card;
CANSerializer serializer;

// wheel diameter is 545 mm, feed into speedo
Speedometer speedo = Speedometer(SPEEDOMETER_INTERRUPT_PIN, 545, 2);

HardwareSerial& fcSerialPort = Serial3;
HardwareSerial& dataSerialPort = Serial2;
HardwareSerial& debugSerialPort = Serial;

// define tasks, types are: input, control, output
void TaskLogFuelCell(void *pvParameters);		// Input task:		Refreshes class variables for fuel cell Volts, Amps, Watts and Energy
void TaskLogCurrentSensor(void *pvParameters);	// Input task:		Refreshes class variables for motor Volts and Amps
void TaskSpeedo(void *pvParameters);	// Control task:	Controls frequency to queue payload from above tasks to output tasks
void TaskLogSendData(void *pvParameters);		// Output task:		Data logged in SD card and sent through XBee. Logged and sent payload should be consistent, hence they are grouped together
void TaskCAN(void* pvParameters);			// In/Out task:		Manages 2-way CAN bus comms
void TaskBlink(void *pvParameters);			// 

//// _______________OPTIONAL_____________
//void TaskReceiveCommands(void *pvParameters);	// Input task:		Enable real-time control of Arduino (if any)
//void TaskRefreshPeripherals(void *pvParameters);// Control task:	Updates all CAN-BUS peripherals

/// <summary>
/// Collects data from 3 different sources: Fuel Cell, Current Sensor, SpeedoMeter.
/// Outputs data to 2 different places: SD card & XBee Serial
/// Manages running lights, brake lights, signal lights.
/// </summary>
void setup() {
	debugSerialPort.begin(9600);
	while (!debugSerialPort);

	// create all files in a new directory
	SD_avail = initSD(card);
	debugSerialPort.print("SD avail: ");
	debugSerialPort.println(SD_avail);
	CAN_avail = serializer.init(CAN_CS_PIN);
	debugSerialPort.print("CAN avail: ");
	debugSerialPort.println(CAN_avail);

	// Now set up all Tasks to run independently. Task functions are found in Tasks.ino
	xTaskCreate(
		TaskLogFuelCell
		, (const portCHAR *)"Fuel"
		, 450
		, NULL
		, 2
		, NULL);
	xTaskCreate(
		TaskLogCurrentSensor
		, (const portCHAR *)"CSensor"
		, 550
		, NULL
		, 3
		, NULL);
	xTaskCreate(
		TaskSpeedo
		, (const portCHAR *)"Enqueue"  // A name just for humans
		, 300  // This stack size can be checked & adjusted by reading the Stack Highwater
		, NULL // Any pointer to pass in
		, 2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
		, NULL);
	xTaskCreate(
		TaskLogSendData
		, (const portCHAR *) "LogSend"
		, 1000  // Stack size
		, NULL
		, 1  // Priority
		, NULL);
	xTaskCreate(
		TaskBlink
		, (const portCHAR *)"SIG"
		, 150 // -25
		, NULL
		, 3
		, &taskBlink);
	xTaskCreate(
		TaskCAN
		, (const portCHAR *) "CAN la!" // where got cannot?
		, 600  // Stack size
		, NULL
		, 2  // Priority
		, NULL);
	//vTaskStartScheduler();
	debugSerialPort.print(F("Free Memory in Bytes: "));
	debugSerialPort.println(freeMemory());
}

void loop()
{
	// Empty. Things are done in Tasks.
}
void TaskLogFuelCell(void *pvParameters)
{
	CANFrame f;
	StructForLogSend s; // this is only for logging FC raw data
	s.setLogSend(true, false, "FCraw.txt");

	fcSerialPort.begin(19200);
	fcSerialPort.setTimeout(50);

	while (1)
	{
		uint8_t bytesRead = fcSerialPort.readBytesUntil('\n', s.data, 100);
		if (bytesRead > 0)
		{
			s.data[bytesRead - 1] = '\0'; // manually null-terminate. Rewrite '\r' into '\0'
			// 1. Log the raw data into the SD card and send via XBee
			s.setLogSend(true, true, "FCraw.txt");
			xQueueSend(queueForLogSend, &s, 100);
			// 2. CAN the processed fuel cell data
			dataFC.insertData(s.data); // THIS will destroy s.data! Don't send s anymore after this line
			dataFC.packCAN(&f);
			xQueueSend(queueForCAN, &f, 100); // subsequently, do not log the processed data, but still send CAN to the dashboard
			/*
			s.data[bytesRead - 1] = '\0'; // manually null-terminate. Rewrite '\r' into '\0'
			// 1. Log the raw data into the SD card
			s.setLogSend(true, false, "FCraw.txt");
			xQueueSend(queueForLogSend, &s, 100);
			// 2. CAN the processed fuel cell data
			dataFC.insertData(s.data); // THIS will destroy s.data! Don't send s anymore after this line
			dataFC.packCAN(&f);
			xQueueSend(queueForCAN, &f, 100); // subsequently, do not log the processed data, but still send CAN to the dashboard
			// 3. Send the processed data via XBee
			s.setLogSend(false, true, "FC.txt");
			dataFC.packString(s.data);
			xQueueSend(queueForLogSend, &s, 100);
			*/
		}
		vTaskDelay(pdMS_TO_TICKS(450));
	}
}
void TaskLogCurrentSensor(void *pvParameters)
{
#define SAMPLES 5
	float getMedian(float floatArray[], int n = SAMPLES);
	CANFrame f;
	StructForLogSend s;
	s.setLogSend(true, true, "CS.txt");
	/* 
	ADS1115 full scale range (FSR) table
	GAIN	FSR(mV)	LSB size(microV)
	0.5		6144	187.5
	1		4096	125
	2		2048	62.5
	4		1024	31.25  <-- we are using setGain(GAIN_FOUR) when measuring voltage, 
	8		512		15.625
	16		256		7.8125 <-- we are using setGain(GAIN_SIXTEEN) when measuring current, for highest resolution. 
	Keeping FSR small increases resolution, but it must be larger than shunt mV or the ADC may be damaged.

	## SHUNTS ##
	75mV 200A shunt: 200A maps to - 75mV / 0.0078125mV per raw unit = 9600 raw unit
	75mV 200A shunt: 1A maps to - 9600 raw unit / 200 amp = 48 raw units per amp
	Hence, amp = raw * ADC scale
	where ADC scale = 0.0078125 * shuntA / shuntmV
	## VOLTAGE DIV ##
	700k and 10k resistors. Using GAIN_FOUR with FSR of 1024 mV and LSB size of 31.25 microV
	SCap volt = divided volt * (total ohm / this resistor ohm)
	divided volt = raw * LSB size
	Merge above 2 eq:
	Scap volt = raw * LSB size * (total ohm / this resistor ohm)
	In this case,
	SCap volt = raw * 0.03125 * 72
	*/
	const float SHUNT_100A75mV_SCALE = 0.0078125 * (100.0 / 75);
	const float VOLTAGEDIVIDER_SCALE = 0.03125 * (720000 / 10000.0) / 1000; // scale 60V to 1.024V maybe?
	/*
	ADS1115 i2c address
	Addr -> VDD = 0x49
	Addr -> GND = 0x48 (default if not connected)
	Addr -> SCL = 0x4B
	Addr -> SDA = 0x4A
	*/
	Adafruit_ADS1115 currentADC = Adafruit_ADS1115(0x49);
	Adafruit_ADS1115 voltageADC = Adafruit_ADS1115(0x48);
	currentADC.setGain(GAIN_SIXTEEN);
	voltageADC.setGain(GAIN_FOUR); // GAIN_FOUR can take 1024mV max. Consult FSR column from the FSR table above
	currentADC.begin();
	voltageADC.begin();

	float capCurrent, motorCurrent, motorVoltage;
	float capCurrents[SAMPLES], motorCurrents[SAMPLES], motorVoltages[SAMPLES];

	dataCSStats.syncTime(); // reset the internal millis count for better 1st reading
	uint8_t syncCounter = 0; // incr on every loop, for less frequent logsend messages
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (1)
	{
		for (int i = 0; i < SAMPLES; i++)
		{
			capCurrents[i] = currentADC.readADC_Differential_2_3() * SHUNT_100A75mV_SCALE;
			motorCurrents[i] = currentADC.readADC_Differential_0_1() * SHUNT_100A75mV_SCALE;
			motorVoltages[i] = voltageADC.readADC_Differential_0_1() * VOLTAGEDIVIDER_SCALE;
			vTaskDelay(pdMS_TO_TICKS(5));
		}
		capCurrent = getMedian(capCurrents);
		float capInCurrent = capCurrent > 0 ? capCurrent : 0;
		float capOutCurrent = capCurrent > 0 ? 0 : -capCurrent;
		motorCurrent = getMedian(motorCurrents); //TODO: Check if median needs to be taken. Current is okay, check voltage measurement.
		motorVoltage = getMedian(motorVoltages);
		dataCS.insertData(motorVoltage, capInCurrent, capOutCurrent, motorCurrent);
		dataCSStats.insertData(motorVoltage, motorCurrent);
		//dataCS.insertData(55, 0, 0, motorCurrent);
		//dataCSStats.insertData(55, motorCurrent);

		s.setLogSend(true, true, "CS.txt");
		dataCS.packString(s.data);
		xQueueSend(queueForLogSend, &s, 100);

		dataCS.packCAN(&f);
		xQueueSend(queueForCAN, &f, 100);

		if (syncCounter % 8 == 0)
		{
			s.setLogSend(true, true, "CSst.txt");
			dataCSStats.packString(s.data);
			xQueueSend(queueForLogSend, &s, 100);

			dataCSStats.packCAN(&f);
			xQueueSend(queueForCAN, &f, 100);
		}

		syncCounter++;

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(400));
	}
}
void TaskSpeedo(void *pvParameters)
{
	CANFrame f;
	StructForLogSend s;

	pinMode(SPEEDOMETER_INTERRUPT_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(SPEEDOMETER_INTERRUPT_PIN), [] {
		speedo.trip();
	}, FALLING);
	while (1)
	{
		float speedKmh = speedo.getSpeedKmh();
		dataSpeedo.insertData(speedKmh);

		s.setLogSend(true, true, "SM.txt");
		dataSpeedo.packString(s.data);
		xQueueSend(queueForLogSend, &s, 100);

		dataSpeedo.packCAN(&f);
		xQueueSend(queueForCAN, &f, 100);

		vTaskDelay(pdMS_TO_TICKS(600));
	}

}
void TaskBlink(void* pvParameters)
{
	bool sigOn = false;
	// initialize light strips
	lightstrip.begin();
	brakestrip.begin();
	lightstrip.setBrightness(50);
	brakestrip.setBrightness(200);
	setRGB(lightstrip, PIXELS, LIGHT_COLOR);
	setRGB(brakestrip, PIXELS, BRAKE_COLOR);

	lstrip.begin();
	rstrip.begin();
	lstrip.setBrightness(50);
	rstrip.setBrightness(50);
	setRGB(lstrip, PIXELS, NO_COLOR);
	setRGB(rstrip, PIXELS, NO_COLOR);
	while (1)
	{
		if (sigOn)
		{
			sigOn = false;
			setRGB(lstrip, PIXELS, NO_COLOR);
			setRGB(rstrip, PIXELS, NO_COLOR);
		}
		else
		{
			if (dataAcc.getHazard() == STATE_EN || dataAcc.getLsig() == STATE_EN)
			{
				sigOn = true;
				setRGB(lstrip, PIXELS, SIG_COLOR);
				debugSerialPort.println(F("lSig ON"));
			}
			if (dataAcc.getHazard() == STATE_EN || dataAcc.getRsig() == STATE_EN)
			{
				sigOn = true;
				setRGB(rstrip, PIXELS, SIG_COLOR);
				debugSerialPort.println(F("rSig ON"));
			}
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
void TaskLogSendData(void *pvParameters)
{
	dataSerialPort.begin(9600);
	StructForLogSend s;

	BaseType_t stringToLog;
	while (1)
	{
		stringToLog = xQueueReceive(queueForLogSend, &s, 0);
		if (stringToLog == pdTRUE)
		{
			if (s.logThis && SD_avail)
			{
				// DO NOT SWITCH OUT THIS TASK IN THE MIDST OF WRITING A FILE ON SD CARD
				vTaskSuspendAll();
				File writtenFile = card.open(s.path, FILE_WRITE);
				writtenFile.println(s.data);
				writtenFile.close();
				xTaskResumeAll();
			}
			if (s.sendThis)
			{
				dataSerialPort.println(s.data);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
void TaskCAN(void *pvParameters)
{
	CANFrame f;
	char str[100];

	BaseType_t canMsgToSend;
	bool prevSigOn = false, thisSigOn = false; // sigOn should be true if any of sig is on (left / right / hazard)
	while (1)
	{
		canMsgToSend = xQueueReceive(queueForCAN, &f, 0);
		if (CAN_avail)
		{
			if (canMsgToSend == pdTRUE)
			{
				serializer.sendCanFrame(&f);
			}
			if (serializer.receiveCanFrame(&f))
			{
				if (dataAcc.checkMatchCAN(&f))
				{
					dataAcc.unpackCAN(&f);
					// react to brake command
					if (dataAcc.getBrake() == STATE_EN)
					{
						debugSerialPort.println(F("BRAKE on"));
						setRGB(brakestrip, PIXELS, BRAKE_COLOR);
						setRGB(lightstrip, PIXELS, BRAKE_COLOR);
					}
					else
					{
						debugSerialPort.println(F("BRAKE off"));
						setRGB(brakestrip, PIXELS, NO_COLOR);
						setRGB(lightstrip, PIXELS, LIGHT_COLOR);
					}
					// react to signal lights command only when sigOn state has changed
					thisSigOn = dataAcc.getLsig() || dataAcc.getRsig() || dataAcc.getHazard();
					if (prevSigOn != thisSigOn)
					{
						prevSigOn = thisSigOn;
						xTaskAbortDelay(taskBlink);
					}
				}
				else
				{
					debugSerialPort.print(F("Unrecognized CAN id: "));
					debugSerialPort.println(f.id);
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}