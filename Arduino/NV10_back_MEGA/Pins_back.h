#pragma once

// Wiring naming convention: DEVICE_PINTYPE_PINNAME

// internal use, no exterior connections should be made to these pins
#define SD_SPI_CS_PIN 44
#define CAN_CS_PIN 48
#define CAN_INT_PIN 19

// rear peripherals
#define SPEEDOMETER_INTERRUPT_PIN 18
#define RUNNINGLIGHT_PIN 45
#define BRAKE_PIN 47
#define LSIG_PIN 46
#define RSIG_PIN 43

// old red box
//#define SD_SPI_CS_PIN 4			// used by shield for SD card
//#define CAN_CS_PIN 48			// used by CAN shield
//#define CAN_INT_PIN 19
//#define RUNNINGLIGHT_PIN 9
//#define BRAKE_PIN 10
//#define LSIG_PIN 11
//#define RSIG_PIN 12
