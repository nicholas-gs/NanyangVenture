# NanyangVenture

Code for NV10 and NV11

Original code courtesy of [@wilsonPhiggsbury](https://github.com/wilsonPhiggsbury)

## DataPoint.h

Serves as the parent class for a series of subclasses which defines how data should be packed as CAN messages & Strings.

1. CAN packing/unpacking functions do not need to be re-implemented by user. CAN messages byteorder format are instead implemented in child classes using pointers to the only union DataPoint member. Refer to DataPointTemplate.h for implementation example.
   Functions defined for CAN: packCAN, unpackCAN, checkMatchCAN
2. Serial packing/unpacking functions need to be custom implemented in child classes.
   Functions to be implemented: packString, unpackString, checkMatchString
3. Broadcast data if dataHasChanged() true

## NV10_CANtoSerial_UNO

This script listens for serial communication and convert to CAN frames, or listen for CAN frames and convert to serial.
Good for acting as a debugging CAN node with Serial monitor on PC, or for acting as CAN translator for RPi / Jetson.

1. CANSerializer - CAN Utility functions using MCP_CAN library(init, receiveFrame, sendFrame, checkError)
2. NV10AccessoriesStatus - Get and set for Lsig, Rsig, Wiper, Hazard, Headlights, Brake lights and pack and unpack string.
3. NV10CurrentSensor - Get voltage, capacitor current IN, capacitor current OUT and motor current
4. NV10CurrentSensorStats - Get time elapsed, power usage, and peak current
5. NV10FuelCell - Get fuel cell pressure, voltage, temperature, and status
6. NV11DataSpeedo - Get speed from speedometer
7. NV11Commands - trigger horn, lap timer reset, pi shutdown(NV11)

## NV10_front_MEGA

Main job is to react to Dashboard's commands and trigger car accessories accordingly.

1. FreeRTOS for multitasking operations
2. NeoPixel to control lights
3. NV10AccessoriesStatus
4. NV11Commands
5. CANSerializer
6. Servo

TaskToggle -> Poll for brake light status, poll for horn press, poll for headlight status  
TaskBlink -> Toggle left and right signal lights for indicator and headlight  
TaskMoveWiper -> Change wiper position based on polled value  
TaskCAN -> If CAN in queue, send CAN. If CAN received, convert to Serial and check if dataAcc(put taskBlink and taskMoveWiper in ready state) or dataCommand.  
If Brake interrupt, immediately set brake light to on

`Lap trig not implemented`

## NV10_ElectroClutchSync

Reads wheel speed and motor speed separately. Acts as middleman between switch and electro-clutch, main purpose to only engage electro-clutch when speed is similar.

1. Wire - I2C communication
2. Speedometer - Get speed and total distance
3. RelayModule - Controlling the relay
4. avr/wdt - WatchDog timer

## NV10_Back_MEGA

The main script for data gathering in NV10.
Gets input from fuel cell, current sensor, and speedometer.  
Outputs data to SD card to serve as "black box", and XBee for ground station live data feed.

1. MemoryFree, FreeRTOS, NeoPixel
2. CANSerializer, NV10FuelCell, NV10CurrentSensor, NV10CurrentSensorStats, NV10AccessoriesStatus, NV11DataSpeedo, Speedometer
3. SdFat - Read/Write SD Card
4. ADS1115 - Analog to Digital converter(Imported as ADS1015)

TaskLogFuelCell -> Refresh class variables for fuel cell Volts, Amps, Watts and Energy  
TaskLogCurrentSensor -> Refreshes class variables for motor Volts and Amps  
TaskSpeedo -> Converts magnet trip period into speed in km/h

The 3 tasks above push data into a queue for below 2 tasks to output through Serial, SD card, CAN bus

TaskLogSendData -> Data logged in SD card and sent through XBee. Logged and sent payload should be consistent, hence they are grouped together  
TaskCAN -> Manage 2-way CAN communication
TaskBlink -> Triggers and times signal lights flashing

All tasks wake up in fixed intervals and post their own data into the same queue.

```
Data Sent:
1. dataCSStats: Motor voltage(max), motor current every 8 loops.(Watthr is also supposed to be sent).
2. dataCS: Motor Voltage and current, Capacitor current in and out.
3. dataSpeedo: speed in km/h
4. dataFC: Fuel Cell volts, amps, pressure, temperature, status
```

## NV10_dashboard_DUE

1. NV10FuelCell, NV10CurrentSensor, NV10CurrentSensorStats, NV10AccessoriesStatus, NV11DataSpeedo, NV11Commands, FREERTOS_ARM
2. DashboardScreens - Show speed, motorAmp, motorVolt, lsigArrow, rsigArrow. If data doesn't arrive, make widget empty(dashboardNextFrame). If it arrives, update value and toggle lsig and rsig.
3. ArrowWidget - draw, wipe arrows
4. BarWidget - draw, set value using DataWidget
5. TextWidget - settings for and value of text

Listen for buttons and utilizes dataAcc and dataCommands to inform the rest of the system whenever a button is pressed. Refresh the dashboard according to FC, Speedo, CS data from telemetry (NV10_back_MEGA).

Custom widget library - Uses AdaFruitGFX library to draw pixels on the screen.

## NV11_CANtoSerial_UNO

Similar to NV10_CANtoSerial_UNO except with different datapoints

1. CANSerializer, NV11DataSpeedo, NV11Commands
2. NV11AccessoriesStatus - Get and set for Lsig, Rsig, hazard, headlights, brake light, wiper, 4-wheel steering, regenerative breaking and pack and unpack string.
3. NV11BMS - From BMS, get voltage, current, temperature, pack and unpack CAN, pack and unpack string.

Incoming data -> Speedo, BMS, Accessories, Commands  
Outgoing data -> Accessories  
If CAN received, convert to string
If string received, convert to CAN

## NV11_speedo_UNO

1. CANSerializer, NV11DataSpeedo, Speedometer, ros
2. ILI9488 - SPI communication to the screen

Get speed from speedo every 4 loops and transmit to CAN.

## NV11_steeringwheel_UNO

Captures buttons from steering wheel and sends them as CAN messages.

1. CANSerializer, NV11AccessoriesStatus, NV11Commands

Shutdown RPi, lap counter, broadcast CAN, set lsig, rsig, hazard, headlight, brake light, regen breaking, 4-wheel steering, wiper and status LED(when sending CAN)

(prevRead is to ensure button is pressed and then released before triggering action.)

## NV11_wiper_UNO

Controls wiper. To test wdt if it works.

1. CANSerializer, NV11AccessoriesStatus, avr\wdt

Set wiper to slow, fast, or off.

## NV11_accessories_MEGA

Controls brake & signal lights.

1. ros, SdFat, CANSerializer, NV11AccessoriesStatus, RelayModule
2. BlockDriver - access to disk drive

Handle signal lights, hazard lights, brake lights and headlights.

## NV10_GroundStation

GUI to visualize the data sent via the XBee. Also saves a local copy of the logged data.

Utilises [LiveCharts](https://github.com/Live-Charts/Live-Charts).

# Hardware

1. For CAN logging emulator, set frequency to half since its oscillator is double. It also has a terminator built.
2. For CAN bus, standard 120Ohm on each endpoint of the CAN bus.
3. In NV11, front CAN hub has terminator built in. Even BMS has terminator built in. Back CAN hub needs to be terminated conditionally(based on if logger or BMS added).
