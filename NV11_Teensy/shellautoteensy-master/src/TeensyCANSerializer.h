#pragma once

#ifndef TEENSYCANSERIALIZER_h
#define TEENSYCANSERIALIZER_h

#include <FlexCAN.h>
#include <NV11DataUltrasonic.h>
#include <NV11DataSpeedo.h>

/*
	Enum of supported baud rates
*/
enum CAN_BUS_SPEED
{
    CAN_BUS_50KBPS = 50000,
    CAN_BUS_100KBPS = 100000,
    CAN_BUS_125KBPS = 125000,
    CAN_BUS_250KBPS = 250000,
    CAN_BUS_500KBPS = 500000,
    CAN_BUS_1000KBPS = 1000000
};

/*
    Object Oriented Callback Interface. This class registers itself so that it receives any incoming frames without
    having to do polling.
*/
class ReceiverClass : public CANListener
{
private:
    NV11DataUltrasonic *ultrasonicData;
    NV11DataSpeedo *speedoData;
    void (*canISR)() = NULL; // Function pointer to function take gets called in frameHandler(...)

public:
    void init(NV11DataUltrasonic *usData, NV11DataSpeedo *spData);
    void attachISR(void (*CAN_ISR)());
    bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);
};

class TeensyCANSerializer
{
private:
    ReceiverClass receiverObj;

public:
    bool init(NV11DataUltrasonic *ultrasonicData, NV11DataSpeedo *speedoData, uint32_t baudRate = CAN_BUS_1000KBPS);
    void attachISR(void (*CAN_ISR)());
    bool sendData(CAN_message_t *CANMessage);
};

#endif // !TEENSYCANSERIALIZER_h