#include "TeensyCANSerializer.h"

/*
	Initialise the CAN0 object in FlexCAN library. Return true if baud rate is supported, false otherwise.
*/
bool TeensyCANSerializer::init(NV11DataUltrasonic *ultrasonicData, NV11DataSpeedo *speedoData, uint32_t baudRate = CAN_BUS_1000KBPS)
{
    switch (baudRate)
    {
    case CAN_BUS_50KBPS:
        Can0.begin(CAN_BUS_50KBPS);
        break;
    case CAN_BUS_100KBPS:
        Can0.begin(CAN_BUS_100KBPS);
        break;
    case CAN_BUS_125KBPS:
        Can0.begin(CAN_BUS_125KBPS);
        break;
    case CAN_BUS_250KBPS:
        Can0.begin(CAN_BUS_250KBPS);
        break;
    case CAN_BUS_500KBPS:
        Can0.begin(CAN_BUS_500KBPS);
        break;
    case CAN_BUS_1000KBPS:
        Can0.begin(CAN_BUS_1000KBPS);
        break;
    default:
        return false;
    }
    receiverObj.init(ultrasonicData, speedoData);
    Can0.attachObj(&receiverObj);
    receiverObj.attachGeneralHandler();
    return true;
}

/*
    Send a CAN_message_t frame through the CAN bus. Returns true if sending was successful, false otherwise
*/
bool TeensyCANSerializer::sendData(CAN_message_t *CANMessage)
{
    bool successful = Can0.write(*CANMessage);
    return successful;
}

/*
    Pass pointers to objects that store the data after unpacking the CAN_message_t CAN frames.
*/
void ReceiverClass::init(NV11DataUltrasonic *usData, NV11DataSpeedo *spData)
{
    ReceiverClass::ultrasonicData = usData;
    ReceiverClass::speedoData = spData;
}

/*
	Function that gets called anytime a new message comes in.
    Unpacks the incoing CAN_message_t into the appropriate object depending on the CAN ID.
*/
bool ReceiverClass::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller)
{
    if (frame.id == ultrasonicData->getCanId())
    {
        ultrasonicData->insertDataTeensy(frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3], frame.buf[4],
                                         frame.buf[5], frame.buf[6]);
    }
    else if (frame.id == speedoData->getCanId())
    {
        float speed = -1;
        memcpy(&speed, frame.buf, 4);
        speedoData->insertData(speed);
    }
}