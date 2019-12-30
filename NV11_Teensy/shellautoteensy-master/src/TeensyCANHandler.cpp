#include "TeensyCANHandler.h"
#include <FlexCAN.h>
#include <NV11DataUltrasonic.h>

void TeensyCANHandler::printFrame(CAN_message_t &frame, int mailbox)
{

    if (frame.id == 0x0A)
    {
        Serial.print("Ultrasonic data is being transmitted");
    }

    Serial.print("ID: ");
    Serial.print(frame.id, HEX);
    Serial.print(" Data: ");
    for (int c = 0; c < frame.len; c++)
    {
        Serial.print(frame.buf[c], HEX);
        Serial.write(' ');
    }
    Serial.write('\r');
    Serial.write('\n');
}

bool TeensyCANHandler::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller)
{
    printFrame(frame, mailbox);
    TeensyCANHandler::unpackCANFrame(frame);

    return true;
}

void TeensyCANHandler::unpackCANFrame(CAN_message_t &frame)
{
    if (frame.id == ULTRASONIC_CAN_FRAME_ID)
    {
        TeensyCANHandler::ultrasonicData.insertDataTeensy(frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3], frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
    }
    TeensyCANHandler::hasData = true;
}

NV11DataUltrasonic TeensyCANHandler::getData()
{
    NV11DataUltrasonic dataCopy;
    dataCopy.insertData(ultrasonicData.getRightFront(), ultrasonicData.getRightSide(), ultrasonicData.getRightBack(),
                        ultrasonicData.getLeftFront(), ultrasonicData.getLeftSide(), ultrasonicData.getLeftBack(), ultrasonicData.getFront());
    TeensyCANHandler::hasData = false;
    return dataCopy;
}