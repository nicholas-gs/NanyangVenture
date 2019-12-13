#include "TeensyCANHandler.h"
#include <FlexCAN.h>

void TeensyCANHandler::printFrame(CAN_message_t &frame, int mailbox)
{

    if (f->id == 0x0A)
    {
        Serial.print("Ultrasonic data is being transmitted")
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

    return true;
}