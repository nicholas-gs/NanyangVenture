#include <FlexCAN.h>
#include <NV11DataUltrasonic.h>

#define ULTRASONIC_CAN_FRAME_ID 0x0A

class TeensyCANHandler : public CANListener
{
private:
    bool hasData = false;
    static volatile NV11DataUltrasonic ultrasonicData;
    /*
        Unpack CAN_message_t frame into NV11DataUltrasonic object
     */
    void unpackCANFrame(CAN_message_t &frame);

public:
    bool hasData();
    /*
        Returns a copy of the ultrasonicData static object
    */
    NV11DataUltrasonic getData();
    void printFrame(CAN_message_t &frame, int mailbox);
    bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);
};
