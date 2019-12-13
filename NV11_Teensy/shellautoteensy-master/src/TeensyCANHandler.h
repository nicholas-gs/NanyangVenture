#include <FlexCAN.h>

class TeensyCANHandler : public CANListener
{
public:
    void printFrame(CAN_message_t &frame, int mailbox);
    bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);
};
