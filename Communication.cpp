#include "Communication.h"
#include "mbed.h"
#include "MessageBuilder.h"

const char max_len = 30;
BufferedSerial *serial_object;
MessageBuilder bin_msg;

void init_communication(BufferedSerial *serial_in)
{
    serial_object = serial_in;
}

void write_bytes(char *ptr, unsigned char len)
{
    for(int i=0; i<len; i++)
    {
      //  serial_object->pc.putc(ptr[i]);
    }
}

void send_odometry(int value1, int value2, int ticks_left, int ticks_right, float x, float y, float theta)
{
    bin_msg.reset();
    bin_msg.add('O');
    bin_msg.add(value1);
    bin_msg.add(value2);
    bin_msg.add(ticks_left);
    bin_msg.add(ticks_right);
    bin_msg.add(x);
    bin_msg.add(y);
    bin_msg.add(theta);

    write_bytes(bin_msg.message, bin_msg.length());
}