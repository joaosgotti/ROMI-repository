#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "mbed.h"

void init_communication(BufferedSerial *serial_in);
void write_bytes(char *ptr, unsigned char len);
void send_odometry(int value1, int value2, int ticks_left, int ticks_right, float x, float y, float theta);

#endif
