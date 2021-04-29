#ifndef MESSAGEBUILDER_H_
#define MESSAGEBUILDER_H_

#include "mbed.h"

class MessageBuilder
{
private:
    static const char max_len = 32;
    char *_pointer;

public:
    char message[max_len];

    MessageBuilder();
    virtual ~MessageBuilder();
    char add(const void* data, size_t len);
    char add(char data);
    char add(float data);
    char add(int data);
    char add(unsigned int data);
    void reset();
    unsigned char available();
    unsigned char length();
};

#endif /* MESSAGEBUILDER_H_ */


