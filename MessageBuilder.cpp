#include "MessageBuilder.h"
#include "mbed.h"

MessageBuilder::MessageBuilder() {
	reset();
}

MessageBuilder::~MessageBuilder() {
	// TODO Auto-generated destructor stub
}

char MessageBuilder::add(const void* data, size_t len) {
	if (available() >= len) {
		memcpy(_pointer, data, len);
		_pointer += len;
		return 0;
	} else {
		return 1;
	}
}

void MessageBuilder::reset() {
	message[0] = 0x06;
	message[1] = 0x85;
	_pointer = &message[2];
}

// Note: if message size grow beyond 32 bytes, return "size_t" insted, because it
// is the most appropriate type for "sizeof" operator. Now, unsgined char is used
// for memory economy.
unsigned char MessageBuilder::available() {
	return &message[max_len - 1] - _pointer + 1;
}

unsigned char MessageBuilder::length() {
	return _pointer - &message[0];
}

char MessageBuilder::add(float data) {
	if (available() >= sizeof(data)) {
		memcpy(_pointer, &data, sizeof(data));
		_pointer += sizeof(data);
		return 0;
	} else {
		return 1;
	}
}

char MessageBuilder::add(int data) {
	if (available() >= sizeof(data)) {
		memcpy(_pointer, &data, sizeof(data));
		_pointer += sizeof(data);
		return 0;
	} else {
		return 1;
	}
}

char MessageBuilder::add(char data) {
	if (available() >= sizeof(data)) {
		memcpy(_pointer, &data, sizeof(data));
		_pointer += sizeof(data);
		return 0;
	} else {
		return 1;
	}
}

char MessageBuilder::add(unsigned int data) {
	if (available() >= sizeof(data)) {
		memcpy(_pointer, &data, sizeof(data));
		_pointer += sizeof(data);
		return 0;
	} else {
		return 1;
	}
}
