/*
 * RingBuffer.h
 *
 * Created: 2/11/2016 11:44:21 PM
 *  Author: dcorey
 */ 


#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <stdint.h>

#define min(x,y) ({ \
    __typeof__ (x) _x = (x);     \
    __typeof__ (y) _y = (y);     \
    _x < _y ? _x : _y; })


// These are circular buffers.
//If head == tail, the buffer is empty
//If head == tail + 1 (or (tail = array_length - 1 and head == 0)), there is one character in the buffer, located at tail

//--------For unsigned 8 bit integers--------
typedef struct RingBufferu8
{
	uint16_t array_length; //Length of the array backing the buffer, not length of data stored in the buffer
	uint16_t head; // address of the newest item in the array
	uint16_t tail; // address of the oldest item added
	uint8_t* buffer;
} RingBufferu8_t;

void rbu8_print(RingBufferu8_t* buffer, const char* data);

void rbu8_write(RingBufferu8_t* buffer, const uint8_t* data, uint16_t length);
//Example usage:
//telemetry = "3731,9832,9283923,..."
//write_to_ring_buffer(xbee_send_buffer, telemetry, telemetry_length);

uint8_t rbu8_read(RingBufferu8_t* buffer, uint8_t* dest, uint16_t length);

uint16_t rbu8_length(RingBufferu8_t* buffer);

void rbu8_delete_oldest(RingBufferu8_t* buffer, uint16_t length);

void rbu8_init(RingBufferu8_t* buffer, uint8_t* backing_array, uint16_t backing_array_length);

uint8_t rbu8_get_nth(RingBufferu8_t* buffer, uint16_t index);


//-------For 16 bit signed integers------------
typedef struct RingBuffer16
{
	uint16_t array_length; //Length of the array backing the buffer, not length of data stored in the buffer
	uint16_t head; // address of the newest item in the array
	uint16_t tail; // address of the oldest item added
	int16_t* buffer;
} RingBuffer16_t;

void rb16_write(RingBuffer16_t* buffer, const int16_t* data, uint16_t length);

uint8_t rb16_read(RingBuffer16_t* buffer, int16_t* dest, uint16_t length);

uint16_t rb16_length(RingBuffer16_t* buffer);

void rb16_delete_oldest(RingBuffer16_t* buffer, uint16_t length);

void rb16_init(RingBuffer16_t* buffer, int16_t* backing_array, uint16_t backing_array_length);

int16_t rb16_get_nth(RingBuffer16_t* buffer, uint16_t index);

//-------For 32 bit signed integers------------
typedef struct RingBuffer32
{
	uint16_t array_length; //Length of the array backing the buffer, not length of data stored in the buffer
	uint16_t head; // address of the newest item in the array
	uint16_t tail; // address of the oldest item added
	int32_t* buffer;
} RingBuffer32_t;

void rb32_write(RingBuffer32_t* buffer, const int32_t* data, uint16_t length);

uint8_t rb32_read(RingBuffer32_t* buffer, int32_t* dest, uint16_t length);

uint16_t rb32_length(RingBuffer32_t* buffer);

void rb32_delete_oldest(RingBuffer32_t* buffer, uint16_t length);

void rb32_init(RingBuffer32_t* buffer, int32_t* backing_array, uint16_t backing_array_length);

int32_t rb32_get_nth(RingBuffer32_t* buffer, uint16_t index);

//-------For testing/debugging-----------
#ifdef DEBUG
void test_ring_bufferu8(void);
void test_ring_buffer32(void);
#endif

#endif /* RINGBUFFER_H_ */