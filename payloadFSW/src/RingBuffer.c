/*
 * RingBuffer.c
 *
 * Created: 2/11/2016 11:44:11 PM
 *  Author: dcorey
 */ 
#include "RingBuffer.h"


void rbu8_print(RingBufferu8_t* buffer, const char* data)
{
	uint8_t i;
	for (i = 0; i < 255; i++)
	{
		if (data[i] == '\0')
			break;
	}
	rbu8_write(buffer, (const uint8_t*) data, i);
}
void rbu8_write(RingBufferu8_t* buffer, const uint8_t* data, uint16_t length)
/* Adds length bytes, taken from the data argument, to the end of buffer */
// Author: William Hankins
{
	for (uint8_t i = 0; i < length; i++)
	{
		buffer->buffer[buffer->head] = data[i]; // data can be not be access greater length-1
		buffer->head++;
		
		if (buffer->head == buffer->array_length) //Next value to write to is beyond the end of the array
		{
			buffer->head = 0;
		} // end if
		
		if (buffer->head == buffer->tail)
		{
			buffer->tail++;
			
			if (buffer->tail == buffer->array_length){
				buffer->tail = 0;
			}// end if
		} // end if
	} // end for
} // end write_to_ring_buffer

void rbu8_init(RingBufferu8_t* buffer, uint8_t* backing_array, uint16_t backing_array_length)
/*	Call to reset the head and tail variables of a RingBuffer. 
	backing_array is the array that the ring buffer actually stores its data in
	backing_array_length is kind of exactly what it says it is
*/
{
	buffer->head = 0; //Beginning of array
	buffer->tail = 0; //End of array
	buffer->buffer = backing_array;
	buffer->array_length = backing_array_length;
}

uint8_t rbu8_read(RingBufferu8_t* buffer, uint8_t* dest, uint16_t length)
/*	Reads length bytes of data from the bottom of buffer.
	dest - resulting data will be stored there. Must be at least length bytes long
	length - number of bytes to be read
	Return values
	* 0 - success
	* 1 - the buffer doesn't have length bytes of data in it, but dest now has everything that was in there, null-terminated
*/
{
	uint16_t index = buffer->tail;
	uint16_t i = 0;
	for (i = 0; i < min(rbu8_length(buffer), length); i++)
	{

		dest[i] = buffer->buffer[index];
		if (index == buffer->array_length - 1) //Reset if we've hit the end
		{
			index = 0;
		}
		else
		{
			index++;
		}
		
	}
	if (length > rbu8_length(buffer))
		return 1;
	else
		return 0;
}

void rbu8_delete_oldest(RingBufferu8_t* buffer, uint16_t length)
/*	Deletes data from the ring buffer
	All it really has to do is move buffer->tail up length bytes or until one byte below buffer->head, whichever is lower
*/
{
	uint16_t move_distance = min(length, rbu8_length(buffer));
	if (move_distance >= rbu8_length(buffer))
	{
		buffer->tail = (buffer->tail + move_distance) % buffer->array_length; //Modulus is so that we don't point to above the buffer's location
		buffer->head = buffer->tail;
	}
	else
	{
		buffer->tail = (buffer->tail + move_distance) % buffer->array_length; //Modulus is so that we don't point to above the buffer's location
	}
}

uint16_t rbu8_length(RingBufferu8_t* buffer)
/*	Returns the number of bytes of data currently stored in the RingBuffer pointed to by its argument
	The return value will always be greater than or equal to 0
	And less than or equal to buffer->array_length
*/
{
	if (buffer->head >= buffer->tail)
	//We are not currently wrapped around the end of the buffer, so we are using the space between the head and tail
	{
		return buffer->head - buffer->tail;
	}
	else
	//We have wrapped around the end of the buffer, so subtract the unused space between the head and tail
	{
		return buffer->array_length - (buffer->tail - buffer->head);
	}
}

uint8_t rbu8_get_nth(RingBufferu8_t* buffer, uint16_t index)
//Returns the nth newest value in the RingBuffer
{
	//index = 0: gets the newest value
	//index = 1: gets the 2nd-newest value
	//etc
	//Does not do bounds checking, so be careful in usage
	//	Verify index < rbu8_length to guard against requesting a value beyond what the buffer currently stores
	//	Verify index < buffer->array_length to guard against requesting a value from some random memory location
	
	//Most recent value is in head - 1
	if (index + 1 <= buffer->head) //Value is between zero and buffer->head
		return buffer->buffer[buffer->head - index - 1];
	else
		return buffer->buffer[buffer->array_length - 1 - index + buffer->head];
}

//--------------16 bit signed functions--------------------
void rb16_write(RingBuffer16_t* buffer, const int16_t* data, uint16_t length)
/* Adds length bytes, taken from the data argument, to the end of buffer */
// Author: William Hankins
{
	for (uint8_t i = 0; i < length; i++)
	{
		buffer->buffer[buffer->head] = data[i]; // data can be not be access greater length-1
		buffer->head++;
		
		if (buffer->head == buffer->array_length) //Next value to write to is beyond the end of the array
		{
			buffer->head = 0;
		} // end if
		
		if (buffer->head == buffer->tail)
		{
			buffer->tail++;
			
			if (buffer->tail == buffer->array_length){
				buffer->tail = 0;
			}// end if
		} // end if
	} // end for
} // end write_to_ring_buffer

void rb16_init(RingBuffer16_t* buffer, int16_t* backing_array, uint16_t backing_array_length)
/*	Call to reset the head and tail variables of a RingBuffer. 
	backing_array is the array that the ring buffer actually stores its data in
	backing_array_length is kind of exactly what it says it is
*/
{
	buffer->head = 0; //Beginning of array
	buffer->tail = 0; //End of array
	buffer->buffer = backing_array;
	buffer->array_length = backing_array_length;
}

uint8_t rb16_read(RingBuffer16_t* buffer, int16_t* dest, uint16_t length)
/*	Reads length bytes of data from the bottom of buffer.
	dest - resulting data will be stored there. Must be at least length bytes long
	length - number of bytes to be read
	Return values
	* 0 - success
	* 1 - the buffer doesn't have length bytes of data in it, but dest now has everything that was in there, null-terminated
*/
{
	uint16_t index = buffer->tail;
	uint16_t i = 0;
	for (i = 0; i < min(rb16_length(buffer), length); i++)
	{

		dest[i] = buffer->buffer[index];
		if (index == buffer->array_length - 1) //Reset if we've hit the end
		{
			index = 0;
		}
		else
		{
			index++;
		}
		
	}
	if (length > rb16_length(buffer))
		return 1;
	else
		return 0;
}

void rb16_delete_oldest(RingBuffer16_t* buffer, uint16_t length)
/*	Deletes data from the ring buffer
	All it really has to do is move buffer->tail up length bytes or until one byte below buffer->head, whichever is lower
*/
{
	uint16_t move_distance = min(length, rb16_length(buffer));
	if (move_distance >= rb16_length(buffer))
	{
		buffer->tail = (buffer->tail + move_distance) % buffer->array_length; //Modulus is so that we don't point to above the buffer's location
		while (1);
		buffer->head = buffer->tail;
	}
	else
	{
		buffer->tail = (buffer->tail + move_distance) % buffer->array_length; //Modulus is so that we don't point to above the buffer's location
	}
}

uint16_t rb16_length(RingBuffer16_t* buffer)
/*	Returns the number of bytes of data currently stored in the RingBuffer pointed to by its argument
	The return value will always be greater than or equal to 0
	And less than or equal to buffer->array_length
*/
{
	if (buffer->head >= buffer->tail)
	//We are not currently wrapped around the end of the buffer, so we are using the space between the head and tail
	{
		return buffer->head - buffer->tail;
	}
	else
	//We have wrapped around the end of the buffer, so subtract the unused space between the head and tail
	{
		return buffer->array_length - (buffer->tail - buffer->head);
	}
}

int16_t rb16_get_nth(RingBuffer16_t* buffer, uint16_t index)
//Returns the nth newest value in the RingBuffer
{
	//index = 0: gets the newest value
	//index = 1: gets the 2nd-newest value
	//etc
	//Does not do bounds checking, so be careful in usage
	//	Verify index < rb16_length() to guard against requesting a value beyond what the buffer currently stores
	//	Verify index < buffer->array_length to guard against requesting a value from some random memory location
	
	//Most recent value is in head - 1
	if (index + 1 <= buffer->head) //Value is between zero and buffer->head
		return buffer->buffer[buffer->head - index - 1];
	else
		return buffer->buffer[buffer->array_length - 1 - index + buffer->head];
}

//--------------32 bit signed functions--------------------
void rb32_write(RingBuffer32_t* buffer, const int32_t* data, uint16_t length)
/* Adds length bytes, taken from the data argument, to the end of buffer */
// Author: William Hankins
{
	for (uint8_t i = 0; i < length; i++)
	{
		buffer->buffer[buffer->head] = data[i]; // data can be not be access greater length-1
		buffer->head++;
		
		if (buffer->head == buffer->array_length) //Next value to write to is beyond the end of the array
		{
			buffer->head = 0;
		} // end if
		
		if (buffer->head == buffer->tail)
		{
			buffer->tail++;
			
			if (buffer->tail == buffer->array_length){
				buffer->tail = 0;
			}// end if
		} // end if
	} // end for
} // end write_to_ring_buffer

void rb32_init(RingBuffer32_t* buffer, int32_t* backing_array, uint16_t backing_array_length)
/*	Call to reset the head and tail variables of a RingBuffer. 
	backing_array is the array that the ring buffer actually stores its data in
	backing_array_length is kind of exactly what it says it is
*/
{
	buffer->head = 0; //Beginning of array
	buffer->tail = 0; //End of array
	buffer->buffer = backing_array;
	buffer->array_length = backing_array_length;
}

uint8_t rb32_read(RingBuffer32_t* buffer, int32_t* dest, uint16_t length)
/*	Reads length bytes of data from the bottom of buffer.
	dest - resulting data will be stored there. Must be at least length bytes long
	length - number of bytes to be read
	Return values
	* 0 - success
	* 1 - the buffer doesn't have length bytes of data in it, but dest now has everything that was in there, null-terminated
*/
{
	uint16_t index = buffer->tail;
	uint16_t i = 0;
	for (i = 0; i < min(rb32_length(buffer), length); i++)
	{

		dest[i] = buffer->buffer[index];
		if (index == buffer->array_length - 1) //Reset if we've hit the end
		{
			index = 0;
		}
		else
		{
			index++;
		}
		
	}
	if (length > rb32_length(buffer))
		return 1;
	else
		return 0;
}

void rb32_delete_oldest(RingBuffer32_t* buffer, uint16_t length)
/*	Deletes data from the ring buffer
	All it really has to do is move buffer->tail up length bytes or until one byte below buffer->head, whichever is lower
*/
{
	uint16_t move_distance = min(length, rb32_length(buffer));
	if (move_distance >= rb32_length(buffer))
	{
		buffer->tail = (buffer->tail + move_distance) % buffer->array_length; //Modulus is so that we don't point to above the buffer's location
		while (1);
		buffer->head = buffer->tail;
	}
	else
	{
		buffer->tail = (buffer->tail + move_distance) % buffer->array_length; //Modulus is so that we don't point to above the buffer's location
	}
}

uint16_t rb32_length(RingBuffer32_t* buffer)
/*	Returns the number of bytes of data currently stored in the RingBuffer pointed to by its argument
	The return value will always be greater than or equal to 0
	And less than or equal to buffer->array_length
*/
{
	if (buffer->head >= buffer->tail)
	//We are not currently wrapped around the end of the buffer, so we are using the space between the head and tail
	{
		return buffer->head - buffer->tail;
	}
	else
	//We have wrapped around the end of the buffer, so subtract the unused space between the head and tail
	{
		return buffer->array_length - (buffer->tail - buffer->head);
	}
}

int32_t rb32_get_nth(RingBuffer32_t* buffer, uint16_t index)
//Returns the nth newest value in the RingBuffer
{
	//index = 0: gets the newest value
	//index = 1: gets the 2nd-newest value
	//etc
	//Does not do bounds checking, so be careful in usage
	//	Verify index < rb16_length() to guard against requesting a value beyond what the buffer currently stores
	//	Verify index < buffer->array_length to guard against requesting a value from some random memory location
	
	//Most recent value is in head - 1
	if (index + 1 <= buffer->head) //Value is between zero and buffer->head
		return buffer->buffer[buffer->head - index - 1];
	else
		return buffer->buffer[buffer->array_length - 1 - index + buffer->head];
}

//----------------Test functions------------------------

#ifdef DEBUG

void test_ring_bufferu8(void)
{
	uint8_t backing_array[10];
	RingBufferu8_t rb;
	rbu8_init(&rb, backing_array, 10);
	uint8_t i;
	for (i = 0; i < 15; i++)
	{
		rbu8_write(&rb, &i, 1);
	}
	uint8_t read[10];
	rbu8_read(&rb, read, 10);
	
	for (i = 0; i < rbu8_length(&rb); i++)
	{
		printf("now reading %i\n", read[i]);
	}
	while (1);
}

void test_ring_buffer32(void)
{
	int32_t barray[11];
	RingBuffer32_t rb;
	rb32_init(&rb, barray, 11);
	int32_t i[] = {3, 7, 2};
	rb32_write(&rb, &i, 3);
	printf("number: %" PRIi32 "\n", (int32_t) (76));
	printf("length is: %i\n", rb32_length(&rb));
	printf("1st is: %" PRIi32 "\n", rb32_get_nth(&rb, 0));
	printf("head is %i\n", rb.head);
	//printf()
}
#endif