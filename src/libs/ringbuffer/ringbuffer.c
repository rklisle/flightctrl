
#include <string.h>
#include "ringbuffer.h"

/**
 * @file
 * Implementation of ring buffer functions.
 */

void ring_buffer_init(ring_buffer_t *buffer, void *buf, size_t buf_size) 
{
  RING_BUFFER_ASSERT(RING_BUFFER_IS_POWER_OF_TWO(buf_size) == 1);
  buffer->buffer = buf;
  buffer->buffer_mask = buf_size - 1;
  buffer->tail_index = 0;
  buffer->head_index = 0;
}

size_t ring_buffer_write(ring_buffer_t *buffer, const void *data, size_t size)
{
    const char *data_ptr = data;
    size_t first_size;
    size_t second_size;
    size_t wt_size = buffer->buffer_mask - ring_buffer_num_items(buffer);
    /* get available to write*/
    wt_size = (wt_size < size) ? wt_size : size;
    /* calculate copy size */
    first_size = buffer->buffer_mask + 1 - buffer->head_index;
    if(first_size > wt_size)
    {
        first_size = wt_size;
        second_size = 0;
    }
    else
    {
        second_size = wt_size - first_size;
    }

    /* Place data in buffer */
    memcpy(&buffer->buffer[buffer->head_index], data_ptr, first_size);
    if(second_size > 0)
    {
        memcpy(buffer->buffer, data_ptr + first_size, second_size);
    }
    /* shift head index*/
    buffer->head_index = (buffer->head_index + wt_size) & RING_BUFFER_MASK(buffer);
    return wt_size;
}

size_t ring_buffer_read(ring_buffer_t *buffer, void *data, size_t len) 
{
    uint8_t *data_ptr = data;
    size_t rd_size;
    size_t first_size;
    size_t second_size;

    if(ring_buffer_is_empty(buffer))
    {
        /* No items */
        return 0;
    }

    /* get available to read*/
    rd_size = ring_buffer_num_items(buffer);
    rd_size = (rd_size < len) ? rd_size : len;

    /* calculate copy size */
    first_size = buffer->buffer_mask + 1 - buffer->tail_index;
    if(first_size > rd_size)
    {
        first_size = rd_size;
        second_size = 0;
    }
    else
    {
        second_size = rd_size - first_size;
    }

    /* copy data from buffer */
    memcpy( data_ptr,&buffer->buffer[buffer->tail_index], first_size);
    if(second_size > 0)
    {
        memcpy( data_ptr + first_size, buffer->buffer, second_size);
    }
    /* shift tail index*/
    buffer->tail_index = (buffer->tail_index + rd_size) & RING_BUFFER_MASK(buffer);
    return rd_size;
}

uint8_t ring_buffer_peek(ring_buffer_t *buffer, uint8_t *data, size_t index) {
  
	if(index >= ring_buffer_num_items(buffer)) 
  {
    /* No items at index */
    return 0;
  }
  
  /* Add index to pointer */
  size_t data_index = ((buffer->tail_index + index) & RING_BUFFER_MASK(buffer));
  *data = buffer->buffer[data_index];
  return 1;
}

