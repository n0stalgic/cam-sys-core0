/*
 * image_buffer.h
 *
 *  Created on: Dec 25, 2025
 *      Author: root
 */

#ifndef IMAGE_BUFFER_H_
#define IMAGE_BUFFER_H_

#include "core_msgs.h"

#define IMAGE_SIZE 			(200 * 1024) // 200 KB
#define NUM_IMAGE_BUFFERS   (8U)

typedef enum
{
	IMG_BUFFER_FREE = 0,
	IMG_BUFFER_CAPTURING,
	IMG_BUFFER_PROCESSING,
	IMG_BUFFER_SAVING
} buffer_state_t;

constexpr u8 IMG_PRIO_HIGH = 3;
constexpr u8 IMG_PRIO_MED  = 2;
constexpr u8 IMG_PRIO_LOW  = 1;

typedef struct
{
	u8* img_data;
	u32 buf_id;
	buffer_state_t buf_state;
	image_metadata_t metadata;

} image_t;

// get any free buffer, returns NULL if none are available
image_t* image_buffer_get_free(void);

// get current state of a buffer
_Bool image_buffer_get_state(u8 buf_id, buffer_state_t* state);

// mark a buffer as capturing
_Bool image_buffer_mark_capturing(u8 buf_id);

// mark a buffer as processing
_Bool image_buffer_mark_processing(u8 buf_id);

// mark a buffer as saving to SD card
_Bool image_buffer_mark_saving(u8 buf_id);

// release a buffer back to the pool
_Bool image_buffer_release(u8 buf_id);

// initialize buffer pool
void image_buffer_init(void);

// Get buffer statistics
void image_buffer_get_stats(u8* free, u8* in_use);


#endif /* IMAGE_BUFFER_H_ */
