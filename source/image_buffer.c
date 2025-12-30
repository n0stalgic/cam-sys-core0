/*
 * image_buffer.c
 *
 *  Created on: Dec 25, 2025
 *      Author: chris
 */

#include <stdlib.h>
#include <string.h>
#include "core_printf.h"
#include "image_buffer.h"
#include "sysdef.h"
#include "FreeRTOS.h"
#include "semphr.h"

// hold 8 images in .data.$RAM2 (BOARD_SDRAM)
__attribute__((section(".data.$BOARD_SDRAM")))
static u8 image_buffer_pool[NUM_IMAGE_BUFFERS][IMAGE_SIZE];

static image_t buffers[NUM_IMAGE_BUFFERS];

static SemaphoreHandle_t buffer_mutex;

void image_buffer_init(void)
{

	buffer_mutex = xSemaphoreCreateMutex();
	if (buffer_mutex == NULL)
	{
		CPRINTF("Image buffer init fail, mutex not created\r\n");
	}

	for (size_t i = {}; i < NUM_IMAGE_BUFFERS; ++i)
	{
		buffers[i].buf_id = i;
		buffers[i].img_data = image_buffer_pool[i];
		buffers[i].buf_state = IMG_BUFFER_FREE;
		memset(&buffers[i].metadata, 0U, sizeof(image_metadata_t));
	}

	CPRINTF("Image buffer memory pool initialized (%d buffers)\r\n",
			NUM_IMAGE_BUFFERS);

}

/**
 * @brief return the first free image buffer slot
 * @return found_img_ptr pointer to an available image slot
 */
image_t* image_buffer_get_free(void)
{
	image_t* found_img_ptr = nullptr;

	ENTER_CRITICAL_SECTION(buffer_mutex);
	for (size_t i = {};i < NUM_IMAGE_BUFFERS; ++i)
	{
		if (buffers[i].buf_state == IMG_BUFFER_FREE)
		{
			found_img_ptr =  &buffers[i];
		}
	}
	EXIT_CRITICAL_SECTION(buffer_mutex);
	// return nullptr if no buffers are available
	return found_img_ptr;
}

_Bool image_buffer_release(u8 buf_id)
{
	if(buf_id >= NUM_IMAGE_BUFFERS)
	{
		return FALSE;
	}

	ENTER_CRITICAL_SECTION(buffer_mutex);
	// simply mark buffer as free, we will not free/realloc
	// memory to save time
	buffers[buf_id].buf_state = IMG_BUFFER_FREE;
	EXIT_CRITICAL_SECTION(buffer_mutex);

	return TRUE;

}

_Bool image_buffer_get_state(u8 buf_id, buffer_state_t* state)
{
	if (buf_id >= NUM_IMAGE_BUFFERS)
	{
		return FALSE;
	}

	ENTER_CRITICAL_SECTION(buffer_mutex);
	*state = buffers[buf_id].buf_state;
	EXIT_CRITICAL_SECTION(buffer_mutex);

	return TRUE;
}


_Bool image_buffer_mark_capturing(u8 buf_id)
{
	if((buf_id >= NUM_IMAGE_BUFFERS) ||
				(buffers[buf_id].buf_state != IMG_BUFFER_FREE))
	{
		return FALSE;
	}

	ENTER_CRITICAL_SECTION(buffer_mutex);
	buffers[buf_id].buf_state = IMG_BUFFER_CAPTURING;
	EXIT_CRITICAL_SECTION(buffer_mutex);

	return TRUE;
}

_Bool image_buffer_mark_processing(u8 buf_id)
{
	if (buf_id >= NUM_IMAGE_BUFFERS)
	{
		return FALSE;
	}

	ENTER_CRITICAL_SECTION(buffer_mutex);
	buffers[buf_id].buf_state = IMG_BUFFER_PROCESSING;
	EXIT_CRITICAL_SECTION(buffer_mutex);

	return TRUE;

}

_Bool image_buffer_mark_saving(u8 buf_id)
{
	if (buf_id >= NUM_IMAGE_BUFFERS)
	{
		return FALSE;
	}

	ENTER_CRITICAL_SECTION(buffer_mutex);
	buffers[buf_id].buf_state = IMG_BUFFER_SAVING;
	EXIT_CRITICAL_SECTION(buffer_mutex);

	return TRUE;

}

void image_buffer_get_stats(u8* free, u8* in_use)
{

	ENTER_CRITICAL_SECTION(buffer_mutex);
	for (size_t i = {}; i < NUM_IMAGE_BUFFERS; ++i)
	{
		if (buffers[i].buf_state == IMG_BUFFER_FREE)
		{
			*free++;
		}
		else
		{
			*in_use++;
		}
	}
	EXIT_CRITICAL_SECTION(buffer_mutex);

}
