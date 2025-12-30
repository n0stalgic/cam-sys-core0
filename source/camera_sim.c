/*
 * camera_sim.c
 *
 *  Created on: Dec 25, 2025
 *      Author: root
 */


#include "camera_sim.h"
#include "sysdef.h"
#include "core_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <stdint.h>

static u32 frame_counter = 0;

void camera_sim_init(void)
{
	CPRINTF("Initializing camera simulator\r\n");
	srand(xTaskGetTickCount());
}

s32 camera_sim_capture(image_t* buffer)
{
    // Simulate capture delay (realistic timing)
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms exposure

	// generate test pattern
    for(u32 i = 0; i < IMAGE_SIZE; i += 4) {
        u32 *ptr = (u32*)(buffer->img_data + i);
        *ptr = (frame_counter << 16) | (i & 0xFFFF);
    }

    // now generate metadata
    buffer->metadata.buffer_id = buffer->buf_id;
    buffer->metadata.timestamp_ms = xTaskGetTickCount();
    buffer->metadata.size_bytes = IMAGE_SIZE;

    // simulate motion
    buffer->metadata.has_motion = (rand() % 100) < 30;

    // simulate license plate
    buffer->metadata.has_license_plate = (rand() % 100) < 5;

    if (buffer->metadata.has_license_plate)
    {
    	buffer->metadata.priority = IMG_PRIO_HIGH;
    }
    else if (buffer->metadata.has_motion)
    {
    	buffer->metadata.priority = IMG_PRIO_MED;
    }
    else
    {
    	buffer->metadata.priority = IMG_PRIO_LOW;
    }

    frame_counter++;

    return 0;

}
