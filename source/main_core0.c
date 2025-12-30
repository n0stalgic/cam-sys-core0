/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <limits.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "core_printf.h"
#include "core_msgs.h"
#include "app.h"
#include "mcmgr.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "rpmsg_addr_mem_cfg.h"
#include "icc.h"
#include "periph_mutex.h"
#include "image_buffer.h"
#include "camera_sim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "version.h"
#include "timers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TASK_STACK_SIZE 	256U
#define TASK_PRI_ICC 	    5U  /* Highest - must drain RPMsg buffer quickly */
#define TASK_PRI_SAVE		2U
#define TASK_PRI_BUF_MGMT	2U
#define TASK_PRI_IMAGE_PROC 3U
#define TASK_PRI_IMAGE_CAPT 4U

#define FREQ_DIV 		1000000U

#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMXRT1170_M7_M4_LINK_ID)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-camera-sys-chan"

#define APP_RPMSG_READY_EVENT_DATA 	  (1U)
#define APP_BEGIN_CAPTURE_EVENT		  (2U)

#define ICC_TX_QUEUE_LEN            (32U)  /* Max pending TX requests */
#define IMG_SAVE_QUEUE_LEN			(16U)
#define IMG_BUFFER_FREE_QUEUE_LEN	(16U)
#define IMG_PROCESS_QUEUE_LEN		(64U)

static QueueHandle_t icc_tx_queue;
static QueueHandle_t image_processing_queue;
static QueueHandle_t image_save_queue;
static QueueHandle_t buffer_free_queue;
static TimerHandle_t icc_timeout_timer;
static volatile bool core1_alive = false;

extern u8 _pvHeapStart[];
extern u8 _pvHeapLimit[];
extern size_t _HeapSize;


#define OCRAM_HEAP_BASE_ADDR _pvHeapStart
#define OCRAM_HEAP_SIZE		 _HeapSize

// this is required for heap 5 allocation
HeapRegion_t xHeapRegions[2U];

__attribute__((section(".noinit.$rpmsg_sh_mem")))
static char rpmsg_lite_base[RPMSG_SH_MEM_SIZE];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ImgProc_EarlyInit(void);
void invalidate_cache_for_core1_image_memory(uint32_t* addr, uint32_t size);
void ImageCaptureTask(void* param);
void ImageProcessTask(void* param);
void ImageBufferMgmtTask(void* param);
void ImageSDCardSaveTask(void* param);
void ICCHdlrTask(void* param);

/*******************************************************************************
 * Task Handles
 ******************************************************************************/
// static TaskHandle_t capture_task_handle;
static TaskHandle_t process_task_handle;
static TaskHandle_t image_capture_task_handle;
static TaskHandle_t rpmsg_init_task_handle;
static TaskHandle_t buffer_mgmt_task_handle;


/*******************************************************************************
 * Code
 ******************************************************************************/


void InitializeHeap(void) {
    // Calculate heap size at runtime
    size_t heap_size = (size_t)(_pvHeapLimit - _pvHeapStart);

    // Initialize heap regions
    xHeapRegions[0].pucStartAddress = (uint8_t*)_pvHeapStart;
    xHeapRegions[0].xSizeInBytes = heap_size;

    // NULL terminator
    xHeapRegions[1].pucStartAddress = NULL;
    xHeapRegions[1].xSizeInBytes = 0;

    // Configure Heap 5
    vPortDefineHeapRegions(xHeapRegions);

    CPRINTF("Heap configured: 0x%08lx, size: %lu bytes\r\n",
            (uint32_t)_pvHeapStart, (uint32_t)heap_size);
}


/**
 * @brief Watchdog timer callback - called when Core1 heartbeat times out
 */
static void icc_timeout_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (core1_alive)
    {
        CPRINTF("*** CORE1 TIMEOUT - No heartbeat! Suspending image processing ***\r\n");
        core1_alive = false;

        /* Suspend image capture to stop sending data to dead core */
        if (image_capture_task_handle != NULL)
        {
            vTaskSuspend(image_capture_task_handle);
        }
    }
}

static void app_nameservice_isr_cb(u32 new_ept, const char *new_ept_name, u32 flags, void *user_data)
{
    u32 *data = (u32 *)user_data;

    *data = new_ept;
}

static volatile u16 RPMsgRemoteReadyEventData = 0U;
static void RPMsgRemoteReadyEventHandler(mcmgr_core_t coreNum, u16 eventData, void *context)
{
    u16 *data = (u16 *)context;

    *data = eventData;
}


void ImageBufferMgmtTask(void* param)
{
	u32 buf_id = {};
	u8 free = {}, in_use = {};

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while (TRUE)
	{

		if(xQueueReceive(buffer_free_queue, &buf_id, 0) == pdTRUE)
		{
			image_buffer_release(buf_id);

			image_buffer_get_stats(&free, &in_use);
		}
	}
}

void ImageCaptureTask(void* param)
{
	TickType_t last_wake = xTaskGetTickCount();

	u32 dropped_frames = {};

	camera_sim_init();

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while(TRUE)
	{
		image_t* buf = image_buffer_get_free();

		if (buf == nullptr)
		{
			CPRINTF("No free buffers, dropping frame\r\n");
			dropped_frames++;
		}
		else
		{
			// capture an image
			camera_sim_capture(buf);

			// send to image processing task
			if(xQueueSend(image_processing_queue, &buf, 0) != pdTRUE)
			{
				CPRINTF("Image processing queue full, dropping frame\r\n");
				dropped_frames++;
				image_buffer_release(buf->buf_id);
			}

		}

		// maintain 30 FPS (33.33ms period)
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(33));
	}
}

void ImageProcessTask(void* param)
{
	image_t* buf;

	image_buffer_init();

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while(TRUE)
	{
		if (xQueueReceive(image_processing_queue, &buf, 0) == pdTRUE)
		{
			image_buffer_mark_processing(buf->buf_id);

			// simulate image processing
			vTaskDelay(pdMS_TO_TICKS(10));

			CPRINTF("processed frame %lu (buf %lu, pri %u, mot %u, lp %u\r\n",
					buf->metadata.frame_number,
					buf->metadata.buffer_id,
					buf->metadata.priority,
					buf->metadata.has_motion,
					buf->metadata.has_license_plate);

			image_buffer_mark_saving(buf->buf_id);

			// send to core1 for storage management
			icc_tx_req_t tx_req = {};
			core_message_t core_msg = {};

			core_msg.type = MSG_IMAGE_READY;
			core_msg.payload.image = buf->metadata;

			tx_req.timeout_ms = 0;
			tx_req.msg = core_msg;

			if(xQueueSend(icc_tx_queue, &tx_req, 0) != pdTRUE)
			{
				CPRINTF("Inter-core queue full, aborting image save request (frame %lu)\r\n",
						buf->metadata.frame_number);

				image_buffer_release(buf->buf_id);
			}
		}
	}
}

void ICCHdlrTask(void* param)
{
	u8 err = FALSE;
	volatile u32 remote_addr = {};
	volatile TestMsg_t local_msg = {};
	core_message_t core_msg = {};
	rpmsg_core1_attrs_t rpmsg_attrs = {};


	/* Wait until the secondary core application signals the rpmsg remote has been initialized and is ready to communicate. */
	CPRINTF("Waiting for core1 ready event...\r\n");
	while (APP_RPMSG_READY_EVENT_DATA != RPMsgRemoteReadyEventData)
	{
		vTaskDelay(pdMS_TO_TICKS(10)); // Use FreeRTOS delay instead of busy-wait
	}
	CPRINTF("core1 ready event received\r\n");

	/* Initialize RPMsg as master */
	rpmsg_attrs.rpmsg_inst = rpmsg_lite_master_init(rpmsg_lite_base,
			RPMSG_SH_MEM_SIZE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
	GOTO_IF_NULL(rpmsg_attrs.rpmsg_inst, init_err, err);

	rpmsg_attrs.rpmsg_queue = rpmsg_queue_create(rpmsg_attrs.rpmsg_inst);
	GOTO_IF_NULL(rpmsg_attrs.rpmsg_queue, init_err, err);


	rpmsg_attrs.rpmsg_ept = rpmsg_lite_create_ept(rpmsg_attrs.rpmsg_inst, CORE0_EPT_ADDR, rpmsg_queue_rx_cb, rpmsg_attrs.rpmsg_queue);
	GOTO_IF_NULL(rpmsg_attrs.rpmsg_ept, init_err, err);


	rpmsg_attrs.ns_handle = rpmsg_ns_bind(rpmsg_attrs.rpmsg_inst, app_nameservice_isr_cb, ((void *)&remote_addr));
	GOTO_IF_NULL(rpmsg_attrs.ns_handle, init_err, err);


	/* Wait until the secondary core application issues the nameservice isr and the remote endpoint address is known. */
	CPRINTF("Waiting for remote endpoint...\r\n");
	while (0U == remote_addr)
	{
		vTaskDelay(10);
	}
	CPRINTF("Remote endpoint ready: %lu\r\n", remote_addr);

	/* Send the first message to the remoteproc */
	/* run a power-on ICC test */
	local_msg.DATA = 0U;
	(void)icc_async_send(&rpmsg_attrs, (void *)&local_msg, sizeof(TestMsg_t));

	/* RPMsg ping-pong communication loop to test */
	u32 msg_len = {};
	while (local_msg.DATA <= 10U)
	{
		(void)icc_sync_recv(&rpmsg_attrs, (void *)&local_msg, &msg_len, sizeof(TestMsg_t));

		local_msg.DATA++;
		(void)icc_sync_send(&rpmsg_attrs, (void *)&local_msg, sizeof(TestMsg_t));

	}

	CPRINTF("Inter-core link online \r\n");

	// we can communicate with the other core,
	// drain the queue before we start protocol handling
	icc_drain_rx_queue(&rpmsg_attrs);

	while (APP_BEGIN_CAPTURE_EVENT != RPMsgRemoteReadyEventData)
	{
		vTaskDelay(pdMS_TO_TICKS(10)); // Use FreeRTOS delay instead of busy-wait
	}
	CPRINTF("Core1 fully initialized. Beginning image capture and processing\r\n");

    /* Mark Core1 as alive and start watchdog timer */
    core1_alive = true;
    xTimerStart(icc_timeout_timer, 0);

    xTaskNotifyGive(image_capture_task_handle);
    xTaskNotifyGive(buffer_mgmt_task_handle);
    xTaskNotifyGive(process_task_handle);

	/* we have a link, begin handling ICC */
	while(TRUE)
	{
		/* let's process recv'd messages first */
		u32 msg_len;
		if (icc_async_recv(&rpmsg_attrs, &core_msg, &msg_len, sizeof(core_message_t)) == RL_SUCCESS)
		{
			if (msg_len > 0)
			{
				switch(core_msg.type)
				{
					case MSG_BUFFER_FREE:
						/* Core1 freed a buffer */
						xQueueSend(buffer_free_queue, &core_msg.payload.buffer_id, 0);
						break;

					case MSG_HEARTBEAT:
						/* Core1 is alive - reset watchdog timer */
						if (!core1_alive)
						{
							CPRINTF("Core1 heartbeat resumed - Restarting image processing\r\n");
							core1_alive = true;
							if (image_capture_task_handle != NULL)
							{
								vTaskResume(image_capture_task_handle);
							}
						}
						xTimerReset(icc_timeout_timer, 0);
						break;

					default:
						CPRINTF("Unknown message type: %d\r\n", core_msg.type);
						break;
				}
			}


		}
		icc_tx_req_t tx_req;
		if (xQueueReceive(icc_tx_queue, &tx_req, 0) == pdTRUE)
		{
			s32 rc;

			if (tx_req.timeout_ms == 0U || tx_req.timeout_ms == RL_BLOCK)
			{
				rc = icc_sync_send(&rpmsg_attrs, &tx_req.msg, sizeof(core_message_t));
			}
			else
			{
				rc = icc_async_send(&rpmsg_attrs, &tx_req.msg, sizeof(core_message_t));
			}

			if (rc != RL_SUCCESS)
			{
				CPRINTF("ICC transmit to core1 fail, err %ld\r\n", rc);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(5U));
	}

init_err:
	if (err)
	{
		if (rpmsg_attrs.rpmsg_ept)
		{
			(void)rpmsg_lite_destroy_ept(rpmsg_attrs.rpmsg_inst, rpmsg_attrs.rpmsg_ept);
			rpmsg_attrs.rpmsg_ept = NULL;
		}

		if (rpmsg_attrs.rpmsg_queue)
		{
			(void)rpmsg_queue_destroy(rpmsg_attrs.rpmsg_inst, rpmsg_attrs.rpmsg_queue);
			rpmsg_attrs.rpmsg_queue = NULL;
		}

		if (rpmsg_attrs.ns_handle)
		{
			(void)rpmsg_ns_unbind(rpmsg_attrs.rpmsg_inst, rpmsg_attrs.ns_handle);
		}

		if (rpmsg_attrs.rpmsg_inst)
		{
			(void)rpmsg_lite_deinit(rpmsg_attrs.rpmsg_inst);
			rpmsg_attrs.rpmsg_inst = NULL;
		}

		CPRINTF("RPMsg de-initialized, task ending\r\n");

	}

	vTaskDelete(NULL); // Task complete, delete itself
}

void invalidate_cache_for_core1_image_memory(uint32_t* addr, uint32_t size)
{
	SCB_InvalidateICache_by_Addr(addr, size);
}

/**
 * @brief initialize the board and start the cm4 core
 */
void ImgProc_EarlyInit(void)
{
	char freqBuf[20];

	/* Initialize MCMGR, install generic event handlers */
	(void)MCMGR_Init();

	/* Init board hardware.*/
	BOARD_InitHardware();

	/* Initialize peripheral mutex for inter-core synchronization */
	periph_mutex_init();

	/* Print the initial banner from Primary core */
	PRINTF("\r\n");
	CPRINTF("Build: %s %s %s\r\n", __MAIN_CORE_0_FW_VER__, __DATE__, __TIME__);
	print_banner("Status", "[ONLINE]");
	sprintf(freqBuf, "%lu MHz", CLOCK_GetCoreSysClkFreq() / FREQ_DIV);
	print_banner("Core Clk", freqBuf);

#ifdef CORE1_IMAGE_COPY_TO_RAM
	/* Copy secondary core image from flash to RAM */
	uint32_t core1_image_size;
	core1_image_size = get_core1_image_size();
	CPRINTF("Copying core1 image to RAM (size: 0x%x)\r\n", core1_image_size);

	(void)memcpy((void *)(char *)CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);

#ifdef APP_INVALIDATE_CACHE_FOR_SECONDARY_CORE_IMAGE_MEMORY
	invalidate_cache_for_core1_image_memory(CORE1_BOOT_ADDRESS, core1_image_size);
#endif
#endif

	/* Register the application event before starting the secondary core */
	(void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RPMsgRemoteReadyEventHandler,
	                         (void *)&RPMsgRemoteReadyEventData);

	/* Boot Secondary core application */
	CPRINTF("Starting core1...\r\n");
	(void)MCMGR_StartCore(kMCMGR_Core1, (void *)(char *)CORE1_BOOT_ADDRESS, (uint32_t)rpmsg_lite_base,
	                     kMCMGR_Start_Synchronous);
}

/*!
 * @brief Main function
 */
int main(void)
{
	ImgProc_EarlyInit();

	InitializeHeap();

	icc_tx_queue = xQueueCreate(ICC_TX_QUEUE_LEN, sizeof(core_message_t));
	CHECK_NULL(icc_tx_queue);

	image_processing_queue = xQueueCreate(IMG_PROCESS_QUEUE_LEN, sizeof(image_t*));
	CHECK_NULL(image_processing_queue);

	buffer_free_queue = xQueueCreate(IMG_BUFFER_FREE_QUEUE_LEN, sizeof(u32));
	CHECK_NULL(buffer_free_queue);

	image_save_queue = xQueueCreate(IMG_SAVE_QUEUE_LEN, sizeof(image_metadata_t));
	CHECK_NULL(image_save_queue);

	/* Create watchdog timer: 3-second timeout, auto-reload disabled */
	icc_timeout_timer = xTimerCreate("Core0_ICCTimer", pdMS_TO_TICKS(3000U), pdFALSE, (void *)0, icc_timeout_callback);
	CHECK_NULL(icc_timeout_timer);

	/* Create RPMsg initialization task - higher priority to run first */
	CHECK_RTOS(xTaskCreate(ICCHdlrTask,         "Core0_ICCHdlrTask",         TASK_STACK_SIZE * 2, NULL, TASK_PRI_ICC,        &rpmsg_init_task_handle));
	CHECK_RTOS(xTaskCreate(ImageCaptureTask,    "Core0_ImageCaptureTask",    TASK_STACK_SIZE,     NULL, TASK_PRI_IMAGE_CAPT, &image_capture_task_handle));
	CHECK_RTOS(xTaskCreate(ImageProcessTask,    "Core0_ImageProcessTask",    TASK_STACK_SIZE,     NULL, TASK_PRI_IMAGE_PROC, &process_task_handle));
	CHECK_RTOS(xTaskCreate(ImageBufferMgmtTask, "Core0_BufferMgmtTask",      TASK_STACK_SIZE,     NULL, TASK_PRI_BUF_MGMT,   &buffer_mgmt_task_handle));

	vTaskStartScheduler();

	/* Should never reach here */
	CPRINTF("ERROR: Scheduler returned!\r\n");
	while(TRUE)
	{
		__asm volatile("nop");
	}
}
