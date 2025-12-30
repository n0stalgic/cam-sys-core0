/*
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_CORE0_H_
#define _APP_CORE0_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
/* Address of memory, from which the secondary core will boot */
#define CORE1_BOOT_ADDRESS (void *)0x80000000

#ifdef CORE1_IMAGE_COPY_TO_RAM
extern const char core1_image_start[];
extern uint32_t core1_image_size;
#define CORE1_IMAGE_START ((void *)core1_image_start)
#define CORE1_IMAGE_SIZE  ((uint32_t)core1_image_size)
#endif
/*${macro:end}*/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif
/*${prototype:end}*/

#endif /* _APP_CORE0_H_ */
