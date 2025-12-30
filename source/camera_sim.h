/*
 * camera_sim.h
 *
 *  Created on: Dec 25, 2025
 *      Author: root
 */

#ifndef CAMERA_SIM_H_
#define CAMERA_SIM_H_

#include "sysdef.h"
#include "image_buffer.h"
#include <stdlib.h>

void camera_sim_init(void);
s32  camera_sim_capture(image_t* buffer);

#endif /* CAMERA_SIM_H_ */
