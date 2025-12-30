# cam-sys-core0
Camera image generation and soon-to-be MIPI CSI-2 capture firmware designed to run on an RT1170-EVKB. Core 0 (Cortex M7) runs the image processing firmware and core 1 (Core M4) handles I/O. Both cores run FreeRTOS and use RPMsg Lite for inter-core messaging
