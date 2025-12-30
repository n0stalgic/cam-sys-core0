        .section .text , "ax" @progbits @preinit_array
        .global core1_image_start
        .type core1_image_start, %object
        .align 2
core1_image_start:
        .incbin "img_housekeeping_cm4_secondary.bin"
        .global core1_image_end
        .type core1_image_end, %object
core1_image_end:
        .global core1_data_image_start
        .type core1_data_image_start, %object
        .align 2
core1_ncache_image_end:
        .global core1_image_size
        .type core1_image_size, %object
        .align 2
core1_image_size:
	.int core1_image_end - core1_image_start
