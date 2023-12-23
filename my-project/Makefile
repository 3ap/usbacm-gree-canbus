PROJECT = awesomesauce
BUILD_DIR = bin

FREERTOS_DIR = ../FreeRTOS-Kernel
SHARED_DIR = ../my-common-code
CFILES = my-project.c
CFILES += api.c
AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32f407vgt6
OOCD_FILE = board/stm32f4discovery.cfg
FREERTOS_PORT = GCC/ARM_CM4F

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

VPATH += $(FREERTOS_DIR)
VPATH += $(FREERTOS_DIR)/portable/$(FREERTOS_PORT)
VPATH += $(FREERTOS_DIR)/portable/MemMang
CFILES += queue.c tasks.c port.c heap_4.c list.c timers.c
INCLUDES += $(patsubst %,-I%, . $(FREERTOS_DIR)/include $(FREERTOS_DIR)/portable/$(FREERTOS_PORT))

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
