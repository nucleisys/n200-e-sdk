TARGET = FreeRTOSv9.0.0
CFLAGS += -O2

BSP_BASE = ../../bsp


C_SRCS += Source/croutine.c  
C_SRCS += Source/list.c  
C_SRCS += Source/queue.c  
C_SRCS += Source/tasks.c  
C_SRCS += Source/timers.c  
C_SRCS += Source/portable/MemMang/heap_4.c  
C_SRCS += Source/portable/GCC/N200/port.c

C_SRCS += Demo/RISCV_N200_GCC/main.c

INCLUDES += -ISource/include
INCLUDES += -IDemo/RISCV_N200_GCC
INCLUDES += -ISource/portable/GCC/N200



ASM_SRCS += Source/portable/GCC/N200/portasm.S

include $(BSP_BASE)/$(BOARD)/n200/env/common.mk
