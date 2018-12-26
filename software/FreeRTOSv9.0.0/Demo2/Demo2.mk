
#Choose test type
ifeq ($(TEST_TYPE), Full)
TEST_DIR := Full
CFLAGS += -D TEST_FULL
else
TEST_DIR := Minimal
endif

C_SRCS += ${DEMO_DIR}/ParTest.c
C_SRCS += ${DEMO_DIR}/serial.c
C_SRCS += Common/Minimal/AbortDelay.c
C_SRCS += Common/${TEST_DIR}/BlockQ.c
C_SRCS += Common/Minimal/blocktim.c
C_SRCS += Common/${TEST_DIR}/comtest.c
C_SRCS += Common/Minimal/countsem.c
C_SRCS += Common/Minimal/crflash.c
C_SRCS += Common/Minimal/crhook.c
C_SRCS += Common/${TEST_DIR}/death.c
C_SRCS += Common/${TEST_DIR}/dynamic.c
C_SRCS += Common/Minimal/EventGroupsDemo.c
C_SRCS += Common/${TEST_DIR}/flash.c
C_SRCS += Common/Minimal/flash_timer.c
C_SRCS += Common/${TEST_DIR}/flop.c
C_SRCS += Common/Minimal/GenQTest.c
C_SRCS += Common/${TEST_DIR}/integer.c
C_SRCS += Common/Minimal/IntSemTest.c
C_SRCS += Common/${TEST_DIR}/PollQ.c
C_SRCS += Common/Minimal/QPeek.c
C_SRCS += Common/Minimal/QueueOverwrite.c
C_SRCS += Common/Minimal/QueueSet.c
C_SRCS += Common/Minimal/QueueSetPolling.c
C_SRCS += Common/Minimal/recmutex.c
C_SRCS += Common/Minimal/semtest.c
C_SRCS += Common/Minimal/StaticAllocation.c
C_SRCS += Common/Minimal/TaskNotify.c
C_SRCS += Common/Minimal/TimerDemo.c

ifeq ($(TEST_TYPE), Full)
C_SRCS += Common/Full/events.c
C_SRCS += Common/Full/print.c
endif

INCLUDES += -I${BASP_BASE}/${BOARD}/n200/drivers
INCLUDES += -I${BASP_BASE}/${BOARD}/soc/drivers
INCLUDES += -ICommon/include

LDFLAGS += -LCommon/Minimal/

