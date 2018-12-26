/*
 * IntTest.c
 *
 *  Created on: 2018Äê10ÔÂ17ÈÕ
 *      Author: danialxie
 *
 *        This is a test board for calling test included in "FreeRTOS/Demo/Common/Minimal"
 *
 */

/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "n200/drivers/n200_pic_tmr.h"
#include "n200/drivers/n200_func.h"

/* Test case header files*/
#include "AbortDelay.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "serial.h"
#include "comtest.h"
#include "countsem.h"
#include "crflash.h"
#include "crhook.h"
#include "death.h"
#include "dynamic.h"
#include "EventGroupsDemo.h"
#include "flash.h"
#include "flash_timer.h"
#include "flop.h"
#include "GenQTest.h"
#include "integer.h"
#include "IntSemTest.h"
#include "mevents.h"
#include "partest.h"
#include "PollQ.h"
#include "print.h"
#include "QPeek.h"
#include "QueueOverwrite.h"
#include "QueueSet.h"
#include "QueueSetPolling.h"
#include "recmutex.h"
#include "semtest.h"
#include "StaticAllocation.h"
#include "TaskNotify.h"
#include "TimerDemo.h"

#define TEST_AUTO_ENABLE      1
#define TEST_INT_ENABLE       1

#define TEST_RESULT_RUN       0
#define TEST_RESULT_PASS      1
#define TEST_RESULT_FAIL      2

#define TEST_CHECK_PERIOD     2000    // ms
#define TEST_STACK_SIZE       256    
#define TEST_TIME_MIN_LAST    30    // s

#define GPIO_INT_SOURCE(x) (SOC_PIC_INT_GPIO_BASE + x)

typedef enum {
   TEST_NONE,
   TEST_ABORT_DELAY = 1,
   TEST_BLOCKING_QUEUE,
   TEST_BLOCK_TIME,
   TEST_COM,
   TEST_COUNT_SEMAPHORE,
   TEST_FLASH_COROUTINE,
  // TEST_HOOK_COROUTINE,
   TEST_DEATH,
   TEST_DYNAMIC_PRIORITY,
   TEST_EVENT_GROUP,
   TEST_LED_FLASH,
   TEST_LED_FLASH_TIMER,
   TEST_MATH,
   TEST_GENERIC_QUEUE,
   TEST_INTEGER,
   TEST_INT_SEMAPHORE,
#ifdef TEST_FULL
   TEST_MULTI_EVENT,
#endif
   TEST_POLLED_QUEUE,
   TEST_QUEUE_PEEK,
   TEST_QUEUE_OVERWRITE,
   TEST_QUEUE_SET,
   TEST_QUEUE_SET_POLLING,
   TEST_RECURSIVE_MUTEX,
   TEST_SEMAPHORE,
   TEST_STATIC_ALLOCATION,
   TEST_TASK_NOTIFY,
   TEST_TIMER_DEMO,
   TEST_MAX_NUM
} TEST_T;

/* Interrupt handler */
void DefaultInterruptHandler(void);
void GPIOInterruptHandler(uint32_t num, uint32_t priority);
void GPIOInterruptHandler8(void);
void GPIOInterruptHandler9(void);
void GPIOInterruptHandler10(void);

static const char HelpMessage[] = {
   "\nThis testing support test cases listed below:\n"
};
static const char *pTestName[] = {
   "Abort Delay Test",
   "Blocking Queue Test",
   "Block Timer Test",
   "COM Test",
   "Counting Semaphore Test",
   "Flash Co-routine Test",
  // "Hook Co-routine Test",
   "Death Test",
   "Dynamic Priority Test",
   "Event Group Test",
   "LED Flash Test",
   "LED Flash Timer Test",
   "Math Test",
   "Generic Queue Test",
   "Integer Test",
   "Interrupt Semaphore Test",
#ifdef TEST_FULL
   "Multiple Event Test",
#endif
   "Polled Queue Test",
   "Queue Peek Test",
   "Queue Over Write Test",
   "Queue Set Test",
   "Queue Set Polling Test",
   "Recursive Mutex Test",
   "Semaphore Test",
   "Static Allocation Test",
   "Task Notify Test",
   "Timer Demo Test"
};

/* Timer & Mutex handle */
static TimerHandle_t xSoftTimer = NULL;
static SemaphoreHandle_t xPrintMutex = NULL;


/* Test selected and running status*/
static TEST_T test = TEST_COM;
static uint32_t xSingleTestLast = TEST_TIME_MIN_LAST;  // Unit: second
static TickType_t xLastStartTime; 
/* function: vGPIOInit */
void vGPIOInit(void)
{
	uint32_t input_gpios = 0x00000700;  // GPIO8-10
	uint32_t output_gpios = 0x000000E0;  // GPIO5-7

   // Set GPIO 0-7 as output pins
	GPIO_REG(GPIO_INPUT_EN)  &= ~output_gpios;
	GPIO_REG(GPIO_OUTPUT_EN)  |= output_gpios;

	// Set GPIO 8-10 as input pins
	GPIO_REG(GPIO_OUTPUT_EN)  &= ~input_gpios;
	GPIO_REG(GPIO_INPUT_EN)  |= input_gpios;
	GPIO_REG(GPIO_PULLUP_EN)  |= input_gpios;

	// Enable GPIO interrupt
	GPIO_REG(GPIO_FALL_IE) |= input_gpios;
}

//enables interrupt and assigns handler
void enable_interrupt(uint32_t int_num, uint32_t int_priority, function_ptr_t handler) 
{
    pic_interrupt_handlers[int_num] = handler;
    pic_set_priority(int_num, int_priority);
    pic_enable_interrupt (int_num);
}

/* function: vPICInit */
static void vPICInit(void) 
{
	// Disable global interrupter
	clear_csr(mstatus, MSTATUS_MIE);

	// Initialize interrupter handler
	for (int i = 0; i < PIC_NUM_INTERRUPTS; i ++){
		pic_interrupt_handlers[i] = DefaultInterruptHandler;
	}

   if(test != TEST_ABORT_DELAY && test != TEST_COM)
   {
      // Enable GPIO interrupter
	   enable_interrupt(GPIO_INT_SOURCE(8),  6, GPIOInterruptHandler8);
	   //enable_interrupt(GPIO_INT_SOURCE(9),  6, GPIOInterruptHandler9);
	   //enable_interrupt(GPIO_INT_SOURCE(10),  6, GPIOInterruptHandler10);
   }

	// Enable global interrupt
	set_csr(mstatus, MSTATUS_MIE);
}
void DefaultInterruptHandler(void){}

/* GPIO interrupt handler */
void GPIOInterruptHandler(uint32_t num, uint32_t priority)
{
   GPIO_REG(GPIO_OUTPUT_VAL) ^= (0x1 << priority);

   //for(uint32_t i = 0; i < 0xffffffff - 1; ++ i);
   //write(1, "\nEnter Interrupt", 16);
   printf("\n>Enter interrupt %d (priority %d)", num, priority);

	// Clear interrupt pending bit
	GPIO_REG(GPIO_FALL_IP) |= (0x1 << num);
}

/* GPIO 8 interrupt handler */
void GPIOInterruptHandler8(void) {
   GPIOInterruptHandler(8, 6);
}

/* GPIO 9 interrupt handler */
void GPIOInterruptHandler9(void) {
   GPIOInterruptHandler(9, 6);
}

/* GPIO 10 interrupt handler */
void GPIOInterruptHandler10(void) {
   GPIOInterruptHandler(10, 6);
}

/********************************************************************/
static void vPrintString(const char* msg)
{
   xSemaphoreTake(xPrintMutex, portMAX_DELAY);
	{
      write(1, msg, strlen(msg));
      fflush(stdout);
	}
   xSemaphoreGive(xPrintMutex);
}

static char GetChar(void)
{
   int32_t input;

   while((input = (int32_t) UART0_REG(UART_REG_RXFIFO)) < 0);

   return input & 0xFF;
}

/* Get a number from console*/
static int xReadNumber(void)
{
   char num_str[16];
   int32_t i = 0;
   char in_char;

   // Read a number from console
   for( i = 0; i < sizeof(num_str) - 1; ++ i) {
      num_str[i] = GetChar();
      if(num_str[i] == '\r' || num_str[i] == '\n') {
         num_str[i] = 0;
         break;
      }
   }
   num_str[i] = 0; // ending char.

   return atoi(num_str);
}

/* Display a text progress bar on console*/
static void vDisplayTestProgress(uint32_t test, TickType_t xTimeLast, BaseType_t result)
{
#define NAME_FIELD_LENGTH  30
#define BAR_FIELD_LENGTH  50
   
   // Display title info
   printf("\r[%2d] %s", test, pTestName[test-1]);
   for(uint32_t i = NAME_FIELD_LENGTH; i > strlen(pTestName[test-1]); -- i)
      printf(" "); // align with spaces
   
   // Display progress bar
   uint32_t bar_len = xTimeLast * BAR_FIELD_LENGTH / (pdMS_TO_TICKS(xSingleTestLast) * 1000);
   if(bar_len > BAR_FIELD_LENGTH) bar_len = BAR_FIELD_LENGTH;
   for(uint32_t i = 0; i < BAR_FIELD_LENGTH; ++ i) 
      printf((i < bar_len) ? "*" : " ");

   // Print result
   if(result == 0) printf("    [%3d%%]", bar_len * 100 / BAR_FIELD_LENGTH);
   else if (result == 1)printf("    [PASS]\n");
   else printf("    [FAIL]\n");

   fflush(stdout);

   // Backup result
   AON_REG(AON_BACKUP1) = (uint32_t)result;
}

static void vAutoSetupTest(void) {
   TEST_T tst_bkp = AON_REG(AON_BACKUP0);

   if(AON_REG(AON_BACKUP1) == TEST_RESULT_PASS || tst_bkp == TEST_NONE) ++ tst_bkp;
   if(tst_bkp >= TEST_MAX_NUM)tst_bkp = 1;

   test = AON_REG(AON_BACKUP0) = tst_bkp;
}

static void vResetSystem(void) {
   //Use WDG to reset system
   taskENTER_CRITICAL();
   AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; // unlock
   AON_REG(AON_WDOGCMP) = 0x01;
   AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE; // unlock
   AON_REG(AON_WDOGCFG) = AON_WDOGCFG_RSTEN | AON_WDOGCFG_ENALWAYS;
   taskEXIT_CRITICAL();
}

/* Running a required single test case */
static void vTestShellTask(void) {
   TEST_T tst_sel;
   int32_t tst_last;

   do {
      printf("\r\nWhich case do you want to test?(1-%d):", TEST_MAX_NUM-1);
      fflush(stdout);

      while((tst_sel = xReadNumber()) == 0);
   } while (tst_sel >= TEST_MAX_NUM);
   test = tst_sel;

   // Get the period a single case will last(minial: 30s, default: 60s).
   printf("\r\nHow long a single test will last for(>=30s)?:");
   fflush(stdout);
   tst_last = xReadNumber();
   if(tst_last >= TEST_TIME_MIN_LAST) xSingleTestLast = tst_last;

   printf("Test %d is chose to running last for %ds\n", test, xSingleTestLast);
   fflush(stdout);
}

/* Running a required single test case */
static void vStartTestTask(void) {
   switch(test) {
      case TEST_ABORT_DELAY: vCreateAbortDelayTasks(); break; 
      case TEST_BLOCKING_QUEUE: vStartBlockingQueueTasks(1); break;
      case TEST_BLOCK_TIME: vCreateBlockTimeTasks(); break;
      case TEST_COM: vAltStartComTestTasks(1, 115200, 0); break;
      case TEST_COUNT_SEMAPHORE: vStartCountingSemaphoreTasks(); break;
      case TEST_FLASH_COROUTINE: vStartFlashCoRoutines(1); break;
      //case TEST_HOOK_COROUTINE: vStartHookCoRoutines(); break;
      case TEST_DEATH: vCreateSuicidalTasks(1); break;
      case TEST_DYNAMIC_PRIORITY: vStartDynamicPriorityTasks(); break;
      case TEST_EVENT_GROUP: vStartEventGroupTasks(); break;
      case TEST_LED_FLASH: vStartLEDFlashTasks(1); break;
      case TEST_LED_FLASH_TIMER: vStartLEDFlashTimers(3); break;
      case TEST_MATH: vStartMathTasks(1); break;
      case TEST_GENERIC_QUEUE: vStartGenericQueueTasks(0); break;
      case TEST_INTEGER: vStartIntegerMathTasks(1); break;
      case TEST_INT_SEMAPHORE: vStartInterruptSemaphoreTasks(); break;
#ifdef TEST_FULL
      case TEST_MULTI_EVENT: vStartMultiEventTasks(); break;
#endif
      case TEST_POLLED_QUEUE: vStartPolledQueueTasks(1); break;
      case TEST_QUEUE_PEEK: vStartQueuePeekTasks(); break;
      case TEST_QUEUE_OVERWRITE: vStartQueueOverwriteTask(1); break;
      case TEST_QUEUE_SET: vStartQueueSetTasks(); break;
      case TEST_QUEUE_SET_POLLING: vStartQueueSetPollingTask(); break;
      case TEST_RECURSIVE_MUTEX: vStartRecursiveMutexTasks(); break;
      case TEST_SEMAPHORE: vStartSemaphoreTasks(1); break;
      case TEST_STATIC_ALLOCATION: vStartStaticallyAllocatedTasks(); break;
      case TEST_TASK_NOTIFY: vStartTaskNotifyTask(); break;
      case TEST_TIMER_DEMO: vStartTimerDemoTask(50); break;
   }
}
/****************************************************/
/* Timer callback function */
static void vCheckTestStatus(TimerHandle_t xTimer) {
   BaseType_t running = pdFALSE;

   switch(test) {
      case TEST_ABORT_DELAY: running = xAreAbortDelayTestTasksStillRunning(); break;
      case TEST_BLOCKING_QUEUE: running = xAreBlockingQueuesStillRunning(); break;
      case TEST_BLOCK_TIME: running = xAreBlockTimeTestTasksStillRunning(); break;
      case TEST_COM: running = xAreComTestTasksStillRunning(); break;      // Fail
      case TEST_COUNT_SEMAPHORE: running = xAreCountingSemaphoreTasksStillRunning(); break;
      case TEST_FLASH_COROUTINE: running = xAreFlashCoRoutinesStillRunning(); break;
      //case TEST_HOOK_COROUTINE: running = xAreHookCoRoutinesStillRunning(); break; // Fail
      case TEST_DEATH: running = xIsCreateTaskStillRunning(); break;
      case TEST_DYNAMIC_PRIORITY: running = xAreDynamicPriorityTasksStillRunning(); break;
      case TEST_EVENT_GROUP: running = xAreEventGroupTasksStillRunning(); break;
      case TEST_LED_FLASH: running = pdTRUE; break;
      case TEST_LED_FLASH_TIMER: running = pdTRUE; break;
      case TEST_MATH: running = xAreMathsTaskStillRunning(); break;
      case TEST_GENERIC_QUEUE: running = xAreGenericQueueTasksStillRunning(); break;
      case TEST_INTEGER: running = xAreIntegerMathsTaskStillRunning(); break;
      case TEST_INT_SEMAPHORE: running = xAreInterruptSemaphoreTasksStillRunning(); break;
#ifdef TEST_FULL
      case TEST_MULTI_EVENT: running = xAreMultiEventTasksStillRunning(); break;
#endif
      case TEST_POLLED_QUEUE: running = xArePollingQueuesStillRunning(); break;
      case TEST_QUEUE_PEEK: running = xAreQueuePeekTasksStillRunning(); break;
      case TEST_QUEUE_OVERWRITE: running = xIsQueueOverwriteTaskStillRunning(); break;
      case TEST_QUEUE_SET: running = xAreQueueSetTasksStillRunning(); break;
      case TEST_QUEUE_SET_POLLING: running = xAreQueueSetPollTasksStillRunning(); break;
      case TEST_RECURSIVE_MUTEX: running = xAreRecursiveMutexTasksStillRunning(); break;
      case TEST_SEMAPHORE: running = xAreSemaphoreTasksStillRunning(); break;
      case TEST_STATIC_ALLOCATION: running = xAreStaticAllocationTasksStillRunning(); break;
      case TEST_TASK_NOTIFY: running = xAreTaskNotificationTasksStillRunning(); break;
      case TEST_TIMER_DEMO: running = xAreTimerDemoTasksStillRunning(5); break;
   }

   if(test != TEST_NONE) {

      //Display progress
      TickType_t xTimeLast = xTaskGetTickCount() - xLastStartTime;
      if(running == pdTRUE) {
         if(xTimeLast >= pdMS_TO_TICKS(xSingleTestLast * 1000)) {
            // Test is successful
            if(xTimerStop(xSoftTimer, 0) == pdPASS) {
               vDisplayTestProgress(test, xTimeLast, TEST_RESULT_PASS);
               vResetSystem();
            }
         } else {
            // Test is still running
            vDisplayTestProgress(test, xTimeLast, TEST_RESULT_RUN);
         }
      } else {
         // Test is failed
         if(xTimerStop(xSoftTimer, 0) == pdPASS) {
            vDisplayTestProgress(test, xTimeLast, TEST_RESULT_FAIL);
	         clear_csr(mstatus, MSTATUS_MIE); //  disable interrupt
            while(1);
            //vResetSystem();
         }
      }
   }
}
/****************************************************/

// Test target board
int main(void)
{
#if TEST_AUTO_ENABLE
   vAutoSetupTest();
#else
   // Print help message
   printf(HelpMessage);

   // Print test name
   for(int i = 1; i < TEST_MAX_NUM; ++ i) {
      printf("    [%2d] %s\r\n", i, pTestName[i-1]);
   }

   vTestShellTask();
#endif

   vGPIOInit();
#if (TEST_INT_ENABLE)
   vPICInit();
#endif

   // Create mutex and semaphore
   xPrintMutex = xSemaphoreCreateMutex();
   if(xPrintMutex == NULL) printf("Create semaphore failed!\n");

	// Create timer
	xSoftTimer = xTimerCreate("Check", pdMS_TO_TICKS(TEST_CHECK_PERIOD), pdTRUE, NULL, vCheckTestStatus);
   if(test != TEST_TIMER_DEMO)xTimerStart(xSoftTimer, 0);  // Timer demo need TMR command to be empty at beginning

   vStartTestTask();
	vTaskStartScheduler();

	for(;;)

	return 0;
}

void vApplicationIdleHook( void )
{
   if(test == TEST_TIMER_DEMO)
   {
      //if(xTimerIsTimerActive(xSoftTimer) == pdFALSE)
         xTimerStart(xSoftTimer, 0);
   }

   if(test != TEST_COM && test != TEST_DEATH)
   {
      for(;;)
      {
         asm volatile ("wfi"); // enter low power mode
      }
   }
   
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
   switch(test) {
      //case TEST_HOOK_COROUTINE: vCoRoutineApplicationTickHook(); break;
      case TEST_EVENT_GROUP: vPeriodicEventGroupsProcessing(); break;
      case TEST_INT_SEMAPHORE: vInterruptSemaphorePeriodicTest(); break;
      case TEST_QUEUE_OVERWRITE: vQueueOverwritePeriodicISRDemo(); break;
      case TEST_QUEUE_SET: vQueueSetAccessQueueSetFromISR(); break;
      case TEST_QUEUE_SET_POLLING: vQueueSetPollingInterruptAccess(); break;
      case TEST_TASK_NOTIFY: xNotifyTaskFromISR(); break;
      case TEST_TIMER_DEMO: vTimerPeriodicISRTests(); break;
   }
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	write(1,"malloc failed\n", 14);
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) xTask;

    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    write(1, "Stack Overflow\n", 15);
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook(void)
{
   vParTestInitialise();
}
/*-----------------------------------------------------------*/

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/
