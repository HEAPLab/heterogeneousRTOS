/*
 * FreeRTOS Kernel V10.4.3
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 * 1 tab == 4 spaces!
 */

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Xilinx includes. */
#include "xscugic.h"

#ifndef configINTERRUPT_CONTROLLER_BASE_ADDRESS
#error configINTERRUPT_CONTROLLER_BASE_ADDRESS must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET
#error configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configUNIQUE_INTERRUPT_PRIORITIES
#error configUNIQUE_INTERRUPT_PRIORITIES must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configSETUP_TICK_INTERRUPT
#error configSETUP_TICK_INTERRUPT() must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif /* configSETUP_TICK_INTERRUPT */

#ifndef configMAX_API_CALL_INTERRUPT_PRIORITY
#error configMAX_API_CALL_INTERRUPT_PRIORITY must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#if configMAX_API_CALL_INTERRUPT_PRIORITY == 0
#error configMAX_API_CALL_INTERRUPT_PRIORITY must not be set to 0
#endif

#if configMAX_API_CALL_INTERRUPT_PRIORITY > configUNIQUE_INTERRUPT_PRIORITIES
#error configMAX_API_CALL_INTERRUPT_PRIORITY must be less than or equal to configUNIQUE_INTERRUPT_PRIORITIES as the lower the numeric priority value the higher the logical interrupt priority
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1
/* Check the configuration. */
#if( configMAX_PRIORITIES > 32 )
#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
#endif
#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* In case security extensions are implemented. */
#if configMAX_API_CALL_INTERRUPT_PRIORITY <= ( configUNIQUE_INTERRUPT_PRIORITIES / 2 )
#error configMAX_API_CALL_INTERRUPT_PRIORITY must be greater than ( configUNIQUE_INTERRUPT_PRIORITIES / 2 )
#endif

/* Some vendor specific files default configCLEAR_TICK_INTERRUPT() in
portmacro.h. */
#ifndef configCLEAR_TICK_INTERRUPT
#define configCLEAR_TICK_INTERRUPT()
#endif

/* A critical section is exited when the critical section nesting count reaches
this value. */
#define portNO_CRITICAL_NESTING			( ( uint32_t ) 0 )

/* In all GICs 255 can be written to the priority mask register to unmask all
(but the lowest) interrupt priority. */
#define portUNMASK_VALUE				( 0xFFUL )

/* Tasks are not created with a floating point context, but can be given a
floating point context after they have been created.  A variable is stored as
part of the tasks context that holds portNO_FLOATING_POINT_CONTEXT if the task
does not have an FPU context, or any other value if the task does have an FPU
context. */
#define portNO_FLOATING_POINT_CONTEXT	( ( StackType_t ) 0 )

/* Constants required to setup the initial task context. */
#define portINITIAL_SPSR				( ( StackType_t ) 0x1f ) /* System mode, ARM mode, IRQ enabled FIQ enabled. */
#define portTHUMB_MODE_BIT				( ( StackType_t ) 0x20 )
#define portINTERRUPT_ENABLE_BIT		( 0x80UL )
#define portTHUMB_MODE_ADDRESS			( 0x01UL )

/* Used by portASSERT_IF_INTERRUPT_PRIORITY_INVALID() when ensuring the binary
point is zero. */
#define portBINARY_POINT_BITS			( ( uint8_t ) 0x03 )

/* Masks all bits in the APSR other than the mode bits. */
#define portAPSR_MODE_BITS_MASK			( 0x1F )

/* The value of the mode bits in the APSR when the CPU is executing in user
mode. */
#define portAPSR_USER_MODE				( 0x10 )

/* The critical section macros only mask interrupts up to an application
determined priority level.  Sometimes it is necessary to turn interrupt off in
the CPU itself before modifying certain hardware registers. */
#define portCPU_IRQ_DISABLE()										\
		__asm volatile ( "CPSID i" ::: "memory" );						\
		__asm volatile ( "DSB" );										\
		__asm volatile ( "ISB" );

#define portCPU_IRQ_ENABLE()										\
		__asm volatile ( "CPSIE i" ::: "memory" );						\
		__asm volatile ( "DSB" );										\
		__asm volatile ( "ISB" );


/* Macro to unmask all interrupt priorities. */
#define portCLEAR_INTERRUPT_MASK()									\
		{																	\
	portCPU_IRQ_DISABLE();											\
	portICCPMR_PRIORITY_MASK_REGISTER = portUNMASK_VALUE;			\
	__asm volatile (	"DSB		\n"								\
			"ISB		\n" );							\
			portCPU_IRQ_ENABLE();											\
		}

#define portINTERRUPT_PRIORITY_REGISTER_OFFSET		0x400UL
#define portMAX_8_BIT_VALUE							( ( uint8_t ) 0xff )
#define portBIT_0_SET								( ( uint8_t ) 0x01 )

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS

/* The space on the stack required to hold the FPU registers.  This is 32 64-bit
registers, plus a 32-bit status register. */
#define portFPU_REGISTER_WORDS	( ( 32 * 2 ) + 1 )

/*-----------------------------------------------------------*/

/*
 * Initialise the interrupt controller instance.
 */
static int32_t prvInitialiseInterruptController( void );

/* Ensure the interrupt controller instance variable is initialised before it is
 * used, and that the initialisation only happens once.
 */
static int32_t prvEnsureInterruptControllerIsInitialised( void );

/*
 * Starts the first task executing.  This function is necessarily written in
 * assembly code so is implemented in portASM.s.
 */
extern void vPortRestoreTaskContext( void );

extern void vPortHandleNewTask( void );

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/* 
 * The instance of the interrupt controller used by this port.  This is required
 * by the Xilinx library API functions.
 */
extern XScuGic xInterruptController;

/*
 * If the application provides an implementation of vApplicationIRQHandler(),
 * then it will get called directly without saving the FPU registers on
 * interrupt entry, and this weak implementation of
 * vApplicationFPUSafeIRQHandler() is just provided to remove linkage errors -
 * it should never actually get called so its implementation contains a
 * call to configASSERT() that will always fail.
 *
 * If the application provides its own implementation of
 * vApplicationFPUSafeIRQHandler() then the implementation of
 * vApplicationIRQHandler() provided in portASM.S will save the FPU registers
 * before calling it.
 *
 * Therefore, if the application writer wants FPU registers to be saved on
 * interrupt entry their IRQ handler must be called
 * vApplicationFPUSafeIRQHandler(), and if the application writer does not want
 * FPU registers to be saved on interrupt entry their IRQ handler must be
 * called vApplicationIRQHandler().
 */
void vApplicationFPUSafeIRQHandler( uint32_t ulICCIAR ) __attribute__((weak) );

/*-----------------------------------------------------------*/

/* A variable is used to keep track of the critical section nesting.  This
variable has to be stored as part of the task context and must be initialised to
a non zero value to ensure interrupts don't inadvertently become unmasked before
the scheduler starts.  As it is stored as part of the task context it will
automatically be set to 0 when the first task is started. */
volatile uint32_t ulCriticalNesting = 9999UL;

/* Saved as part of the task context.  If ulPortTaskHasFPUContext is non-zero then
a floating point context must be saved and restored for the task. */
volatile uint32_t ulPortTaskHasFPUContext = pdFALSE;

/* Set to 1 to pend a context switch from an ISR. */
volatile uint32_t ulPortYieldRequired = pdFALSE;

/* Counts the interrupt nesting depth.  A context switch is only performed if
if the nesting depth is 0. */
volatile uint32_t ulPortInterruptNesting = 0UL;
/*
 * Global counter used for calculation of run time statistics of tasks.
 * Defined only when the relevant option is turned on
 */
#if (configGENERATE_RUN_TIME_STATS==1)
volatile uint32_t ulHighFrequencyTimerTicks;
#endif

/* Used in the asm file. */
__attribute__(( used )) const uint32_t ulICCIAR = portICCIAR_INTERRUPT_ACKNOWLEDGE_REGISTER_ADDRESS;
__attribute__(( used )) const uint32_t ulICCEOIR = portICCEOIR_END_OF_INTERRUPT_REGISTER_ADDRESS;
__attribute__(( used )) const uint32_t ulICCPMR	= portICCPMR_PRIORITY_MASK_REGISTER_ADDRESS;
__attribute__(( used )) const uint32_t ulMaxAPIPriorityMask = ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	/* Setup the initial stack of the task.  The stack is set exactly as
	expected by the portRESTORE_CONTEXT() macro.

	The fist real value on the stack is the status register, which is set for
	system mode, with interrupts enabled.  A few NULLs are added first to ensure
	GDB does not try decoding a non-existent return address. */
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) portINITIAL_SPSR;

	if( ( ( uint32_t ) pxCode & portTHUMB_MODE_ADDRESS ) != 0x00UL )
	{
		/* The task will start in THUMB mode. */
		*pxTopOfStack |= portTHUMB_MODE_BIT;
	}

	pxTopOfStack--;

	/* Next the return address, which in this case is the start of the task. */
	*pxTopOfStack = ( StackType_t ) pxCode;
	pxTopOfStack--;

	/* Next all the registers other than the stack pointer. */
	*pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12121212;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x11111111;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x10101010;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x09090909;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x08080808;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x07070707;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x06060606;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x05050505;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x04040404;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x03030303;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x02020202;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x01010101;	/* R1 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
	pxTopOfStack--;

	/* The task will start with a critical nesting count of 0 as interrupts are
	enabled. */
	*pxTopOfStack = portNO_CRITICAL_NESTING;

#if( configUSE_TASK_FPU_SUPPORT == 1 )
	{
		/* The task will start without a floating point context.  A task that
		uses the floating point hardware must call vPortTaskUsesFPU() before
		executing any floating point instructions. */
		pxTopOfStack--;
		*pxTopOfStack = portNO_FLOATING_POINT_CONTEXT;
	}
#elif( configUSE_TASK_FPU_SUPPORT == 2 )
	{
		/* The task will start with a floating point context.  Leave enough
		space for the registers - and ensure they are initialised to 0. */
		pxTopOfStack -= portFPU_REGISTER_WORDS;
		memset( pxTopOfStack, 0x00, portFPU_REGISTER_WORDS * sizeof( StackType_t ) );

		pxTopOfStack--;
		*pxTopOfStack = pdTRUE;
		ulPortTaskHasFPUContext = pdTRUE;
	}
#else
	{
#error Invalid configUSE_TASK_FPU_SUPPORT setting - configUSE_TASK_FPU_SUPPORT must be set to 1, 2, or left undefined.
	}
#endif

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

//FPGA INFOS fedit


////NEXT TASK HANDLER
//#define NEXT_TASK_HANDLER_BASEADDR 0x43C00000
//
//int enableNewTaskInterrupt() {
//	NEXT_TASK_HANDLER_EnableInterrupt(NEXT_TASK_HANDLER_BASEADDR);
//	return 0;
//}
//
//
//
////GPIO
//#include "xparameters.h"
//#include "xgpio.h"
//#define GPIO0VAL 0x01
//#define GPIO_START_SCHED_CHANNEL 1
//#define GPIO_START_SCHED_DEVICE_ID XPAR_GPIO_0_DEVICE_ID
//XGpio prvGpio0;
//
//struct prvSchedControlStruct {
//	volatile u8 data; //2byte
//	volatile u8 control; //1byte
//} prvSchedControl;
//
//void prvWriteSchedControl() {
//	u16 prvSchedControlU16;
//	memcpy(&prvSchedControlU16, &prvSchedControl, sizeof(prvSchedControl));
//	xil_printf("written %x to GPIO0\n", prvSchedControlU16);
//	XGpio_DiscreteWrite(&prvGpio0, GPIO_START_SCHED_CHANNEL, prvSchedControlU16);
//}

//DMA____________________________
//
//#include "xaxicdma.h"
//#include "xdebug.h"
//#include "xil_exception.h"
//#include "xil_cache.h"
//#include <math.h>
//#include "xscugic.h"
//
////#define DMA_NUMBER_OF_TRANSFERS	2	/* Number of simple transfers to do */
//#define DMA_CTRL_DEVICE_ID 	XPAR_AXICDMA_0_DEVICE_ID
//#define DMA_INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
//#define DMA_CTRL_IRPT_INTR	XPAR_FABRIC_AXI_CDMA_0_CDMA_INTROUT_INTR
//
//static volatile int prvDmaDone = 0;	/* Dma transfer is done */
//static volatile int prvDmaError = 0;	/* Dma Bus Error occurs */
//
//static u32 prvDmaRTTasksListDestAddr 	= 0xC0000000;
//
//static XAxiCdma prvDmaAxiCdmaInstance;	/* Instance of the XAxiCdma */
//static XScuGic prvDmaIntcController;	/* Instance of the Interrupt Controller */
//
//static void prvDmaDisableIntrSystem()
//{
//
//	XScuGic *IntcInstancePtr = &prvDmaIntcController;
//	u32 IntrId = DMA_CTRL_IRPT_INTR;
//
//	XScuGic_Disable(IntcInstancePtr ,IntrId );
//	XScuGic_Disconnect(IntcInstancePtr ,IntrId );
//
//}
//
//static int prvDmaSetupIntrSystem(XScuGic *IntcInstancePtr, XAxiCdma *InstancePtr,
//			u32 IntrId)
//
//{
//	int Status;
//
//
//	/*
//	 * Initialize the interrupt controller driver
//	 */
//	XScuGic_Config *IntcConfig;
//
//
//	/*
//	 * Initialize the interrupt controller driver so that it is ready to
//	 * use.
//	 */
//	IntcConfig = XScuGic_LookupConfig(DMA_INTC_DEVICE_ID);
//	if (NULL == IntcConfig) {
//		return XST_FAILURE;
//	}
//
//	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
//					IntcConfig->CpuBaseAddress);
//	if (Status != XST_SUCCESS) {
//		return XST_FAILURE;
//	}
//
//	XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId, 0xA0, 0x3);
//////////////////////
//	XScuGic_SetPriorityTriggerType(IntcInstancePtr, XPAR_FABRIC_NEXT_TASK_HANDLER_0_IRQ_INTR, 0xA0, 0x3);
////////////////////
//	/*
//	 * Connect the device driver handler that will be called when an
//	 * interrupt for the device occurs, the handler defined above performs
//	 * the specific interrupt processing for the device.
//	 */
//	Status = XScuGic_Connect(IntcInstancePtr, IntrId,
//				(Xil_InterruptHandler)XAxiCdma_IntrHandler,
//				InstancePtr);
//	if (Status != XST_SUCCESS) {
//		return Status;
//	}
//
//	////////////////////
//	Status = XScuGic_Connect(IntcInstancePtr, XPAR_FABRIC_NEXT_TASK_HANDLER_0_IRQ_INTR,
//				(Xil_InterruptHandler)newTaskHandler,
//				(void *) IntcInstancePtr);
//	if (Status != XST_SUCCESS) {
//		return Status;
//	}
//	////////////////////
//
//	/*
//	 * Enable the interrupt for the DMA device.
//	 */
//	XScuGic_Enable(IntcInstancePtr, IntrId);
//
/////////////
//	XScuGic_Enable(IntcInstancePtr, XPAR_FABRIC_NEXT_TASK_HANDLER_0_IRQ_INTR);
//////////////////
//
//	Xil_ExceptionInit();
//
//	/*
//	 * Connect the interrupt controller interrupt handler to the hardware
//	 * interrupt handling logic in the processor.
//	 */
//	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
//				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
//				IntcInstancePtr);
//
//
//	/*
//	 * Enable interrupts in the Processor.
//	 */
//	Xil_ExceptionEnable();
//
//
//	return XST_SUCCESS;
//}
//
//int prvDmaInit() {
//	int Status=XST_FAILURE;
//	XAxiCdma_Config *CfgPtr;
//
//	XScuGic *IntcInstancePtr = &prvDmaIntcController;
//	XAxiCdma *InstancePtr = &prvDmaAxiCdmaInstance;
//	u16 DeviceId = DMA_CTRL_DEVICE_ID;
//	u32 IntrId = DMA_CTRL_IRPT_INTR;
//
//	/* Initialize the XAxiCdma device.
//	 */
//	CfgPtr = XAxiCdma_LookupConfig(DeviceId);
//	if (!CfgPtr) {
//		xil_printf("init failure");
//		return XST_FAILURE;
//	}
//
//	Status = XAxiCdma_CfgInitialize(InstancePtr, CfgPtr, CfgPtr->BaseAddress);
//	if (Status != XST_SUCCESS) {
//		xil_printf("CfgInitialize failure");
//		xil_printf("status %i", Status);
//
//		return XST_FAILURE;
//	}
//
//	/* Setup the interrupt system
//	 */
//	Status = prvDmaSetupIntrSystem(IntcInstancePtr, InstancePtr, IntrId);
//	if (Status != XST_SUCCESS) {
//		return XST_FAILURE;
//	}
//
//	/* Enable all (completion/error/delay) interrupts
//	 */
//	XAxiCdma_IntrEnable(InstancePtr, XAXICDMA_XR_IRQ_ALL_MASK);
//	return XST_SUCCESS;
//}
//
//static void prvDmaCdma_CallBack(void *CallBackRef, u32 IrqMask, int *IgnorePtr)
//{
//	if (IrqMask & XAXICDMA_XR_IRQ_ERROR_MASK) {
//		prvDmaError = TRUE;
//		xil_printf("\r\n--- Transfer Error --- \r\n");
//	}
//
//	if (IrqMask & XAXICDMA_XR_IRQ_IOC_MASK) {
//		xil_printf("\r\n--- Transfer Done --- \r\n");
//		prvDmaDone = TRUE;
//	}
//}
//
///* Performs a blocking transfer (waits for transfer completion); Retries are the n */
//static int prvDmaCDMABlockingTransfer(XAxiCdma *InstancePtr, int Length, u32 SourceAddr, u32 DestAddr)
//{
//
//	int Status;
//
//	prvDmaDone = 0;
//	prvDmaError = 0;
//
//
//	xil_printf("Start Transfer \n\r");
//	/* Try to start the DMA transfer
//	 */
//
//		/* Flush the SrcBuffer before the DMA transfer, in case the Data Cache
//		 * is enabled
//		 */
//		Xil_DCacheFlushRange((u32)SourceAddr, Length);
//
//		Status = XAxiCdma_SimpleTransfer(InstancePtr,
//										(u32)(SourceAddr),
//										(u32)(DestAddr),
//										Length,
//										prvDmaCdma_CallBack,
//										(void *)InstancePtr);
//
//		if (Status == XST_FAILURE) {
//			xil_printf("Error in Transfer  \n\r");
//			return 1;
//		}
//
//
//
//		/* Wait until the DMA transfer is done
//			 */
//			while (!prvDmaDone && !prvDmaError) {
//				/* Wait */
//				xil_printf("waiting Transfer to finish \n\r");
//			}
//
//			if (prvDmaError) {
//				return XST_FAILURE;
//			}
//		/* Invalidate the DestBuffer before receiving the data, in case the
//		 * Data Cache is enabled
//		 */
//		//Xil_DCacheInvalidateRange((u32)DestAddr, Length); THIS ROW CAUSED TO HANG UP
//
//	return XST_SUCCESS;
//}
//
///*Transfers the task set via DMA to the FPGA and waits for transfer completion. Then scheduler can be started. */
//
//int prvDmaBlockingTransferFreeByteSize( u32 prvDmaDestAddr, u32 prvDmaSourceAddr, u32 byteSize ) {
//	XAxiCdma *InstancePtr=&prvDmaAxiCdmaInstance;
//	int index;
//	int Status=XST_FAILURE;
//	u32 BUFFER_BYTESIZE	= (XPAR_AXI_CDMA_0_M_AXI_DATA_WIDTH * XPAR_AXI_CDMA_0_M_AXI_MAX_BURST_LEN);
//
//	if (byteSize > BUFFER_BYTESIZE) {
//		//int Tries = DMA_NUMBER_OF_TRANSFERS;
//		int bursts = ceil( byteSize / BUFFER_BYTESIZE );
//		u32 remainder = byteSize % BUFFER_BYTESIZE;
//
//		for (index = 0; index < bursts; index++) {
//			Status = prvDmaCDMABlockingTransfer(InstancePtr,
//					(remainder != 0 && index == bursts -1) ? remainder : byteSize, prvDmaSourceAddr, prvDmaDestAddr);
//		}
//	} else {
//		Status = prvDmaCDMABlockingTransfer(InstancePtr,
//				byteSize, (u32) prvDmaSourceAddr, (u32)prvDmaDestAddr);
//	}
//
//	if(Status != XST_SUCCESS) {
//		return XST_FAILURE;
//	}
//	return XST_SUCCESS;
//}
//




//	status=prvDmaInit();
//	if (status!=XST_SUCCESS) {
//		xil_printf("DMA init failed");
//		return status;
//	}
////SINGLE TRANSACTION IF SOURCE ADDRESSES ARE CONTIGUOUS
//	status=prvDmaBlockingTransferFreeByteSize( prvDmaRTTasksListDestAddr, prvRTTasksListPtr, prvRTTasksListByteSize + prvOrderedQueuesByteSize + orderedDeadlineActivationQPayloadByteSize);
//	if (status!=XST_SUCCESS) {
//		xil_printf("DMA RTTTaskSet transfer failed");
//		return status;
//	};
//	//MULTIPLE TRANSACTIONS
///*
//	status=prvDmaBlockingTransferFreeByteSize( prvDmaRTTasksListDestAddr, prvRTTasksListPtr, prvRTTasksListByteSize);
//	if (status!=XST_SUCCESS) {
//		xil_printf("DMA RTTTaskSet transfer failed");
//		return status;
//	};
//
//	status=prvDmaBlockingTransferFreeByteSize( prvDmaRTTasksListDestAddr+prvRTTasksListByteSize, prvOrderedQueuesPtr, prvOrderedQueuesByteSize );
//	if (status!=XST_SUCCESS) {
//		xil_printf("DMA OrderedDeadlineActivationQueue transfer failed");
//		return status;
//	};
//
//	status=prvDmaBlockingTransferFreeByteSize( prvDmaRTTasksListDestAddr+prvRTTasksListByteSize+prvOrderedQueuesByteSize, orderedDeadlineActivationQPayload, orderedDeadlineActivationQPayloadByteSize );
//	if (status!=XST_SUCCESS) {
//		xil_printf("DMA OrderedDeadlineActivationQueue transfer failed");
//		return status;
//	};
//
//*/
//	//TODO CHECK WHETHER DISABLE OR NOT
//	//prvDmaDisableIntrSystem();
//	enableNewTaskInterrupt();
//
//
//	status = XGpio_Initialize(&prvGpio0, GPIO_START_SCHED_DEVICE_ID);
//	if (status != XST_SUCCESS) {
//		xil_printf("Gpio Initialization Failed\r\n");
//		return status;
//	}
//
//	XGpio_SetDataDirection(&prvGpio0, GPIO_START_SCHED_CHANNEL, ~GPIO0VAL);
//
//	prvSchedControl.control=1;
//	prvSchedControl.data = numberOfTasks;
//	prvWriteSchedControl();



//// END OF DMA FUNCTIONS________________________________


static void prvTaskExitError( void )
{
	/* A function that implements a task must not exit or attempt to return to
	its caller as there is nothing to return to.  If a task wants to exit it
	should instead call vTaskDelete( NULL ).

	Artificially force an assert() to be triggered if configASSERT() is
	defined, then stop here so application writers can catch the error. */
	xil_printf("Warning: return statement has been called from task %s, deleting it\n",pcTaskGetName(NULL));
	if (uxTaskGetNumberOfTasks() == 2)
	{
		xil_printf("Warning: Kernel does not have any task to manage other than idle task\n");
	}
	vTaskDelete( NULL );
}
/*-----------------------------------------------------------*/

BaseType_t xPortInstallInterruptHandler( uint8_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef )
{
	int32_t lReturn;

	/* An API function is provided to install an interrupt handler */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
		lReturn = XScuGic_Connect( &xInterruptController, ucInterruptID, pxHandler, pvCallBackRef );
	}
	if( lReturn == XST_SUCCESS )
	{
		lReturn = pdPASS;
	}
	configASSERT( lReturn == pdPASS );

	return lReturn;
}
/*-----------------------------------------------------------*/

static int32_t prvEnsureInterruptControllerIsInitialised( void )
{
	static int32_t lInterruptControllerInitialised = pdFALSE;
	int32_t lReturn;

	/* Ensure the interrupt controller instance variable is initialised before
	it is used, and that the initialisation only happens once. */
	if( lInterruptControllerInitialised != pdTRUE )
	{
		lReturn = prvInitialiseInterruptController();

		if( lReturn == pdPASS )
		{
			lInterruptControllerInitialised = pdTRUE;
		}
	}
	else
	{
		lReturn = pdPASS;
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

static int32_t prvInitialiseInterruptController( void )
{
	BaseType_t xStatus;
	XScuGic_Config *pxGICConfig;

	/* Initialize the interrupt controller driver. */
	pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );
	xStatus = XScuGic_CfgInitialize( &xInterruptController, pxGICConfig, pxGICConfig->CpuBaseAddress );	
	if( xStatus == XST_SUCCESS )
	{
		xStatus = pdPASS;
	}
	else
	{
		xStatus = pdFAIL;
	}
	configASSERT( xStatus == pdPASS );

	return xStatus;
}
/*-----------------------------------------------------------*/

void vPortEnableInterrupt( uint8_t ucInterruptID )
{
	int32_t lReturn;

	/* An API function is provided to enable an interrupt in the interrupt
	controller. */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
		XScuGic_Enable( &xInterruptController, ucInterruptID );
	}	
	configASSERT( lReturn );
}
/*-----------------------------------------------------------*/

void vPortDisableInterrupt( uint8_t ucInterruptID )
{
	int32_t lReturn;

	/* An API function is provided to disable an interrupt in the interrupt
	controller. */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
		XScuGic_Disable( &xInterruptController, ucInterruptID );
	}
	configASSERT( lReturn );
}
/*-----------------------------------------------------------*/


//fedit add
/* Initializes the scheduler. */

#include "xparameters.h"
#include "scheduler.h"
#define SCHEDULER_BASEADDR XPAR_SCHEDULER_0_S_AXI_BASEADDR
//#define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
#define SCHEDULER_INTR XPAR_FABRIC_SCHEDULER_0_IRQ_INTR

typedef struct __attribute__((__packed__)) {
	TCB_t * pxNextTcb;
	char executionMode; //normal, reexecution due to fault, reexecution due to timing fail
} newTaskDescrStr;
//#define PXNEXTTCB 0x10000000
#define NEWTASKDESCRPTR 0x10000000
//static XScuGic intControllerInstance;

TCB_t** pxCurrentTCB_ptr;

#define CPU_BASEADDR		XPAR_SCUGIC_CPU_BASEADDR
#define EXECUTIONMODE_NORMAL 0
#define EXECUTIONMODE_REEXECUTION_TIMING 1
#define EXECUTIONMODE_REEXECUTION_FAULT 2

//typedef struct {
//	u8 checkId;
//	u8 taskId;
//	u16 uniId;
//	char command;
//	float AOV[configMAX_AOV_DIM];
//} FAULTDETECTOR_controlStr;

//char ExecutionMode[configMAX_RT_TASKS];
//OutcomeStr Errors[configMAX_RT_TASKS];
//#define FAULTDETECTOR_CONTROLSTREAM_BASEADDR XPAR_AXI_MM2S_MAPPER_0_BASEADDR
#define FAULTDETECTOR_BASEADDR XPAR_RUN_0_S_AXI_CONTROL_BASEADDR

#define COMMAND_INIT 1
#define COMMAND_TEST 2
#define COMMAND_TRAIN 3

//#include "xrun.h"


XRun FAULTDETECTOR_InstancePtr;
FAULTDETECTOR_controlStr controlForFaultDet;

//#include "xaxidma.h"
//XAxiDma AxiDma;
#define FAULTDETECTOR_DEVICEID XPAR_RUN_0_DEVICE_ID
//#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID

void FAULTDET_getLastError(FAULTDETECTOR_OutcomeStr* dest) {
	FAULTDETECTOR_getLastError(&FAULTDETECTOR_InstancePtr, ((*pxCurrentTCB_ptr)->uxTaskNumber)-1, dest);
}

void FAULTDETECTOR_init(region_t trainedRegions[FAULTDETECTOR_MAX_CHECKS][FAULTDETECTOR_MAX_REGIONS], u8 n_regions[FAULTDETECTOR_MAX_CHECKS]) {
	XRun_Config* configPtr=XRun_LookupConfig(FAULTDETECTOR_DEVICEID);
	XRun_CfgInitialize(&FAULTDETECTOR_InstancePtr, configPtr);
	FAULTDETECTOR_MoveRegions(&FAULTDETECTOR_InstancePtr, trainedRegions);
	FAULTDETECTOR_MoveNRegions(&FAULTDETECTOR_InstancePtr, n_regions);
	XRun_Set_inputAOV(&FAULTDETECTOR_InstancePtr, (u64) (&controlForFaultDet));
	XRun_Set_copyInputAOV(&FAULTDETECTOR_InstancePtr, 0x0);
	XRun_Start(&FAULTDETECTOR_InstancePtr);


//	XAxiDma_Config *CfgPtr;
//	int Status;
//
//	/* Initialize the XAxiDma device.
//	 */
//	CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
////	if (!CfgPtr) {
////		xil_printf("No config found for %d\r\n", DMA_DEV_ID);
////		return XST_FAILURE;
////	}
//
//	Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
////	if (Status != XST_SUCCESS) {
////		xil_printf("Initialization failed %d\r\n", Status);
////		return XST_FAILURE;
////	}
//
//	/* Disable interrupts, we use polling mode
//	 */
//	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
//						XAXIDMA_DEVICE_TO_DMA);
//	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
//						XAXIDMA_DMA_TO_DEVICE);



}

void FAULTDETECTOR_Train(FAULTDETECTOR_controlStr* contr) {
	contr->command=COMMAND_TRAIN;

	while(!FAULTDETECTOR_isReadyForNextControl(&FAULTDETECTOR_InstancePtr)) {}
	controlForFaultDet=*contr;
	FAULTDETECTOR_processNextControl(&FAULTDETECTOR_InstancePtr);
//	while(!FAULTDETECTOR_isReadyForNextControl(&FAULTDETECTOR_InstancePtr)) {
//		xil_printf("notReady");
//	}



//	int Status = XAxiDma_SimpleTransfer(&AxiDma,(UINTPTR) &contr,
//			sizeof(FAULTDETECTOR_controlStr), XAXIDMA_DMA_TO_DEVICE);
//	//xil_printf("status %d", Status);
//	while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) //||
//		//(XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)))
//	{
//			/* Wait */
//	}

}
void FAULTDETECTOR_Test(FAULTDETECTOR_controlStr* contr) {
	contr->command=COMMAND_TEST;

	while(!FAULTDETECTOR_isReadyForNextControl(&FAULTDETECTOR_InstancePtr)) {}
	controlForFaultDet=*contr;
	FAULTDETECTOR_processNextControl(&FAULTDETECTOR_InstancePtr);
//	while(!FAULTDETECTOR_isReadyForNextControl(&FAULTDETECTOR_InstancePtr)) {
//		xil_printf("notReady");
//	}
//	int Status;
//	Status = XAxiDma_SimpleTransfer(&AxiDma,(UINTPTR) &contr,
//				sizeof(FAULTDETECTOR_controlStr), XAXIDMA_DMA_TO_DEVICE);
////	xil_printf("%d", Status);
//	while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) //||
//		//(XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)))
//	{
//			/* Wait */
//	}

}

void initFaultDetection() {
	//int taskId=(*pxCurrentTCB_ptr)->uxTaskNumber;
	if ((*pxCurrentTCB_ptr)->executionMode==EXECUTIONMODE_REEXECUTION_FAULT)
		FAULTDETECTOR_getLastError(&FAULTDETECTOR_InstancePtr, (*pxCurrentTCB_ptr)->uxTaskNumber-1, &((*pxCurrentTCB_ptr)->lastError));
	//Errors[taskId]=copyFromFaultDetector
}
void endFaultDetection() {
	//int taskId=(*pxCurrentTCB_ptr)->uxTaskNumber;
	(*pxCurrentTCB_ptr)->executionMode=EXECUTIONMODE_NORMAL;
}
char isFault() {
	return FAULTDETECTOR_isFault(&FAULTDETECTOR_InstancePtr, ((*pxCurrentTCB_ptr)->uxTaskNumber)-1);
}
void resetFault() {
	FAULTDETECTOR_resetFault(&FAULTDETECTOR_InstancePtr, ((*pxCurrentTCB_ptr)->uxTaskNumber)-1);
}
void readOutcome() {
	FAULTDETECTOR_OutcomeStr out;
	FAULTDETECTOR_getLastError(&FAULTDETECTOR_InstancePtr, (*pxCurrentTCB_ptr)->uxTaskNumber-1, &out);
}

void testPoint(int uniId, int checkId, int argCount, ...) {
	va_list ap;
	va_start(ap, argCount);
	if (argCount>FAULTDETECTOR_MAX_AOV_DIM) //MAX_AOV_DIM
		return; //error

	FAULTDETECTOR_controlStr contr;
	TCB_t* tcbPtr=*pxCurrentTCB_ptr;

	contr.uniId=uniId;
	contr.checkId=checkId;
	//contr.taskId=taskId;
	contr.taskId=tcbPtr->uxTaskNumber-1;

//	for (int i=0; i<argCount; i++) {
//		contr.AOV[i]=*va_arg(ap, float*);
//	}
//	for (int i=argCount; i<FAULTDETECTOR_MAX_AOV_DIM; i++) {
//		contr.AOV[i]=0.0;
//	}
	for (int i=0; i<FAULTDETECTOR_MAX_AOV_DIM; i++) {
		contr.AOV[i]=100.0;
	}

	if (tcbPtr->executionMode==EXECUTIONMODE_REEXECUTION_FAULT && tcbPtr->lastError.uniId==uniId && tcbPtr->lastError.checkId==checkId) {
		tcbPtr->lastError.uniId=-1;
		tcbPtr->lastError.checkId=-1;
		if (memcmp(tcbPtr->lastError.AOV, contr.AOV, sizeof(contr.AOV))==0)
			FAULTDETECTOR_Train(&contr);
		else
			FAULTDETECTOR_Test(&contr);
	} else {
		FAULTDETECTOR_Test(&contr);
	}
	va_end(ap);
}

void trainPoint(int checkId, int argCount, ...) {
	va_list ap;
	va_start(ap, argCount);
	if (argCount>FAULTDETECTOR_MAX_AOV_DIM) //MAX_AOV_DIM
		return; //error

	FAULTDETECTOR_controlStr contr;
	TCB_t* tcbPtr=*pxCurrentTCB_ptr;

	contr.uniId=-1;
	contr.checkId=checkId;
	//contr.taskId=taskId;
	contr.taskId=tcbPtr->uxTaskNumber-1;

	for (int i=0; i<argCount; i++) {
		contr.AOV[i]=*(va_arg(ap, float*));
	}
	for (int i=argCount; i<FAULTDETECTOR_MAX_AOV_DIM; i++) {
		contr.AOV[i]=0.0;
	}

	FAULTDETECTOR_Train(&contr);

	/*if (tcbPtr->executionMode==EXECUTIONMODE_REEXECUTION_FAULT && tcbPtr->lastError.uniId==uniId && tcbPtr->lastError.checkId==checkId) {
		tcbPtr->lastError.uniId=-1;
		tcbPtr->lastError.checkId=-1;
		if (memcmp(tcbPtr->lastError.AOV, contr.AOV, sizeof(contr.AOV))==0)
			FAULTDETECTOR_Train(contr);
		else
			FAULTDETECTOR_Test(contr);
	} else {
		FAULTDETECTOR_Test(contr);
	}*/
	va_end(ap);
}


void xPortScheduleNewTask(void)
{

	newTaskDescrStr* newtaskdesc=(newTaskDescrStr*) NEWTASKDESCRPTR;
	//*pxCurrentTCB_ptr = *((TCB_t**) NEWTASKDESCRPTR);
	*pxCurrentTCB_ptr = newtaskdesc->pxNextTcb;
	xil_printf("| NEW, %X ", *pxCurrentTCB_ptr);
	(*(pxCurrentTCB_ptr))->jobEnded=0;

	if (newtaskdesc->executionMode!=EXECUTIONMODE_NORMAL) {
		//ExecutionMode[(*(pxCurrentTCB_ptr))->uxTCBNumber-1]=newtaskdesc->executionMode;
		//reset to begin
		(*pxCurrentTCB_ptr)->executionMode=newtaskdesc->executionMode;
	}

	if (newtaskdesc->executionMode==EXECUTIONMODE_REEXECUTION_FAULT) {
		//reset to begin
	}

	SCHEDULER_ACKInterrupt((void *) SCHEDULER_BASEADDR);
	/*	xil_printf(" initial SP: %X ", ((*pxCurrentTCB_ptr)->pxStack));
	xil_printf(" SP: %X ", ((*pxCurrentTCB_ptr)->pxTopOfStack));
	xil_printf(" PC address: %X ", ((*pxCurrentTCB_ptr)->pxTopOfStack + 13));
	xil_printf(" PC instr: %X |", *((*pxCurrentTCB_ptr)->pxTopOfStack + 13));*/

}

void xPortSchedulerResumeTask(u16 uxTaskNumber) {
	SCHEDULER_resumeTask((void*) SCHEDULER_BASEADDR, uxTaskNumber);
}

void xPortSchedulerSignalTaskSuspended(u16 uxTaskNumber) {
	SCHEDULER_signalTaskSuspended((void*) SCHEDULER_BASEADDR, uxTaskNumber);
}

void xPortSchedulerSignalJobEnded(u16 uxTaskNumber) {
	SCHEDULER_signalJobEnded((void*) SCHEDULER_BASEADDR, uxTaskNumber);
}

void xPortSchedulerSignalTaskEnded(u16 uxTaskNumber)
{
	SCHEDULER_signalTaskEnded((void*) SCHEDULER_BASEADDR, uxTaskNumber);
}
BaseType_t xPortInitScheduler( u32 numberOfTasks,
		void* tasksTCBPtrs,
		void* tasksWCETs,
		void* tasksDeadlines,
		void* tasksPeriods,
		u32* pxCurrentTCBPtr )
{
	pxCurrentTCB_ptr=(TCB_t **)pxCurrentTCBPtr;
//	int status;

	Xil_DisableMMU();

	SCHEDULER_setNumberOfTasks((void*) SCHEDULER_BASEADDR, (u32) numberOfTasks);

	SCHEDULER_copyTCBPtrs((void*) SCHEDULER_BASEADDR, tasksTCBPtrs);
	SCHEDULER_copyWCETs((void*) SCHEDULER_BASEADDR, tasksWCETs);
	SCHEDULER_copyDeadlines((void*) SCHEDULER_BASEADDR, tasksDeadlines);
	SCHEDULER_copyPeriods((void*) SCHEDULER_BASEADDR, tasksPeriods);

	//	SCHEDULER_copyOrderedDeadlineQIndex((void*) SCHEDULER_BASEADDR, orderedDeadlineQTaskNums);
	//	SCHEDULER_copyOrderedActivationQIndex((void*) SCHEDULER_BASEADDR, orderedActivationQTaskNums);
	//	SCHEDULER_copyOrderedDeadlineQ((void*) SCHEDULER_BASEADDR, orderedDeadlineQPayload);
	//	SCHEDULER_copyOrderedActivationQ((void*) SCHEDULER_BASEADDR, orderedActivationQPayload);
	//	SCHEDULER_copyOrderedDeadlineQReverseIndex((void*) SCHEDULER_BASEADDR, orderedReverseDeadlineQTaskNums);
	//	SCHEDULER_copyOrderedActivationQReverseIndex((void*) SCHEDULER_BASEADDR, orderedReverseActivationQTaskNums);
	//		/*
	//		 * Initialize the interrupt controller driver so that it is ready to
	//		 * use.
	//		 */
	//		XScuGic_Config* intCConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	//		if (intCConfig == NULL) {
	//			return XST_FAILURE;
	//		}
	//
	//		status = XScuGic_CfgInitialize(&intControllerInstance, intCConfig,
	//				intCConfig->CpuBaseAddress);
	//		if (status != XST_SUCCESS) {
	//			return XST_FAILURE;
	//		}

	//		XScuGic_SetPriorityTriggerType(&intControllerInstance, SCHEDULER_INTR, 0xA0, 0x3);
	//		/*
	//		 * Connect the device driver handler that will be called when an
	//		 * interrupt for the device occurs, the handler defined above performs
	//		 * the specific interrupt processing for the device.
	//		 */
	//
	//		status = XScuGic_Connect(&intControllerInstance, SCHEDULER_INTR,
	//					(Xil_InterruptHandler)vPortHandleNewTask,
	//					(void *) &intControllerInstance);
	//		if (status != XST_SUCCESS) {
	//			return status;
	//		}
	//
	//		/*
	//		 * Enable the interrupt for the SCHEDULER device.
	//		 */
	//
	//		XScuGic_Enable(&intControllerInstance, SCHEDULER_INTR);
	//
	//		Xil_ExceptionInit();
	//
	//		/*
	//		 * Connect the interrupt controller interrupt handler to the hardware
	//		 * interrupt handling logic in the processor.
	//		 */
	//		Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
	//					(Xil_ExceptionHandler)XScuGic_InterruptHandler,
	//					&intControllerInstance);


	/*
	 * Enable interrupts in the Processor.
	 */
	//Xil_ExceptionEnable(); In this case, interrupts will be automatically enabled when a new task is scheduled
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_FIQ_INT,
			(Xil_ExceptionHandler) vPortHandleNewTask,
			(void *)CPU_BASEADDR);
	/*
	 * Enable FIQ/IRQ in the ARM
	 */
	//Xil_ExceptionEnableMask(XIL_EXCEPTION_ALL);

	Xil_ExceptionEnable();

	return pdPASS;
}

BaseType_t xPortStartScheduler( void )
{
	uint32_t ulAPSR;

#if( configASSERT_DEFINED == 1 )
	{
		volatile uint32_t ulOriginalPriority;
		volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( configINTERRUPT_CONTROLLER_BASE_ADDRESS + portINTERRUPT_PRIORITY_REGISTER_OFFSET );
		volatile uint8_t ucMaxPriorityValue;

		/* Determine how many priority bits are implemented in the GIC.

		Save the interrupt priority value that is about to be clobbered. */
		ulOriginalPriority = *pucFirstUserPriorityRegister;

		/* Determine the number of priority bits available.  First write to
		all possible bits. */
		*pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

		/* Read the value back to see how many bits stuck. */
		ucMaxPriorityValue = *pucFirstUserPriorityRegister;

		/* Shift to the least significant bits. */
		while( ( ucMaxPriorityValue & portBIT_0_SET ) != portBIT_0_SET )
		{
			ucMaxPriorityValue >>= ( uint8_t ) 0x01;
		}

		/* Sanity check configUNIQUE_INTERRUPT_PRIORITIES matches the read
		value. */
		configASSERT( ucMaxPriorityValue == portLOWEST_INTERRUPT_PRIORITY );

		/* Restore the clobbered interrupt priority register to its original
		value. */
		*pucFirstUserPriorityRegister = ulOriginalPriority;
	}
#endif /* conifgASSERT_DEFINED */


	/* Only continue if the CPU is not in User mode.  The CPU must be in a
	Privileged mode for the scheduler to start. */
	__asm volatile ( "MRS %0, APSR" : "=r" ( ulAPSR ) :: "memory" );
	ulAPSR &= portAPSR_MODE_BITS_MASK;
	configASSERT( ulAPSR != portAPSR_USER_MODE );

	if( ulAPSR != portAPSR_USER_MODE )
	{
		/* Only continue if the binary point value is set to its lowest possible
		setting.  See the comments in vPortValidateInterruptPriority() below for
		more information. */
		configASSERT( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE );

		if( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE )
		{
			/* Interrupts are turned off in the CPU itself to ensure tick does
			not execute	while the scheduler is being started.  Interrupts are
			automatically turned back on in the CPU when the first task starts
			executing. */
			portCPU_IRQ_DISABLE();

			/* Start the timer that generates the tick ISR. */
			//fedit remove: DISABLED TICKS INTERRUPT initialisation at scheduler startup which triggers periodic scheduler routine - in port.c since architecture specific: interrupts for context switch will be generated by FPGA, we don't want CPU tick interrupt
			//configSETUP_TICK_INTERRUPT();

			/* Start the scheduler on hardware */
			//			prvSchedControl.control=2;
			//			prvSchedControl.data = 0;
			//			prvWriteSchedControl();
			SCHEDULER_EnableInterrupt((void*) SCHEDULER_BASEADDR);
			SCHEDULER_start((void*) SCHEDULER_BASEADDR);
			/* Start the first task executing. */
			vPortRestoreTaskContext();
		}
	}

	/* Will only get here if vTaskStartScheduler() was called with the CPU in
	a non-privileged mode or the binary point register was not set to its lowest
	possible value.  prvTaskExitError() is referenced to prevent a compiler
	warning about it being defined but not referenced in the case that the user
	defines their own exit address. */
	( void ) prvTaskExitError;
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Not implemented in ports where there is nothing to return to.
	Artificially force an assert. */
	SCHEDULER_stop((void*) SCHEDULER_BASEADDR);
	SCHEDULER_DisableInterrupt((void*) SCHEDULER_BASEADDR);
	configASSERT( ulCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	/* Mask interrupts up to the max syscall interrupt priority. */
	ulPortSetInterruptMask();

	/* Now interrupts are disabled ulCriticalNesting can be accessed
	directly.  Increment ulCriticalNesting to keep a count of how many times
	portENTER_CRITICAL() has been called. */
	ulCriticalNesting++;

	/* This is not the interrupt safe version of the enter critical function so
	assert() if it is being called from an interrupt context.  Only API
	functions that end in "FromISR" can be used in an interrupt.  Only assert if
	the critical nesting count is 1 to protect against recursive calls if the
	assert function also uses a critical section. */
	if( ulCriticalNesting == 1 )
	{
		configASSERT( ulPortInterruptNesting == 0 );
	}
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		/* Decrement the nesting count as the critical section is being
		exited. */
		ulCriticalNesting--;

		/* If the nesting level has reached zero then all interrupt
		priorities must be re-enabled. */
		if( ulCriticalNesting == portNO_CRITICAL_NESTING )
		{
			/* Critical nesting has reached zero so all interrupt priorities
			should be unmasked. */
			portCLEAR_INTERRUPT_MASK();
		}
	}
}
/*-----------------------------------------------------------*/

void FreeRTOS_Tick_Handler( void )
{
	//	/*
	//	 * The Xilinx implementation of generating run time task stats uses the same timer used for generating
	//	 * FreeRTOS ticks. In case user decides to generate run time stats the tick handler is called more
	//	 * frequently (10 times faster). The timer/tick handler uses logic to handle the same. It handles
	//	 * the FreeRTOS tick once per 10 interrupts.
	//	 * For handling generation of run time stats, it increments a pre-defined counter every time the
	//	 * interrupt handler executes.
	//	 */
	//#if (configGENERATE_RUN_TIME_STATS == 1)
	//	ulHighFrequencyTimerTicks++;
	//	if (!(ulHighFrequencyTimerTicks % 10))
	//#endif
	//	{
	//	/* Set interrupt mask before altering scheduler structures.   The tick
	//	handler runs at the lowest priority, so interrupts cannot already be masked,
	//	so there is no need to save and restore the current mask value.  It is
	//	necessary to turn off interrupts in the CPU itself while the ICCPMR is being
	//	updated. */
	//	portCPU_IRQ_DISABLE();
	//	portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
	//	__asm volatile (	"dsb		\n"
	//						"isb		\n" ::: "memory" );
	//	portCPU_IRQ_ENABLE();
	//
	//	/* Increment the RTOS tick. */
	//	if( xTaskIncrementTick() != pdFALSE )
	//	{
	//		ulPortYieldRequired = pdTRUE;
	//	}
	//	}
	//
	//	/* Ensure all interrupt priorities are active again. */
	//	portCLEAR_INTERRUPT_MASK();
	//	configCLEAR_TICK_INTERRUPT();
}
/*-----------------------------------------------------------*/

#if( configUSE_TASK_FPU_SUPPORT != 2 )

void vPortTaskUsesFPU( void )
{
	uint32_t ulInitialFPSCR = 0;

	/* A task is registering the fact that it needs an FPU context.  Set the
		FPU flag (which is saved as part of the task context). */
	ulPortTaskHasFPUContext = pdTRUE;

	/* Initialise the floating point status register. */
	__asm volatile ( "FMXR 	FPSCR, %0" :: "r" (ulInitialFPSCR) : "memory" );
}

#endif /* configUSE_TASK_FPU_SUPPORT */
/*-----------------------------------------------------------*/

void vPortClearInterruptMask( uint32_t ulNewMaskValue )
{
	if( ulNewMaskValue == pdFALSE )
	{
		portCLEAR_INTERRUPT_MASK();
	}
}
/*-----------------------------------------------------------*/

uint32_t ulPortSetInterruptMask( void )
{
	uint32_t ulReturn;

	/* Interrupt in the CPU must be turned off while the ICCPMR is being
	updated. */
	portCPU_IRQ_DISABLE();
	if( portICCPMR_PRIORITY_MASK_REGISTER == ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT ) )
	{
		/* Interrupts were already masked. */
		ulReturn = pdTRUE;
	}
	else
	{
		ulReturn = pdFALSE;
		portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
		__asm volatile (	"dsb		\n"
				"isb		\n" ::: "memory" );
	}
	portCPU_IRQ_ENABLE();

	return ulReturn;
}
/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

void vPortValidateInterruptPriority( void )
{
	/* The following assertion will fail if a service routine (ISR) for
		an interrupt that has been assigned a priority above
		configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
		function.  ISR safe FreeRTOS API functions must *only* be called
		from interrupts that have been assigned a priority at or below
		configMAX_SYSCALL_INTERRUPT_PRIORITY.

		Numerically low interrupt priority numbers represent logically high
		interrupt priorities, therefore the priority of the interrupt must
		be set to a value equal to or numerically *higher* than
		configMAX_SYSCALL_INTERRUPT_PRIORITY.

		FreeRTOS maintains separate thread and ISR API functions to ensure
		interrupt entry is as fast and simple as possible. */
	configASSERT( portICCRPR_RUNNING_PRIORITY_REGISTER >= ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT ) );

	/* Priority grouping:  The interrupt controller (GIC) allows the bits
		that define each interrupt's priority to be split between bits that
		define the interrupt's pre-emption priority bits and bits that define
		the interrupt's sub-priority.  For simplicity all bits must be defined
		to be pre-emption priority bits.  The following assertion will fail if
		this is not the case (if some bits represent a sub-priority).

		The priority grouping is configured by the GIC's binary point register
		(ICCBPR).  Writing 0 to ICCBPR will ensure it is set to its lowest
		possible value (which may be above 0). */
	configASSERT( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE );
}

#endif /* configASSERT_DEFINED */
/*-----------------------------------------------------------*/

void vApplicationFPUSafeIRQHandler( uint32_t ulICCIAR )
{
	( void ) ulICCIAR;
	configASSERT( ( volatile void * ) NULL );
}

#if( configGENERATE_RUN_TIME_STATS == 1 )
/*
 * For Xilinx implementation this is a dummy function that does a redundant operation
 * of zeroing out the global counter.
 * It is called by FreeRTOS kernel.
 */
void xCONFIGURE_TIMER_FOR_RUN_TIME_STATS (void)
{
	ulHighFrequencyTimerTicks = 0;
}
/*
 * For Xilinx implementation this function returns the global counter used for
 * run time task stats calculation.
 * It is called by FreeRTOS kernel task handling logic.
 */
uint32_t xGET_RUN_TIME_COUNTER_VALUE (void)
{
	return ulHighFrequencyTimerTicks;
}
#endif
