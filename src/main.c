/*
    Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
    Copyright (c) 2012 - 2022 Xilinx, Inc. All Rights Reserved.
	SPDX-License-Identifier: MIT


    http://www.FreeRTOS.org
    http://aws.amazon.com/freertos


    1 tab == 4 spaces!
 */

/* FreeRTOS includes. */
//#include "xil_printf.h"

//#define testingCampaign
//#define FAULTDETECTOR_EXECINSW
//#define trainMode
#define onOutputOnly

#include "FreeRTOS.h"
#include "task.h"
/* Xilinx includes. */

#include "portable.h"

#include "perf_timer.h"

//#define imgscalingBench
#define latnavBench
//
//#ifdef trainMode
#ifdef imgscalingBench

#define IMG_HEIGHT 16
#define IMG_WIDTH 16
static unsigned char mat_in[IMG_HEIGHT][IMG_WIDTH];
unsigned char mat_out[IMG_HEIGHT][IMG_WIDTH];
#endif

#include "simple_random.h"

//#else
//#include "scaling_image.h"
//#endif
/*-----------------------------------------------------------*/

#include <math.h>
static void prvTaskOne( void *pvParameters );
static void prvTaskTwo( void *pvParameters );
static void prvTaskThree( void *pvParameters );
static void prvTaskFour( void *pvParameters );
//static void prvImgAcquireAndScaleTask( void *pvParameters );
#include <stdio.h>


static FAULTDETECTOR_region_t trainedRegions[FAULTDETECTOR_MAX_CHECKS][FAULTDETECTOR_MAX_REGIONS];
static u8 n_regions[FAULTDETECTOR_MAX_CHECKS];

void printBits(size_t const size, void const * const ptr)
{
	printf(" ");
	unsigned char *b = (unsigned char*) ptr;
	unsigned char byte;
	int i, j;

	for (i = size-1; i >= 0; i--) {
		for (j = 7; j >= 0; j--) {
			byte = (b[i] >> j) & 1;
			printf("%u", byte);
		}
	}
	puts("");
	printf(" ");
}


int main( void )
{
	//		xRTTaskCreate( prvTaskOne,
	//				( const char * ) "One",
	//				configMINIMAL_STACK_SIZE,
	//				NULL,
	//				tskIDLE_PRIORITY,
	//				NULL,
	//				50000000, //deadline
	//				50000000, //period
	//				40000000); //wcet
	//
	//		xRTTaskCreate( prvTaskTwo,
	//				( const char * ) "Two",
	//				configMINIMAL_STACK_SIZE,
	//				NULL,
	//				tskIDLE_PRIORITY,
	//				NULL,
	//				100000000, //deadline
	//				100000000, //period
	//				40000000); //wcet
	//
	//		xRTTaskCreate( prvTaskThree,
	//				( const char * ) "Three",
	//				configMINIMAL_STACK_SIZE,
	//				NULL,
	//				tskIDLE_PRIORITY,
	//				NULL,
	//				150000000, //deadline
	//				150000000, //period
	//				40000000); //wcet


	//	xRTTaskCreate( prvImgAcquireAndScaleTask,
	//			( const char * ) "Four",
	//			configMINIMAL_STACK_SIZE,
	//			NULL,
	//			tskIDLE_PRIORITY,
	//			NULL,
	//			999999997, //deadline
	//			999999997, //period
	//			999999997); //wcet

	//	for (int i=0; i<8096; i++) {
	//		testtt[i]=i;
	//	}

	xRTTaskCreate( prvTaskFour,
			( const char * ) "Four",
			configMINIMAL_STACK_SIZE,
			NULL,
			tskIDLE_PRIORITY,
			NULL,
			999999997, //deadline
			999999997, //period
			999999997); //wcet

	for (int i=0; i<FAULTDETECTOR_MAX_CHECKS; i++) {
		n_regions[i]=0;
		for (int j=0; j<FAULTDETECTOR_MAX_REGIONS; j++) {
			for (int k=0; k<=FAULTDETECTOR_MAX_AOV_DIM; k++) {
				trainedRegions[i][j].center[k]=0.0f;
				trainedRegions[i][j].max[k]=0.0f;
				trainedRegions[i][j].min[k]=0.0f;

//				if (j>=0 && j<=2) {
//					if (k>=0 && k<1) {
//					trainedRegions[i][j].center[k]=0.0f;
//					trainedRegions[i][j].max[k]=1000.0f;
//					trainedRegions[i][j].min[k]=-1000.0f;
//					} else if ( k<=2 ) {
//						trainedRegions[i][j].center[k]=1000.0f;
//						trainedRegions[i][j].max[k]=1000.0f;
//						trainedRegions[i][j].min[k]=1000.0f;
//					}
//				} else if (j>=3 && j<=5) {
//					if (k>=0 && k<=)
//					trainedRegions[i][j].center[k]=0.0f;
//					trainedRegions[i][j].max[k]=1000.0f;
//					trainedRegions[i][j].min[k]=-1000.0f;
//				} else if (j==6) {
//					trainedRegions[i][j].center[k]=0.0f;
//					trainedRegions[i][j].max[k]=1000.0f;
//					trainedRegions[i][j].min[k]=-1000.0f;
//				}

			}
		}

	}

	vTaskStartFaultDetector(
			//#ifdef trainMode
			//			0, //do not load from sd, load from supplied trainedRegions and n_regions instead
			//#else
			//			1,
			//#endif
			0,
			trainedRegions,
			n_regions);


	vTaskStartScheduler();

	//should never reach this point
	for( ;; );
}


/*-----------------------------------------------------------*/
static void prvTaskOne( void *pvParameters )
{
	for (;;) {
		printf(" One ");

		vTaskJobEnd();
	}
}

/*-----------------------------------------------------------*/
static void prvTaskTwo( void *pvParameters )
{
	for (;;) {
		printf(" Two ");

		vTaskJobEnd();
	}
}

static void prvTaskThree( void *pvParameters )
{
	for (;;) {
		printf(" Three ");

		vTaskJobEnd();
	}
}


u8 injectingErrors=0;
float r1, r2, r3, r4;



#ifdef latnavBench

#define TIME_STEP 0.001

#define CLAMP(x, minim, maxim) (x < minim ? minim : (x > maxim ? maxim : x))

#define ITERATIONS 100


typedef struct pid_controller_s {

	float p;
	float i;
	float d;
	float b;

	float prev_error;
	float integral_sum;
	float backpropagation;

} pid_controller_t;



float run_pid(pid_controller_t* pid, float error, int executionId) {
	float output;
#ifdef testingCampaign
	FAULTDET_testing_injectFault32(pid->p, executionId, 32*0, (32*1)-1, injectingErrors);
	FAULTDET_testing_injectFault32(pid->i, executionId, 32*1, (32*2)-1, injectingErrors);
	FAULTDET_testing_injectFault32(pid->d, executionId, 32*2, (32*3)-1, injectingErrors);
	FAULTDET_testing_injectFault32(pid->b, executionId, 32*3, (32*4)-1, injectingErrors);
	FAULTDET_testing_injectFault32(pid->prev_error, executionId, 32*4, (32*5)-1, injectingErrors);
	FAULTDET_testing_injectFault32(pid->integral_sum, executionId, 32*5, (32*6)-1, injectingErrors);
	FAULTDET_testing_injectFault32(pid->backpropagation, executionId, 32*6, (32*7)-1, injectingErrors);
#endif
	output = error * pid->p;
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(output, executionId, 32*7, (32*8)-1, injectingErrors);
#endif
	pid->integral_sum += ((error * pid->i) + (pid->b * pid->backpropagation)) * TIME_STEP;
#ifdef testingCampaign
	FAULTDET_testing_injectFault32(pid->integral_sum, executionId, 32*8, (32*9)-1, injectingErrors);
#endif
	output += pid->integral_sum;
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(output, executionId, 32*9, (32*10)-1, injectingErrors);
#endif
	output += pid->d * (error - pid->prev_error) / TIME_STEP;
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(output, executionId, 32*10, (32*11)-1, injectingErrors);
#endif
	pid->prev_error = error;

	return output;
}

float roll_limiter(float desired_roll, float speed, int executionId) {
	float limit_perc,limit;
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(speed, executionId, 32*0, (32*1)-1, injectingErrors);
	FAULTDET_testing_injectFault32(desired_roll, executionId, 32*4, (32*5)-1, injectingErrors);
#endif
	if (speed <= 140) {
		return CLAMP(desired_roll, -30, 30);
	}
	if (speed >= 300) {
		return CLAMP(desired_roll, -40, 40);
	}
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(speed, executionId, 32*1, (32*2)-1, injectingErrors);
#endif
	limit_perc = (speed < 220) ? (speed-140) / 80 : ((speed-220) / 80);
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(limit_perc, executionId, 32*2, (32*3)-1, injectingErrors);
#endif
	limit = (speed < 220) ? (30 + limit_perc * 37) : (40 + (1-limit_perc) * 27);
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(limit, executionId, 32*3, (32*4)-1, injectingErrors);
#endif
	return CLAMP (desired_roll, -limit, limit);
}

float roll_rate_limiter(float desired_roll_rate, float roll, int executionId) {
#ifdef testingCampaign
	FAULTDET_testing_injectFault32(roll, executionId, 32*0, (32*1)-1, injectingErrors);
	FAULTDET_testing_injectFault32(desired_roll_rate, executionId, 32*1, (32*2)-1, injectingErrors);
#endif

	if (roll < 20 && roll > -20) {
		return CLAMP (desired_roll_rate, -5, 5);
	} else if (roll < 30 && roll > -30) {
		return CLAMP (desired_roll_rate, -3, 3);
	} else {
		return CLAMP (desired_roll_rate, -1, 1);
	}
}

float ailerons_limiter(float aileron, int executionId) {
#ifdef testingCampaign

	FAULTDET_testing_injectFault32(aileron, executionId, 32*0, (32*1)-1, injectingErrors);
#endif
	return CLAMP(aileron, -30, 30);
}

float r1, r2, r3, r4;

volatile unsigned long long int clk_count_bench_total=0;
volatile unsigned int clk_count_bench_total_times=0;

char first=0xFF;

void latnav(int roundId, int executionId) {

#ifndef FAULTDETECTOR_EXECINSW
	FAULTDET_ExecutionDescriptor inst;
	FAULTDET_initFaultDetection(&inst);


	inst.lastTest.checkId = first ? 0:6;
	inst.lastTest.executionId=0;
	inst.lastTest.uniId= first ? 0:1;
	inst.testedOnce=0xFF;
	first=0;
#endif

	perf_reset_and_start_clock();


	pid_controller_t pid_roll_rate,pid_roll,pid_heading;
	float curr_heading,curr_roll,curr_roll_rate;
	int i;


	pid_roll_rate.p = 0.31;
	pid_roll_rate.i = 0.25;
	pid_roll_rate.d = 0.46;
	pid_roll_rate.b = 0.15;


	pid_roll.p = 0.81;
	pid_roll.i = 0.63;
	pid_roll.d = 0.39;
	pid_roll.b = 0.42;

	pid_heading.p = 0.15;
	pid_heading.i = 0.64;
	pid_heading.d = 0.33;
	pid_heading.b = 0.55;

	pid_roll_rate.integral_sum= pid_roll_rate.prev_error= pid_roll_rate.backpropagation= 0;
	pid_roll.integral_sum     = pid_roll.prev_error     = pid_roll.backpropagation     = 0;
	pid_heading.integral_sum  = pid_heading.prev_error  = pid_heading.backpropagation  = 0;


	curr_heading = r1;
	curr_roll = r2;
	curr_roll_rate = r3;




	for(i=0; i<1; i++) {

		float desired_roll,actual_roll,desired_roll_rate,actual_roll_rate,desired_ailerons,actual_ailerons;

		float err=curr_heading-r4;
		float pid_heading_backpropagation_orig=pid_heading.backpropagation;
		float err_orig=err;

		desired_roll = run_pid(&pid_heading, err, executionId);

#ifndef FAULTDETECTOR_EXECINSW
		FAULTDET_blockIfFaultDetectedInTask(&inst);
#endif

		if (executionId<-1) {
			FAULTDET_trainPoint(
					1,
					0,  //checkId
					3,
					//					/*&(pid_heading.b),*/ &(pid_heading_backpropagation_orig), /*&(pid_heading.d), &(pid_heading.i), &(pid_heading.p),*/ /*&(pid_heading.prev_error),*/ &err_orig, &desired_roll);//, &actual_roll);
					/*&(pid_heading.b),*/ &(pid_heading.backpropagation), /*&(pid_heading.d), &(pid_heading.i), &(pid_heading.p),*/ /*&(pid_heading.prev_error),*/ &err, &desired_roll);//, &actual_roll);

		} else {
			FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
					&inst,
#endif
					1, //uniId
					0, //checkId
					0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
					injectingErrors,
					2,
					2,
					roundId,
					executionId,
#endif
					3, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
					//					/*&(pid_heading.b),*/ &(pid_heading_backpropagation_orig), /*&(pid_heading.d), &(pid_heading.i), &(pid_heading.p),*/ /*&(pid_heading.prev_error),*/ &err_orig, &desired_roll);//, &actual_roll);
					/*&(pid_heading.b),*/ &(pid_heading.backpropagation), /*&(pid_heading.d), &(pid_heading.i), &(pid_heading.p),*/ /*&(pid_heading.prev_error),*/ &err, &desired_roll);//, &actual_roll);
		}

		actual_roll = roll_limiter(desired_roll, 400, executionId-(32*11));


		if (executionId<-1) {
			FAULTDET_trainPoint(
					1,
					3,  //checkId
					2,
					&desired_roll, &actual_roll);
		} else {
			FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
					&inst,
#endif
					1, //uniId
					3, //checkId
					0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
					injectingErrors,
					1,
					1,
					roundId,
					executionId,
#endif
					2, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
					&desired_roll, &actual_roll);
		}



		pid_heading.backpropagation = actual_roll - desired_roll;

		float pid_roll_backpropagation_orig=pid_roll.backpropagation;

		float err1=curr_roll - actual_roll;
		float err1_orig=curr_roll - actual_roll;
		desired_roll_rate = run_pid(&pid_roll, err1, executionId-(32*11)-(32*5));

		if (executionId<-1) {
			FAULTDET_trainPoint(
					1,
					1,  //ceckId
					3,
					//					/*&(pid_roll.b),*/ &(pid_roll_backpropagation_orig), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err1_orig, &desired_roll_rate);//, &actual_roll_rate);
					/*&(pid_roll.b),*/ &(pid_roll.backpropagation), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err1, &desired_roll_rate);//, &actual_roll_rate);

		} else {
			FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
					&inst,
#endif
					1, //uniId
					1, //checkId
					0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
					injectingErrors,
					2,
					2,
					roundId,
					executionId,
#endif
					3, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
					//					/*&(pid_roll.b),*/ &(pid_roll_backpropagation_orig), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err1_orig, &desired_roll_rate);//, &actual_roll_rate);
					/*&(pid_roll.b),*/ &(pid_roll.backpropagation), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err1, &desired_roll_rate);//, &actual_roll_rate);
		}

		actual_roll_rate = roll_rate_limiter(desired_roll_rate, curr_roll, executionId-(32*11)-(32*5)-(32*11));

		if (executionId<-1) {
			FAULTDET_trainPoint(
					1,
					4,  //checkId
					3,
					&actual_roll_rate, &curr_roll, &desired_roll_rate);
		} else {
			FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
					&inst,
#endif
					1, //uniId
					4, //checkId
					0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
					injectingErrors,
					0,
					0,
					roundId,
					executionId,
#endif
					3, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
					&actual_roll_rate, &curr_roll, &desired_roll_rate);
		}

		pid_roll.backpropagation = actual_roll_rate - desired_roll_rate;
		pid_roll_backpropagation_orig=pid_roll.backpropagation;

		float err2=curr_roll_rate - actual_roll_rate;
		float err2_orig=err2;
		desired_ailerons = run_pid(&pid_roll, err2, executionId-(32*11)-(32*5)-(32*11)-(32*2));

		if (executionId<-1) {
			FAULTDET_trainPoint(
					1,
					2,  //checkId
					3,
					//					/*&(pid_roll.b),*/ &(pid_roll_backpropagation_orig), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err2_orig, &desired_ailerons);//, &actual_ailerons);
					/*&(pid_roll.b),*/ &(pid_roll.backpropagation), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err2, &desired_ailerons);//, &actual_ailerons);

		} else {
			FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
					&inst,
#endif
					1, //uniId
					2, //checkId
					0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
					injectingErrors,
					2,
					2,
					roundId,
					executionId,
#endif
					3, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
					//					/*&(pid_roll.b),*/ &(pid_roll_backpropagation_orig), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err2_orig, &desired_ailerons);//, &actual_ailerons);
					/*&(pid_roll.b),*/ &(pid_roll.backpropagation), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ /*&(pid_roll.prev_error),*/ &err2, &desired_ailerons);//, &actual_ailerons);
		}
		actual_ailerons = ailerons_limiter(desired_ailerons, executionId-(32*11)-(32*5)-(32*11)-(32*2)-(32*11));

		if (executionId<-1) {
			FAULTDET_trainPoint(
					1,
					5,  //checkId
					2,
					&desired_ailerons, &actual_ailerons);
		} else {
			FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
					&inst,
#endif
					1, //uniId
					5, //checkId
					0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
					injectingErrors,
					1,
					1,
					roundId,
					executionId,
#endif
					2, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
					&desired_ailerons, &actual_ailerons);
		}

		pid_roll.backpropagation = actual_ailerons - desired_ailerons;


		if (executionId<-1) {
			FAULTDET_trainPoint(
					1,
					6,  //checkId
					4,
					/*&(pid_roll.b),*/ &(curr_heading), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ &(curr_roll), &curr_roll_rate, &actual_ailerons);
		}  else {
			FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
					&inst,
#endif
					1, //uniId
					6, //checkId
					0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
					injectingErrors,
					3,
					3,
					roundId,
					executionId,
#endif
					4, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
					/*&(pid_roll.b),*/ &(curr_heading), /*&(pid_roll.d), &(pid_roll.i), &(pid_roll.p),*/ &(curr_roll), &curr_roll_rate, &actual_ailerons);
		}



		/* Just a random plane model*/
		//		FAULTDET_testing_injectFault32(curr_roll, executionId, (32*11)+(32*5)+(32*11)+(32*2)+(32*11)+(32*1), (32*11)+(32*5)+(32*11)+(32*2)+(32*11)+(32*1)+(32)-1, injectingErrors);
		//		FAULTDET_testing_injectFault32(curr_roll_rate, executionId, (32*11)+(32*5)+(32*11)+(32*2)+(32*11)+(32*1)+(32), (32*11)+(32*5)+(32*11)+(32*2)+(32*11)+(32*1)+(32)+(32)-1,  injectingErrors);
		//		FAULTDET_testing_injectFault32(desired_ailerons, executionId, (32*11)+(32*5)+(32*11)+(32*2)+(32*11)+(32*1)+(32)+(32), (32*11)+(32*5)+(32*11)+(32*2)+(32*11)+(32*1)+(32)+(32)+(32)-1,  injectingErrors);

		//				curr_heading += curr_roll/10 * TIME_STEP;
		//				curr_roll += curr_roll_rate * TIME_STEP;
		//				curr_roll_rate += desired_ailerons / 5;


		//		FAULTDET_testing_injectFault32(curr_heading, executionId, 1408, 1408+32-1, injectingErrors);
		//		FAULTDET_testing_injectFault32(curr_roll, executionId, 1408+32, 1408+32+32-1,  injectingErrors);
		//		FAULTDET_testing_injectFault32(curr_roll_rate, executionId, 1408+32+32, 1408+32+32+32-1 /*1503*/,  injectingErrors);

	}
	perf_stop_clock();
	if (executionId>=-1) {
		clk_count_bench_total+=get_clock_L();
		clk_count_bench_total_times++;
		if (get_clock_U()!=0)
			printf("err up not 0");
	}

#ifdef testingCampaign
	if (executionId>=-1) {
		FAULTDET_testing_commitTmpStatsAndReset(injectingErrors);
	}

	if (executionId==-1) {

		injectingErrors=0xFF;
	}

#endif
}


#ifdef imgscalingBench
//SOURCED FROM ABSURD BENCHMARK SUITE BY HEAPLAB - POLIMI
#define SCALING_FACTOR 2

static float hermit_poly(float p_0, float p_1, float p_2, float p_3, float x){
	float a,b,c,d;

	a = -p_0/2 + (3 *p_1)/2 - (3 *p_2)/2 + p_3/2;
	b = p_0 - (5 *p_1)/2 + 2*p_2 - p_3/2;
	c = - p_0/2 +p_2/2;
	d = p_1;

	return  a*x*x*x + b*x*x +c*x+d;
}

static unsigned int get_pixel(int x, int y){
	return mat_in[x][y];
}

void imgScaling(int executionId) {
#ifndef FAULTDETECTOR_EXECINSW
	FAULTDET_ExecutionDescriptor inst;
	FAULTDET_initFaultDetection(&inst);
#endif

	//			float hour=get_hour_of_day();
	int i,j;

	//#ifdef trainMode
	if (executionId<=-1) {
		//				if (executionId==-1)
		//					random_set_seed(512);
		//				else
		random_set_seed(-executionId);
		for (i = 0; i < IMG_HEIGHT; i++){
			for (j = 0; j < IMG_WIDTH; j++){
				mat_in[i][j]=random_get()*256;
			}
		}
	}
	//#endif


	//				#endif
	//				perf_reset_and_start_timer();
	//				unsigned int clk_l=get_clock_L();
	//				unsigned int clk_hi=get_clock_U();
	//				printf("initial value %x%x", clk_hi, clk_l);
	//printf(" initial time, lsbits only %d", get_delay_for_number_of_cycles(clk_l));


	unsigned int ctr=0;



	for(i=0;i<SCALING_FACTOR*IMG_HEIGHT;i++){
		for(j=0;j<SCALING_FACTOR*IMG_WIDTH;j++){
			int x,y;
			float dx,dy;

			x = i/SCALING_FACTOR;
			y = j/SCALING_FACTOR;
			dx=i/SCALING_FACTOR-x;
			dy=j/SCALING_FACTOR-y;

			float col0,col1,col2,col3,res;
			static unsigned int i0, i1, i2, i3;//, accum;
			float i0f, i1f, i2f, i3f;//, accum;
			float df;

			i0=get_pixel(x-1,y-1);
			i1=get_pixel(x,y-1);
			i2=get_pixel(x+1,y-1);
			i3=get_pixel(x+2,y-1);



#ifdef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;

#endif

			//#ifndef trainMode

			FAULTDET_testing_injectFault32(i0, executionId, 0, 31, injectingErrors);
			FAULTDET_testing_injectFault32(i1, executionId, 32, 63, injectingErrors);
			FAULTDET_testing_injectFault32(i2, executionId, 64, 95, injectingErrors);
			FAULTDET_testing_injectFault32(i3, executionId, 96, 127, injectingErrors);

			//						FAULTDET_testing_injectFault32(col0, executionId, 128, 159, injectingErrors);
			FAULTDET_testing_injectFault32(dx, executionId, 160, 191, injectingErrors);
			//						#endif

#ifndef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;
#endif



			//				accum=i0+i1+i2+i3;

			//				/*check if the sum of the colours of 4 adiacent pixels
			//				 is not uncommon for the hour of the day and the coordinates in the image, assuming camera
			//				 is fixed and working in a super secure room with restricted access,
			//				 so people won't usually enter there*/
			//				FAULTDET_testPoint(&inst,
			//						uniIdBase+1, //uniId from 1 to 65534 (included)
			//						0, //checkId
			//						0, /*non blocking.
			//										BLOCKING OR NON BLOCKING:
			//										if BLOCKING, FAULTDET_blockIfFaultDetectedInTask (see later for details) is called if a test is performed,
			//										otherwise, after writing the controlStr to RAM and informing the fault detector
			//										that a new controlStr is available, the function returns.*/
			//						4, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM, unused elements will be initialised to 0)
			//						&accum, &x, &y, &hour); //values of AOV
			col0=hermit_poly(i0,i1,i2,i3,dx);
			//#ifdef trainMode
			if (executionId<-1) {
				FAULTDET_trainPoint(ctr*5+1, 0, 6, &i0f, &i1f, &i2f, &i3f, &col0, &dx);
				//#else
			} else {
				FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
						&inst,
#endif
						ctr*5+1, //uniId
						0, //checkId
						0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
						injectingErrors,
#endif
						6, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
						&i0f, &i1f, &i2f, &i3f, &col0, &df);
				//#endif
			}
			i0=get_pixel(x-1,y);
			i1=get_pixel(x,y);
			i2=get_pixel(x+1,y);
			i3=get_pixel(x+2,y);



#ifdef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;
#endif
			//#ifndef trainMode
			FAULTDET_testing_injectFault32(i0, executionId, 192, 223, injectingErrors);
			FAULTDET_testing_injectFault32(i1, executionId, 224, 255, injectingErrors);
			FAULTDET_testing_injectFault32(i2, executionId, 256, 287, injectingErrors);
			FAULTDET_testing_injectFault32(i3, executionId, 288, 319, injectingErrors);

			//						FAULTDET_testing_injectFault32(col1, executionId, 320, 351, injectingErrors);
			FAULTDET_testing_injectFault32(dx, executionId, 352, 383, injectingErrors);
			//#endif
#ifndef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;

#endif


			//				accum=i0+i1+i2+i3;
			//				FAULTDET_testPoint(&inst,
			//						uniIdBase+2, //uniId
			//						0, //checkId
			//						0, //non blocking.
			//						4, //SIZE OF THIS SPECIFIC AOV
			//						&accum, &x, &y, &hour); //values of AOV
			col1=hermit_poly(i0,i1,i2,i3,dx);

			//#ifdef trainMode
			if (executionId<-1) {
				FAULTDET_trainPoint(ctr*5+2, 0, 6, &i0f, &i1f, &i2f, &i3f, &col1, &dx);
				//#else
			} else {
				FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
						&inst,
#endif
						ctr*5+2, //uniId
						0, //checkId
						0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
						injectingErrors,
#endif
						6, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
						&i0f, &i1f, &i2f, &i3f, &col1, &df);
				//#endif
			}
			i0=get_pixel(x-1,y+1);
			i1=get_pixel(x,y+1);
			i2=get_pixel(x+1,y+1);
			i3=get_pixel(x+2,y+1);


#ifdef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;

#endif
			//#ifndef trainMode

			FAULTDET_testing_injectFault32(i0, executionId, 384, 415, injectingErrors);
			FAULTDET_testing_injectFault32(i1, executionId, 416, 447, injectingErrors);
			FAULTDET_testing_injectFault32(i2, executionId, 448, 479, injectingErrors);
			FAULTDET_testing_injectFault32(i3, executionId, 480, 511, injectingErrors);

			//						FAULTDET_testing_injectFault32(col2, executionId, 512, 543, injectingErrors);
			FAULTDET_testing_injectFault32(dx, executionId, 544, 575, injectingErrors);
			//#endif

#ifndef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;

#endif



			//				accum=i0+i1+i2+i3;
			//				FAULTDET_testPoint(&inst,
			//						uniIdBase+3, //uniId
			//						0, //checkId
			//						0, //non blocking.
			//						4, //SIZE OF THIS SPECIFIC AOV
			//						&accum, &x, &y, &hour); //values of AOV
			col2=hermit_poly(i0,i1,i2,i3,dx);
			//#ifdef trainMode
			if (executionId<-1) {
				FAULTDET_trainPoint(ctr*5+3, 0, 6, &i0f, &i1f, &i2f, &i3f, &col2, &dx);
				//#else
			} else {
				FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
						&inst,
#endif
						ctr*5+3, //uniId
						0, //checkId
						0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
						injectingErrors,
#endif
						6, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
						&i0f, &i1f, &i2f, &i3f, &col2, &df);
				//#endif
			}
			i0=get_pixel(x-1,y+2);
			i1=get_pixel(x,y+2);
			i2=get_pixel(x+1,y+2);
			i3=get_pixel(x+2,y+2);


#ifdef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;

#endif
			//#ifndef trainMode


			FAULTDET_testing_injectFault32(i0, executionId, 576, 607, injectingErrors);
			FAULTDET_testing_injectFault32(i1, executionId, 608, 639, injectingErrors);
			FAULTDET_testing_injectFault32(i2, executionId, 640, 671, injectingErrors);
			FAULTDET_testing_injectFault32(i3, executionId, 672, 703, injectingErrors);

			//						FAULTDET_testing_injectFault32(col3, executionId, 704, 735, injectingErrors);
			FAULTDET_testing_injectFault32(dx, executionId, 736, 767, injectingErrors);
			//#endif

#ifndef onOutputOnly
			i0f=i0;
			i1f=i1;
			i2f=i2;
			i3f=i3;
			df=dx;
#endif

			//				accum=i0+i1+i2+i3;
			//				FAULTDET_testPoint(&inst,
			//						uniIdBase+4, //uniId
			//						0, //checkId
			//						0, //non blocking.
			//						4, //SIZE OF THIS SPECIFIC AOV
			//						&accum, &x, &y, &hour); //values of AOV
			col3=hermit_poly(i0,i1,i2,i3,dx);
			float col0f, col1f, col2f, col3f;
			//#ifdef trainMode
			if (executionId<-1) {
				FAULTDET_trainPoint(ctr*5+4, 0, 6, &i0f, &i1f, &i2f, &i3f, &col3, &df);
			} else {
				FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
						&inst,
#endif
						ctr*5+4, //uniId
						0, //checkId
						0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
						injectingErrors,
#endif
						6, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
						&i0f, &i1f, &i2f, &i3f, &col3, &dx);
				//#endif
			}

#ifdef onOutputOnly
			col0f=col0;
			col1f=col1;
			col2f=col2;
			col3f=col3;
			df=dy;

#endif
			//#ifndef trainMode
#ifdef testingCampaign
			FAULTDET_testing_injectFault32(col0, executionId, 768, 799, injectingErrors);
			FAULTDET_testing_injectFault32(col1, executionId, 800, 831, injectingErrors);
			FAULTDET_testing_injectFault32(col2, executionId, 832, 863, injectingErrors);
			FAULTDET_testing_injectFault32(col3, executionId, 864, 895, injectingErrors);
#endif
			//						FAULTDET_testing_injectFault32(res, executionId, 896, 927, injectingErrors);
			FAULTDET_testing_injectFault32(dy, executionId, 928, 959, injectingErrors);
			//#endif
#ifndef onOutputOnly
			col0f=col0;
			col1f=col1;
			col2f=col2;
			col3f=col3;
			df=dy;


#endif
			res=hermit_poly(col0,col1,col2,col3,dy);
			if (executionId<-1) {
				//#ifdef trainMode
				FAULTDET_trainPoint(ctr*5+5, 0, 6, &col0f, &col1f, &col2f, &col3f, &res, &dy);
				//#else
			} else {
				FAULTDET_testPoint(
#ifndef FAULTDETECTOR_EXECINSW
						&inst,
#endif
						ctr*5+5, //uniId
						0, //checkId
						0, //BLOCKING OR NON BLOCKING, non blocking
#ifdef testingCampaign
						injectingErrors,
#endif
						6, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
						&col0, &col1, &col2, &col3, &res, &df);
				//#endif
			}

#ifdef testingCampaign
			//#ifndef trainMode
			if (injectingErrors) {
				FAULTDET_testing_commitTmpStatsAndReset();
			}
			//#endif
#endif

			mat_out[i][j]=res;
			ctr++;
		}
	}

	//				clk_l=get_clock_L();
	//				clk_hi=get_clock_U();
	//				printf("final value %x%x", clk_hi, clk_l);
	//				printf(" final time, lsbits only %d", get_delay_for_number_of_cycles(clk_l));


	/*we want to commit what we processed. Check no fault has happened.
			As soon as a fault is detected by fault detector and the condition allows it,
			task is killed and reexecuted, so we just have to wait here.
			Most likely, if a fault is detected, we won't reach this point,
			the task will be re executed earlier.*/
#ifndef FAULTDETECTOR_EXECINSW
	FAULTDET_blockIfFaultDetectedInTask(&inst);
#endif

	//no fault, commit changes
	//		replace_old_img(mat_out);

	//signal to the scheduler the end of the job

	//		vTaskJobEnd();


#ifdef testingCampaign
	//#ifndef trainMode
	if (executionId>=-1) {
		printf(" total: %d ", FAULTDET_testing_getTotal());
		printf(" ok: %d ", FAULTDET_testing_getOk());
		if (injectingErrors) {
			printf(" fn: %d\n", FAULTDET_testing_getFalseNegatives());
		} else {
			printf(" fp: %d\n", FAULTDET_testing_getFalsePositives());
		}
		if (!injectingErrors) {
			//			FAULTDET_testing_resetStats();
		}
		injectingErrors=0xFF;
	}
	//#endif
#endif

}
#endif

#endif

#include "perf_timer.h"

//static void prvImgAcquireAndScaleTask( void *pvParameters )
static void prvTaskFour( void *pvParameters )
{
	xPortSchedulerDisableIntr(); //if uncommented, task will execute continuously
	printf("start\n");

	random_set_seed(1);
	//	float a;
	//	float b;
	//	float c;
	//
	//	for (int i=0; i<50000; i++) {
	//
	//		a=random_get();
	//		b=random_get();
	//		c=random_get();
	//
	//		FAULTDET_trainPoint(
	//				1,
	//				0,  //checkId
	//				3,
	//				&a, &b, &c);
	//	}
	//
	//	for (int i=0; i<10000; i++) {
	//		a=random_get();
	//		b=random_get();
	//		c=random_get();
	//
	//		FAULTDET_testPoint(
	//#ifndef FAULTDETECTOR_EXECINSW
	//				NULL,
	//#endif
	//				1, //uniId
	//				0, //checkId
	//				0, //BLOCKING OR NON BLOCKING, non blocking
	//#ifdef testingCampaign
	//				injectingErrors,
	//				1,
	//				1,
	//				roundId,
	//				executionId,
	//#endif
	//				3, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
	//				&a, &b, &c);
	//#ifndef FAULTDETECTOR_EXECINSW
	//		FAULTDET_resetFault();
	//#endif
	//	}
	//	printf("clk train mean: %u clk test mean: %u", getMeanTrainClock(), getMeanTestClock());
	for (int i=0; i<25000; i++) {
		r1=random_get();
		r2=random_get();
		r3=random_get();
		r4=random_get();
		latnav(0, -5);
	}
	for (int i=0; i<50000; i++) {
		r1=random_get();
		r2=random_get();
		r3=random_get();
		r4=random_get();
		latnav(0, -1);
//		injectingErrors=0x0;
//		FAULTDET_testing_resetGoldens();
	}
	unsigned int benchtime=clk_count_bench_total/clk_count_bench_total_times;
	printf("bench time %u", benchtime);
//	printf("\"total_pos\": %d, ", FAULTDET_testing_getTotal_golden());
//	printf("\"true_pos\": %d, ", FAULTDET_testing_getOk_golden());
//	printf("\"false_pos\": %d, ", FAULTDET_testing_getFalsePositives_golden());
}



#ifdef aaaa


//	unsigned long long int ovh_sum=0;
//	for (int i=0; i<10000; i++) {
//		perf_reset_and_start_clock();
//		perf_stop_clock();
//		ovh_sum+=get_clock_L();
//	}
//	unsigned int ovh=ovh_sum/10000;
//	printf("measure overhead %u", ovh);

perf_reset_and_start_clock();
perf_stop_clock();
printf("measure overhead %u", get_clock_L());

random_set_seed(1);

//	for (int i=0; i<1000; i++) {
//		injectingErrors=0x0;

int executions=1000;
int trainIter=1000;

for (int executionId=-trainIter-1 ;executionId<-1/*960*/; executionId++) {
	r1=random_get();
	r2=random_get();
	r3=random_get();
	r4=random_get();
	latnav(0, executionId);
}

for (int i=0; i<executions; i++) {
	FAULTDET_testing_resetGoldens();
	injectingErrors=0x0;
	r1=random_get();
	r2=random_get();
	r3=random_get();
	r4=random_get();
	for (int executionId=-1 ;executionId<1312/*1503*//*960*/; executionId++) {
		latnav(i, executionId);
	}
	//		}
	//#ifdef trainMode
	//		printf("Training done. ");
	//		FAULTDET_dumpRegions();
	//		while(1) {
	//			//printf("done train");
	//
	//		}
	//#endif
	//			FAULTDET_hotUpdateRegions(trainedRegions, n_regions);
}
#ifdef testingCampaign
printf("clk train mean: %u clk test mean: %u", getMeanTrainClock(), getMeanTestClock());
printf("], ");
printf("\"total_pos\": %d, ", FAULTDET_testing_getTotal_golden());
printf("\"true_pos\": %d, ", FAULTDET_testing_getOk_golden());
printf("\"false_pos\": %d, ", FAULTDET_testing_getFalsePositives_golden());

printf("\"total_bitflips\":%d, ", FAULTDET_testing_getTotal());
printf("\"true_neg\": %d, ", FAULTDET_testing_getOk());
printf("\"false_neg\":%d, ", FAULTDET_testing_getFalseNegatives());
//	printf("%d|", FAULTDET_testing_getOk_wtolerance());
//	printf("%d|", FAULTDET_testing_getFalseNegatives_wtolerance());
printf("\"no_effect_bitflips\": %d", FAULTDET_testing_getNoEffects());
printf("}");
#endif

}
#endif

//
//
//	static void prvTaskFour( void *pvParameters )
//	{
//#ifdef testingCampaign
//		xPortSchedulerDisableIntr(); //if uncommented, task will execute continuously
//		for (int executionId=-1 ;executionId<96; executionId++) {
//#else
//			for (;;) {
//#endif
//#ifndef FAULTDETECTOR_EXECINSW
//				FAULTDET_ExecutionDescriptor inst;
//				FAULTDET_initFaultDetection(&inst);
//#endif
//
//				//		FAULTDET_testing_initTesting();
//				//FAULTDET_resetFault(); //not needed, automatically done by the faultdetector when a command from the same check but with different UniId is received
//
//				float v1=150.0;
//				float v2=50.0;
//				float v3=50.0;
//
//#ifdef testingCampaign
//
//				printf("Exec id %d", executionId);
//
//				if (executionId>=0) {
//					printBits(sizeof(u32), &v1);
//					FAULTDET_testing_injectFault32(v1, executionId, 0, 31, injectingErrors);
//					printBits(sizeof(u32), &v1);
//
//					printBits(sizeof(u32), &v2);
//					FAULTDET_testing_injectFault32(v2, executionId, 32, 63, injectingErrors);
//					printBits(sizeof(u32), &v2);
//
//					printBits(sizeof(u32), &v3);
//					FAULTDET_testing_injectFault32(v3, executionId, 64, 95, injectingErrors);
//					printBits(sizeof(u32), &v3);
//				}
//#endif
//
//				FAULTDET_testPoint(
//#ifndef FAULTDETECTOR_EXECINSW
//						&inst,
//#endif
//						1, //uniId
//						0, //checkId
//						0, //BLOCKING OR NON BLOCKING, non blocking
//#ifdef testingCampaign
//						injectingErrors,
//#endif
//						8, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
//						&v1, &v2, &v3, &v3, &v3, &v3, &v3, &v3);
//
//				/*we want to commit what we processed. Check no fault has happened.
//			As soon as a fault is detected by fault detector and the condition allows it,
//			task is killed and reexecuted, so we just have to wait here.
//			Most likely, if a fault is detected, we won't reach this point,
//			the task will be re executed earlier.*/
//				//		FAULTDET_blockIfFaultDetectedInTask(&inst);
//
//#ifdef testingCampaign
//				printf(" total: %d ", FAULTDET_testing_getTotal());
//				printf(" ok: %d ", FAULTDET_testing_getOk());
//				printf(" fp: %d ", FAULTDET_testing_getFalsePositives());
//				printf(" fn: %d ", FAULTDET_testing_getFalseNegatives());
//
//				//after the first execution (generating goldens), inject faults
//				injectingErrors=0xFF;
//#endif
//
//				//commit changes
//				//writeOutput(...)
//
//				//signal to the scheduler the end of the job
//#ifndef testingCampaign
//				vTaskJobEnd();
//#endif
//			}
//		}
