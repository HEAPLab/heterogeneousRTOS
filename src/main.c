/*
    Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
    Copyright (c) 2012 - 2022 Xilinx, Inc. All Rights Reserved.
	SPDX-License-Identifier: MIT


    http://www.FreeRTOS.org
    http://aws.amazon.com/freertos


    1 tab == 4 spaces!
 */

//#include "xil_xil_printf.h"

#define onOutputOnly

#include "FreeRTOS.h"
#include "task.h"
#include "xil_printf.h"

#include "portable.h"

#include "perf_timer.h"

//#define gaussianBench
#define ANNBench
//#define imgscalingBench
//#define FFTBench
#define testMode
//#define latnavBench

#include "simple_random.h"

/*-----------------------------------------------------------*/

#include <math.h>
static void prvTaskOne( void *pvParameters );
static void prvTaskTwo( void *pvParameters );
static void prvTaskThree( void *pvParameters );
static void prvTaskFour( void *pvParameters );
#include <stdio.h>


static FAULTDETECTOR_region_t trainedRegions[FAULTDETECTOR_MAX_CHECKS][FAULTDETECTOR_MAX_REGIONS];
static u8 n_regions[FAULTDETECTOR_MAX_CHECKS];

int main( void )
{
	//			xRTTaskCreate( prvTaskOne,
	//					( const char * ) "One",
	//					configMINIMAL_STACK_SIZE,
	//					NULL,
	//					tskIDLE_PRIORITY,
	//					NULL,
	//					50000000, //deadline
	//					50000000, //period
	//					40000000); //wcet
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
			50000, //deadline
			50000, //period
			1,
			10000,
			10000); //wcet

	for (int i=0; i<FAULTDETECTOR_MAX_CHECKS; i++) {
		n_regions[i]=0;
		for (int j=0; j<FAULTDETECTOR_MAX_REGIONS; j++) {
			for (int k=0; k<=FAULTDETECTOR_MAX_AOV_DIM; k++) {
				trainedRegions[i][j].center[k]=0.0f;
				trainedRegions[i][j].max[k]=0.0f;
				trainedRegions[i][j].min[k]=0.0f;
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

	//	xil_printf("%u\n", sizeof(FAULTDETECTOR_testpointShortDescriptorStr));

	vTaskStartScheduler();

	//should never reach this point
	for( ;; );
}

FAULTDETECTOR_controlStr* contr;

/*-----------------------------------------------------------*/
static void prvTaskOne( void *pvParameters )
{
	for (;;) {
		xil_printf(" One ");

		vTaskJobEnd();
	}
}

/*-----------------------------------------------------------*/
static void prvTaskTwo( void *pvParameters )
{
	for (;;) {
		xil_printf(" Two ");

		vTaskJobEnd();
	}
}

static void prvTaskThree( void *pvParameters )
{
	for (;;) {
		xil_printf(" Three ");

		vTaskJobEnd();
	}
}

//FAULTDETECTOR_controlStr contr;

#ifdef gaussianBench


//at least one of them must be even
#define IMG_HEIGHT 128
#define IMG_WIDTH 128

#define KERNEL_SIZE 5

#define SIGMA 1.0

#ifndef USER_GAUSS_FILTER
static unsigned char mat_in[IMG_HEIGHT][IMG_WIDTH];
static unsigned char mat_out[IMG_HEIGHT][IMG_WIDTH];
#endif

/* KERNEL_SIZExKERNEL_SIZE gaussian filter with origin in (1,1) */
static float kernel[KERNEL_SIZE][KERNEL_SIZE];

/**
 * @brief It generates a KERNEL_SIZE x KERNEL_SIZE gaussian kernel
 *
 */
static void gaussian_kernel_init(){
	int i,j;
	float sum=0;
	for (i = 0; i < KERNEL_SIZE; i++) {
		for (j = 0; j < KERNEL_SIZE; j++) {
			float x = i - (KERNEL_SIZE - 1) / 2.0;
			float y = j - (KERNEL_SIZE - 1) / 2.0;
			kernel[i][j] =  exp(((pow(x, 2) + pow(y, 2)) / ((2 * pow(SIGMA, 2)))) * (-1));
			sum += kernel[i][j];
		}
	}

	for (i = 0; i < KERNEL_SIZE; i++) {
		for (j = 0; j < KERNEL_SIZE; j++) {
			kernel[i][j] /= sum;
		}
	}
}

/**
 * @brief Performs 2D convolution of KERNEL_SIZExKERNEL_SIZE kernel with mat_in
 *
 * @param p_x  center point x coordinate
 * @param p_y center point y coordinate
 * @return int result of 2d convolution of kernel centred in mat_in[p_x][p_y]
 */
char horizontalAccumulate=0x0;
float oldtemp;
int uniIdCtr=0;
//float diff;

static int convolution2D_train(int p_x, int p_y){
	int k_r,offset_x,offset_y,i,j;
	float temp;

	/*Kernel radius*/
	k_r=KERNEL_SIZE/2;

	/*kernel can be superimposed? if not we are on borders, then we keep the values unchanged*/
	if(p_x-k_r<0 || p_y-k_r<0 || p_x+k_r>=IMG_HEIGHT || p_y+k_r>=IMG_WIDTH){
		unsigned char res=mat_in[p_x][p_y];
		return res;
	}
	/*offset between kernel's indexes and array's ones*/
	offset_x=p_x-k_r;
	offset_y=p_y-k_r;

	temp=0;

	if (horizontalAccumulate) {
		int loop1ctr=0;
		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
			int loop2ctr=0;
			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
				unsigned char in=mat_in[i][j];
				float kernelin=kernel[i-offset_x][j-offset_y];
				temp+=kernelin * in;
				contr->AOV[loop1ctr]-=in;
				loop2ctr++;
			}
			loop1ctr++;
		}

		//		diff=temp>oldtemp ? temp-oldtemp : oldtemp-temp;

		contr->uniId=uniIdCtr;
		contr->checkId=0;
		contr->AOV[5]=temp>oldtemp ? temp-oldtemp : oldtemp-temp;

		FAULTDET_trainPoint();
		uniIdCtr++;
	} else {
		for (int i=0; i<KERNEL_SIZE; i++) {
			contr->AOV[i]=0;
		}
		int loop1ctr=0;
		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
			int loop2ctr=0;
			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
				unsigned char in=mat_in[i][j];
				float kernelin=kernel[i-offset_x][j-offset_y];
				temp+=kernelin * in;
				contr->AOV[loop1ctr]+=in;
				loop2ctr++;
			}
			loop1ctr++;
		}
	}
	oldtemp=temp;
	horizontalAccumulate=!horizontalAccumulate;
	return temp;
}


FAULTDETECTOR_controlStr* contr;

static int convolution2D_test(int p_x, int p_y){
	int k_r,offset_x,offset_y,i,j;
	float temp;

	/*Kernel radius*/
	k_r=KERNEL_SIZE/2;

	/*kernel can be superimposed? if not we are on borders, then we keep the values unchanged*/
	if(p_x-k_r<0 || p_y-k_r<0 || p_x+k_r>=IMG_HEIGHT || p_y+k_r>=IMG_WIDTH){
		unsigned char res=mat_in[p_x][p_y];
		return res;
	}
	/*offset between kernel's indexes and array's ones*/
	offset_x=p_x-k_r;
	offset_y=p_y-k_r;

	temp=0;

	if (horizontalAccumulate) {
		int loop1ctr=0;
		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
			int loop2ctr=0;
			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
				unsigned char in=mat_in[i][j];
				float kernelin=kernel[i-offset_x][j-offset_y];
				temp+=kernelin * in;
				contr->AOV[loop1ctr]-=in;
				loop2ctr++;
			}
			loop1ctr++;
		}

		//diff=temp>oldtemp ? temp-oldtemp : oldtemp-temp;

		contr->uniId=uniIdCtr;
		contr->checkId=0;

		contr->AOV[5]=temp>oldtemp ? temp-oldtemp : oldtemp-temp;;

		FAULTDET_testPoint();
		uniIdCtr++;
	} else {
		for (int i=0; i<KERNEL_SIZE; i++) {
			contr->AOV[i]=0;
		}
		int loop1ctr=0;
		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
			int loop2ctr=0;
			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
				unsigned char in=mat_in[i][j];
				float kernelin=kernel[i-offset_x][j-offset_y];
				temp+=kernelin * in;
				contr->AOV[loop1ctr]+=in;
				loop2ctr++;
			}
			loop1ctr++;
		}
	}
	oldtemp=temp;
	horizontalAccumulate=!horizontalAccumulate;
	return temp;
}
static int convolution2D_test_prequel(int p_x, int p_y){
	int k_r,offset_x,offset_y,i,j;
	float temp;

	/*Kernel radius*/
	k_r=KERNEL_SIZE/2;

	/*kernel can be superimposed? if not we are on borders, then we keep the values unchanged*/
	if(p_x-k_r<0 || p_y-k_r<0 || p_x+k_r>=IMG_HEIGHT || p_y+k_r>=IMG_WIDTH){
		unsigned char res=mat_in[p_x][p_y];
		return res;
	}
	/*offset between kernel's indexes and array's ones*/
	offset_x=p_x-k_r;
	offset_y=p_y-k_r;

	temp=0;

	if (horizontalAccumulate) {
		int loop1ctr=0;
		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
			int loop2ctr=0;
			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
				unsigned char in=mat_in[i][j];
				float kernelin=kernel[i-offset_x][j-offset_y];
				temp+=kernelin * in;
				contr->AOV[loop1ctr]=0xFFFFFFF;
				loop2ctr++;
			}
			loop1ctr++;
		}

		//		diff=temp>oldtemp ? temp-oldtemp : oldtemp-temp;

		contr->uniId=uniIdCtr;
		contr->checkId=0;

		contr->AOV[5]=0xFFFFFFF;

		FAULTDET_testPoint();
		uniIdCtr++;
	} else {
		for (int i=0; i<KERNEL_SIZE; i++) {
			contr->AOV[i]=0;
		}
		int loop1ctr=0;
		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
			int loop2ctr=0;
			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
				unsigned char in=mat_in[i][j];
				float kernelin=kernel[i-offset_x][j-offset_y];
				temp+=kernelin * in;
				contr->AOV[loop1ctr]=0xFFFFFFF;
				loop2ctr++;
			}
			loop1ctr++;
		}
	}
	oldtemp=temp;
	horizontalAccumulate=!horizontalAccumulate;
	return temp;
}


//
//static int convolution2D_train(int p_x, int p_y){
//	int k_r,offset_x,offset_y,i,j;
//	float temp;
//
//	/*Kernel radius*/
//	k_r=KERNEL_SIZE/2;
//
//	/*kernel can be superimposed? if not we are on borders, then we keep the values unchanged*/
//	if(p_x-k_r<0 || p_y-k_r<0 || p_x+k_r>=IMG_HEIGHT || p_y+k_r>=IMG_WIDTH){
//		unsigned char res=mat_in[p_x][p_y];
//		return res;
//	}
//	/*offset between kernel's indexes and array's ones*/
//	offset_x=p_x-k_r;
//	offset_y=p_y-k_r;
//
//	temp=0;
//
//	for (int i=0; i<KERNEL_SIZE; i++) {
//		faultdet_vals[i]=0;
//	}
//	unsigned int checkId;
//
//	if (horizontalAccumulate) {
//		int loop1ctr=0;
//		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1TOTAL
//			int loop2ctr=0;
//			for(j=p_y-k_r;j<=p_y+k_r;j++) { //LOOP2TOTAL
//				unsigned char in=mat_in[i][j];
//				float kernelin=kernel[i-offset_x][j-offset_y];
//				temp+=kernelin * in;
//				faultdet_vals[loop2ctr]+=in;
//				loop2ctr++;
//			}
//			loop1ctr++;
//		}
//	} else {
//		int loop1ctr=0;
//		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
//			int loop2ctr=0;
//			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
//				unsigned char in=mat_in[i][j];
//				float kernelin=kernel[i-offset_x][j-offset_y];
//				temp+=kernelin * in;
//				faultdet_vals[loop1ctr]+=in;
//				loop2ctr++;
//			}
//			loop1ctr++;
//		}
//	}
//	if (offset_x!=0)
//		diff= temp>oldtemp ? temp-oldtemp : oldtemp-temp;
//	else diff=0;
//
//	contr->uniId=uniIdCtr;
//	contr->checkId=0;
//	contr->AOV[0]=faultdet_vals[0];
//	contr->AOV[1]=faultdet_vals[1];
//	contr->AOV[2]=faultdet_vals[2];
//	contr->AOV[3]=faultdet_vals[3];
//	contr->AOV[4]=faultdet_vals[4];
//	contr->AOV[5]=temp;
//	contr->AOV[6]=diff;
//	FAULTDET_trainPoint(&contr);
//	uniIdCtr++;
//
//	oldtemp=temp;
//
//	horizontalAccumulate=!horizontalAccumulate;
//	return temp;
//}
//
//
//static int convolution2D_test(int p_x, int p_y){
//	int k_r,offset_x,offset_y,i,j;
//	float temp;
//
//	/*Kernel radius*/
//	k_r=KERNEL_SIZE/2;
//
//	/*kernel can be superimposed? if not we are on borders, then we keep the values unchanged*/
//	if(p_x-k_r<0 || p_y-k_r<0 || p_x+k_r>=IMG_HEIGHT || p_y+k_r>=IMG_WIDTH){
//		unsigned char res=mat_in[p_x][p_y];
//		return res;
//	}
//	/*offset between kernel's indexes and array's ones*/
//	offset_x=p_x-k_r;
//	offset_y=p_y-k_r;
//
//	temp=0;
//
//	for (int i=0; i<KERNEL_SIZE; i++) {
//		faultdet_vals[i]=0;
//	}
//	unsigned int checkId;
//
//	if (horizontalAccumulate) {
//		int loop1ctr=0;
//		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1TOTAL
//			int loop2ctr=0;
//			for(j=p_y-k_r;j<=p_y+k_r;j++) { //LOOP2TOTAL
//				unsigned char in=mat_in[i][j];
//				float kernelin=kernel[i-offset_x][j-offset_y];
//				temp+=kernelin * in;
//				faultdet_vals[loop2ctr]+=in;
//				loop2ctr++;
//			}
//			loop1ctr++;
//		}
//	} else {
//		int loop1ctr=0;
//		for(i=p_x-k_r;i<=p_x+k_r;i++){ //LOOP1
//			int loop2ctr=0;
//			for(j=p_y-k_r;j<=p_y+k_r;j++){ //LOOP2
//				unsigned char in=mat_in[i][j];
//				float kernelin=kernel[i-offset_x][j-offset_y];
//				temp+=kernelin * in;
//				faultdet_vals[loop1ctr]+=in;
//				loop2ctr++;
//			}
//			loop1ctr++;
//		}
//	}
//	if (offset_x!=0)
//		diff= temp>oldtemp ? temp-oldtemp : oldtemp-temp;
//	else diff=0;
//
//	contr->uniId=uniIdCtr;
//	contr->checkId=0;
//	contr->AOV[0]=faultdet_vals[0];
//	contr->AOV[1]=faultdet_vals[1];
//	contr->AOV[2]=faultdet_vals[2];
//	contr->AOV[3]=faultdet_vals[3];
//	contr->AOV[4]=faultdet_vals[4];
//	contr->AOV[5]=temp;
//	contr->AOV[6]=diff;
//	FAULTDET_testPoint(&contr);
//	uniIdCtr++;
//
//	oldtemp=temp;
//
//	horizontalAccumulate=!horizontalAccumulate;
//	return temp;
//}


/**
 * @brief Actual gaussian filter implementation
 *
 */
static void gauss_filter_routine_test_fault() {
	contr->uniId=0;
	contr->checkId=0;
	contr->AOV[0]=0xFFFFFFF;
	contr->AOV[1]=0xFFFFFFF;
	contr->AOV[2]=0xFFFFFFF;
	contr->AOV[3]=0xFFFFFFF;
	contr->AOV[4]=0xFFFFFFF;
	contr->AOV[5]=0xFFFFFFF;
	FAULTDET_testPoint();
#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_blockIfFaultDetectedInTask();
#endif
}

static void gauss_filter_routine_test() {

	perf_reset_and_start_clock();

	horizontalAccumulate=0x0;
	uniIdCtr=0;

	FAULTDET_initFaultDetection();
	int i=0;
	int j=0;

	for(;i<2;i++){
		for(j=0;j<IMG_WIDTH;j++){
			mat_out[i][j]=convolution2D_test(i,j);
		}
	}
	j=0;
	//TRAINING benchmark
	mat_out[i][j]=convolution2D_test(i,j); //j==0
	j++;
	mat_out[i][j]=convolution2D_test(i,j); //j==1
	j++;
	mat_out[i][j]=convolution2D_test_prequel(i,j); //j==2
	j++;
	mat_out[i][j]=convolution2D_test_prequel(i,j); //j==3
	j++;
	for(;i<IMG_HEIGHT;i++){
		for(;j<IMG_WIDTH;j++){
			mat_out[i][j]=convolution2D_test(i,j);
		}
		j=0;
	}

#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_blockIfFaultDetectedInTask(&contr);
#endif

	perf_stop_clock();
	xil_printf("%u\n", get_clock_L());
	if (get_clock_U()!=0)
		xil_printf("err up not 0");
}

static void gauss_filter_routine_train() {

	horizontalAccumulate=0x0;
	uniIdCtr=0;

	int i,j;

	for(i=0;i<IMG_HEIGHT;i++){
		for(j=0;j<IMG_WIDTH;j++){
			mat_out[i][j]=convolution2D_train(i,j);
		}
	}

	if (horizontalAccumulate) {
		xil_printf("ERROR, HEIGHT OR WIDTH MUST BE EVEN\n");
	}
}


void init_img_matrix() {
	for (int i = 0; i < IMG_HEIGHT; i++){
		for (int j = 0; j < IMG_WIDTH; j++){
			mat_in[i][j]=random_get()*256;
		}
	}
}

#endif

#ifdef ANNBench

#include "simple_random.h"

#include <math.h>

#define IN_NODES 4

#define HIDDEN_NODES 2
#define HIDDEN_LAYERS 5

#define OUT_NODES 1
#define LR 0.1
#define NN_EPOCH 1

#define ARRAY_LENGTH 8
#define BURST_LENGTH 8

static float test_in[BURST_LENGTH][IN_NODES];
static float test_out[BURST_LENGTH][OUT_NODES];

static float train_in[ARRAY_LENGTH][IN_NODES];
static float train_out[ARRAY_LENGTH][OUT_NODES];
static float net_out[OUT_NODES];

static float in_weight[IN_NODES][HIDDEN_NODES];

static float hl_weight[HIDDEN_LAYERS][HIDDEN_NODES][HIDDEN_NODES];
static float hl_bias[HIDDEN_LAYERS][HIDDEN_NODES];

static float out_weight[HIDDEN_NODES][OUT_NODES];
static float out_bias[OUT_NODES];

static float temp_out[HIDDEN_LAYERS][HIDDEN_NODES];
static float delta_out[OUT_NODES];
static float delta_hidden[HIDDEN_LAYERS][HIDDEN_NODES];

static float sigmoid(float x){
	return 1/(1+exp(-x));
}

static float d_sigmoid(float x){
	return x*(1-x);
}
static void init_train_data(){
	int i;
	for (i = 0; i < ARRAY_LENGTH; i++){
		for (int a=0; a<IN_NODES;a++) {
			train_in[i][a]=random_get();
		}
	}
	for (i = 0; i < ARRAY_LENGTH; i++){
		for (int a=0; a<OUT_NODES;a++) {
			train_out[i][a]=random_get();
		}
	}
}


static void init_test_data(){
	for (int i=0; i<BURST_LENGTH; i++) {
		for (int a=0; a<IN_NODES;a++) {
			test_in[i][a]=random_get();
		}
	}
}


static void init_weights(){
	int i,h,l;

	for(i=0;i<IN_NODES;i++){
		for ( h = 0; h < HIDDEN_NODES; h++){
			in_weight[i][h]=random_get();
		}

	}
	for(l=0;l<HIDDEN_LAYERS;l++){
		for ( h = 0; h < HIDDEN_NODES; h++){
			hl_bias[l][h]=random_get();
			for(i=0;i<HIDDEN_NODES;i++){
				hl_weight[l][h][i]=random_get();
			}
		}

	}

	for(i=0;i<OUT_NODES;i++){
		out_bias[i]=random_get();
		for ( h = 0; h < HIDDEN_NODES; h++){
			out_weight[h][i]=random_get();
		}
	}

}
static void forward_pass(int train_idx){
	int h,l,y;

	for(h=0;h<HIDDEN_NODES;h++){
		int x;
		float activation;

		activation=hl_bias[0][h];
		for(x=0;x<IN_NODES;x++){
			activation+=(in_weight[x][h]*train_in[train_idx][x]);
		}
		temp_out[0][h]=sigmoid(activation);
	}
	for(l=1;l<HIDDEN_LAYERS;l++){
		for(h=0;h<HIDDEN_NODES;h++){
			float activation;
			int x;

			activation=hl_bias[l][h];
			for(x=0;x<HIDDEN_NODES;x++){
				activation+=(hl_weight[l][h][x]*temp_out[l-1][h]);
			}
			temp_out[l][h]=sigmoid(activation);
		}
	}
	for(y=0;y<OUT_NODES;y++){
		float activation;

		activation=out_bias[y];
		for(h=0;h<HIDDEN_NODES;h++){
			activation+=(out_weight[h][y]*temp_out[HIDDEN_LAYERS-1][h]);
		}
		net_out[y]=sigmoid(activation);
	}
}

#define LOOP3TOTAL (32*3)
#define LOOP2TOTAL (64+LOOP3TOTAL*IN_NODES)

#define LOOP6TOTAL (32*3)
#define LOOP5TOTAL (64+LOOP6TOTAL*HIDDEN_NODES)
#define LOOP4TOTAL (LOOP5TOTAL*HIDDEN_NODES)

#define LOOP8TOTAL (32*3)
#define LOOP7TOTAL (64+LOOP8TOTAL*HIDDEN_NODES)

#define LOOP1TOTAL (LOOP2TOTAL*HIDDEN_NODES+LOOP4TOTAL*(HIDDEN_LAYERS-1)+LOOP7TOTAL*OUT_NODES)

static void forward_pass_test_burst_train(){
	int h,l,y;


	for (int b=0; b<BURST_LENGTH; b++) { //LOOP1

		for(h=0;h<HIDDEN_NODES;h++){ //LOOP2
			int x;
			float activation;

			activation=hl_bias[0][h];

			for(x=0;x<IN_NODES;x++){ //LOOP3
				float v1=in_weight[x][h];
				float v2=test_in[b][x];

				activation+=(v1*v2);
			}
			temp_out[0][h]=sigmoid(activation); //64
		}

		for(l=1;l<HIDDEN_LAYERS;l++){ //LOOP4
			for(h=0;h<HIDDEN_NODES;h++){ //LOOP5
				float activation;
				int x;

				activation=hl_bias[l][h];

				for(x=0;x<HIDDEN_NODES;x++){ //LOOP6
					float v1=hl_weight[l][h][x];
					float v2=temp_out[l-1][h];

					activation+=(v1*v2);

				}
				temp_out[l][h]=sigmoid(activation);
			}
		}

		for(y=0;y<OUT_NODES;y++){ //LOOP7
			float activation;

			activation=out_bias[y];

			for(h=0;h<HIDDEN_NODES;h++){ //LOOP8
				float v1=out_weight[h][y];
				float v2=temp_out[HIDDEN_LAYERS-1][h];

				activation+=(v1*v2);
			}
			float out=sigmoid(activation);

			contr->uniId=b;
			contr->checkId=0;

			contr->AOV[0]=test_in[b][0];
			contr->AOV[1]=test_in[b][1];
			contr->AOV[2]=test_in[b][2];
			contr->AOV[3]=test_in[b][3];
			contr->AOV[4]=test_in[b][4];

			contr->AOV[5]=out;


			FAULTDET_trainPoint();

			test_out[b][y]=out;
		}

	}
}

static void forward_pass_test_burst_test_fault() {
	contr->uniId=0;
	contr->checkId=0;

	contr->AOV[0]=0xFFFFFFF;
	contr->AOV[1]=0xFFFFFFF;
	contr->AOV[2]=0xFFFFFFF;
	contr->AOV[3]=0xFFFFFFF;
	contr->AOV[4]=0xFFFFFFF;

	contr->AOV[5]=0xFFFFFFF;

	FAULTDET_testPoint();
#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_blockIfFaultDetectedInTask();
#endif
}

static void forward_pass_test_burst_test(){
	int h,l,y;
	perf_reset_and_start_clock();
	FAULTDET_initFaultDetection();

	//dummy test
	for(h=0;h<HIDDEN_NODES;h++){ //LOOP2
		int x;
		float activation;

		activation=hl_bias[0][h];

		for(x=0;x<IN_NODES;x++){ //LOOP3
			float v1=in_weight[x][h];
			float v2=test_in[0][x];

			activation+=(v1*v2);
		}
		temp_out[0][h]=sigmoid(activation); //64
	}

	for(l=1;l<HIDDEN_LAYERS;l++){ //LOOP4
		for(h=0;h<HIDDEN_NODES;h++){ //LOOP5
			float activation;
			int x;

			activation=hl_bias[l][h];

			for(x=0;x<HIDDEN_NODES;x++){ //LOOP6
				float v1=hl_weight[l][h][x];
				float v2=temp_out[l-1][h];

				activation+=(v1*v2);

			}
			temp_out[l][h]=sigmoid(activation);
		}
	}

	for(y=0;y<OUT_NODES;y++){ //LOOP7
		float activation;

		activation=out_bias[y];

		for(h=0;h<HIDDEN_NODES;h++){ //LOOP8
			float v1=out_weight[h][y];
			float v2=temp_out[HIDDEN_LAYERS-1][h];

			activation+=(v1*v2);
		}
		float out=sigmoid(activation);

		contr->uniId=0;
		contr->checkId=0;

		contr->AOV[0]=0xFFFFFFF;
		contr->AOV[1]=0xFFFFFFF;
		contr->AOV[2]=0xFFFFFFF;
		contr->AOV[3]=0xFFFFFFF;
		contr->AOV[4]=0xFFFFFFF;

		contr->AOV[5]=0xFFFFFFF;

		FAULTDET_testPoint();

		test_out[0][y]=out;


	}
	//_________________________


	for (int b=1; b<BURST_LENGTH; b++) { //LOOP1

		for(h=0;h<HIDDEN_NODES;h++){ //LOOP2
			int x;
			float activation;

			activation=hl_bias[0][h];

			for(x=0;x<IN_NODES;x++){ //LOOP3
				float v1=in_weight[x][h];
				float v2=test_in[b][x];

				activation+=(v1*v2);
			}
			temp_out[0][h]=sigmoid(activation); //64
		}

		for(l=1;l<HIDDEN_LAYERS;l++){ //LOOP4
			for(h=0;h<HIDDEN_NODES;h++){ //LOOP5
				float activation;
				int x;

				activation=hl_bias[l][h];

				for(x=0;x<HIDDEN_NODES;x++){ //LOOP6
					float v1=hl_weight[l][h][x];
					float v2=temp_out[l-1][h];

					activation+=(v1*v2);

				}
				temp_out[l][h]=sigmoid(activation);
			}
		}

		for(y=0;y<OUT_NODES;y++){ //LOOP7
			float activation;

			activation=out_bias[y];

			for(h=0;h<HIDDEN_NODES;h++){ //LOOP8
				float v1=out_weight[h][y];
				float v2=temp_out[HIDDEN_LAYERS-1][h];

				activation+=(v1*v2);
			}
			float out=sigmoid(activation);

			contr->uniId=b;
			contr->checkId=0;

			contr->AOV[0]=test_in[b][0];
			contr->AOV[1]=test_in[b][1];
			contr->AOV[2]=test_in[b][2];
			contr->AOV[3]=test_in[b][3];
			contr->AOV[4]=test_in[b][4];

			contr->AOV[5]=out;

			FAULTDET_testPoint();

			test_out[b][y]=out;


		}

	}
#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_blockIfFaultDetectedInTask(&contr);
#endif

	perf_stop_clock();
	xil_printf("%u\n", get_clock_L());
	if (get_clock_U()!=0)
		xil_printf("err up not 0");
}


static void back_propagation(int train_idx){
	int y,h,l,x;
	/*Compute deltas for OUTPUT LAYER*/
	for(y=0;y<OUT_NODES;y++){
		delta_out[y] = (train_out[train_idx][y]-net_out[y])*d_sigmoid(net_out[y]);
	}
	/* Compute deltas for HIDDEN LAYER */
	for(h=0;h<HIDDEN_NODES;h++){
		float d_error;

		d_error=0;
		for(y=0;y<OUT_NODES;y++){
			d_error+=delta_out[y]*out_weight[h][y];
		}
		delta_hidden[HIDDEN_LAYERS-1][h]=d_error*sigmoid(temp_out[HIDDEN_LAYERS-1][h]);
	}
	for(l=HIDDEN_NODES-2;l>=0;l--){
		for(h=0;h<HIDDEN_NODES;h++){
			float d_error;

			d_error=0;
			for(y=0;y<HIDDEN_NODES;y++){
				d_error+=delta_hidden[l+1][y]*hl_weight[l][h][y];
			}
			delta_hidden[l][h]=d_error*sigmoid(temp_out[l][h]);
		}
	}

	/*Update weights*/
	for(y=0;y<OUT_NODES;y++){
		out_bias[y]+=delta_out[y]*LR;
		for(h=0;h<HIDDEN_NODES;h++){
			out_weight[h][y]+=temp_out[HIDDEN_LAYERS-1][h]*delta_out[y]*LR;
		}
	}
	for(l=HIDDEN_NODES-2;l>0;l--){
		for(h=0;h<HIDDEN_NODES;h++){
			hl_bias[l][h]+=delta_hidden[l][h]*LR;
			for(x=0;x<IN_NODES;x++){
				hl_weight[l][h][x]+=temp_out[l-1][x]*delta_hidden[l][h]*LR;
			}
		}
	}

	for(h=0;h<HIDDEN_NODES;h++){
		hl_bias[0][h]+=delta_hidden[0][h]*LR;
		for(x=0;x<IN_NODES;x++){
			in_weight[x][h]+=train_in[train_idx][x]*delta_hidden[0][h]*LR;
		}
	}

}

static void train_ann_routine(){
	int i,j;

	init_weights();
	for(i=0; i<NN_EPOCH;i++){

		for(j=0; j<ARRAY_LENGTH; j++){
			forward_pass(j);

			back_propagation(j);
		}

	}
}
#endif


#ifdef FFTBench

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FFT_LENGTH 512
#define CHECKPERIODICITY 8
typedef struct{
	float re,im;
} complex;

static complex array_in[FFT_LENGTH];
static complex array_out[FFT_LENGTH];
volatile char faulty=0;


/**
 * @brief It performs sum between two complex numbers
 *
 * @param a first complex number
 * @param b second complex number
 * @return complex sum of a and b
 */

//192
static complex complex_sum(complex a, complex b){
	complex res;
	res.re=a.re + b.re;
	res.im=a.im + b.im;

	return res;
}
/**
 * @brief It performs multiplication between two complex numbers
 *
 * @param a first complex number
 * @param b second complex number
 * @return complex mult of a and b
 */

//192
static complex complex_mult(complex a, complex b){
	complex res;
	res.re=(a.re * b.re) - (a.im*b.im);
	res.im=(a.im*b.re) + (a.re*b.im);
	return res;
}
/**
 * @brief It translates exponential form to Re and Im components
 *
 * @param x exponent
 * @return Re and Im components
 */

//96
static complex complex_exp(float x){
	/* e^(i*x)=cos(x) + i*sin(x)*/
	//	FAULTDET_testing_injectFault32(x, executionId, 32*0, (32*1)-1, injectingErrors);

	complex res;
	res.re=cos(x);
	res.im=sin(x);

	//	FAULTDET_testing_injectFault32(res.re, executionId, 32*1, (32*2)-1, injectingErrors);
	//	FAULTDET_testing_injectFault32(res.im, executionId, 32*2, (32*3)-1, injectingErrors);

	return res;
}

/**
 * @brief Actual fft implementation using Cooley–Tukey algorithm (radix-2 DIT case)
 *
 * @return Fourier transform for input array
 */

#define checksNum FAULTDETECTOR_MAX_CHECKS
int checks_idx[checksNum-1];

static void fft_routine_test_fault(){
	contr->uniId=14;
	contr->checkId=0;
	contr->AOV[0]=0xFFFFFFF;
	contr->AOV[1]=0xFFFFFFF;
	contr->AOV[2]=0xFFFFFFF;
	contr->AOV[3]=0xFFFFFFF;
	contr->AOV[4]=0xFFFFFFF;
	contr->AOV[5]=0xFFFFFFF;
	FAULTDET_testPoint();
	#ifndef configFAULTDETECTOR_SOFTWARE
		FAULTDET_blockIfFaultDetectedInTask();
	#endif
}



static void fft_routine_test(){
	int k=0;
	int n=0;
	complex even_sum,odd_sum;

	even_sum.re=0;
	even_sum.im=0;

#ifdef testMode
	contr->AOV[0]=0;
	contr->AOV[1]=0;
	contr->AOV[2]=0;
	contr->AOV[3]=0;
	int idx=0;
	int mul=0;
	complex tmp;
	tmp.im=0;
	tmp.re=0;
#endif

	complex cmplxexp;
	complex n_term;


	perf_reset_and_start_clock();

	FAULTDET_initFaultDetection();



	//loop unroll
	cmplxexp=complex_exp((-2*M_PI*n*k)/FFT_LENGTH); //96
	n_term = complex_mult(array_in[n], cmplxexp); //192


#ifdef testMode
	tmp=complex_sum(tmp, n_term); //192

	for(;n<FFT_LENGTH;n=n+2){
		if (idx<CHECKPERIODICITY/2) {
			contr->AOV[0]=0xFFFFFFF;
			contr->AOV[2]=0xFFFFFFF;
			idx++;
		} else if (idx==CHECKPERIODICITY-1) {
			contr->AOV[1]=0xFFFFFFF;
			contr->AOV[3]=0xFFFFFFF;

			mul=n*k;

			int chkid=checksNum-1;
			for (int i=0; i<checksNum-1; i++) {
				if (mul<=checks_idx[i]){
					chkid=i;
					break;
				}
			}

			contr->uniId=14;
			contr->checkId=0;
			contr->AOV[4]=0xFFFFFFF;
			contr->AOV[5]=0xFFFFFFF;

			FAULTDET_testPoint();

			even_sum=complex_sum(even_sum,tmp); //192
			//				even_sum2=complex_sum(even_sum2,tmp); //192
			//				if (memcmp(&even_sum, &even_sum, sizeof(complex))!=0)
			//					faulty=0xFF;

			idx=0;
			tmp.im=0;
			tmp.re=0;

			contr->AOV[0]=0;
			contr->AOV[1]=0;
			contr->AOV[2]=0;
			contr->AOV[3]=0;

			break;
		} else {
			contr->AOV[1]=0xFFFFFFF;
			contr->AOV[3]=0xFFFFFFF;
			idx++;
		}
	}

#endif
	n=n+2;

	//finished unroll

	for(;k<FFT_LENGTH;k++){


		/*X_k=[sum{0,N/2-1} x_2n * e^(i*(-2*pi*2n*k)/N)] + [sum{0,N/2-1} x_(2n+1) * e^(i*(-2*pi*(2n+1)*k)/N)]*/



		for(;n<FFT_LENGTH;n=n+2){
			cmplxexp=complex_exp((-2*M_PI*n*k)/FFT_LENGTH); //96
			n_term = complex_mult(array_in[n], cmplxexp); //192


#ifdef testMode
			tmp=complex_sum(tmp, n_term); //192

			if (idx<CHECKPERIODICITY/2) {
				contr->AOV[0]+=array_in[n].re;
				contr->AOV[2]+=array_in[n].im;
				idx++;
			} else if (idx==CHECKPERIODICITY-1) {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;

				mul=n*k;

				int chkid=checksNum-1;
				for (int i=0; i<checksNum-1; i++) {
					if (mul<=checks_idx[i]){
						chkid=i;
						break;
					}
				}

				contr->uniId=n;
				contr->checkId=chkid;
				contr->AOV[4]=tmp.re;
				contr->AOV[5]=tmp.im;

				FAULTDET_testPoint();

				even_sum=complex_sum(even_sum,tmp); //192
				//				even_sum2=complex_sum(even_sum2,tmp); //192
				//				if (memcmp(&even_sum, &even_sum, sizeof(complex))!=0)
				//					faulty=0xFF;

				idx=0;
				tmp.im=0;
				tmp.re=0;

				contr->AOV[0]=0;
				contr->AOV[1]=0;
				contr->AOV[2]=0;
				contr->AOV[3]=0;
			} else {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;
				idx++;
			}
#else
			even_sum=complex_sum(even_sum,n_term); //192
#endif
		}


		even_sum.re=0;
		even_sum.im=0;

		odd_sum.re=0;
		odd_sum.im=0;

#ifdef testMode
		idx=0;
		mul=0;

		tmp.im=0;
		tmp.re=0;
#endif

		for(n=1;n<FFT_LENGTH;n=n+2){
			cmplxexp=complex_exp((-2*M_PI*n*k)/FFT_LENGTH); //96
			n_term = complex_mult(array_in[n], cmplxexp); //192

#ifdef testMode
			tmp=complex_sum(tmp,n_term); //192

			if (idx<CHECKPERIODICITY/2) {
				contr->AOV[0]+=array_in[n].re;
				contr->AOV[2]+=array_in[n].im;
				idx++;
			} else if (idx==CHECKPERIODICITY-1) {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;

				mul=n*k;
				int chkid=checksNum-1;
				for (int i=0; i<checksNum-1; i++) {
					if (mul<=checks_idx[i]){
						chkid=i;
						break;
					}
				}

				contr->uniId=n;
				contr->checkId=chkid;
				contr->AOV[4]=tmp.re;
				contr->AOV[5]=tmp.im;

				FAULTDET_testPoint();

				odd_sum=complex_sum(odd_sum,tmp); //192
				//				odd_sum2=complex_sum(odd_sum2,tmp); //192
				//				if (memcmp(&odd_sum, &odd_sum2, sizeof(complex))!=0)
				//					faulty=0xFF;

				idx=0;
				tmp.im=0;
				tmp.re=0;

				contr->AOV[0]=0;
				contr->AOV[1]=0;
				contr->AOV[2]=0;
				contr->AOV[3]=0;
			} else {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;
				idx++;
			}
#else
			odd_sum=complex_sum(odd_sum,n_term); //192
#endif
		}

		complex out=complex_sum(even_sum,odd_sum);
#ifdef testMode
		//		complex out2=complex_sum(even_sum,odd_sum);
		//		if (memcmp(&out, &out2, sizeof(complex))!=0)
		//			faulty=0xFF;

#ifndef configFAULTDETECTOR_SOFTWARE
		FAULTDET_blockIfFaultDetectedInTask();
#endif
#endif
		array_out[k] = out; //192

#ifdef testMode
		//		contr->AOV[0]=0; //redundant
		//		contr->AOV[1]=0;
		//		contr->AOV[2]=0;
		//		contr->AOV[3]=0;

		idx=0;
		mul=0;

		tmp.im=0;
		tmp.re=0;
#endif
	}
	perf_stop_clock();
	xil_printf("%u\n", get_clock_L());
	if (get_clock_U()!=0)
		xil_printf("err up not 0");
}

//static void fft_routine_test(){
//	int k;
//
//	//for TRAINING benchmark
////	char executed=0;
//
//	perf_reset_and_start_clock();
//
//	FAULTDET_initFaultDetection();
//
//	for(k=0;k<FFT_LENGTH;k++){
//
//
//		/*X_k=[sum{0,N/2-1} x_2n * e^(i*(-2*pi*2n*k)/N)] + [sum{0,N/2-1} x_(2n+1) * e^(i*(-2*pi*(2n+1)*k)/N)]*/
//		int n;
//		complex even_sum,odd_sum;
//		even_sum.re=0;
//		even_sum.im=0;
//#ifdef testMode
//		complex even_sum2, odd_sum2;
//
//		contr->AOV[0]=0;
//		contr->AOV[1]=0;
//		contr->AOV[2]=0;
//		contr->AOV[3]=0;
//
//
//
//		int idx=0;
//		int mul=0;
//		complex tmp;
//		tmp.im=0;
//		tmp.re=0;
//#endif
//
//
//		for(n=0;n<FFT_LENGTH;n=n+2){
//			complex cmplxexp=complex_exp((-2*M_PI*n*k)/FFT_LENGTH); //96
//			complex n_term = complex_mult(array_in[n], cmplxexp); //192
//
//
//#ifdef testMode
//			tmp=complex_sum(tmp, n_term); //192
//
//			if (idx<CHECKPERIODICITY/2) {
//				contr->AOV[0]+=array_in[n].re;
//				contr->AOV[2]+=array_in[n].im;
//				idx++;
//			} else if (idx==CHECKPERIODICITY-1) {
//				contr->AOV[1]+=array_in[n].re;
//				contr->AOV[3]+=array_in[n].im;
//
//				mul=n*k;
//
//				int chkid=checksNum-1;
//				for (int i=0; i<checksNum-1; i++) {
//					if (mul<=checks_idx[i]){
//						chkid=i;
//						break;
//					}
//				}
//
//				contr->uniId=n;
//				contr->checkId=chkid;
//				contr->AOV[4]=tmp.re;
//				contr->AOV[5]=tmp.im;
////				if (!executed) {
////					contr->AOV[0]=0xFFFFFFF;
////					contr->AOV[1]=0xFFFFFFF;
////					contr->AOV[2]=0xFFFFFFF;
////					contr->AOV[3]=0xFFFFFFF;
////					contr->AOV[4]=0xFFFFFFF;
////					contr->AOV[5]=0xFFFFFFF;
////					executed=0xFF;
////				}
//				FAULTDET_testPoint();
//
//				even_sum=complex_sum(even_sum,tmp); //192
//				//				even_sum2=complex_sum(even_sum2,tmp); //192
//				//				if (memcmp(&even_sum, &even_sum, sizeof(complex))!=0)
//				//					faulty=0xFF;
//
//				idx=0;
//				tmp.im=0;
//				tmp.re=0;
//
//				contr->AOV[0]=0;
//				contr->AOV[1]=0;
//				contr->AOV[2]=0;
//				contr->AOV[3]=0;
//			} else {
//				contr->AOV[1]+=array_in[n].re;
//				contr->AOV[3]+=array_in[n].im;
//				idx++;
//			}
//#else
//			even_sum=complex_sum(even_sum,n_term); //192
//#endif
//		}
//
//		odd_sum.re=0;
//		odd_sum.im=0;
//
//#ifdef testMode
//		idx=0;
//		mul=0;
//
//		tmp.im=0;
//		tmp.re=0;
//#endif
//
//		for(n=1;n<FFT_LENGTH;n=n+2){
//			complex cmplxexp=complex_exp((-2*M_PI*n*k)/FFT_LENGTH); //96
//			complex n_term = complex_mult(array_in[n], cmplxexp); //192
//
//#ifdef testMode
//			tmp=complex_sum(tmp,n_term); //192
//
//			if (idx<CHECKPERIODICITY/2) {
//				contr->AOV[0]+=array_in[n].re;
//				contr->AOV[2]+=array_in[n].im;
//				idx++;
//			} else if (idx==CHECKPERIODICITY-1) {
//				contr->AOV[1]+=array_in[n].re;
//				contr->AOV[3]+=array_in[n].im;
//
//				mul=n*k;
//				int chkid=checksNum-1;
//				for (int i=0; i<checksNum-1; i++) {
//					if (mul<=checks_idx[i]){
//						chkid=i;
//						break;
//					}
//				}
//
//				contr->uniId=n;
//				contr->checkId=chkid;
//				contr->AOV[4]=tmp.re;
//				contr->AOV[5]=tmp.im;
//
//				FAULTDET_testPoint();
//
//				odd_sum=complex_sum(odd_sum,tmp); //192
//				//				odd_sum2=complex_sum(odd_sum2,tmp); //192
//				//				if (memcmp(&odd_sum, &odd_sum2, sizeof(complex))!=0)
//				//					faulty=0xFF;
//
//				idx=0;
//				tmp.im=0;
//				tmp.re=0;
//
//				contr->AOV[0]=0;
//				contr->AOV[1]=0;
//				contr->AOV[2]=0;
//				contr->AOV[3]=0;
//			} else {
//				contr->AOV[1]+=array_in[n].re;
//				contr->AOV[3]+=array_in[n].im;
//				idx++;
//			}
//#else
//			odd_sum=complex_sum(odd_sum,n_term); //192
//#endif
//		}
//
//		complex out=complex_sum(even_sum,odd_sum);
//#ifdef testMode
//		//		complex out2=complex_sum(even_sum,odd_sum);
//		//		if (memcmp(&out, &out2, sizeof(complex))!=0)
//		//			faulty=0xFF;
//
//#ifndef configFAULTDETECTOR_SOFTWARE
//		FAULTDET_blockIfFaultDetectedInTask();
//#endif
//#endif
//		array_out[k] = out; //192
//	}
//	perf_stop_clock();
//	xil_printf("%u\n", get_clock_L());
//	if (get_clock_U()!=0)
//		xil_printf("err up not 0");
//}



//	for(k=0;k<FFT_LENGTH;k++){
//
//
//		/*X_k=[sum{0,N/2-1} x_2n * e^(i*(-2*pi*2n*k)/N)] + [sum{0,N/2-1} x_(2n+1) * e^(i*(-2*pi*(2n+1)*k)/N)]*/
//		int n;
//		complex even_sum,odd_sum;
//
//		even_sum.re=0;
//		even_sum.im=0;
//
//		//		FAULTDET_testing_injectFault32(even_sum.re, executionId, 32*0+k*LOOP1TOTAL, (32*1)-1+k*LOOP1TOTAL, injectingErrors);
//		//		FAULTDET_testing_injectFault32(even_sum.im, executionId, 32*1+k*LOOP1TOTAL, (32*2)-1+k*LOOP1TOTAL, injectingErrors);
//
//		for(n=0;n<FFT_LENGTH;n=n+2){
//			complex n_term = complex_mult(array_in[n],complex_exp((-2*M_PI*n*k)/FFT_LENGTH)); //192
//
//			complex_sum(even_sum,n_term); //192
//		}
//
//
//		odd_sum.re=0;
//		odd_sum.im=0;
//
//		//		FAULTDET_testing_injectFault32(odd_sum.re, executionId, (32*2)+192*2+k*LOOP1TOTAL, (32*3)-1+192*2+k*LOOP1TOTAL, injectingErrors);
//		//		FAULTDET_testing_injectFault32(odd_sum.im, executionId, (32*3)+192*2+k*LOOP1TOTAL, (32*4)-1+192*2+k*LOOP1TOTAL, injectingErrors);
//
//		for(n=1;n<FFT_LENGTH;n=n+2){
//			complex n_term = complex_mult(array_in[n],complex_exp((-2*M_PI*n*k)/FFT_LENGTH) ); //192
//
//			complex_sum(odd_sum,n_term); //192
//		}
//
//		array_out[k] = complex_sum(even_sum,odd_sum); //192
//	}
//
//
//	contr->uniId=1;
//	contr->checkId=0;
//	//		contr->taskId=0;
//	//		contr->executionId=0;
//	//		contr->command=2;
//	contr->AOV[0]=array_in[0].re;
//	contr->AOV[1]=array_in[0].im;
//	contr->AOV[2]=array_in[1].re;
//	contr->AOV[3]=array_in[1].im;
//	contr->AOV[4]=array_out[0].re;
//	contr->AOV[5]=array_out[0].im;
//	contr->AOV[6]=array_out[1].re;
//	contr->AOV[7]=array_out[1].im;
//	//		FAULTDETECTOR_startCopy(&FAULTDETECTOR_InstancePtr);
//	//		FAULTDET_Test(&contr);
//	//	FAULTDET_testPoint(&contr);
//perf_stop_clock();
//xil_printf("%u\n", get_clock_L());
//if (get_clock_U()!=0)
//	xil_printf("err up not 0");
//}
static void fft_routine_train(){
	int k;

	perf_reset_and_start_clock();

	for(k=0;k<FFT_LENGTH;k++){


		/*X_k=[sum{0,N/2-1} x_2n * e^(i*(-2*pi*2n*k)/N)] + [sum{0,N/2-1} x_(2n+1) * e^(i*(-2*pi*(2n+1)*k)/N)]*/
		int n;
		complex even_sum,odd_sum, even_sum2, odd_sum2;

		contr->AOV[0]=0;
		contr->AOV[1]=0;
		contr->AOV[2]=0;
		contr->AOV[3]=0;

		even_sum.re=0;
		even_sum.im=0;

		int idx=0;
		int mul=0;
		complex tmp;
		tmp.im=0;
		tmp.re=0;


		for(n=0;n<FFT_LENGTH;n=n+2){
			complex cmplxexp=complex_exp((-2*M_PI*n*k)/FFT_LENGTH); //96
			complex n_term = complex_mult(array_in[n], cmplxexp); //192

			tmp=complex_sum(tmp, n_term); //192

			if (idx<CHECKPERIODICITY/2) {
				contr->AOV[0]+=array_in[n].re;
				contr->AOV[2]+=array_in[n].im;
				idx++;
			} else if (idx==CHECKPERIODICITY-1) {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;

				mul=n*k;

				int chkid=checksNum-1;
				for (int i=0; i<checksNum-1; i++) {
					if (mul<=checks_idx[i]){
						chkid=i;
						break;
					}
				}

				contr->uniId=n;
				contr->checkId=chkid;
				contr->AOV[4]=tmp.re;
				contr->AOV[5]=tmp.im;
				FAULTDET_trainPoint();

				even_sum=complex_sum(even_sum,tmp); //192

				idx=0;
				tmp.im=0;
				tmp.re=0;

				contr->AOV[0]=0;
				contr->AOV[1]=0;
				contr->AOV[2]=0;
				contr->AOV[3]=0;
			} else {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;
				idx++;
			}
		}

		odd_sum.re=0;
		odd_sum.im=0;

		idx=0;
		mul=0;

		tmp.im=0;
		tmp.re=0;

		for(n=1;n<FFT_LENGTH;n=n+2){
			complex cmplxexp=complex_exp((-2*M_PI*n*k)/FFT_LENGTH); //96
			complex n_term = complex_mult(array_in[n], cmplxexp); //192

			tmp=complex_sum(tmp,n_term); //192

			if (idx<CHECKPERIODICITY/2) {
				contr->AOV[0]+=array_in[n].re;
				contr->AOV[2]+=array_in[n].im;
				idx++;
			} else if (idx==CHECKPERIODICITY-1) {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;

				mul=n*k;
				int chkid=checksNum-1;
				for (int i=0; i<checksNum-1; i++) {
					if (mul<=checks_idx[i]){
						chkid=i;
						break;
					}
				}

				contr->uniId=n;
				contr->checkId=chkid;
				contr->AOV[4]=tmp.re;
				contr->AOV[5]=tmp.im;
				FAULTDET_trainPoint();

				odd_sum=complex_sum(odd_sum,tmp); //192

				idx=0;
				tmp.im=0;
				tmp.re=0;

				contr->AOV[0]=0;
				contr->AOV[1]=0;
				contr->AOV[2]=0;
				contr->AOV[3]=0;
			} else {
				contr->AOV[1]+=array_in[n].re;
				contr->AOV[3]+=array_in[n].im;
				idx++;
			}
		}

		complex out=complex_sum(even_sum,odd_sum);

		array_out[k] = out; //192
	}
}
#endif


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



float run_pid(pid_controller_t* pid, float error) {
	float output;
	output = error * pid->p;
	pid->integral_sum += ((error * pid->i) + (pid->b * pid->backpropagation)) * TIME_STEP;
	output += pid->integral_sum;
	output += pid->d * (error - pid->prev_error) / TIME_STEP;
	pid->prev_error = error;

	return output;
}

float roll_limiter(float desired_roll, float speed) {
	float limit_perc,limit;

	if (speed <= 140) {
		return CLAMP(desired_roll, -30, 30);
	}
	if (speed >= 300) {
		return CLAMP(desired_roll, -40, 40);
	}

	limit_perc = (speed < 220) ? (speed-140) / 80 : ((speed-220) / 80);

	limit = (speed < 220) ? (30 + limit_perc * 37) : (40 + (1-limit_perc) * 27);

	return CLAMP (desired_roll, -limit, limit);
}

float roll_rate_limiter(float desired_roll_rate, float roll) {
	if (roll < 20 && roll > -20) {
		return CLAMP (desired_roll_rate, -5, 5);
	} else if (roll < 30 && roll > -30) {
		return CLAMP (desired_roll_rate, -3, 3);
	} else {
		return CLAMP (desired_roll_rate, -1, 1);
	}
}

float ailerons_limiter(float aileron) {
	return CLAMP(aileron, -30, 30);
}

void latnav_train() {

	//	perf_reset_and_start_clock();


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


	//	for(i=0; i<1; i++) {

	float desired_roll,actual_roll,desired_roll_rate,actual_roll_rate,desired_ailerons,actual_ailerons;

	float err=curr_heading-r4;
	float pid_heading_backpropagation_orig=pid_heading.backpropagation;
	float err_orig=err;

	desired_roll = run_pid(&pid_heading, err);
	actual_roll = roll_limiter(desired_roll, 400);

	//	contr->uniId=1;
	//	contr->checkId=0;
	//	//		contr->taskId=0;
	//	//		contr->executionId=0;
	//	//		contr->command=2;
	//	contr->AOV[0]=pid_heading.backpropagation;
	//	contr->AOV[1]=err;
	//	contr->AOV[2]=desired_roll;
	//	contr->AOV[3]=actual_roll;
	//	contr->AOV[4]=0;
	//	//		FAULTDETECTOR_startCopy(&FAULTDETECTOR_InstancePtr);
	//	//		FAULTDET_Test(&contr);
	//	FAULTDET_trainPoint(&contr);

	contr->uniId=1;
	contr->checkId=0;
	//		contr->taskId=0;
	//		contr->executionId=0;
	//		contr->command=2;
	contr->AOV[0]=curr_heading;
	contr->AOV[1]=r4;
	contr->AOV[2]=desired_roll;
	contr->AOV[3]=actual_roll;
	//		FAULTDETECTOR_startCopy(&FAULTDETECTOR_InstancePtr);
	//		FAULTDET_Test(&contr);
	FAULTDET_trainPoint(&contr);


	pid_heading.backpropagation = actual_roll - desired_roll;

	float pid_roll_backpropagation_orig=pid_roll.backpropagation;

	float err1=curr_roll - actual_roll;
	float err1_orig=curr_roll - actual_roll;
	desired_roll_rate = run_pid(&pid_roll, err1);

	actual_roll_rate = roll_rate_limiter(desired_roll_rate, curr_roll);

	//	contr->uniId=1;
	//	contr->checkId=1;
	//	//		contr->taskId=0;
	//	//		contr->executionId=0;
	//	//		contr->command=2;
	//	contr->AOV[0]=pid_roll.backpropagation;
	//	contr->AOV[1]=err1;
	//	contr->AOV[2]=desired_roll_rate;
	//	contr->AOV[3]=actual_roll_rate;
	//	contr->AOV[4]=curr_roll;
	//	//		FAULTDETECTOR_startCopy(&FAULTDETECTOR_InstancePtr);
	//	//		FAULTDET_Test(&contr);
	//	FAULTDET_trainPoint(&contr);

	pid_roll.backpropagation = actual_roll_rate - desired_roll_rate;
	pid_roll_backpropagation_orig=pid_roll.backpropagation;

	float err2=curr_roll_rate - actual_roll_rate;
	float err2_orig=err2;
	desired_ailerons = run_pid(&pid_roll, err2);
	actual_ailerons = ailerons_limiter(desired_ailerons);

	//	contr->uniId=1;
	//	contr->checkId=2;
	//	//		contr->taskId=0;
	//	//		contr->executionId=0;
	//	//		contr->command=2;
	//	contr->AOV[0]=pid_roll.backpropagation;
	//	contr->AOV[1]=err2;
	//	contr->AOV[2]=desired_ailerons;
	//	contr->AOV[3]=actual_ailerons;
	//	contr->AOV[4]=0;
	//	//		FAULTDETECTOR_startCopy(&FAULTDETECTOR_InstancePtr);
	//	//		FAULTDET_Test(&contr);
	//	FAULTDET_trainPoint(&contr);

	pid_roll.backpropagation = actual_ailerons - desired_ailerons;


	//	contr->uniId=1;
	//	contr->checkId=3;
	//	//		contr->taskId=0;
	//	//		contr->executionId=0;
	//	//		contr->command=2;
	//	contr->AOV[0]=curr_heading;
	//	contr->AOV[1]=curr_roll;
	//	contr->AOV[2]=curr_roll_rate;
	//	contr->AOV[3]=actual_ailerons;
	//	contr->AOV[4]=0;


	contr->uniId=1;
	contr->checkId=1;
	//		contr->taskId=0;
	//		contr->executionId=0;
	//		contr->command=2;
	contr->AOV[0]=curr_roll;
	contr->AOV[1]=actual_roll_rate;
	contr->AOV[2]=curr_roll_rate;
	contr->AOV[3]=actual_ailerons;

	FAULTDET_trainPoint(&contr);

	/* Just a random plane model*/

	//	curr_heading += curr_roll/10 * TIME_STEP;
	//	curr_roll += curr_roll_rate * TIME_STEP;
	//	curr_roll_rate += desired_ailerons / 5;

	//	}
	//	perf_stop_clock();
	//	clk_count_train_total+=get_clock_L();
	//	clk_count_train_total_times++;
	//	if (get_clock_U()!=0)
	//		xil_printf("err up not 0");

}


void latnav_test_fault() {

	contr->uniId=1;
	contr->checkId=0;
	contr->AOV[0]=0xFFFFF;
	contr->AOV[1]=0xFFFFF;
	contr->AOV[2]=0xFFFFF;
	contr->AOV[3]=0xFFFFF;
	FAULTDET_testPoint();
#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_blockIfFaultDetectedInTask();
#endif
}

void latnav_test() {


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

	//	XFaultdetector FAULTDETECTOR_InstancePtr=FAULTDET_getInstancePtr();
	//	FAULTDETECTOR_controlStr* contr=&contrtmp;
	//	FAULTDETECTOR_controlStr contr;
	curr_heading = r1;
	curr_roll = r2;
	curr_roll_rate = r3;

	perf_reset_and_start_clock();

	//	for(i=0; i<1; i++) {
	FAULTDET_initFaultDetection();

	float desired_roll,actual_roll,desired_roll_rate,actual_roll_rate,desired_ailerons,actual_ailerons;

	float err=curr_heading-r4;
	float pid_heading_backpropagation_orig=pid_heading.backpropagation;
	float err_orig=err;

	desired_roll = run_pid(&pid_heading, err);
	actual_roll = roll_limiter(desired_roll, 400);

	contr->uniId=1;
	contr->checkId=0;
	contr->AOV[0]=0xFFFFF;
	contr->AOV[1]=0xFFFFF;
	contr->AOV[2]=0xFFFFF;
	contr->AOV[3]=0xFFFFF;
	FAULTDET_testPoint();

	//	contr->uniId=1;
	//	contr->checkId=0;
	//	//		contr->taskId=0;
	//	//		contr->executionId=0;
	//	//		contr->command=2;
	//	contr->AOV[0]=curr_heading;
	//	contr->AOV[1]=r4;
	//	contr->AOV[2]=desired_roll;
	//	contr->AOV[3]=actual_roll;
	//	//		FAULTDETECTOR_startCopy(&FAULTDETECTOR_InstancePtr);
	//	//		FAULTDET_Test(&contr);
	//	FAULTDET_testPoint();


	pid_heading.backpropagation = actual_roll - desired_roll;

	float pid_roll_backpropagation_orig=pid_roll.backpropagation;

	float err1=curr_roll - actual_roll;
	float err1_orig=curr_roll - actual_roll;
	desired_roll_rate = run_pid(&pid_roll, err1);

	actual_roll_rate = roll_rate_limiter(desired_roll_rate, curr_roll);

	pid_roll.backpropagation = actual_roll_rate - desired_roll_rate;
	pid_roll_backpropagation_orig=pid_roll.backpropagation;

	float err2=curr_roll_rate - actual_roll_rate;
	float err2_orig=err2;
	desired_ailerons = run_pid(&pid_roll, err2);
	actual_ailerons = ailerons_limiter(desired_ailerons);


	pid_roll.backpropagation = actual_ailerons - desired_ailerons;

	//	contr->uniId=1;
	//	contr->checkId=0;
	//	//		contr->taskId=0;
	//	//		contr->executionId=0;
	//	//		contr->command=2;
	//	contr->AOV[0]=curr_heading;
	//	contr->AOV[1]=curr_roll;
	//	contr->AOV[2]=curr_roll_rate;
	//	contr->AOV[3]=actual_ailerons;
	//
	//	FAULTDET_testPoint(&contr);

	contr->uniId=2;
	contr->checkId=1;
	//		contr->taskId=0;
	//		contr->executionId=0;
	//		contr->command=2;
	contr->AOV[0]=curr_roll;
	contr->AOV[1]=actual_roll_rate;
	contr->AOV[2]=curr_roll_rate;
	contr->AOV[3]=actual_ailerons;

	FAULTDET_testPoint();

	/* Just a random plane model*/

#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_blockIfFaultDetectedInTask();
#endif

	//	curr_heading += curr_roll/10 * TIME_STEP;
	//	curr_roll += curr_roll_rate * TIME_STEP;
	//	curr_roll_rate += desired_ailerons / 5;

	//	}
	perf_stop_clock();
	xil_printf("%u\n", get_clock_L());
	if (get_clock_U()!=0)
		xil_printf("err up not 0");
}
#endif


#ifdef imgscalingBench
//SOURCED FROM ABSURD BENCHMARK SUITE BY HEAPLAB - POLIMI
#define SCALING_FACTOR 2

#define IMG_HEIGHT 16
#define IMG_WIDTH 16
static unsigned char mat_in[IMG_HEIGHT][IMG_WIDTH];
unsigned char mat_out[IMG_HEIGHT][IMG_WIDTH];

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
		//			8000	else
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
	//				xil_printf("initial value %x%x", clk_hi, clk_l);
	//xil_printf(" initial time, lsbits only %d", get_delay_for_number_of_cycles(clk_l));


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
#ifdef detectionPerformanceMeasurement
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
#ifdef detectionPerformanceMeasurement
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
#ifdef detectionPerformanceMeasurement
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


			FAULTDET8000_testing_injectFault32(i0, executionId, 576, 607, injectingErrors);
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
#ifdef detectionPerformanceMeasurement
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
#ifdef detectionPerformanceMeasurement
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
#ifdef detectionPerformanceMeasurement
						injectingErrors,
#endif
						6, //SIZE OF THIS SPECIFIC AOV (<=FAULTDETECTOR_MAX_AOV_DIM , unused elements will be initialised to 0)
						&col0, &col1, &col2, &col3, &res, &df);
				//#endif
			}

#ifdef detectionPerformanceMeasurement
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
	//				xil_printf("final value %x%x", clk_hi, clk_l);
	//				xil_printf(" final time, lsbits only %d", get_delay_for_number_of_cycles(clk_l));


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


#ifdef detectionPerformanceMeasurement
	//#ifndef trainMode
	if (executionId>=-1) {
		xil_printf(" total: %d ", FAULTDET_testing_getTotal());
		xil_printf(" ok: %d ", FAULTDET_testing_getOk());
		if (injectingErrors) {
			xil_printf(" fn: %d\n", FAULTDET_testing_getFalseNegatives());
		} else {
			xil_printf(" fp: %d\n", FAULTDET_testing_getFalsePositives());
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


#include "perf_timer.h"

//static void prvImgAcquireAndScaleTask( void *pvParameters )

static void prvTaskFour( void *pvParameters )
{
	//#ifdef configIGNORE_FAULTS_DETECTED_BY_SW_FAULT_DETECTOR
	//#ifndef configSCHEDULER_SOFTWARE
	xPortSchedulerDisableIntr(); //if uncommented, task will execute continuously
	//#endif
	//#endif

	contr=FAULTDET_initFaultDetection();
	for (int i=0; i<FAULTDETECTOR_MAX_AOV_DIM;i++)
		contr->AOV[i]=0;

	for (int i=0; i<100000; i++) {}

	random_set_seed(1);

#ifdef ANNBench

	init_train_data();
	train_ann_routine();

	//	init_test_data();
	//	forward_pass_test_burst(-2);

	for (int i=-800; i<-1; i++) {
		init_test_data();
		forward_pass_test_burst_train();
	}


	FAULTDET_dumpRegionsToDest(trainedRegions, n_regions);
#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_start ();
#endif


	for (int i=0; i<100000; i++) {
		forward_pass_test_burst_test_fault();

		vTaskDBG_setExecutionModeFault();

		init_test_data();

		forward_pass_test_burst_test();

//		FAULTDET_dumpRegionsToDest(trainedRegions, n_regions); //DEBUG

#ifndef configFAULTDETECTOR_SOFTWARE
		FAULTDET_StopRunMode();
#endif
		vTaskStartFaultDetector(0, trainedRegions, n_regions);
	}
#endif

#ifdef gaussianBench

	gaussian_kernel_init();

	contr=FAULTDET_initFaultDetection();

	for (int i=-1000; i<-1; i++) {
		init_img_matrix();
		gauss_filter_routine_train();
	}

	FAULTDET_dumpRegions(); //to sd card
	FAULTDET_dumpRegionsToDest(trainedRegions, n_regions); //to ram
#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_start ();
#endif

	for (int i=0; i<10000; i++) {
		init_img_matrix();

		gauss_filter_routine_test_fault();

		vTaskDBG_setExecutionModeFault();

		gauss_filter_routine_test();

//		FAULTDET_dumpRegionsToDest(trainedRegions, n_regions); //DEBUG

#ifndef configFAULTDETECTOR_SOFTWARE
		FAULTDET_StopRunMode();
#endif
		vTaskStartFaultDetector(0, trainedRegions, n_regions);
	}

#endif

#ifdef FFTBench
	int step=FFT_LENGTH*FFT_LENGTH/checksNum;
	int part=0;
	for (int i=0; i<checksNum-1; i++) {
		part+=step;
		checks_idx[i]=part;
	}

#ifdef testMode
	for (int executionId=-5000; executionId<-1; executionId++) {
		for(int i=0; i<FFT_LENGTH;i++){
			complex x;
			x.re=random_get();
			x.im=random_get();

			array_in[i]=x;
		}
		fft_routine_train();
	}
	FAULTDET_dumpRegions(); //to SD card
	FAULTDET_dumpRegionsToDest(trainedRegions, n_regions);
#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_start ();
#endif
#endif
	for (int i=0; i<10000; i++) {
		for(int i=0; i<FFT_LENGTH;i++){
			complex x;
			x.re=random_get();
			x.im=random_get();

			array_in[i]=x;
		}
#ifdef testMode
		faulty=0x0;
#endif
		fft_routine_test_fault();

		vTaskDBG_setExecutionModeFault();

		fft_routine_test();

//		FAULTDET_dumpRegionsToDest(trainedRegions, n_regions); //DEBUG

#ifndef configFAULTDETECTOR_SOFTWARE
		FAULTDET_StopRunMode();
#endif
		vTaskStartFaultDetector(0, trainedRegions, n_regions);
	}
#endif

	//	for (int i=0; i<FAULTDETECTOR_MAX_CHECKS; i++) {
	//		n_regions[i]=0;
	//		for (int j=0; j<FAULTDETECTOR_MAX_REGIONS; j++) {
	//			for (int k=0; k<=FAULTDETECTOR_MAX_AOV_DIM; k++) {
	//				trainedRegions[i][j].center[k]=0.0f;
	//				trainedRegions[i][j].max[k]=0.0f;
	//				trainedRegions[i][j].min[k]=0.0f;
	//			}
	//		}
	//	}
	//
	//	FAULTDET_dumpRegionsToDest(trainedRegions, n_regions);

#ifdef latnavBench
	for (int i=0; i<4000; i++) {
		r1=random_get();
		r2=random_get();
		r3=random_get();
		r4=random_get();
		latnav_train();
	}

	FAULTDET_dumpRegionsToDest(trainedRegions, n_regions);
	FAULTDET_dumpRegions(); //to sd card

#ifndef configFAULTDETECTOR_SOFTWARE
	FAULTDET_start ();
#endif

	for (int i=0; i<100000; i++) {
		r1=random_get();
		r2=random_get();
		r3=random_get();
		r4=random_get();


		//				injectingErrors=0x0;
		//				FAULTDET_testing_resetGoldens();

		latnav_test_fault();

		vTaskDBG_setExecutionModeFault();

		latnav_test();

		//		FAULTDET_dumpRegionsToDest(trainedRegions, n_regions); //FOR DEBUG PURPOSES
#ifndef configFAULTDETECTOR_SOFTWARE
		FAULTDET_StopRunMode();
#endif
		vTaskStartFaultDetector(0, trainedRegions, n_regions);

	}
#endif
	for (;;) {}
}
