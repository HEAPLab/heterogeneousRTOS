#ifndef FAULTDETECTOR_SW_HEADER
#define FAULTDETECTOR_SW_HEADER

#include "xil_types.h"

//#define COMMAND_TEST 2
//#define COMMAND_TRAIN 3

#ifndef XFAULTDETECTOR_H
#define FAULTDETECTOR_MAX_CHECKS 64
#define FAULTDETECTOR_MAX_TASKS 16

#define FAULTDETECTOR_MAX_AOV_DIM 8

#define FAULTDETECTOR_MAX_REGIONS 16

#define FAULTDETECTOR_THRESH_CONSTANT (1e-10)

typedef struct {
	u8 checkId;
	u8 taskId;
	u8 executionId;
	u16 uniId;
	char command;
	char gap0[2];
	float AOV[FAULTDETECTOR_MAX_AOV_DIM];
} FAULTDETECTOR_controlStr;

typedef struct {
	float min[FAULTDETECTOR_MAX_AOV_DIM];
	float max[FAULTDETECTOR_MAX_AOV_DIM];
	float center[FAULTDETECTOR_MAX_AOV_DIM];
} FAULTDETECTOR_region_t;
#endif

char FAULTDETECTOR_SW_test(FAULTDETECTOR_controlStr* in);
void FAULTDETECTOR_SW_train(FAULTDETECTOR_controlStr* in);

#endif
