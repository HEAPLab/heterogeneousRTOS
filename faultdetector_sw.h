#ifndef FAULTDETECTOR_SW_HEADER
#define FAULTDETECTOR_SW_HEADER

#include "xil_types.h"

//#define COMMAND_TEST 2
//#define COMMAND_TRAIN 3

#ifndef XFAULTDETECTOR_H
#define FAULTDETECTOR_MAX_CHECKS 4
#define FAULTDETECTOR_MAX_TASKS 4

#define FAULTDETECTOR_MAX_AOV_DIM 4

#define FAULTDETECTOR_MAX_REGIONS 8

typedef struct {
	u16 uniId;
	u8 checkId;
	u8 executionId;
	float AOV[FAULTDETECTOR_MAX_AOV_DIM];
	u8 taskId;
	char command;
	char gap0[2];
} __attribute__((packed, aligned(4))) FAULTDETECTOR_controlStr;

typedef struct {
	float min[FAULTDETECTOR_MAX_AOV_DIM];
	float max[FAULTDETECTOR_MAX_AOV_DIM];
	float center[FAULTDETECTOR_MAX_AOV_DIM];
} FAULTDETECTOR_region_t;

typedef struct {
	u16 uniId;
	u8 checkId;
	u8 executionId;
	float AOV[FAULTDETECTOR_MAX_AOV_DIM];
} __attribute__((packed, aligned(4))) FAULTDETECTOR_testpointDescriptorStr;

typedef struct {
	u16 uniId;
	u8 checkId;
	u8 executionId;
} __attribute__((packed, aligned(4))) FAULTDETECTOR_testpointShortDescriptorStr;

#endif

void FAULTDETECTOR_SW_initRegions(FAULTDETECTOR_region_t trainedRegions[FAULTDETECTOR_MAX_CHECKS][FAULTDETECTOR_MAX_REGIONS], u8 n_regions_in[FAULTDETECTOR_MAX_CHECKS]);
void FAULTDETECTOR_SW_dumpRegions(FAULTDETECTOR_region_t trainedRegions[FAULTDETECTOR_MAX_CHECKS][FAULTDETECTOR_MAX_REGIONS], u8 n_regions_out[FAULTDETECTOR_MAX_CHECKS]);

char FAULTDETECTOR_SW_test(FAULTDETECTOR_controlStr* in);
void FAULTDETECTOR_SW_train(FAULTDETECTOR_controlStr* in);

#endif
