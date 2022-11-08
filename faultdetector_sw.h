#ifndef FAULTDETECTOR
#define FAULTDETECTOR

#define COMMAND_TEST 2
#define COMMAND_TRAIN 3


#define FAULTDETECTOR_MAX_CHECKS 64
#define FAULTDETECTOR_MAX_TASKS 16

#define FAULTDETECTOR_MAX_AOV_DIM 8

#define FAULTDETECTOR_MAX_REGIONS 16

#define FAULTDETECTOR_THRESH 1e-10

struct FAULTDETECTOR_controlStr {
	ap_uint<8> checkId;
	ap_uint<8> taskId;
	ap_uint<8> executionId;
	ap_uint<16> uniId;
	char command;
	char gap0[2];
	float AOV[FAULTDETECTOR_MAX_AOV_DIM];
};

typedef struct REGION_T{
			float min[FAULTDETECTOR_MAX_AOV_DIM];
			float max[FAULTDETECTOR_MAX_AOV_DIM];
			float center[FAULTDETECTOR_MAX_AOV_DIM];
		} FAULTDETECTOR_region_t;

void FAULTDETECTOR_SW_testPoint (FAULTDETECTOR_controlStr& in);

#endif