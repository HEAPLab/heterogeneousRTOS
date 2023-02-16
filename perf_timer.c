#include "perf_timer.h"

/* For delay calculation using global timer */
#define SCU_GLOBAL_TIMER_COUNT_L32	0xF8F00200
#define SCU_GLOBAL_TIMER_COUNT_U32	0xF8F00204
#define SCU_GLOBAL_TIMER_CONTROL	0xF8F00208

#define APU_FREQ  666666687


/* start timer */
void perf_start_clock(void)
{
	*(volatile unsigned int*)SCU_GLOBAL_TIMER_CONTROL = ((1 << 0) | // Timer Enable
			(1 << 3) | // Auto-increment
			(0 << 8) // Pre-scale
	); 
}



/* Compute mask for given delay in miliseconds*/
unsigned int get_number_of_cycles_for_delay(unsigned int delay)
{
	// GTC is always clocked at 1/2 of the CPU frequency (CPU_3x2x)
	return (APU_FREQ*delay/(2*1000));

}

unsigned int get_delay_for_number_of_cycles(unsigned int number_of_cycles)
{
	return ((2*1000)*number_of_cycles/APU_FREQ);
}

/* stop timer */
void perf_stop_clock(void)
{
	*(volatile unsigned int*)SCU_GLOBAL_TIMER_CONTROL = 0;
}


/* stop timer and reset timer count regs */
void perf_reset_clock(void)
{
	perf_stop_clock();
	*(volatile unsigned int*)SCU_GLOBAL_TIMER_COUNT_L32 = 0;
	*(volatile unsigned int*)SCU_GLOBAL_TIMER_COUNT_U32 = 0;
}

void perf_reset_and_start_clock()
{
	perf_reset_clock();
	perf_start_clock();
}

typedef struct timerLongStr {
	unsigned int lo;
	unsigned int hi;
};

volatile unsigned long long int get_clock() {
	return *(volatile unsigned long long int*)SCU_GLOBAL_TIMER_COUNT_L32;
}

volatile unsigned int get_clock_L() {
	return *(volatile unsigned int*)SCU_GLOBAL_TIMER_COUNT_L32;
}

volatile unsigned int get_clock_U() {
	return *(volatile unsigned int*)SCU_GLOBAL_TIMER_COUNT_U32;
}

