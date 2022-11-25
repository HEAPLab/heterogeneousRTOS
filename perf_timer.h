#ifndef PERF_TIMER_HEADER
#define PERF_TIMER_HEADER

void perf_start_clock(void);
unsigned int get_number_of_cycles_for_delay(unsigned int delay);
unsigned int get_delay_for_number_of_cycles(unsigned int number_of_cycles);

void perf_stop_clock(void);
void perf_reset_clock(void);
void perf_reset_and_start_clock();

volatile unsigned long long int get_clock();
volatile unsigned int get_clock_L();
volatile unsigned int get_clock_U();


#endif
