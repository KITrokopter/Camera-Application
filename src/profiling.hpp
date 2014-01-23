#ifndef PROFILING_HPP
#define PROFILING_HPP

#include <time.h>

// Use this to output profiling information.
#define QC_PROFILE

#ifdef QC_PROFILE

#define START_CLOCK(clock) long clock = getNanoTime();

#define STOP_CLOCK(clock, message) std::cout << message << getNanoTime() - clock << std::endl;

#else
#define START_CLOCK(clock) ;
#define STOP_CLOCK(clock, message) ;
#endif

long int getNanoTime()
{
	timespec ts;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
	return 1000000000 * ts.tv_sec + ts.tv_nsec;
}

#endif // PROFILING_HPP
