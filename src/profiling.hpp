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

long int getNanoTime();

#endif // PROFILING_HPP
