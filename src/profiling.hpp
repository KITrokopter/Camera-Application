#ifndef PROFILING_HPP
#define PROFILING_HPP

#include <time.h>

/**
 * Tools for profiling the tracking.
 *
 * @author Sebastian Schmidt
 */

// Use this to output profiling information.
// #define QC_PROFILE

#ifdef QC_PROFILE

#define START_CLOCK(clock) long clock = getNanoTime();

#define STOP_CLOCK(clock, message) std::cout << message << getNanoTime() - clock << std::endl;

#else
#define START_CLOCK(clock) ;
#define STOP_CLOCK(clock, message) ;
#endif

/**
 * Returns the current system time.
 *
 * @return The current system time in nano second resolution.
 */
long int getNanoTime();

#endif // PROFILING_HPP
