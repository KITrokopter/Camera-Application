#ifndef MUTEX_HPP
#define MUTEX_HPP

#include <pthread.h>

/**
 * C++ interface for pthread mutexes.
 *
 * @author http://openkinect.org/wiki/C%2B%2BOpenCvExample
 * @author Sebastian Schmidt
 */
class Mutex {
private:
	pthread_mutex_t m_mutex;

public:
	Mutex();
	void lock();
	void unlock();
	~Mutex();
};

#endif // MUTEX_HPP
