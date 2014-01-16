#ifndef MUTEX_HPP
#define MUTEX_HPP

#include <pthread.h>

class Mutex
{
	private:
		pthread_mutex_t m_mutex;

	public:
		Mutex();
		
		void lock();
		
		void unlock();
};

#endif // MUTEX_HPP
