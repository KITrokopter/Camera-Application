#include "Mutex.hpp"

/**
 * Creates a new pthread mutex.
 */
Mutex::Mutex()
{
	pthread_mutex_init(&m_mutex, NULL);
}

/**
 * Locks the pthread mutex.
 */
void Mutex::lock()
{
	pthread_mutex_lock(&m_mutex);
}

/**
 * Unlocks the pthread mutex.
 */
void Mutex::unlock()
{
	pthread_mutex_unlock(&m_mutex);
}

/**
 * Destroys the pthread mutex.
 */
Mutex::~Mutex()
{
	pthread_mutex_destroy(&m_mutex);
}