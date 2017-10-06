#pragma once

#include "Framework/ConcurrentScheduler.h"

class Schedulers{
public:
	/* The schedulers. Minimally you will likely want one periodic scheduler to run the normal tasks.
	 * Additionally schedulers could be ConsecutiveSchedulers for entire autonomous movments or preconfigure manuervers.
	 * Use "public static" because they are single objects(singleton). */
	static CTRE::Tasking::Schedulers::ConcurrentScheduler* PeriodicTasks;
};
