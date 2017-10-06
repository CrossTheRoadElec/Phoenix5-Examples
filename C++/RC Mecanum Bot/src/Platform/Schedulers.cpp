#include "Platform/Schedulers.h"

CTRE::Tasking::Schedulers::ConcurrentScheduler* Schedulers::PeriodicTasks = new CTRE::Tasking::Schedulers::ConcurrentScheduler(10);
