#pragma once

#include <vector>
#include "ctrlib/Tasking/ILoopable.h"
#include "ctrlib/Tasking/IProcessable.h"

namespace CTRE {
namespace Tasking {
namespace Schedulers {

class ConcurrentScheduler: public ILoopable, public IProcessable{
public:
	std::vector<ILoopable*> _loops;
	std::vector<bool> _enabs;

	ConcurrentScheduler();
	virtual ~ConcurrentScheduler();
	void Add(ILoopable *aLoop, bool enable = true);
	void RemoveAll();
	void Start(ILoopable *toStart);
	void Stop(ILoopable *toStop);
	void StartAll();
	void StopAll();

	//IProcessable
	void Process();

	//ILoopable
	bool Iterated();
	void OnStart();
	void OnLoop();
	void OnStop();
	bool IsDone();
};
}}}
