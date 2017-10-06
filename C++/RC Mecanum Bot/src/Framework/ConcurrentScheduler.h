#pragma once

#include "set"
#include "ctrlib/Tasking/ILoopable.h"
#include "ctrlib/Tasking/IProcessable.h"

namespace CTRE {
namespace Tasking {
namespace Schedulers {

class ConcurrentScheduler: public ILoopable, public IProcessable{
public:
	bool _running = true;
	int _timeout;
	bool _iterated = false;
	std::set<ILoopable*> _loops;
	std::set<bool> _enabs;

	ConcurrentScheduler(int timeout);
	virtual ~ConcurrentScheduler();
	void Add(ILoopable *aLoop);
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
