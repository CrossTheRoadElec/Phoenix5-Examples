#include "ConcurrentScheduler.h"

namespace CTRE {
namespace Tasking{
namespace Schedulers {

ConcurrentScheduler::ConcurrentScheduler() {
}
ConcurrentScheduler::~ConcurrentScheduler(){}
void ConcurrentScheduler::Add(ILoopable *aLoop, bool enable) {
	_loops.push_back(aLoop);
	_enabs.push_back(enable);
}
void ConcurrentScheduler::RemoveAll() {
	_loops.clear();
	_enabs.clear();
}
void ConcurrentScheduler::Start(ILoopable* toStart) {
	for (int i = 0; i < (int)_loops.size(); ++i) {
		ILoopable* lp = (ILoopable*) _loops[i];

		if (lp == toStart) {
			_enabs[i] = true;
			lp->OnStart();
			return;
		}
	}

}
void ConcurrentScheduler::Stop(ILoopable* toStop) {
	for (int i = 0; i < (int)_loops.size(); ++i) {
		ILoopable* lp = (ILoopable*) _loops[i];

		if (lp == toStop) {
			_enabs[i] = false;
			lp->OnStop();
			return;
		}
	}
}
void ConcurrentScheduler::StartAll() {	//All Loops
	for(auto loop : _loops){
		loop->OnStart();
	}
	for(auto enable : _enabs){
		enable = true;
	}
}
void ConcurrentScheduler::StopAll() {	//All Loops
	for(auto loop : _loops){
		loop->OnStop();
	}
	for(auto enable : _enabs){
		enable = false;
	}
}
void ConcurrentScheduler::Process() {
	for(int i = 0; i< (int)_loops.size(); ++i){
		ILoopable* loop = (ILoopable*)_loops[i];
		bool en = (bool)_enabs[i];
		if(en){
			loop->OnLoop();
		}
		else
		{
			/* Current ILoopable is turned off, don't call OnLoop for it */
		}
	}
}
/* ILoopable */
void ConcurrentScheduler::OnStart() {
	ConcurrentScheduler::StartAll();
}
void ConcurrentScheduler::OnLoop() {
	ConcurrentScheduler::Process();
}
void ConcurrentScheduler::OnStop() {
	ConcurrentScheduler::StopAll();
}
bool ConcurrentScheduler::IsDone() {
		return false;
}
}}}
