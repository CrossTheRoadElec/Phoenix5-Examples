#include "ConcurrentScheduler.h"

namespace CTRE {
namespace Tasking{
namespace Schedulers {

ConcurrentScheduler::ConcurrentScheduler(int timeout) {
	_timeout = timeout;
}
ConcurrentScheduler::~ConcurrentScheduler(){}
void ConcurrentScheduler::Add(ILoopable *aLoop) {
	for(auto loop : _loops){
		if(loop == aLoop)
		{
			return;	//Already have loop
		}
	}
	//We add another loop and enable state
	_loops.insert(aLoop);
	_enabs.insert(true);
}
void ConcurrentScheduler::RemoveAll() {
	_loops.clear();
}
void ConcurrentScheduler::Start(ILoopable* toStart) {
//	auto iter = _loops.find(toStart);
//	ILoopable* loop = *iter;
//	loop->OnStart();
	for(auto loop : _loops){
		if(loop == toStart)
		{
			return;	//Already have loop
		}
	}
	_loops.insert(toStart);	//Just throw it back in
}

void ConcurrentScheduler::Stop(ILoopable* toStop) {
//	auto iter = _loops.find(toStop);
//	ILoopable* loop = *iter;
//	loop->OnStop();
	for(auto loop : _loops){
		if(loop == toStop)
		{
			_loops.erase(toStop);	//remove from the set, would this remove persistent stuff?
		}
	}
	return;
}
void ConcurrentScheduler::StartAll() {	//All Loops
	for(auto loop : _loops){
		loop->OnStart();
	}
	_running = true;
}
void ConcurrentScheduler::StopAll() {	//All Loops
	for(auto loop : _loops){
		loop->OnStop();
	}
	_running = false;
}
void ConcurrentScheduler::Process() {
	for(auto loop : _loops){
		loop->OnLoop();
	}
}
/* ILoopable */
bool ConcurrentScheduler::Iterated() {
	return _iterated;
}
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
	if (_running == false)
		return true;
	else
		return false;
}
}}}
