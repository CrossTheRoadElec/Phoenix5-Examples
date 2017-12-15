package org.usfirst.frc.team3539.robot.Framework;

import com.ctre.phoenix.ILoopable;
import java.util.ArrayList;

public class ConcurrentScheduler implements com.ctre.phoenix.ILoopable {
	ArrayList<com.ctre.phoenix.ILoopable> _loops = new ArrayList<com.ctre.phoenix.ILoopable>();
	ArrayList<Boolean> _enabs = new ArrayList<Boolean>();

	public ConcurrentScheduler() {
	}

	public void Add(ILoopable newLoop, boolean bEnabled) {
		for (ILoopable loop : _loops) {
			if (loop == newLoop)
				return; /* Loop exists, ignore it */
		}
		_loops.add(newLoop);
		_enabs.add(bEnabled);

		if (bEnabled)
			Start(newLoop);
	}

	/* Method overloaded to provide a default state */
	public void Add(ILoopable newLoop) {
		for (ILoopable loop : _loops) {
			if (loop == newLoop)
				return; /* Loop exists, ignore it */
		}
		_loops.add(newLoop);
		_enabs.add(true);

		if (true)
			Start(newLoop);
	}

	public void Start(ILoopable toStart) {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			if (lp == toStart) {
				lp.OnStart();
				_enabs.set(i, true);

				return;
			}
		}
		System.out.format("CTR: Could not find object in scheduler");
	}

	public void Stop(ILoopable toStart) {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);
			boolean en = (boolean) _enabs.get(i);

			if (lp == toStart) {
				if (en == true) {
					lp.OnStop();
					_enabs.set(i, false);
				}
				return;
			}
		}
		System.out.format("CTR: Could not find object in scheduler");
	}

	public void RemoveAll() {
		_loops.clear();
		_enabs.clear();
	}

	public void StartAll() {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			lp.OnStart();
			_enabs.set(i, true);
		}
	}

	public void StopAll() {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			lp.OnStop();
			_enabs.set(i, false);
		}
	}

	public void Process() {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);
			boolean en = (boolean) _enabs.get(i);

			if (en) {
				lp.OnLoop();
			} else {
				/* Current ILoopable is turned off, don't call OnLoop() for it */
			}
		}
	}

	// --- ILoopable ---/
	public void OnStart() {
		StartAll();
	}

	public void OnLoop() {
		Process();
	}

	public boolean IsDone() {
		return false;
	}

	public void OnStop() {
		StopAll();
	}
}
