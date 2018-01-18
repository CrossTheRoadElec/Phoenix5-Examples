package org.usfirst.frc.team3539.robot.Framework;

import com.ctre.phoenix.ILoopable;
import java.util.ArrayList;

public class ConcurrentScheduler implements com.ctre.phoenix.ILoopable {
	ArrayList<com.ctre.phoenix.ILoopable> _loops = new ArrayList<com.ctre.phoenix.ILoopable>();
	ArrayList<Boolean> _enabs = new ArrayList<Boolean>();

	public ConcurrentScheduler() {
	}

	public void add(ILoopable newLoop, boolean bEnabled) {
		for (ILoopable loop : _loops) {
			if (loop == newLoop)
				return; /* Loop exists, ignore it */
		}
		_loops.add(newLoop);
		_enabs.add(bEnabled);

		if (bEnabled)
			start(newLoop);
	}

	/* Method overloaded to provide a default state */
	public void add(ILoopable newLoop) {
		for (ILoopable loop : _loops) {
			if (loop == newLoop)
				return; /* Loop exists, ignore it */
		}
		_loops.add(newLoop);
		_enabs.add(true);

		if (true)
			start(newLoop);
	}

	public void start(ILoopable toStart) {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			if (lp == toStart) {
				lp.onStart();
				_enabs.set(i, true);

				return;
			}
		}
		System.out.format("CTR: Could not find object in scheduler");
	}

	public void stop(ILoopable toStart) {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);
			boolean en = (boolean) _enabs.get(i);

			if (lp == toStart) {
				if (en == true) {
					lp.onStop();
					_enabs.set(i, false);
				}
				return;
			}
		}
		System.out.format("CTR: Could not find object in scheduler");
	}

	public void removeAll() {
		_loops.clear();
		_enabs.clear();
	}

	public void startAll() {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			lp.onStart();
			_enabs.set(i, true);
		}
	}

	public void stopAll() {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			lp.onStop();
			_enabs.set(i, false);
		}
	}

	public void process() {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);
			boolean en = (boolean) _enabs.get(i);

			if (en) {
				lp.onLoop();
			} else {
				/*
				 * Current ILoopable is turned off, don't call OnLoop() for it
				 */
			}
		}
	}

	// --- ILoopable ---/
	public void onStart() {
		startAll();
	}

	public void onLoop() {
		process();
	}

	public boolean isDone() {
		return false;
	}

	public void onStop() {
		stopAll();
	}
}
