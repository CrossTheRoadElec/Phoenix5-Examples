package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {

	/** our test talon */
	TalonSRX _tal = new TalonSRX(0);
	/* our test gamepad */
	Joystick _joy = new Joystick(0);
	/** save buttons each loop */
	boolean[] _btnsLast = {false, false, false, false, false, false, false, false, false, false};
	/** desired brake mode, init value assigned here */
	boolean _brake = true;

	/** c'tor Select the brake mode to start with. */
	public Robot() {
		/* override brake setting programmatically */
		_tal.setNeutralMode(_brake ? NeutralMode.Brake : NeutralMode.Coast);
		System.out.println("brake:" + _brake); /* instrument to console */
	}

	/** Every loop, flip brake mode if button1 when is pressed. */
	public void commonloop() {
		/* get buttons */
		boolean[] btns = new boolean[_btnsLast.length];
		for (int i = 1; i < _btnsLast.length; ++i)
			btns[i] = _joy.getRawButton(i);

		/* flip brake when btn1 is pressed */
		if (btns[1] && !_btnsLast[1]) {
			_brake = !_brake;
			/* override brake setting programmatically */
			_tal.setNeutralMode(_brake ? NeutralMode.Brake : NeutralMode.Coast);
			/* instrument to console */
			System.out.println("brake:" + _brake); 
		}

		/* save buttons states for on-press detection */
		for (int i = 1; i < 10; ++i)
			_btnsLast[i] = btns[i];
	}

	public void disabledPeriodic() {
		commonloop(); /* just call a "common" loop */
	}

	public void teleopPeriodic() {
		commonloop(); /* just call a "common" loop */
	}
}
