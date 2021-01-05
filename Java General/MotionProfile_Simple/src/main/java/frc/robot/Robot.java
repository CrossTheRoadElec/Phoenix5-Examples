/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The TalonSRX class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is a TimedRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * or find Motion Profile Generator.xlsx in the Project folder.
 * 
 * Controls:
 * Button 1: When held, streams and fires the MP.  When released, contorl is back to PercentOutput Mode.
 * Button 2: Prints MP status to the console when held.
 * Left Joystick Y-Axis: Throttle Talon SRX forward and reverse when not running MP.
 * 
 * Gains for Motion Profile may need to be adjusted in Constants.java
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motion.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {

    /** very simple state machine to prevent calling set() while firing MP. */
    int _state = 0;

    /** a master talon, add followers if need be. */
    TalonSRX _master = new WPI_TalonSRX(0);

    /** gamepad for control */
    Joystick _joy = new Joystick(0);

    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStream = new BufferedTrajectoryPointStream();

    /* talon configs */
    TalonSRXConfiguration _config = new TalonSRXConfiguration(); // factory default settings
    
    /* quick and dirty plotter to smartdash */
    PlotThread _plotThread = new PlotThread(_master);

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(_master, 0.75, 6800, true);
    }
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public void robotInit() {
        /* fill our buffer object with the excel points */
        initBuffer(MotionProfile.Points, MotionProfile.kNumPoints);

        /* _config the master specific settings */
        _config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        _config.neutralDeadband = Constants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        _config.slot0.kF = Constants.kGains_MotProf.kF;
        _config.slot0.kP = Constants.kGains_MotProf.kP;
        _config.slot0.kI = Constants.kGains_MotProf.kI;
        _config.slot0.kD = Constants.kGains_MotProf.kD;
        _config.slot0.integralZone = (int) Constants.kGains_MotProf.kIzone;
        _config.slot0.closedLoopPeakOutput = Constants.kGains_MotProf.kPeakOutput;
        // _config.slot0.allowableClosedloopError // left default for this example
        // _config.slot0.maxIntegralAccumulator; // left default for this example
        // _config.slot0.closedLoopPeriod; // left default for this example
        _master.configAllSettings(_config);

        /* pick the sensor phase and desired direction */
        _master.setSensorPhase(true);
        _master.setInverted(false);
    }

    public void robotPeriodic() {
        /* get joystick button and stick */
        boolean bPrintValues = _joy.getRawButton(2);
        boolean bFireMp = _joy.getRawButton(1);
        double axis = _joy.getRawAxis(1);

        /* if button is up, just drive the motor in PercentOutput */
        if (bFireMp == false) {
            _state = 0;
        }

        switch (_state) {
            /* drive master talon normally */
            case 0:
                _master.set(ControlMode.PercentOutput, axis);
                if (bFireMp == true) {
                    /* go to MP logic */
                    _state = 1;
                }
                break;

            /* fire the MP, and stop calling set() since that will cancel the MP */
            case 1:
                /* wait for 10 points to buffer in firmware, then transition to MP */
                _master.startMotionProfile(_bufferedStream, 10, ControlMode.MotionProfile);
                _state = 2;
                Instrum.printLine("MP started");
                break;

            /* wait for MP to finish */
            case 2:
                System.out.println("Position: " + _master.getSelectedSensorPosition() + ", Velocity: " + _master.getSelectedSensorVelocity());
                System.out.println("TarPos: " + _master.getActiveTrajectoryPosition() + ", TarVel: " + _master.getActiveTrajectoryVelocity());
                if (_master.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
                    _state = 3;
                }
                break;

            /* MP is finished, nothing to do */
            case 3:
                break;
        }

        /* print MP values */
        Instrum.loop(bPrintValues, _master);
    }

    /**
     * Fill _bufferedStream with points from csv/generated-table.
     *
     * @param profile  generated array from excel
     * @param totalCnt num points in profile
     */
    private void initBuffer(double[][] profile, int totalCnt) {

        boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
                                // since you can use negative numbers in profile).

        TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
                                                       // automatically, you can alloc just one

        /* clear the buffer, in case it was used elsewhere */
        _bufferedStream.Clear();

        /* Insert every point into buffer, no limit on size */
        for (int i = 0; i < totalCnt; ++i) {

            double direction = forward ? +1 : -1;
            double positionRot = profile[i][0];
            double velocityRPM = profile[i][1];
            int durationMilliseconds = (int) profile[i][2];

            /* for each point, fill our structure and pass it to API */
            point.timeDur = durationMilliseconds;
            point.position = direction * positionRot * Constants.kSensorUnitsPerRotation; // Convert Revolutions to
                                                                                          // Units
            point.velocity = direction * velocityRPM * Constants.kSensorUnitsPerRotation / 600.0; // Convert RPM to
                                                                                                  // Units/100ms
            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            point.zeroPos = (i == 0); /* set this to true on the first point */
            point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

            _bufferedStream.Write(point);
        }
    }
}
