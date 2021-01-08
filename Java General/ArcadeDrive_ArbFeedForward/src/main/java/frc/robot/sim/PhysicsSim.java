package frc.robot.sim;

import java.util.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }

    /**
     * Adds a TalonSRX controller to the simulator.
     * 
     * @param talon
     *        The TalonSRX device
     * @param accelToFullTime
     *        The time the motor takes to accelerate from 0 to full, in seconds
     * @param fullVel
     *        The maximum motor velocity, in ticks per 100ms
     */
    public void addTalonSRX(TalonSRX talon, final double accelToFullTime, final double fullVel) {
        addTalonSRX(talon, accelToFullTime, fullVel, false);
    }

    /**
     * Adds a TalonSRX controller to the simulator.
     * 
     * @param talon
     *        The TalonSRX device
     * @param accelToFullTime
     *        The time the motor takes to accelerate from 0 to full, in seconds
     * @param fullVel
     *        The maximum motor velocity, in ticks per 100ms
     * @param sensorPhase
     *        The phase of the TalonSRX sensors
     */
    public void addTalonSRX(TalonSRX talon, final double accelToFullTime, final double fullVel, final boolean sensorPhase) {
        if (talon != null) {
            TalonSRXSimProfile simTalon = new TalonSRXSimProfile(talon, accelToFullTime, fullVel, sensorPhase);
            _simProfiles.add(simTalon);
        }
    }

    /**
     * Adds a VictorSPX controller to the simulator.
     * 
     * @param victor
     *        The VictorSPX device
     */
    public void addVictorSPX(VictorSPX victor) {
        if (victor != null) {
            VictorSPXSimProfile simVictor = new VictorSPXSimProfile(victor);
            _simProfiles.add(simVictor);
        }
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate TalonSRX sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
    static double random(double min, double max) {
        return (max - min) / 2 * Math.sin(Math.IEEEremainder(Math.random(), 2 * 3.14159)) + (max + min) / 2;
    }
    static double random(double max) {
        return random(0, max);
    }

    
    /**
     * Holds information about a simulated device.
     */
    static class SimProfile {
        private long _lastTime;
        private boolean _running = false;

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        public void run() {}

        /**
         * Returns the time since last call, in milliseconds.
         */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!_running) {
                _lastTime = System.nanoTime();
                _running = true;
            }
            
            long now = System.nanoTime();
            final double period = (now - _lastTime) / 1000000.;
            _lastTime = now;

            return period;
        }
    }
}
