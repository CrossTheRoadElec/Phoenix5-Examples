package frc.robot;

import java.util.*;
import com.ctre.phoenix.unmanaged.*;
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
        // Enable the robot
        Unmanaged.feedEnable(100);
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
    private static double random(double min, double max) {
        return (max - min) / 2 * Math.sin(Math.IEEEremainder(Math.random(), 2 * 3.14159)) + (max + min) / 2;
    }
    private static double random(double max) {
        return random(0, max);
    }

    
    /**
     * Holds information about a simulated device.
     */
    private static class SimProfile {
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

    /**
     * Holds information about a simulated TalonSRX.
     */
    private static class TalonSRXSimProfile extends SimProfile {
        private final TalonSRX _talon;
        private final double _accelToFullTime;
        private final double _fullVel;
        private final boolean _sensorPhase;
    
        /** The current position */
        private double _pos = 0;
        /** The current velocity */
        private double _vel = 0;
    
        /**
         * Creates a new simulation profile for a TalonSRX device.
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
        public TalonSRXSimProfile(final TalonSRX talon, final double accelToFullTime, final double fullVel, final boolean sensorPhase) {
            this._talon = talon;
            this._accelToFullTime = accelToFullTime;
            this._fullVel = fullVel;
            this._sensorPhase = sensorPhase;
        }

        /**
         * Runs the simulation profile.
         */
        public void run() {
            final double period = getPeriod();
            final double accelAmount = _fullVel / _accelToFullTime * period / 1000;

            /// DEVICE SPEED SIMULATION

            double outPerc = _talon.getMotorOutputPercent();
            if (_sensorPhase) {
                outPerc *= -1;
            }
            // Calculate theoretical velocity with some randomness
            double theoreticalVel = outPerc * _fullVel * random(0.95, 1);
            // Simulate motor load
            if (theoreticalVel > _vel + accelAmount) {
                _vel += accelAmount;
            }
            else if (theoreticalVel < _vel - accelAmount) {
                _vel -= accelAmount;
            }
            else {
                _vel += 0.9 * (theoreticalVel - _vel);
            }
            _pos += _vel * period / 100;

            /// SET SIM PHYSICS INPUTS

            _talon.getSimCollection().addQuadraturePosition((int)(_vel * period / 100));
            _talon.getSimCollection().setQuadratureVelocity((int)_vel);
            
            _talon.getSimCollection().setSupplyCurrent(Math.abs(outPerc) * 20 * random(0.95, 1.05));
            _talon.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
        }
    }

    /**
     * Holds information about a simulated VictorSPX.
     */
    private static class VictorSPXSimProfile extends SimProfile {
        public final VictorSPX _victor;
    
        /**
         * Creates a new simulation profile for a VictorSPX device.
         * 
         * @param victor
         *        The VictorSPX device
         */
        public VictorSPXSimProfile(final VictorSPX victor) {
            this._victor = victor;
        }

        /**
         * Runs the simulation profile.
         */
        public void run() {
            final double period = getPeriod();
    
            // Device voltage simulation
            double outPerc = _victor.getMotorOutputPercent();
            _victor.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
        }
    }
}
