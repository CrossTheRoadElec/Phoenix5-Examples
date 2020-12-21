package frc.robot;

import java.util.*;
import com.ctre.phoenix.unmanaged.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    /**
     * Holds information about a simulated TalonSRX.
     */
    public static class SimTalonSRX {
        public final TalonSRX talon;
        public final double accelToFullTime;
        public final double fullVel;
        public final boolean sensorPhase;
    
        /**
         * Creates a new instance of simulated TalonSRX info.
         * 
         * @param talon
         *        The TalonSRX device
         * @param accelToFullTime
         *        The time the motor takes to accelerate from 0 to full, in seconds
         * @param fullVel
         *        The maximum motor velocity, in ticks per 100ms
         */
        public SimTalonSRX(final TalonSRX talon, final double accelToFullTime, final double fullVel) {
            this(talon, accelToFullTime, fullVel, false);
        }
    
        /**
         * Creates a new instance of simulated TalonSRX info.
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
        public SimTalonSRX(final TalonSRX talon, final double accelToFullTime, final double fullVel, final boolean sensorPhase) {
            this.talon = talon;
            this.accelToFullTime = accelToFullTime;
            this.fullVel = fullVel;
            this.sensorPhase = sensorPhase;
        }
            
        private long lastTime;
        private boolean running = false;
    
        /** The current position */
        private double pos = 0;
        /** The current velocity */
        private double vel = 0;
    }
    /**
     * Holds information about a simulated VictorSPX.
     */
    public static class SimVictorSPX {
        public final VictorSPX victor;
    
        /**
         * Creates a new instance of simulated TalonSRX info.
         * 
         * @param victor
         *        The VictorSPX device
         */
        public SimVictorSPX(final VictorSPX victor) {
            this.victor = victor;
        }
            
        private long lastTime;
        private boolean running = false;
    }
    
    private static final PhysicsSim sim = new PhysicsSim();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }

    /**
     * Adds TalonSRX controllers to the simulator.
     */
    public void addTalonSRXs(SimTalonSRX... simTalonSRXs) {
        Collections.addAll(this.simTalonSRXs, simTalonSRXs);
    }

    /**
     * Adds VictorSPX controllers to the simulator.
     */
    public void addVictorSPXs(SimVictorSPX... simVictorSPXs) {
        Collections.addAll(this.simVictorSPXs, simVictorSPXs);
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
        for (SimTalonSRX simTalonSrx : simTalonSRXs) {
            if (simTalonSrx.talon != null) {
                talonSRXSimulator(simTalonSrx);
            }
        }
        for (SimVictorSPX simVictorSPX : simVictorSPXs) {
            if (simVictorSPX.victor != null) {
                victorSPXSimulator(simVictorSPX);
            }
        }
    }

    private final ArrayList<SimTalonSRX> simTalonSRXs = new ArrayList<SimTalonSRX>();
    private final ArrayList<SimVictorSPX> simVictorSPXs = new ArrayList<SimVictorSPX>();

    private double random(double min, double max) {
        return (max - min) / 2 * Math.sin(Math.IEEEremainder(Math.random(), 2 * 3.14159)) + (max + min) / 2;
    }
    private double random(double max) {
        return random(0, max);
    }

    /**
     * Simulates the TalonSRX sensors.
     */
    private void talonSRXSimulator(SimTalonSRX simTalonSRX) {
        if (!simTalonSRX.running) {
            simTalonSRX.lastTime = System.nanoTime();
            simTalonSRX.running = true;
        }
        
        long now = System.nanoTime();
        final double period = (now - simTalonSRX.lastTime) / 1000000.;
        simTalonSRX.lastTime = now;
        
        final double accelAmount = simTalonSRX.fullVel / simTalonSRX.accelToFullTime * period / 1000;

        // Device speed simulation
        double outPerc = simTalonSRX.talon.getMotorOutputPercent();
        if (simTalonSRX.sensorPhase) {
            outPerc *= -1;
        }
        double theoreticalVel = outPerc * simTalonSRX.fullVel * random(0.95, 1);
        if (theoreticalVel > simTalonSRX.vel + accelAmount) {
            simTalonSRX.vel += accelAmount;
        }
        else if (theoreticalVel < simTalonSRX.vel - accelAmount) {
            simTalonSRX.vel -= accelAmount;
        }
        else {
            simTalonSRX.vel += 0.9 * (theoreticalVel - simTalonSRX.vel);
        }
        simTalonSRX.pos += simTalonSRX.vel * period / 100;

        // simTalonSRX.talon.getSimCollection().setQuadraturePosition((int)simTalon.pos);
        simTalonSRX.talon.getSimCollection().addQuadraturePosition((int)(simTalonSRX.vel * period / 100));
        simTalonSRX.talon.getSimCollection().setQuadratureVelocity((int)simTalonSRX.vel);
        
        simTalonSRX.talon.getSimCollection().setSupplyCurrent(Math.abs(outPerc) * 30 * random(0.95, 1.05));
        simTalonSRX.talon.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));

        simTalonSRX.talon.getSimCollection().setLimitFwd(simTalonSRX.talon.getSelectedSensorPosition() > 50000);
        simTalonSRX.talon.getSimCollection().setLimitRev(simTalonSRX.talon.getSelectedSensorPosition() < -50000);
    }

    /**
     * Simulates the VictorSPX sensors.
     */
    private void victorSPXSimulator(SimVictorSPX simVictorSPX) {
        if (!simVictorSPX.running) {
            simVictorSPX.lastTime = System.nanoTime();
            simVictorSPX.running = true;
        }
        
        long now = System.nanoTime();
        final double period = (now - simVictorSPX.lastTime) / 1000000.;
        simVictorSPX.lastTime = now;

        // Device voltage simulation
        double outPerc = simVictorSPX.victor.getMotorOutputPercent();
        simVictorSPX.victor.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
    }
}
