#pragma once

#include <vector>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

/**
 * Holds information about a simulated TalonSRX.
 */
class SimTalonSRX {
public:
    TalonSRX* const talon;
    double const accelToFullTime;
    double const fullVel;

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
    SimTalonSRX(TalonSRX* const talon, double const accelToFullTime, double const fullVel)
        : talon(talon), accelToFullTime(accelToFullTime), fullVel(fullVel) {}
    
private:
    std::chrono::steady_clock::time_point lastTime;
    bool running = false;

    /** The current position */
    double pos = 0;
    /** The current velocity */
    double vel = 0;

    friend class PhysicsSim;
};
/**
 * Holds information about a simulated VictorSPX.
 */
class SimVictorSPX {
public:
    VictorSPX* const victor;

    /**
     * Creates a new instance of simulated VictorSPX info.
     * 
     * @param victor
     *        The VictorSPX device
     */
    SimVictorSPX(VictorSPX* const victor) : victor(victor) {}
    
private:
    std::chrono::steady_clock::time_point lastTime;
    bool running = false;

    friend class PhysicsSim;
};

/**
 * Manages physics simulation for CTRE products.
 */
class PhysicsSim
{
public:
    /**
     * Gets the robot simulator instance.
     */
    static PhysicsSim& GetInstance() {
        static PhysicsSim sim;
        return sim;
    }

    /**
     * Adds TalonSRX controllers to the simulator.
     */
    void AddTalonSRXs(std::initializer_list<SimTalonSRX*> simTalonSRXs) {
        this->simTalonSRXs.insert(this->simTalonSRXs.end(), simTalonSRXs);
    }

    /**
     * Adds VictorSPX controllers to the simulator.
     */
    void AddVictorSPXs(std::initializer_list<SimVictorSPX*> simVictorSPXs) {
        this->simVictorSPXs.insert(this->simVictorSPXs.end(), simVictorSPXs);
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate TalonSRX sensors
     */
    void Run() {
        // Enable the robot
        unmanaged::Unmanaged::FeedEnable(100);
        // Simulate devices
        for (SimTalonSRX* simTalonSRX : simTalonSRXs) {
            if (simTalonSRX && simTalonSRX->talon) {
                TalonSRXSimulator(simTalonSRX);
            }
        }
        for (SimVictorSPX* simVictorSPX : simVictorSPXs) {
            if (simVictorSPX && simVictorSPX->victor) {
                VictorSPXSimulator(simVictorSPX);
            }
        }
    }

private:
    std::vector<SimTalonSRX*> simTalonSRXs;
    std::vector<SimVictorSPX*> simVictorSPXs;

    double random(double min, double max) {
        return (max - min) / 2 * sin(fmod(rand(), 2 * 3.14159)) + (max + min) / 2;
    }
    double random(double max) {
        return random(0, max);
    }

    /**
     * Simulates the TalonSRX sensors.
     */
    void TalonSRXSimulator(SimTalonSRX* simTalonSRX) {
        if (!simTalonSRX->running) {
            simTalonSRX->lastTime = std::chrono::steady_clock::now();
            simTalonSRX->running = true;
        }
        
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed(now - simTalonSRX->lastTime);
        double const period = elapsed.count();
        simTalonSRX->lastTime = now;
        
        double const accelAmount = simTalonSRX->fullVel / simTalonSRX->accelToFullTime * period / 1000;

        // Device speed simulation
        double outPerc = simTalonSRX->talon->GetMotorOutputPercent();
        double theoreticalVel = outPerc * simTalonSRX->fullVel * random(0.95, 1);
        if (theoreticalVel > simTalonSRX->vel + accelAmount) {
            simTalonSRX->vel += accelAmount;
        }
        else if (theoreticalVel < simTalonSRX->vel - accelAmount) {
            simTalonSRX->vel -= accelAmount;
        }
        else {
            simTalonSRX->vel += 0.9 * (theoreticalVel - simTalonSRX->vel);
        }
        simTalonSRX->pos += simTalonSRX->vel * period / 100;

        // simTalonSRX->talon->GetSimCollection().SetQuadraturePosition(simTalonSRX->pos);
        simTalonSRX->talon->GetSimCollection().AddQuadraturePosition(simTalonSRX->vel * period / 100);
        simTalonSRX->talon->GetSimCollection().SetQuadratureVelocity(simTalonSRX->vel);
        
        simTalonSRX->talon->GetSimCollection().SetCurrent(10 + outPerc * 30 * random(0.95, 1.05));
        simTalonSRX->talon->GetSimCollection().SetBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
    }

    /**
     * Simulates the VictorSPX sensors.
     */
    void VictorSPXSimulator(SimVictorSPX* simVictorSPX) {
        if (!simVictorSPX->running) {
            simVictorSPX->lastTime = std::chrono::steady_clock::now();
            simVictorSPX->running = true;
        }
        
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed(now - simVictorSPX->lastTime);
        double const period = elapsed.count();
        simVictorSPX->lastTime = now;

        // Device voltage simulation
        double outPerc = simVictorSPX->victor->GetMotorOutputPercent();
        simVictorSPX->victor->GetSimCollection().SetBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
    }
};
