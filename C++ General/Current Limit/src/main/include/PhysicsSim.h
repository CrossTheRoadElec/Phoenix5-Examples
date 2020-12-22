#pragma once

#include <vector>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

/**
 * Manages physics simulation for CTRE products.
 */
class PhysicsSim
{
    class SimProfile;
    class TalonSRXSimProfile;
    class VictorSPXSimProfile;
    
public:
    /**
     * Gets the physics simulator instance.
     */
    static PhysicsSim& GetInstance() {
        static PhysicsSim sim;
        return sim;
    }

    ~PhysicsSim() {
        for (auto simProfile : _simProfiles) {
            delete simProfile;
        }
        _simProfiles.clear();
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
    void AddTalonSRX(TalonSRX &talon, double const accelToFullTime, double const fullVel, bool const sensorPhase = false) {
        TalonSRXSimProfile* simTalon = new TalonSRXSimProfile(talon, accelToFullTime, fullVel, sensorPhase);
        this->_simProfiles.insert(this->_simProfiles.end(), simTalon);
    }

    /**
     * Adds a VictorSPX controller to the simulator.
     * 
     * @param victor
     *        The VictorSPX device
     */
    void AddVictorSPX(VictorSPX &victor) {
        VictorSPXSimProfile* simVictor = new VictorSPXSimProfile(victor);
        this->_simProfiles.insert(this->_simProfiles.end(), simVictor);
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
        for (auto simProfile : _simProfiles) {
            simProfile->Run();
        }
    }

private:
    /* list of simulation profiles */
    std::vector<SimProfile*> _simProfiles;

    /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
    static double random(double min, double max) {
        return (max - min) / 2 * sin(fmod(rand(), 2 * 3.14159)) + (max + min) / 2;
    }
    static double random(double max) {
        return random(0, max);
    }
    
    /**
     * Holds information about a simulated device.
     */
    class SimProfile {
    private:
        std::chrono::steady_clock::time_point _lastTime;
        bool _running = false;

    public:
        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        virtual void Run() {}

    protected:
        /**
         * Returns the time since last call, in milliseconds.
         */
        double GetPeriod() {
            // set the start time if not yet running
            if (!_running) {
                _lastTime = std::chrono::steady_clock::now();
                _running = true;
            }

            // get time since last call
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> elapsed(now - _lastTime);
            double const period = elapsed.count();
            _lastTime = now;

            return period;
        }
    };

    /**
     * Holds information about a simulated TalonSRX.
     */
    class TalonSRXSimProfile : public SimProfile {
    private:
        TalonSRX &_talon;
        double const _accelToFullTime;
        double const _fullVel;
        bool const _sensorPhase;

        /** The current position */
        double _pos = 0;
        /** The current velocity */
        double _vel = 0;

    public:
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
        TalonSRXSimProfile(TalonSRX &talon, double const accelToFullTime, double const fullVel, bool const sensorPhase = false)
            : _talon(talon), _accelToFullTime(accelToFullTime), _fullVel(fullVel), _sensorPhase(sensorPhase) {}
        
        /**
         * Runs the simulation profile.
         */
        void Run() override {
            double const period = GetPeriod();
            double const accelAmount = _fullVel / _accelToFullTime * period / 1000;

            /// DEVICE SPEED SIMULATION

            double outPerc = _talon.GetMotorOutputPercent();
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

            _talon.GetSimCollection().AddQuadraturePosition(_vel * period / 100);
            _talon.GetSimCollection().SetQuadratureVelocity(_vel);
            
            _talon.GetSimCollection().SetSupplyCurrent(fabs(outPerc) * 20 * random(0.95, 1.05));
            _talon.GetSimCollection().SetBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
        }
    };

    /**
     * Holds information about a simulated VictorSPX.
     */
    class VictorSPXSimProfile : public SimProfile {
    private:
        VictorSPX &_victor;

    public:
        /**
         * Creates a new simulation profile for a VictorSPX device.
         * 
         * @param victor
         *        The VictorSPX device
         */
        VictorSPXSimProfile(VictorSPX &victor) : _victor(victor) {}

        /**
         * Runs the simulation profile.
         */
        void Run() override {
            double const period = GetPeriod();

            // Device voltage simulation
            double outPerc = _victor.GetMotorOutputPercent();
            _victor.GetSimCollection().SetBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
        }
    };
};
