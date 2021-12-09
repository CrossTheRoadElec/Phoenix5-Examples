#include "PhysicsSim.h"
#include "ctre/Phoenix.h"

/**
 * Runs the simulation profile.
 * 
 * This uses very rudimentary physics simulation and exists to allow users to test
 * features of our products in simulation using our examples out of the box.
 * Users may modify this to utilize more accurate physics simulation.
 */
void PhysicsSim::TalonFXSimProfile::Run() {
    double const period = GetPeriod();
    double const accelAmount = _fullVel / _accelToFullTime * period / 1000;

    /// DEVICE SPEED SIMULATION

    double outPerc = _falcon.GetSimCollection().GetMotorOutputLeadVoltage() / 12;
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

    _falcon.GetSimCollection().AddIntegratedSensorPosition(_vel * period / 100);
    _falcon.GetSimCollection().SetIntegratedSensorVelocity(_vel);

    double supplyCurrent = fabs(outPerc) * 30 * random(0.95, 1.05);
    double statorCurrent = outPerc == 0 ? 0 : supplyCurrent / fabs(outPerc);
    _falcon.GetSimCollection().SetSupplyCurrent(supplyCurrent);
    _falcon.GetSimCollection().SetStatorCurrent(statorCurrent);

    _falcon.GetSimCollection().SetBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
}