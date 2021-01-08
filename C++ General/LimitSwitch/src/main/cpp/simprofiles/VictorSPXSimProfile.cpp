#include "PhysicsSim.h"
#include "ctre/Phoenix.h"

/**
 * Runs the simulation profile.
 * 
 * This uses very rudimentary physics simulation and exists to allow users to test
 * features of our products in simulation using our examples out of the box.
 * Users may modify this to utilize more accurate physics simulation.
 */
void PhysicsSim::VictorSPXSimProfile::Run() {
    double const period = GetPeriod();

    // Device voltage simulation
    double outPerc = _victor.GetMotorOutputPercent();
    _victor.GetSimCollection().SetBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
}