#include "PhysicsSim.h"
#include "ctre/Phoenix.h"

void PhysicsSim::VictorSPXSimProfile::Run() {
    double const period = GetPeriod();

    // Device voltage simulation
    double outPerc = _victor.GetMotorOutputPercent();
    _victor.GetSimCollection().SetBusVoltage(12 - outPerc * outPerc * 3/4 * random(0.95, 1.05));
}