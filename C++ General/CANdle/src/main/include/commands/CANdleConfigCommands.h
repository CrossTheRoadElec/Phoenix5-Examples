#pragma once

#include <frc2/command/InstantCommand.h>
#include "subsystems/CANdleSystem.h"

namespace CANdleConfigCommands{

class ConfigBrightness : public frc2::InstantCommand {
public:
    ConfigBrightness(CANdleSystem *candleSystem, double brightnessPercent);
};

class ConfigLosBehavior : public frc2::InstantCommand {
public:
    ConfigLosBehavior(CANdleSystem *candleSystem, bool ledOffWhenLos);
};

}
