#pragma once

#include <frc2/command/InstantCommand.h>
#include "subsystems/CANdleSystem.h"

namespace CANdlePrintCommands{

class PrintVBat : public frc2::InstantCommand {
public:
    PrintVBat(CANdleSystem *candleSystem);
};

class Print5V : public frc2::InstantCommand {
public:
    Print5V(CANdleSystem *candleSystem);
};

class PrintCurrent : public frc2::InstantCommand {
public:
    PrintCurrent(CANdleSystem *candleSystem);
};

class PrintTemperature : public frc2::InstantCommand {
public:
    PrintTemperature(CANdleSystem *candleSystem);
};

class PrintModulatedOutput : public frc2::InstantCommand {
public:
    PrintModulatedOutput(CANdleSystem *candleSystem);
};

}
