#include "commands/CANdleConfigCommands.h"

using namespace CANdleConfigCommands;

ConfigBrightness::ConfigBrightness(CANdleSystem *candleSystem, double brightnessPercent) :
    frc2::InstantCommand([this, candleSystem, brightnessPercent]{
        candleSystem->ConfigBrightness(brightnessPercent);
        }){

}


ConfigLosBehavior::ConfigLosBehavior(CANdleSystem *candleSystem, bool ledOffWhenLos) :
    frc2::InstantCommand([this, candleSystem, ledOffWhenLos]{
        candleSystem->ConfigLos(ledOffWhenLos);
        }){

}