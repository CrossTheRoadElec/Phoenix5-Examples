#include "commands/CANdlePrintCommands.h"
#include <iostream>

using namespace CANdlePrintCommands;

PrintVBat::PrintVBat(CANdleSystem *candleSystem) :
    frc2::InstantCommand([this, candleSystem]{ 
        std::cout << "VBat is " << candleSystem->GetVbat() << "V" << std::endl;
    }) {
    
}

Print5V::Print5V(CANdleSystem *candleSystem) :
    frc2::InstantCommand([this, candleSystem]{ 
        std::cout << "5V is " << candleSystem->Get5V() << "V" << std::endl;
    }) {
    
}

PrintCurrent::PrintCurrent(CANdleSystem *candleSystem) :
    frc2::InstantCommand([this, candleSystem]{ 
        std::cout << "Current is " << candleSystem->GetCurrent() << "A" << std::endl;
    }) {
    
}

PrintTemperature::PrintTemperature(CANdleSystem *candleSystem) :
    frc2::InstantCommand([this, candleSystem]{ 
        std::cout << "Temperature is " << candleSystem->GetTemperature() << "C" << std::endl;
    }) {
    
}


PrintModulatedOutput::PrintModulatedOutput(CANdleSystem *candleSystem) :
    frc2::InstantCommand([this, candleSystem]{ 
        std::cout << "Modulated Output is " << candleSystem->GetModulatedOutput() * 100 << "%" << std::endl;
    }) {
    
}

