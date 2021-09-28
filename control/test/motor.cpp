#include "mtrain.hpp"
#include "SPI.hpp"
#include "iodefs.h"
#include "drivers/MCP23017.hpp"
#include "LockedStruct.hpp"
#include "MicroPackets.hpp"
#include "drivers/MCP23017.hpp"
#include "drivers/RotarySelector.hpp"
#include "drivers/IOExpanderDigitalInOut.hpp"
#include "drivers/FPGA.hpp"
#include "MicroPackets.hpp"
#include <memory>

DebugInfo debugInfo;

int main()
{
    std::unique_ptr<SPI> fpgaKickerSPI = std::make_unique<SPI>(FPGA_SPI_BUS, std::nullopt, 16'000'000);
    FPGA fpga(std::move(fpgaKickerSPI), p31, p14, p13, p15);
    MotorCommand motorCommand;
    MotionCommand motionCommand;

    while (true)
    {

        float duty = 1;

        motorCommand.isValid = true;
        motionCommand.isValid = true;
        motorCommand.lastUpdate = HAL_GetTick();
        for (int i = 0; i < 4; i++)
        {
            //motionCommand.bodyXVel = 0.1;
            motionCommand.bodyYVel = 0.1;
            motorCommand.wheels[i] = duty;
        }
        motorCommand.dribbler = duty * 127;
        fpga.send_config();
        //HAL_Delay(100);
    }
}
