#include "mtrain.hpp"
#include "SPI.hpp"
#include "I2C.hpp"

#include "iodefs.h"
// #include "modules/GenericModule.hpp"
// #include "modules/BatteryModule.hpp"
#include "modules/FPGAModule.hpp"
// #include "modules/IMUModule.hpp"
// #include "modules/KickerModule.hpp"
#include "modules/LEDModule.hpp"
#include "drivers/MCP23017.hpp"
#include "modules/MotionControlModule.hpp"
#include "modules/RadioModule.hpp"
#include "modules/RotaryDialModule.hpp"
//#include "modules/RotaryDialModule.hpp"
#include <LockedStruct.hpp>
#include <Micropackets.hpp>
#include "MicroPackets.hpp"
#include "drivers/MCP23017.hpp"
//#include "drivers/RotarySelector.hpp"
#include "drivers/IOExpanderDigitalInOut.hpp"

DebugInfo debugInfo;
std::vector<const char *> failed_modules;
struct MotorTesterDrivers
{

    LockedStruct<MCP23017> *mcp23017;
};

[[noreturn]] void startModule(void *pvModule)
{
    GenericModule *module = static_cast<GenericModule *>(pvModule);

    printf("[INFO] Starting module %s\r\n", module->name);
    module->start();
    printf("[INFO] Finished starting module %s\r\n", module->name);

    TickType_t last_wait_time = xTaskGetTickCount();
    TickType_t increment = module->period.count();

    while (true)
    {
        module->entry();
        vTaskDelayUntil(&last_wait_time, increment);
    }
}

void createModule(GenericModule *module)
{
    BaseType_t result = xTaskCreate(startModule,
                                    module->name,
                                    module->stackSize,
                                    module,
                                    module->priority,
                                    &(module->handle));
    if (result != pdPASS)
    {
        printf("[ERROR] Failed to initialize task %s for reason %x:\r\n", module->name, module->stackSize);
        failed_modules.push_back(module->name);
    }
    else
    {
        printf("[INFO] Initialized task %s.\r\n", module->name);
    }
}

[[noreturn]] void motorTesterModule(void *pvDrivers)
{
    MotorTesterDrivers *drivers = reinterpret_cast<MotorTesterDrivers *>(pvDrivers);
    LockedStruct<MCP23017> &ioExpander = *(drivers->mcp23017);
    LockedStruct<RobotID> robotID{};
    // static RotaryDialModule dial(ioExpander,
    //                              robotID);
    //createModule(&dial);

    // RotarySelector<IOExpanderDigitalInOut> dial({IOExpanderDigitalInOut(ioExpander, HEX_SWITCH_BIT0, MCP23017::DIR_INPUT),
    //                                              IOExpanderDigitalInOut(ioExpander, HEX_SWITCH_BIT1, MCP23017::DIR_INPUT),
    //                                              IOExpanderDigitalInOut(ioExpander, HEX_SWITCH_BIT2, MCP23017::DIR_INPUT),
    //                                              IOExpanderDigitalInOut(ioExpander, HEX_SWITCH_BIT3, MCP23017::DIR_INPUT)});

    DigitalOut led1(LED1);
    DigitalOut led2(LED2);

    led1 = 1;

    // // Get initial dial value
    //dial.start();
    led2 = 1;
    auto robotIDLock = robotID.lock();

    while (robotIDLock->isValid != false)
    {
        vTaskDelay(100);
        printf("Dial is %d\n", robotIDLock->lastUpdate);
    }
}

int main()
{
    printf("Hi you entered main()");
    LockedStruct<RobotID> robotID{};
    auto robotIDLock = robotID.lock();
    MotorCommand motorCommand;
    //auto motorCommandLock = motorCommand.lock();
    motorCommand.isValid = false;
    motorCommand.lastUpdate = 0;
    for (int i = 0; i < 4; i++)
    {
        motorCommand.wheels[i] = 0;
    }
    motorCommand.dribbler = 0;

    FPGAStatus fpgaStatus;
    // //auto fpgaStatusLock = fpgaStatus.lock();
    MotorFeedback motorFeedback;
    // //auto motorFeedbackLock = motorFeedback.lock();

    //auto robotIDLock = robotID.lock();

    LockedStruct<I2C> sharedI2C(SHARED_I2C_BUS);
    auto _I2C = sharedI2C.lock();
    std::unique_ptr<SPI> fpgaKickerSPI = std::make_unique<SPI>(FPGA_SPI_BUS, std::nullopt, 16'000'000);
    // //LockedStruct<MCP23017> ioExpander(MCP23017{sharedI2C, 0x42});
    //std::shared_ptr<I2C> sharedI2C = std::make_shared<I2C>(SHARED_I2C_BUS);
    // //std::shared_ptr<MCP23017>
    // //std::shared_ptr<MCP23017> ioExpander = std::make_shared<MCP23017>(sharedI2C, 0x42);
    // // ioExpander.config(0x00FF, 0x00FF, 0x00FF);
    LockedStruct<MCP23017> ioExpander(MCP23017{sharedI2C, 0x42});
    //std::shared_ptr<MCP23017>& ioExpander = std::make_shared<MCP23017>(_I2C, 0x42);
    ioExpander.unsafe_value()->config(0x00FF, 0x00FF, 0x00FF);
    // RotaryDialModule dial (ioExpander, robotID);
    // createModule(&dial);

    // static FPGAModule fpga(std::move(fpgaKickerSPI),
    //                        motorCommand,
    //                        fpgaStatus,
    //                        motorFeedback);
    // createModule(&fpga);
    std::array<int16_t, 5> dutyCycles{10, 10, 10, 10, 10};

    MotorTesterDrivers drivers{&ioExpander};
    TaskHandle_t handle = nullptr;
    BaseType_t result = xTaskCreate(motorTesterModule,
                                    "Motor tester",
                                    4096,
                                    &drivers,
                                    0,
                                    &handle);
    vTaskStartScheduler();

    //     while (true)
    //     {

    //         printf("RobotID: %d\r\n", robotIDLock->robotID);

    //         float duty = ((robotIDLock->robotID ^ 8) - 8) % 8 / 8.0;

    //         motorCommand.isValid = true;
    //         motorCommand.lastUpdate = HAL_GetTick();
    //         for (int i = 0; i < 4; i++)
    //         {
    //             motorCommand.wheels[i] = duty;
    //         }
    //         motorCommand.dribbler = abs(duty * 127);
    //         int16_t duty_cycles[5] = {50, 50, 50, 50, 50};
    //         size_t length = 5;

    //         //fpga.set_duty_cycles(duty_cycles, length);
    //         HAL_Delay(100);
    //     }
}
