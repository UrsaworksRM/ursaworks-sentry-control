/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */


#include "tap/board/board.hpp"
#include "modm/platform/timer/timer_1.hpp"

#include "modm/architecture/interface/delay.hpp"
#include "modm/architecture/interface/interrupt.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "src/drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "src/robot/robot_control.hpp"

#include "tap/communication/gpio/pwm.hpp"

#include "taproot/src/tap/algorithms/smooth_pid.hpp"

#include "taproot/src/tap/communication/serial/remote_serial_constants.hpp"

#include "taproot/src/tap/communication/serial/remote.hpp"

static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
static constexpr float MAHONY_KP = 0.1f;

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(tap::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(tap::Drivers *drivers);

using namespace xcysrc::standard;

static constexpr tap::motor::MotorId agitatorID = tap::motor::MOTOR7;
static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS1;
static constexpr int DESIRED_RPM =1000;
tap::motor::DjiMotor agimotor(::DoNotUse_getDrivers(),agitatorID,CAN_BUS2,false,"cool motor");

tap::motor::DjiMotor l1(::DoNotUse_getDrivers(),tap::motor::MOTOR1,CAN_BUS2,false,"cool motor");


static void initializePWM(tap::Drivers *drivers)
{
    drivers->leds.set(tap::gpio::Leds::Blue, true);
    modm::delay_ms(1000);
    drivers->leds.set(tap::gpio::Leds::Blue, false);
    tap::gpio::Pwm::Pin pwmPin1= tap::gpio::Pwm::Pin::C1;
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 100);
    tap::gpio::Pwm::Pin pwmPin2= tap::gpio::Pwm::Pin::C2;
    drivers->pwm.write(.2,pwmPin1);
    drivers->pwm.write(.2,pwmPin2);
    modm::delay_ms(2000);
    drivers->pwm.write(0.06,pwmPin1);
    drivers->pwm.write(0.06,pwmPin2);
    modm::delay_ms(2000);
    drivers->pwm.write(0.11,pwmPin1);
    drivers->pwm.write(0.11,pwmPin2);
    modm::delay_ms(500);
    
// every time robot got killed or power off, initialize the gpio again
// better to make it to the button
}



static void agitatorSpin(tap::Drivers *drivers)
{
    /*tap::motor::DjiMotor agitatorMotor(::DoNotUse_getDrivers(), agitatorID, CAN_BUS,false,"cool motor"); */
    bool spin = (drivers->remote.getSwitch(tap::Remote::Switch::LEFT_SWITCH) == tap::Remote::SwitchState::UP) || drivers->remote.getMouseL();
    bool inv = (drivers->remote.getSwitch(tap::Remote::Switch::LEFT_SWITCH) == tap::Remote::SwitchState::DOWN) || drivers->remote.getMouseR();
    if(spin){
        agimotor.setDesiredOutput(static_cast<int32_t>(2000));  
    }
    else if(inv){
        agimotor.setDesiredOutput(static_cast<int32_t>(-2000));  
    }
    else{
        agimotor.setDesiredOutput(static_cast<int32_t>(0));  
    }
    drivers->djiMotorTxHandler.processCanSendData();
}

static void rotate(tap::Drivers *drivers) {
    float wheelInput = drivers->remote.getWheel() / 660.0F;
    drivers->leds.set(tap::gpio::Leds::Blue, wheelInput > 0.8F);
    drivers->leds.set(tap::gpio::Leds::Red, wheelInput < -0.8F);
    l1.setDesiredOutput(static_cast<int16_t>(5000));
    drivers->djiMotorTxHandler.processCanSendData();
}


int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    Drivers *drivers = DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    drivers->leds.set(tap::gpio::Leds::Red, true);
    modm::delay_ms(1000);
    drivers->leds.set(tap::gpio::Leds::Red, false);
    initSubsystemCommands(drivers);
    drivers->leds.set(tap::gpio::Leds::Green, true);
    modm::delay_ms(1000);
    drivers->leds.set(tap::gpio::Leds::Green, false);   
    agimotor.initialize();
    initializePWM(drivers); 
    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));
        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.processCanSendData, ());
            agitatorSpin(drivers);
            rotate(drivers);
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(tap::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->remote.initialize();
    drivers->refSerial.initialize();
}

static void updateIo(tap::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
}
