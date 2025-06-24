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

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

#include <cstdio>
#include <cstring>

/* communication includes ---------------------------------------------------*/
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "robot/robot_control.hpp"
#include "src/control/turret/turret_subsystem.hpp"

// AIM ASSIST DEFINITIONS
static constexpr float AIM_ASSIST_SENSITIVITY = 0.05f;
static constexpr float USER_OVERRIDE_THRESHOLD = 0.8f;

extern xcysrc::control::turret::TurretSubsystem turret;

enum class ExternalCommsState
{
    WAITING_FOR_ACK,
    RECEIVING_AIM_DATA
};

static ExternalCommsState commsState = ExternalCommsState::WAITING_FOR_ACK;
static tap::arch::PeriodicMilliTimer sendColorTimer(250); // Send color every 250ms until ACK
static char uartBuffer[128];
static uint8_t uartBufferIdx = 0;


static void parseAndHandleSerial(tap::Drivers* drivers);
static void handleAimAssist(tap::Drivers* drivers, float externalPitch, float externalYaw);
// END AIM ASSIST DEFINITIONS

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

static void sendTeamColor(tap::Drivers* drivers)
{
    const auto& robotData = drivers->refSerial.getRobotData();
    const char* teamColorString = "UNKNOWN\n";
    if (robotData.robotId != tap::communication::serial::RefSerialData::RobotId::INVALID)
    {
        teamColorString = tap::communication::serial::RefSerialData::isBlueTeam(robotData.robotId) ? "BLUE\n" : "RED\n";
    }
    drivers->uart.write(tap::communication::serial::Uart::Uart1, reinterpret_cast<const uint8_t*>(teamColorString), strlen(teamColorString));
}

using namespace xcysrc::standard;

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

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (commsState == ExternalCommsState::WAITING_FOR_ACK && drivers->refSerial.getRefSerialReceivingData() && sendColorTimer.execute())
        {
            sendTeamColor(drivers);
        }

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.processCanSendData, ());
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
    drivers->uart.init<tap::communication::serial::Uart::Uart1, 9600>();
}

static void updateIo(tap::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();

    uint8_t byte;
    while(drivers->uart.read(tap::communication::serial::Uart::Uart1, &byte, 1))
    {
        if (byte == '\n' || uartBufferIdx == sizeof(uartBuffer) - 1)
        {
            uartBuffer[uartBufferIdx] = '\0';
            parseAndHandleSerial(drivers);
            uartBufferIdx = 0;
        }
        else
        {
            uartBuffer[uartBufferIdx++] = byte;
        }
    }
}

static void parseAndHandleSerial(tap::Drivers* drivers)
{
    if (commsState == ExternalCommsState::WAITING_FOR_ACK)
    {
        if (strcmp(uartBuffer, "ACK") == 0)
        {
            commsState = ExternalCommsState::RECEIVING_AIM_DATA;
        }
    }
    else if (commsState == ExternalCommsState::RECEIVING_AIM_DATA)
    {
        float pitch, yaw;
        if (sscanf(uartBuffer, "P%fY%f", &pitch, &yaw) == 2)
        {
            handleAimAssist(drivers, pitch, yaw);
        }
    }
}

static void handleAimAssist(tap::Drivers* drivers, float externalPitch, float externalYaw)
{
    // Get user input from remote
    float userPitch = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    float userYaw = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

    float finalPitch = turret.pitchMotor.getChassisFrameSetpoint();
    float finalYaw = turret.yawMotor.getChassisFrameSetpoint();

    // --- Pitch Calculation ---
    // If user input is strong, override aim assist
    if (fabs(userPitch) > USER_OVERRIDE_THRESHOLD) {
        finalPitch += userPitch * AIM_ASSIST_SENSITIVITY;
    } else {
        // Otherwise, blend user input with external aim assist
        float pitchError = externalPitch - turret.pitchMotor.getChassisFrameUnwrappedMeasuredAngle();
        finalPitch = turret.pitchMotor.getChassisFrameUnwrappedMeasuredAngle() + pitchError + (userPitch * AIM_ASSIST_SENSITIVITY);
    }
    
    // --- Yaw Calculation ---
    if (fabs(userYaw) > USER_OVERRIDE_THRESHOLD) {
        finalYaw += userYaw * AIM_ASSIST_SENSITIVITY;
    } else {
        float yawError = externalYaw - turret.yawMotor.getChassisFrameUnwrappedMeasuredAngle();
        finalYaw = turret.yawMotor.getChassisFrameUnwrappedMeasuredAngle() + yawError + (userYaw * AIM_ASSIST_SENSITIVITY);
    }
    
    turret.pitchMotor.setChassisFrameSetpoint(finalPitch);
    turret.yawMotor.setChassisFrameSetpoint(finalYaw);
}
