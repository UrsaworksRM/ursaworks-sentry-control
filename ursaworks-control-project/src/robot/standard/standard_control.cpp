
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "src/drivers_singleton.hpp"

// #include "src/control/agitator/velocity_agitator_subsystem.hpp"
// #include "src/control/a/example_subsystem.hpp"
// #include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "src/control/chassis/chassis_autorotate_command.hpp"
// #include "aruwsrc/control/client-display/client_display_command.hpp"
// #include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "src/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "src/control/turret/user/turret_user_world_relative_command.hpp"
#include "src/control/chassis/chassis_subsystem.hpp"

using namespace tap::control::setpoint;
using namespace xcysrc::standard;
using namespace tap::control;
using namespace xcysrc::control;

using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace standard_control
{
    static constexpr xcysrc::control::turret::TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 4778,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

static constexpr xcysrc::control::turret::TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 6300,
    .minAngle = modm::toRadian(80),
    .maxAngle = modm::toRadian(120),
    .limitMotorAngles = true,
};
namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 59'183.1f,
    .ki = 0.0f,
    .kd = 5000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 30.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 72183.1f,
    .ki = 100.0f,
    .kd = 1000.0f,
    .maxICumulative = 10.0f,
    .maxOutput = 20000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace chassis_rel
/* define subsystems --------------------------------------------------------*/

tap::motor::DjiMotor pitchMotor(drivers(), tap::motor::MOTOR6, tap::can::CanBus::CAN_BUS1, false, "Pitch Turret");

tap::motor::DjiMotor yawMotor(
    drivers(),
    tap::motor::MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    false,
    "Yaw Turret");
xcysrc::control::turret::TurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG);


xcysrc::chassis::MecanumChassisSubsystem chassis(drivers());

// VelocityAgitatorSubsystem agitator(
//     drivers(),
//     constants::AGITATOR_PID_CONFIG,
//     constants::AGITATOR_CONFIG);

// ExampleSubsystem agitator(
//     drivers(),
//     constants::AGITATOR_PID_CONFIG,
//     constants::AGITATOR_CONFIG);

// xcysrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
//     aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
//     frictionWheels(
//         drivers(),
//         xcysrc::control::launcher::LEFT_MOTOR_ID,
//         xcysrc::control::launcher::RIGHT_MOTOR_ID,
//         xcysrc::control::launcher::CAN_BUS_MOTORS,
//         &getTurretMCBCanComm(),
//         tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);



/* define commands ----------------------------------------------------------*/

xcysrc::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor,
    xcysrc::chassis::ChassisSymmetry::SYMMETRICAL_180);

// xcysrc::chassis::BeybladeCommand beybladeCommand(
//     drivers(),
//     &chassis,
//     &turret.yawMotor,
//     (drivers()->controlOperatorInterface));

// Turret controllers
xcysrc::control::turret::algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController( //todo
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

xcysrc::control::turret::algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

// algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
//     *drivers(),
//     turret.yawMotor,
//     world_rel_chassis_imu::YAW_PID_CONFIG);


// turret commands
xcysrc::control::turret::user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    // &worldFrameYawChassisImuController,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    0.02f,
    0.02f);

// MoveIntegralCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

// rotates agitator when aiming at target and within heat limit

//xcysrc::control::launcher::FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
//     drivers(),
//     &frictionWheels,
//     15.0f,
//     false,
//     tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

// xcysrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
//     drivers(),
//     &frictionWheels,
//     0.0f,
//     true,
//     tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);


/* define command mappings --------------------------------------------------*/
// Remote related mappings

// HoldCommandMapping leftSwitchDown(
//     drivers(),
//     {&beybladeCommand},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

// Keyboard/Mouse related mappings

// ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));

// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.


// The user can press q or e to manually rotate the chassis left or right.
// The user can press q and e simultaneously to enable wiggle driving. Wiggling is cancelled
// automatically once a different drive mode is chosen.

PressCommandMapping xPressed(
    drivers(),
    {&chassisAutorotateCommand},
    RemoteMapState({tap::Remote::Key::X}));

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(Drivers *drivers)
{
    // drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    // drivers->commandScheduler.registerSubsystem(&frictionWheels);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    chassis.initialize();
    // agitator.initialize();
    // frictionWheels.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    // frictionWheels.setDefaultCommand(&spinFrictionWheels);
}


/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&xPressed);
}
}  // namespace standard_control

namespace xcysrc::standard
{
void initSubsystemCommands(Drivers *drivers)
{
    standard_control::initializeSubsystems();
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}
}  // namespace xcysrc::standard

