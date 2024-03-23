#include "src/control/chassis/chassis_subsystem.hpp"
#include "tap/algorithms/math_user_utils.hpp"
using namespace tap::algorithms;

namespace xcysrc
{
namespace chassis
{

modm::Pair<int, float> MecanumChassisSubsystem::lastComputedMaxWheelSpeed =
    CHASSIS_POWER_TO_MAX_SPEED_LUT[0];

MecanumChassisSubsystem::MecanumChassisSubsystem(tap::Drivers* drivers)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      leftFrontMotor(
          drivers,
          tap::motor::MOTOR2,
          tap::can::CanBus::CAN_BUS1,
          false,
          "left front drive motor"),
      leftBackMotor(
          drivers,
          tap::motor::MOTOR3,
          tap::can::CanBus::CAN_BUS1,
          false,
          "left back drive motor"),
      rightFrontMotor(
          drivers,
          tap::motor::MOTOR1,
          tap::can::CanBus::CAN_BUS1,
          false,
          "right front drive motor"),
      rightBackMotor(
          drivers,
          tap::motor::MOTOR4,
          tap::can::CanBus::CAN_BUS1,
          false,
          "right back drive motor"),
      velocityPid{
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT)}
{
    motors[LF] = &leftFrontMotor;
    motors[RF] = &rightFrontMotor;
    motors[LB] = &leftBackMotor;
    motors[RB] = &rightBackMotor;
}

void MecanumChassisSubsystem::initialize()
{
    for (int i = 0; i < getNumChassisMotors(); i++)
    {
        motors[i]->initialize();
    }
}

void MecanumChassisSubsystem::calculateOutput(float x, float y, float r, float maxWheelSpeed)
{
    // this is the distance between the center of the chassis to the wheel
    float chassisRotationRatio = 1.0f;
    // sqrtf(powf(WIDTH_BETWEEN_WHEELS_X / 2.0f, 2.0f) + powf(WIDTH_BETWEEN_WHEELS_Y / 2.0f, 2.0f));

    // to take into account the location of the turret so we rotate around the turret rather
    // than the center of the chassis, we calculate the offset and than multiply however
    // much we want to rotate by
    float leftFrontRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFrontRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float chassisRotateTranslated = modm::toDegree(r) / chassisRotationRatio;
    desiredWheelRPM[LF] = limitVal(
        -y + x - chassisRotateTranslated * leftFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[RF] = limitVal(
        -y - x - chassisRotateTranslated * rightFrontRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[LB] = limitVal(
        y + x - chassisRotateTranslated * leftBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);
    desiredWheelRPM[RB] = limitVal(
        y - x - chassisRotateTranslated * rightBackRotationRatio,
        -maxWheelSpeed,
        maxWheelSpeed);

    desiredRotation = r;
}

void MecanumChassisSubsystem::updateMotorRpmPid(
    modm::Pid<float>* pid,
    tap::motor::DjiMotor* const motor,
    float desiredRpm)
{
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

void MecanumChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    calculateOutput(x, y, r, 4500);
}

void MecanumChassisSubsystem::refresh()
{
    for (int i = 0; i < 4; i++)
    {
        updateMotorRpmPid(&velocityPid[i], motors[i], desiredWheelRPM[i]);
    }
    // rightFrontMotor.setDesiredOutput(static_cast<int32_t>(700));
}

float MecanumChassisSubsystem::calculateRotationTranslationalGain(
    float chassisRotationDesiredWheelspeed)
{
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotation is greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
    {
        const float maxWheelSpeed = MecanumChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // power(max revolve speed + min rotation threshold - specified revolve speed, 2) /
        // power(max revolve speed, 2)
        rTranslationalGain = powf(
            (maxWheelSpeed + MIN_ROTATION_THRESHOLD - fabsf(chassisRotationDesiredWheelspeed)) /
                maxWheelSpeed,
            2.0f);

        rTranslationalGain = limitVal(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain;
}

float MecanumChassisSubsystem::chassisSpeedRotationPID(float currentAngleError, float errD)
{
    // P
    float currRotationPidP = currentAngleError * AUTOROTATION_PID_KP;
    currRotationPidP = limitVal(currRotationPidP, -AUTOROTATION_PID_MAX_P, AUTOROTATION_PID_MAX_P);

    // D
    float currentRotationPidD = errD * AUTOROTATION_PID_KD;

    currentRotationPidD =
        limitVal(currentRotationPidD, -AUTOROTATION_PID_MAX_D, AUTOROTATION_PID_MAX_D);

    float wheelRotationSpeed = limitVal(
        currRotationPidP + currentRotationPidD,
        -AUTOROTATION_PID_MAX_OUTPUT,
        AUTOROTATION_PID_MAX_OUTPUT);

    return wheelRotationSpeed;
}


}  // namespace chassis
}  // namespace xcysrc