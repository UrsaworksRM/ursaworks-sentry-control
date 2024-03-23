#include "modm/math/filter.hpp"
#include "modm/math/geometry.hpp"
#include "modm/math/interpolation/linear.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/drivers.hpp"

namespace xcysrc
{
namespace chassis
{
static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {
    {50, 4'500},
    {60, 5'700},
    {70, 6'400},
    {80, 6'700},
    {100, 7'000},
    {120, 8'000},
};

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_SPEED_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

static constexpr float VELOCITY_PID_KP = 20.0f;
static constexpr float VELOCITY_PID_KI = 0.2f;
static constexpr float VELOCITY_PID_KD = 0.0f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 5'000.0f;
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16'000.0f;
static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.385f;
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.366f;
static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;
static constexpr float AUTOROTATION_PID_KP = 5'729.6f;
static constexpr float AUTOROTATION_PID_KD = 57.3f;
static constexpr float AUTOROTATION_PID_MAX_P = 4'000.0f;
static constexpr float AUTOROTATION_PID_MAX_D = 5'000.0f;
static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5'500.0f;
static constexpr float AUTOROTATION_MIN_SMOOTHING_ALPHA = 0.001f;

class MecanumChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    MecanumChassisSubsystem(tap::Drivers* drivers);
    void initialize() override;
    void setDesiredOutput(float x, float y, float r);
    inline int getNumChassisMotors() const override { return MODM_ARRAY_SIZE(motors); }
    inline int16_t getLeftFrontRpmActual() const  { return leftFrontMotor.getShaftRPM(); }
    inline int16_t getLeftBackRpmActual() const  { return leftBackMotor.getShaftRPM(); }
    inline int16_t getRightFrontRpmActual() const  { return rightFrontMotor.getShaftRPM(); }
    inline int16_t getRightBackRpmActual() const  { return rightBackMotor.getShaftRPM(); }
    enum WheelRPMIndex
    {
        LF = 0,
        RF = 1,
        LB = 2,
        RB = 3,
    };
    void refresh() override;
    float chassisSpeedRotationPID(float currentAngleError, float errD);
    inline float getDesiredRotation() const { return desiredRotation; }
    static inline float getMaxWheelSpeed(bool refSerialOnline, int chassisPower)
    {
        if (!refSerialOnline)
        {
            chassisPower = 0;
        }

        // only re-interpolate when needed (since this function is called a lot and the chassis
        // power rarely changes, this helps cut down on unnecessary array searching/interpolation)
        if (lastComputedMaxWheelSpeed.first != chassisPower)
        {
            lastComputedMaxWheelSpeed.first = chassisPower;
            lastComputedMaxWheelSpeed.second =
                CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(chassisPower);
        }

        return lastComputedMaxWheelSpeed.second;
    }

    tap::motor::DjiMotor leftFrontMotor;
    tap::motor::DjiMotor leftBackMotor;
    tap::motor::DjiMotor rightFrontMotor;
    tap::motor::DjiMotor rightBackMotor;
    static modm::Pair<int, float> lastComputedMaxWheelSpeed;
    mockable float calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed);
    inline void setZeroRPM()
    {
        desiredWheelRPM[0] = 0;
        desiredWheelRPM[1] = 0;
        desiredWheelRPM[2] = 0;
        desiredWheelRPM[3] = 0;
    }

private:
    void calculateOutput(float x, float y, float r, float maxWheelSpeed);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        tap::motor::DjiMotor* const motor,
        float desiredRpm);

    modm::Pid<float> velocityPid[4];
    tap::motor::DjiMotor* motors[4];
    float desiredWheelRPM[4];
    float desiredRotation = 0;
};

}  // namespace chassis
}  // namespace xcysrc
