#include "hardware/motors.h"
#include <math.h>

static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t PWM_RES = 8;
static constexpr int16_t PWM_MAX = 255;

Motor::Motor(uint8_t in1, uint8_t in2, uint8_t ch1, uint8_t ch2)
    : _in1(in1), _in2(in2), _ch1(ch1), _ch2(ch2) {}

void Motor::begin(uint32_t freq, uint8_t resBits)
{
    ledcSetup(_ch1, freq, resBits);
    ledcSetup(_ch2, freq, resBits);
    ledcAttachPin(_in1, _ch1);
    ledcAttachPin(_in2, _ch2);
    stop();
}

void Motor::stop()
{
    ledcWrite(_ch1, 0);
    ledcWrite(_ch2, 0);
}

void Motor::set(int16_t speed)
{
    if (speed > PWM_MAX)
        speed = PWM_MAX;
    if (speed < -PWM_MAX)
        speed = -PWM_MAX;

    if (speed >= 0)
    {
        ledcWrite(_ch1, (uint8_t)speed);
        ledcWrite(_ch2, 0);
    }
    else
    {
        ledcWrite(_ch1, 0);
        ledcWrite(_ch2, (uint8_t)(-speed));
    }
}

int16_t DifferentialDrive::clampI16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return (int16_t)v;
}

float DifferentialDrive::clampF(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

void DifferentialDrive::begin()
{
    _left.begin(PWM_FREQ, PWM_RES);
    _right.begin(PWM_FREQ, PWM_RES);
    stop();
}

void DifferentialDrive::stop()
{
    _left.stop();
    _right.stop();
}

void DifferentialDrive::drive(int16_t left, int16_t right)
{
    _left.set(left);
    _right.set(right);
}

void DifferentialDrive::twist(float linear, float angular)
{
    linear = clampF(linear, -1.0f, 1.0f);
    angular = clampF(angular, -1.0f, 1.0f);

    float l = linear - angular;
    float r = linear + angular;

    float m = fabsf(l);
    if (fabsf(r) > m)
        m = fabsf(r);

    if (m > 1.0f)
    {
        l /= m;
        r /= m;
    }

    int16_t left = clampI16((int32_t)lroundf(l * PWM_MAX), -PWM_MAX, PWM_MAX);
    int16_t right = clampI16((int32_t)lroundf(r * PWM_MAX), -PWM_MAX, PWM_MAX);

    drive(left, right);
}

MotorController::MotorController(SystemState *state) : _state(state) {}

void MotorController::begin()
{
    _drive.begin();
}

void MotorController::startTask()
{
    if (_task)
        return;

    xTaskCreatePinnedToCore(
        taskTrampoline,
        "MotorTask",
        3072,
        this,
        2,
        &_task,
        1);
}

void MotorController::stopTask()
{
    if (!_task)
        return;
    vTaskDelete(_task);
    _task = nullptr;
}

void MotorController::taskTrampoline(void *arg)
{
    static_cast<MotorController *>(arg)->taskLoop();
}

void MotorController::taskLoop()
{
    for (;;)
    {
        DriveMode mode;
        float lin, ang;
        bool obsL, obsR;

        _state->lock();
        mode = _state->driveMode;
        lin = _state->linearVelocity;
        ang = _state->angularVelocity;
        obsL = _state->obstacleLeft;
        obsR = _state->obstacleRight;
        _state->unlock();

        if (mode == MANUAL)
        {
            if ((obsL || obsR) && lin > 0.05f)
                _drive.stop();
            else
                _drive.twist(lin, ang);
        }
        else
        {
            _drive.stop();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
