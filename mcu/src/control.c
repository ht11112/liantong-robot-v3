#include "control.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f

static void clear_command(Control *control)
{
    control->command_active = false;
    control->linear = 0;
    control->angular = 0;
}

static void break_odometry(Control *control)
{
    if (control->odom_valid) {
        control->odom_valid = false;
        ++control->segment;
    }
    control->v = 0;
    control->w = 0;
}

void control_init(Control *control, uint64_t boot)
{
    memset(control, 0, sizeof(*control));
    control->boot = boot;
    control->segment = 1;
}

void control_command(Control *control, float linear, float angular, uint32_t now)
{
    if (!control->stop_latched && control->fault == FAULT_NONE) {
        control->linear = linear;
        control->angular = angular;
        control->command_ms = now;
        control->command_active = true;
    }
}

void control_stop(Control *control)
{
    control->stop_latched = true;
    clear_command(control);
    control->quick_stop = true;
}

void control_resume(Control *control)
{
    control->stop_latched = false;
    clear_command(control);
}

void control_tick(Control *control, uint32_t now)
{
    if (control->command_active && now - control->command_ms >= COMMAND_TIMEOUT_MS) {
        clear_command(control);
        control->quick_stop = true;
    }
    if (control->odom_valid && now - control->measurement_ms >= ODOM_MAX_AGE_MS) {
        break_odometry(control);
    }
}

void control_fault(Control *control, Fault fault)
{
    clear_command(control);
    control->fault = fault;
    break_odometry(control);
}

void control_sample(Control *control, int16_t left_tenths, int16_t right_tenths, uint32_t now)
{
    const float meters_per_tenth_rpm = PI * WHEEL_DIAMETER_M / 600.0f;
    float left = left_tenths * meters_per_tenth_rpm;
    float right = -right_tenths * meters_per_tenth_rpm;
    float v = (left + right) * 0.5f;
    float w = (right - left) / TRACK_WIDTH_M;
    uint32_t dt_ms = now - control->measurement_ms;

    if (control->odom_valid && dt_ms >= ODOM_MAX_AGE_MS) {
        break_odometry(control);
    }
    if (control->odom_valid && dt_ms > 0) {
        float dt = dt_ms * 0.001f;
        float distance = (control->v + v) * 0.5f * dt;
        float angle = (control->w + w) * 0.5f * dt;
        float middle_yaw = control->yaw + angle * 0.5f;
        control->x += distance * cosf(middle_yaw);
        control->y += distance * sinf(middle_yaw);
        control->yaw = atan2f(sinf(control->yaw + angle), cosf(control->yaw + angle));
    }
    control->v = v;
    control->w = w;
    control->measurement_ms = now;
    ++control->sequence;
    control->odom_valid = true;
    control->fault = FAULT_NONE;
}

static int16_t target_rpm(float meters_per_second)
{
    float target = meters_per_second * (60.0f / (PI * WHEEL_DIAMETER_M));
    if (target >= INT16_MAX) {
        return INT16_MAX;
    }
    if (target <= INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)roundf(target);
}

void control_targets(const Control *control, int16_t *left, int16_t *right)
{
    float half_turn = control->angular * TRACK_WIDTH_M * 0.5f;
    *left = target_rpm(control->linear - half_turn);
    *right = target_rpm(-control->linear - half_turn);
}
