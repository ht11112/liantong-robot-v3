#ifndef V3_CONTROL_H
#define V3_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define COMMAND_TIMEOUT_MS 510u
#define ODOM_MAX_AGE_MS 150u
#define WHEEL_DIAMETER_M 0.20f
#define TRACK_WIDTH_M 0.30f

typedef enum {
    FAULT_NONE, FAULT_CAN_SEND, FAULT_CAN_TIMEOUT, FAULT_CAN_REPLY,
    FAULT_DRIVE_CONFIG, FAULT_CAN_BUS_OFF
} Fault;

typedef struct {
    uint64_t boot;
    uint32_t command_ms, measurement_ms, sequence, segment;
    float linear, angular;
    float x, y, yaw, v, w;
    Fault fault;
    bool command_active, stop_latched, quick_stop, odom_valid;
} Control;

void control_init(Control *control, uint64_t boot);
void control_command(Control *control, float linear, float angular, uint32_t now);
void control_stop(Control *control);
void control_resume(Control *control);
void control_tick(Control *control, uint32_t now);
void control_fault(Control *control, Fault fault);
void control_sample(Control *control, int16_t left_tenths, int16_t right_tenths, uint32_t now);
void control_targets(const Control *control, int16_t *left, int16_t *right);

#endif
