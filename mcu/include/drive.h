#ifndef V3_DRIVE_H
#define V3_DRIVE_H

#include "control.h"

#define CAN_RESPONSE_MS 50u
#define CAN_SAMPLE_PERIOD_MS 50u

typedef bool (*DriveSend)(void *user, const uint8_t frame[8]);

typedef enum {
    SDO_IDLE, SDO_ZERO, SDO_QUICK, SDO_MODE, SDO_DUAL_AXIS, SDO_UNITS, SDO_CONTROL_WORD,
    SDO_STATUS, SDO_SPEED, SDO_TARGET
} SdoOperation;

typedef struct {
    DriveSend send;
    void *user;
    SdoOperation pending;
    uint32_t sent_ms, speed_ms, applied_ms, retry_ms, sent_value, applied_value;
    uint16_t index;
    uint8_t sub, bytes, enable_step;
    bool write, need_zero, need_quick, quick_first, mode_ok, dual_axis, units_ok, enabled, backoff;
} Drive;

void drive_init(Drive *drive, DriveSend send, void *user, uint32_t now);
void drive_receive(Drive *drive, Control *control, const uint8_t frame[8], uint32_t now);
void drive_poll(Drive *drive, Control *control, bool bus_off, uint32_t now);

#endif
