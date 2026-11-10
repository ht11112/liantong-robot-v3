#include "drive.h"

#include <string.h>

static void stop_outputs(Drive *drive)
{
    drive->need_zero = true;
    drive->need_quick = true;
    drive->quick_first = false;
    drive->enabled = false;
    drive->enable_step = 0;
}

static void fail(Drive *drive, Control *control, Fault fault, uint32_t now)
{
    bool quick_first = drive->pending == SDO_ZERO || drive->pending == SDO_TARGET;
    control_fault(control, fault);
    stop_outputs(drive);
    drive->quick_first = quick_first;
    drive->pending = SDO_IDLE;
    drive->mode_ok = false;
    drive->dual_axis = false;
    drive->units_ok = false;
    drive->backoff = true;
    drive->retry_ms = now;
}

void drive_init(Drive *drive, DriveSend send, void *user, uint32_t now)
{
    memset(drive, 0, sizeof(*drive));
    drive->send = send;
    drive->user = user;
    drive->need_zero = true;
    drive->speed_ms = now - CAN_SAMPLE_PERIOD_MS;
}

static void send_sdo(Drive *drive, Control *control, SdoOperation operation,
                     uint16_t index, uint8_t sub, uint8_t bytes,
                     bool write, uint32_t value, uint32_t now)
{
    uint8_t frame[8] = {0x40, (uint8_t)index, (uint8_t)(index >> 8), sub};
    if (write) {
        frame[0] = (uint8_t)(0x23u | ((4u - bytes) << 2));
        for (unsigned i = 0; i < bytes; ++i) {
            frame[4 + i] = (uint8_t)(value >> (8 * i));
        }
    }
    drive->pending = operation;
    drive->index = index;
    drive->sub = sub;
    drive->bytes = bytes;
    drive->write = write;
    drive->sent_ms = now;
    drive->sent_value = value;
    if (!drive->send(drive->user, frame)) {
        fail(drive, control, FAULT_CAN_SEND, now);
    }
}

void drive_receive(Drive *drive, Control *control, const uint8_t frame[8], uint32_t now)
{
    static const uint8_t status_steps[] = {1, 3, 7};
    uint16_t index = (uint16_t)(frame[1] | ((uint16_t)frame[2] << 8));
    if (drive->pending == SDO_IDLE || index != drive->index || frame[3] != drive->sub) {
        return;
    }
    if (now - drive->sent_ms >= CAN_RESPONSE_MS) {
        fail(drive, control, FAULT_CAN_TIMEOUT, now);
        return;
    }
    uint8_t expected = drive->write ? 0x60u : (uint8_t)(0x43u | ((4u - drive->bytes) << 2));
    if (frame[0] != expected) {
        fail(drive, control, FAULT_CAN_REPLY, now);
        return;
    }
    uint32_t value = 0;
    for (unsigned i = 0; i < drive->bytes; ++i) {
        value |= (uint32_t)frame[4 + i] << (8 * i);
    }
    switch (drive->pending) {
    case SDO_ZERO:
        drive->need_zero = false;
        drive->applied_value = 0;
        drive->applied_ms = now;
        break;
    case SDO_QUICK:
        drive->need_quick = false;
        break;
    case SDO_MODE:
        if (value != 3) {
            fail(drive, control, FAULT_DRIVE_CONFIG, now);
            return;
        }
        drive->mode_ok = true;
        break;
    case SDO_DUAL_AXIS:
    case SDO_UNITS:
        if (value != 1) {
            fail(drive, control, FAULT_DRIVE_CONFIG, now);
            return;
        }
        if (drive->pending == SDO_DUAL_AXIS) {
            drive->dual_axis = true;
        } else {
            drive->units_ok = true;
        }
        break;
    case SDO_CONTROL_WORD:
        if (!drive->need_quick) {
            ++drive->enable_step;
        }
        break;
    case SDO_STATUS: {
        if (drive->need_quick) {
            break;
        }
        uint8_t status = status_steps[drive->enable_step / 2];
        if ((value & 15u) != status || ((value >> 16) & 15u) != status) {
            fail(drive, control, FAULT_DRIVE_CONFIG, now);
            return;
        }
        drive->enabled = ++drive->enable_step == 6;
        break;
    }
    case SDO_SPEED:
        control_sample(control, (int16_t)value, (int16_t)(value >> 16), now);
        break;
    case SDO_TARGET:
        drive->applied_value = drive->sent_value;
        drive->applied_ms = now;
        break;
    case SDO_IDLE:
        break;
    }
    drive->pending = SDO_IDLE;
}

void drive_poll(Drive *drive, Control *control, bool bus_off, uint32_t now)
{
    bool had_odometry = control->odom_valid;
    control_tick(control, now);
    if (control->quick_stop) {
        stop_outputs(drive);
        control->quick_stop = false;
    }
    if (bus_off) {
        if (control->fault != FAULT_CAN_BUS_OFF) {
            fail(drive, control, FAULT_CAN_BUS_OFF, now);
        } else {
            control_fault(control, FAULT_CAN_BUS_OFF);
        }
        return;
    }
    if ((had_odometry && !control->odom_valid) ||
        (drive->pending != SDO_IDLE && now - drive->sent_ms >= CAN_RESPONSE_MS)) {
        fail(drive, control, FAULT_CAN_TIMEOUT, now);
    }
    if (drive->pending != SDO_IDLE ||
        (drive->backoff && now - drive->retry_ms < CAN_SAMPLE_PERIOD_MS)) {
        return;
    }
    drive->backoff = false;

    /* 零目标未应答时先尝试快停。停车路径完成前不发普通轮询。 */
    if (drive->need_zero && !(drive->need_quick && drive->quick_first)) {
        send_sdo(drive, control, SDO_ZERO, 0x60ff, 3, 4, true, 0, now);
        return;
    }
    if (drive->need_quick) {
        send_sdo(drive, control, SDO_QUICK, 0x6040, 0, 2, true, 2, now);
        return;
    }
    if (!drive->mode_ok) {
        send_sdo(drive, control, SDO_MODE, 0x6061, 0, 1, false, 0, now);
        return;
    }
    if (!drive->dual_axis) {
        send_sdo(drive, control, SDO_DUAL_AXIS, 0x200f, 0, 2, false, 0, now);
        return;
    }
    if (!drive->units_ok) {
        send_sdo(drive, control, SDO_UNITS, 0x2026, 5, 2, false, 0, now);
        return;
    }

    int16_t left, right;
    control_targets(control, &left, &right);
    uint32_t packed = (uint16_t)left | ((uint32_t)(uint16_t)right << 16);
    if (drive->enabled && packed == 0 && drive->applied_value != 0) {
        send_sdo(drive, control, SDO_TARGET, 0x60ff, 3, 4, true, 0, now);
        return;
    }
    if (now - drive->speed_ms >= CAN_SAMPLE_PERIOD_MS) {
        drive->speed_ms = now;
        send_sdo(drive, control, SDO_SPEED, 0x606c, 3, 4, false, 0, now);
        return;
    }
    if (!control->odom_valid || packed == 0) {
        return;
    }
    if (!drive->enabled) {
        static const uint16_t control_words[] = {6, 7, 15};
        if (drive->enable_step % 2 == 0) {
            send_sdo(drive, control, SDO_CONTROL_WORD, 0x6040, 0, 2, true,
                     control_words[drive->enable_step / 2], now);
        } else {
            send_sdo(drive, control, SDO_STATUS, 0x6041, 0, 4, false, 0, now);
        }
        return;
    }
    if (drive->applied_value != packed || now - drive->applied_ms >= 100u) {
        send_sdo(drive, control, SDO_TARGET, 0x60ff, 3, 4, true, packed, now);
    }
}
