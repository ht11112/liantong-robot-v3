#include "drive.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint16_t index;
    uint32_t value;
    bool write;
} Frame;

typedef struct {
    Control control;
    Drive drive;
    uint32_t now;
    Frame request;
    Frame frames[256];
    unsigned sent;
    uint8_t mode, status, units;
    bool waiting, drop, bus_off, refuse, abort;
    uint32_t speed;
} Peer;

static bool transmit(void *user, const uint8_t bytes[8])
{
    Peer *peer = user;
    if (peer->refuse) {
        return false;
    }
    Frame frame = {0};
    frame.index = (uint16_t)(bytes[1] | ((uint16_t)bytes[2] << 8));
    frame.write = bytes[0] != 0x40;
    for (unsigned i = 0; i < 4; ++i) {
        frame.value |= (uint32_t)bytes[4 + i] << (8 * i);
    }
    assert(peer->sent < sizeof(peer->frames) / sizeof(peer->frames[0]));
    peer->frames[peer->sent++] = frame;
    peer->request = frame;
    peer->waiting = !peer->drop;
    return true;
}

static void tick(Peer *peer)
{
    ++peer->now;
    if (peer->waiting) {
        peer->waiting = false;
        Frame frame = peer->request;
        uint8_t bytes[8] = {0x43, (uint8_t)frame.index, (uint8_t)(frame.index >> 8), 0};
        uint32_t value = 0;
        if (peer->abort) {
            bytes[0] = 0x80;
        } else if (frame.write) {
            assert(frame.index == 0x6040 || frame.index == 0x60ff);
            bytes[0] = 0x60;
            bytes[3] = frame.index == 0x60ff ? 3 : 0;
            if (frame.index == 0x6040) {
                peer->status = frame.value == 6 ? 1 : frame.value == 7 ? 3 : 7;
            }
        } else if (frame.index == 0x6061) {
            bytes[0] = 0x4f;
            value = peer->mode;
        } else if (frame.index == 0x200f || frame.index == 0x2026) {
            bytes[0] = 0x4b;
            bytes[3] = frame.index == 0x2026 ? 5 : 0;
            value = frame.index == 0x2026 ? peer->units : 1;
        } else if (frame.index == 0x6041) {
            value = peer->status | ((uint32_t)peer->status << 16);
        } else {
            assert(frame.index == 0x606c);
            bytes[3] = 3;
            value = peer->speed;
        }
        for (unsigned i = 0; i < 4; ++i) {
            bytes[4 + i] = (uint8_t)(value >> (8 * i));
        }
        drive_receive(&peer->drive, &peer->control, bytes, peer->now);
    }
    drive_poll(&peer->drive, &peer->control, peer->bus_off, peer->now);
}

static void run(Peer *peer, unsigned ms)
{
    for (unsigned i = 0; i < ms; ++i) {
        tick(peer);
    }
}

static void init(Peer *peer)
{
    memset(peer, 0, sizeof(*peer));
    peer->mode = 3;
    peer->units = 1;
    control_init(&peer->control, 1);
    drive_init(&peer->drive, transmit, peer, 0);
}

static unsigned target_count(const Peer *peer, unsigned from)
{
    unsigned count = 0;
    for (unsigned i = from; i < peer->sent; ++i) {
        Frame frame = peer->frames[i];
        count += frame.write && frame.index == 0x60ff && frame.value != 0;
    }
    return count;
}

int main(void)
{
    Peer peer;
    init(&peer);
    run(&peer, 100);
    assert(peer.frames[0].index == 0x60ff && peer.frames[0].value == 0);
    assert(peer.control.odom_valid && peer.control.sequence == 2);
    assert(!peer.drive.enabled && target_count(&peer, 0) == 0);
    unsigned before = peer.sent;
    control_command(&peer.control, 0.18f, 0, peer.now);
    run(&peer, 30);
    assert(peer.drive.enabled && target_count(&peer, before) == 1);
    unsigned step = 0;
    const uint32_t control_words[] = {6, 7, 15};
    for (unsigned i = before; i < peer.sent; ++i) {
        Frame frame = peer.frames[i];
        if (frame.write && frame.index == 0x6040) {
            assert(step < 3 && frame.value == control_words[step++]);
        }
    }
    assert(step == 3);
    assert((int16_t)peer.drive.applied_value == 17 &&
           (int16_t)(peer.drive.applied_value >> 16) == -17);
    puts("PASS real SDO enable path, verified RPM units and uncapped differential targets");

    before = peer.sent;
    control_stop(&peer.control);
    run(&peer, 15);
    assert(peer.frames[before].index == 0x60ff && peer.frames[before].value == 0);
    assert(peer.frames[before + 1].index == 0x6040 && peer.frames[before + 1].value == 2);
    assert(!peer.drive.enabled && peer.control.stop_latched);
    control_command(&peer.control, 0.25f, 0, peer.now);
    run(&peer, 30);
    control_resume(&peer.control);
    run(&peer, 30);
    assert(target_count(&peer, before) == 0);
    control_command(&peer.control, 0.25f, 0, peer.now);
    run(&peer, 510);
    assert(target_count(&peer, before) > 0 && !peer.control.stop_latched && !peer.control.command_active);
    run(&peer, 10);
    assert(!peer.drive.enabled && peer.drive.applied_value == 0);
    puts("PASS STOP zero then Quick Stop, no replay on RESUME or command timeout");

    control_command(&peer.control, 0.2f, 0, peer.now);
    run(&peer, 15);
    assert(peer.drive.enabled);
    uint32_t segment = peer.control.segment;
    peer.drop = true;
    run(&peer, 105);
    assert(peer.control.fault == FAULT_CAN_TIMEOUT && !peer.control.odom_valid);
    assert(peer.control.segment == segment + 1 && !peer.control.command_active);
    control_command(&peer.control, 0.2f, 0, peer.now);
    assert(!peer.control.command_active);
    float x = peer.control.x;
    before = peer.sent;
    peer.drop = false;
    run(&peer, 170);
    assert(peer.control.odom_valid && peer.control.fault == FAULT_NONE && peer.control.x == x);
    assert(target_count(&peer, before) == 0);
    control_command(&peer.control, 0.2f, 0, peer.now);
    run(&peer, 20);
    assert(target_count(&peer, before) == 1);
    puts("PASS CAN deadline cuts odom, ignores targets during fault and recovers on fresh command");

    peer.bus_off = true;
    before = peer.sent;
    run(&peer, 10);
    assert(peer.control.fault == FAULT_CAN_BUS_OFF && !peer.control.odom_valid);
    assert(!peer.control.command_active && peer.sent == before);
    init(&peer);
    peer.refuse = true;
    run(&peer, 10);
    assert(peer.control.fault == FAULT_CAN_SEND && !peer.control.odom_valid);
    peer.refuse = false;
    peer.abort = true;
    run(&peer, 100);
    assert(peer.control.fault == FAULT_CAN_REPLY && !peer.control.odom_valid);
    init(&peer);
    peer.units = 10;
    run(&peer, 200);
    assert(peer.control.fault == FAULT_DRIVE_CONFIG && !peer.control.odom_valid && !peer.drive.enabled);
    puts("PASS bus-off, send rejection, SDO abort and unverified drive settings");

    Control odom;
    control_init(&odom, 1);
    control_sample(&odom, 100, -100, 10);
    control_sample(&odom, 100, -100, 60);
    assert(fabsf(odom.x - 0.00523599f) < 0.000001f && odom.yaw == 0);
    control_sample(&odom, -100, -100, 110);
    control_sample(&odom, -100, -100, 160);
    control_sample(&odom, -100, -100, 210);
    assert(odom.yaw > 0.08f && odom.w > 0);
    x = odom.x;
    control_sample(&odom, 100, -100, 360);
    assert(odom.x == x && odom.segment == 2);
    puts("PASS real wheel signs, SI units, accumulated yaw and missing-source gap");
    return 0;
}
