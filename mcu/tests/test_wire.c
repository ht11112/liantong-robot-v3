#include "protocol.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static Reply line(Parser *parser, Control *control, const char *text, uint32_t now)
{
    Reply reply = REPLY_NONE;
    for (; *text; ++text) {
        Reply result = protocol_byte(parser, control, *text, now);
        if (result != REPLY_NONE) {
            assert(reply == REPLY_NONE);
            reply = result;
        }
    }
    return reply;
}

static void golden(Control *control, uint32_t now, const char *expected)
{
    char text[REPLY_CAPACITY];
    size_t size = protocol_reply(text, REPLY_ODOM, control, now);
    assert(size == strlen(text) && size < sizeof(text));
    assert(strcmp(text, expected) == 0);
    printf("GOLDEN|%s", text);
}

int main(void)
{
    Parser parser = {0};
    Control control;
    control_init(&control, UINT64_C(0x0123456789abcdef));
    assert(line(&parser, &control, "QUERY\r\n", 5) == REPLY_ODOM);
    golden(&control, 5, "ODO,0123456789abcdef,0,0,5,0,1,0.000000,0.000000,0.000000,0.000000,0.000000,0,0\n");
    assert(line(&parser, &control, "cMd,0.18,abc\r\n", 100) == REPLY_COMMAND);
    assert(fabsf(control.linear - 0.18f) < 0.00001f && control.angular == 0);
    assert(line(&parser, &control, "foo,0.20tail\n", 101) == REPLY_COMMAND);
    assert(control.linear == 0 && fabsf(control.angular - 0.2f) < 0.00001f);
    assert(line(&parser, &control, "CMD,+4e0,-0x1p+1\n", 102) == REPLY_COMMAND);
    assert(control.linear == 1 && control.angular == -1);
    assert(line(&parser, &control, "CMD,nan,0\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "CMD,0,INF\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "CMD,0,1e999\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "CMD,0\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "STOP,0.2\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "QUERY,0.2\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "CMDX,0.2\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "STARTED\n", 103) == REPLY_INPUT);
    assert(line(&parser, &control, "UNKNOWN\n", 103) == REPLY_INPUT);
    for (unsigned i = 0; i < LINE_CAPACITY + 1; ++i) {
        assert(protocol_byte(&parser, &control, 'A', 103) == REPLY_NONE);
    }
    assert(line(&parser, &control, "STOP\r\n", 103) == REPLY_INPUT);
    assert(!control.stop_latched && control.command_ms == 102);
    puts("PASS ASCII compatibility, full keywords, CR/LF, nonfinite and overlong lines");

    assert(line(&parser, &control, "ESTOP\n", 110) == REPLY_STOP);
    assert(control.stop_latched && control.quick_stop && !control.command_active);
    assert(line(&parser, &control, "CMD,0.2,0\n", 111) == REPLY_COMMAND);
    assert(!control.command_active);
    assert(line(&parser, &control, "START\n", 112) == REPLY_RESUME);
    assert(!control.stop_latched && !control.command_active);
    control_fault(&control, FAULT_CAN_REPLY);
    assert(line(&parser, &control, "CMD,0.2,0\n", 113) == REPLY_COMMAND);
    assert(!control.command_active);
    control_sample(&control, 100, -100, 150);
    assert(!control.command_active && control.fault == FAULT_NONE);
    golden(&control, 160, "ODO,0123456789abcdef,1,150,160,1,1,0.000000,0.000000,0.000000,0.104720,0.000000,0,0\n");
    golden(&control, 180, "ODO,0123456789abcdef,1,150,180,1,1,0.000000,0.000000,0.000000,0.104720,0.000000,0,0\n");
    assert(line(&parser, &control, "STOP\n", 200) == REPLY_STOP);
    control_tick(&control, 300);
    control_fault(&control, FAULT_CAN_TIMEOUT);
    golden(&control, 301, "ODO,0123456789abcdef,1,150,301,0,2,0.000000,0.000000,0.000000,0.000000,0.000000,1,2\n");
    control_sample(&control, 0, 0, 350);
    golden(&control, 360, "ODO,0123456789abcdef,2,350,360,1,2,0.000000,0.000000,0.000000,0.000000,0.000000,1,0\n");
    puts("PASS STOP and RESUME, CAN-fault command discard, truthful and repeated ODO");

    control_init(&control, 1);
    uint32_t started = UINT32_MAX - 400u;
    assert(line(&parser, &control, "CMD,0.2,0\n", started) == REPLY_COMMAND);
    control_tick(&control, started + 509u);
    assert(control.command_active && !control.quick_stop);
    assert(line(&parser, &control, "QUERY\n", started + 510u) == REPLY_ODOM);
    control_tick(&control, started + 510u);
    assert(!control.command_active && !control.stop_latched && control.quick_stop);
    assert(line(&parser, &control, "0.2,0\n", started + 511u) == REPLY_COMMAND);
    assert(control.command_active);
    puts("PASS precise 510ms command timeout, tick wrap and non-renewing QUERY");
    return 0;
}
