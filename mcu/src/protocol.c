#include "protocol.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *after_prefix(const char *line, const char *command)
{
    while (*command != '\0') {
        if (toupper((unsigned char)*line++) != *command++) {
            return NULL;
        }
    }
    return line;
}

static bool matches(const char *line, const char *command)
{
    const char *end = after_prefix(line, command);
    return end && *end == '\0';
}

static bool reserved_prefix(const char *line)
{
    static const char *const commands[] = {"CMD", "STOP", "ESTOP", "START", "RESUME", "QUERY"};
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        if (after_prefix(line, commands[i])) {
            return true;
        }
    }
    return false;
}

static Reply parse(char *line, Control *control, uint32_t now)
{
    while (isspace((unsigned char)*line)) {
        ++line;
    }
    size_t length = strlen(line);
    while (length > 0 && isspace((unsigned char)line[length - 1])) {
        line[--length] = '\0';
    }
    if (length == 0) {
        return REPLY_NONE;
    }
    if (matches(line, "STOP") || matches(line, "ESTOP")) {
        control_stop(control);
        return REPLY_STOP;
    }
    if (matches(line, "RESUME") || matches(line, "START")) {
        control_resume(control);
        return REPLY_RESUME;
    }
    if (matches(line, "QUERY")) {
        return REPLY_ODOM;
    }
    char *comma = strchr(line, ',');
    if (!comma) {
        return REPLY_INPUT;
    }
    *comma = '\0';
    if (matches(line, "CMD")) {
        line = comma + 1;
        comma = strchr(line, ',');
        if (!comma) {
            return REPLY_INPUT;
        }
        *comma = '\0';
    } else if (reserved_prefix(line)) {
        return REPLY_INPUT;
    }
    /* strtod 的无转换值和数值后的尾字沿用原小车语义。 */
    double linear = strtod(line, NULL);
    double angular = strtod(comma + 1, NULL);
    if (!isfinite(linear) || !isfinite(angular)) {
        return REPLY_INPUT;
    }
    control_command(control, linear < -1.0 ? -1.0f : linear > 1.0 ? 1.0f : (float)linear,
                    angular < -1.0 ? -1.0f : angular > 1.0 ? 1.0f : (float)angular, now);
    return REPLY_COMMAND;
}

Reply protocol_byte(Parser *parser, Control *control, char byte, uint32_t now)
{
    if (byte == '\n' || byte == '\r') {
        parser->line[parser->used] = '\0';
        Reply reply = parser->discard ? REPLY_INPUT : parse(parser->line, control, now);
        parser->used = 0;
        parser->discard = false;
        return reply;
    }
    if (byte == '\0' || parser->used == sizeof(parser->line) - 1) {
        parser->discard = true;
    }
    if (!parser->discard) {
        parser->line[parser->used++] = byte;
    }
    return REPLY_NONE;
}

static char *unsigned_digits(char *out, uint32_t value, unsigned base, size_t width)
{
    static const char digits[] = "0123456789abcdef";
    char reversed[10];
    size_t count = 0;
    do {
        reversed[count++] = digits[value % base];
        value /= base;
    } while (value != 0);
    for (size_t i = count; i < width; ++i) {
        *out++ = '0';
    }
    while (count > 0) {
        *out++ = reversed[--count];
    }
    return out;
}

static char *decimal(char *out, float value)
{
    /* 固定六位，避免 Cortex-M 镜像为 printf 浮点格式引入整套转换。 */
    if (value < 0) {
        *out++ = '-';
    }
    float magnitude = fabsf(value);
    uint32_t whole = (uint32_t)magnitude;
    uint32_t fraction = (uint32_t)roundf((magnitude - (float)whole) * 1000000.0f);
    if (fraction == 1000000u) {
        ++whole;
        fraction = 0;
    }
    out = unsigned_digits(out, whole, 10, 0);
    *out++ = '.';
    return unsigned_digits(out, fraction, 10, 6);
}

size_t protocol_reply(char *out, Reply reply, const Control *control, uint32_t now)
{
    static const char *const replies[] = {"", "ERR,INPUT\n", "ACK,CMD\n", "ACK,STOP\n", "ACK,RESUME\n"};
    if (reply != REPLY_ODOM) {
        size_t length = strlen(replies[reply]);
        memcpy(out, replies[reply], length + 1);
        return length;
    }

    bool valid = control->odom_valid && now - control->measurement_ms < ODOM_MAX_AGE_MS;
    memcpy(out, "ODO,", 4);
    char *next = unsigned_digits(out + 4, (uint32_t)(control->boot >> 32), 16, 8);
    next = unsigned_digits(next, (uint32_t)control->boot, 16, 8);
    uint32_t integers[] = {control->sequence, control->measurement_ms, now, valid, control->segment};
    for (size_t i = 0; i < sizeof(integers) / sizeof(integers[0]); ++i) {
        *next++ = ',';
        next = unsigned_digits(next, integers[i], 10, 0);
    }
    float values[] = {control->x, control->y, control->yaw, valid ? control->v : 0, valid ? control->w : 0};
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        *next++ = ',';
        next = decimal(next, values[i]);
    }
    *next++ = ',';
    *next++ = control->stop_latched ? '1' : '0';
    *next++ = ',';
    next = unsigned_digits(next, control->fault, 10, 0);
    *next++ = '\n';
    *next = '\0';
    return (size_t)(next - out);
}
