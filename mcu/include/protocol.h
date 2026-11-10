#ifndef V3_PROTOCOL_H
#define V3_PROTOCOL_H

#include <stddef.h>
#include "control.h"

#define LINE_CAPACITY 96u
#define REPLY_CAPACITY 192u
#define REPLY_QUIET_MS 2u

typedef enum {
    REPLY_NONE, REPLY_INPUT, REPLY_COMMAND, REPLY_STOP, REPLY_RESUME, REPLY_ODOM
} Reply;

typedef struct {
    char line[LINE_CAPACITY];
    size_t used;
    bool discard;
} Parser;

Reply protocol_byte(Parser *parser, Control *control, char byte, uint32_t now);
size_t protocol_reply(char *out, Reply reply, const Control *control, uint32_t now);

#endif
