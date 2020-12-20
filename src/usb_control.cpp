#include <Fsm.h>
#include "usb_control.h"
#include "events.h"

extern Fsm barndoor;

typedef struct {
    char const *command;
    const int len;
    void (*action)(char *params);
} Command;


void actionStart(char *p) {
    barndoor.trigger(EVENT_START_BUTTON);
    Serial.print(F(">OK\r"));
}

void actionStop(char *p) {
    barndoor.trigger(EVENT_STOP_BUTTON);
    Serial.print(F(">OK\r"));
}

void actionRewind(char *p) {
    barndoor.trigger(EVENT_REWIND_BUTTON);
    Serial.print(F(">OK\r"));
}

void actionAutoHome(char *p) {
    barndoor.trigger(EVENT_AUTO_HOME);
    Serial.print(F(">OK\r"));
}

void actionSetMinOpening(char *p) {
    Serial.print(F(">ERR: NOT SUPPORTED\r"));
}

void actionSetMaxOpening(char *p) {
    Serial.print(F(">ERR: NOT SUPPORTED\r"));
}

void actionSaveConfig(char *p) {
    Serial.print(F(">ERR: NOT SUPPORTED\r"));
}


#define DEF_COMMAND(cmd)         static const char COMMAND_##cmd[] PROGMEM = {#cmd};
#define COMMAND(cmd, action)     {COMMAND_##cmd, sizeof(#cmd)-1, action}
#define NULL_COMMAND             {NULL, 0, NULL}

DEF_COMMAND(START);
DEF_COMMAND(STOP);
DEF_COMMAND(REWIND);
DEF_COMMAND(AUTO_HOME);
DEF_COMMAND(SET_MIN_OPENING);
DEF_COMMAND(SET_MAX_OPENING);
DEF_COMMAND(SAVE_CONFIG);

const Command Commands[] = {
    COMMAND(START, actionStart),
    COMMAND(STOP, actionStop),
    COMMAND(REWIND, actionRewind),
    COMMAND(AUTO_HOME, actionAutoHome),
    COMMAND(SET_MIN_OPENING, actionSetMinOpening),
    COMMAND(SET_MAX_OPENING, actionSetMaxOpening),
    COMMAND(SAVE_CONFIG, actionSaveConfig),
    NULL_COMMAND
};


void UsbControl::loop(long timeleft) {
    if (timeleft <= 0) return;

    int count = Serial.available();
    if(count) {
        if(count > COMMAND_BUFFER_SIZE - bufferLen)
        {
            count = COMMAND_BUFFER_SIZE - bufferLen;
        }
        char *pIn = buffer + bufferLen;
        Serial.readBytes(pIn, count);
        bufferLen += count;
        char *pEnd = (char *)memchr(pIn, LINE_TERMINATOR, count);
        if(pEnd) {
            *pEnd = 0;
            pIn += count;
            processLine();
            bufferLen = pIn - pEnd;
            memmove(buffer, ++pEnd, bufferLen);
        }
    }
}


void UsbControl::processLine() {
    const Command *pCmd = Commands;
    while(pCmd->action) {
        int res = strcmp_P(buffer, pCmd->command);
        if(res == 0 || (res > 0 && buffer[pCmd->len] == ' ')) {
            pCmd->action(buffer + pCmd->len);
            return;
        }
        pCmd++;
    }
    Serial.print(F(">UKNOWN COMMAND\r"));
}