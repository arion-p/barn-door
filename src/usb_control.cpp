#include <Fsm.h>
#include "usb_control.h"
#include "defines.h"
#include "events.h"
#include "settings.h"

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
    char *end;
    float newMinOpening = strtod(p, &end);
    if(end == p || *end != 0)
    {
        InitialOpening = newMinOpening;
        init();
        Serial.print(F(">OK\r"));
    } else {
        Serial.print(F(">ERR: INVALID PARAMETER\r"));
    }
}


void actionSetMaxOpening(char *p) {
    char *end;
    float newMaxOpening = strtod(p, &end);
    if(end == p || *end != 0)
    {
        MaximumOpening = newMaxOpening;
        init();
        Serial.print(F(">OK\r"));
    } else {
        Serial.print(F(">ERR: INVALID PARAMETER\r"));
    }
}

void actionGetTime(char *p) {
    Serial.print(F(">OK: "));
    Serial.print(micros());
    Serial.print('\r');
}

void actionSetPlanTimes(char *p) {
    long trueTime;
    long sysTime;
    if(2 == sscanf(p, "%ld,%ld", &trueTime, &sysTime)){
        PlanAheadTrueTimeMs = trueTime;
        PlanAheadSysTimeMs = sysTime;
        Serial.print(F(">OK\r"));
    } else {
        Serial.print(F(">ERR: INVALID PARAMETER\r"));
    }
}

void actionSaveConfig(char *p) {
    Serial.print(F(">ERR: NOT SUPPORTED\r"));
}


#define DEF_COMMAND(cmd)         static const char COMMAND_##cmd[] PROGMEM = {#cmd};
#define COMMAND(cmd, action)     {COMMAND_##cmd, sizeof(#cmd)-1, action}
#define NULL_COMMAND             {NULL, 0, NULL}

DEF_COMMAND(GET_TIME);
DEF_COMMAND(START);
DEF_COMMAND(STOP);
DEF_COMMAND(REWIND);
DEF_COMMAND(AUTO_HOME);
DEF_COMMAND(SET_MIN_OPENING);
DEF_COMMAND(SET_MAX_OPENING);
DEF_COMMAND(SET_PLAN_TIMES);
DEF_COMMAND(SAVE_CONFIG);

// Commands are scanned using linear search in the following array
// Place more common or more time sensitive commands first
const Command Commands[] = {
    COMMAND(GET_TIME, actionGetTime), // define this first to minimize delays, since this command is time sensitive
    COMMAND(START, actionStart),
    COMMAND(STOP, actionStop),
    COMMAND(REWIND, actionRewind),
    COMMAND(AUTO_HOME, actionAutoHome),
    COMMAND(SET_MIN_OPENING, actionSetMinOpening),
    COMMAND(SET_MAX_OPENING, actionSetMaxOpening),
    COMMAND(SET_PLAN_TIMES, actionSetPlanTimes),
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
        int res = strncmp_P(buffer, pCmd->command, pCmd->len);
        char delim = buffer[pCmd->len];
        if(res == 0 && (delim == ' ' || delim =='\0')) {
            pCmd->action(buffer + pCmd->len);
            return;
        }
        pCmd++;
    }
    Serial.print(F(">UKNOWN COMMAND\r"));
}