#pragma once
#include "headfile.h"

typedef enum {
    REMOTE_LINK_IDLE = 0,
    REMOTE_LINK_WAIT_ACK,
    REMOTE_LINK_READY,
    REMOTE_LINK_LAUNCH_DELAY,
    REMOTE_LINK_RUNNING,
    REMOTE_LINK_Q3_TIMING,
    REMOTE_LINK_Q3_DONE
} RemoteLinkState;

void CMD_Init(void);
void CMD_RX(void);

void Remote_Link_Tick(uint32_t now_us);
uint8_t Remote_Link_GetQuestion(void);
RemoteLinkState Remote_Link_GetState(void);
void Remote_Link_SetQuestion(uint8_t question);
void Remote_Link_StartPrepare(void);
void Remote_Link_CancelPrepare(void);
uint8_t Remote_Link_RequestLaunch(uint32_t start_us);
void Remote_Link_Stop(void);
uint32_t Remote_Link_GetQ3ElapsedMs(void);
