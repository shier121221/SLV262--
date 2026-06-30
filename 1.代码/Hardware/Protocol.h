#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f10x.h"

#define PROTOCOL_HEAD 0x55
#define PROTOCOL_MAX_PAYLOAD 80
#define PROTOCOL_MAX_FRAME (PROTOCOL_MAX_PAYLOAD + 4)

#define PROTOCOL_MSG_CAR_STATUS      0x01
#define PROTOCOL_MSG_ENV_BATTERY     0x02
#define PROTOCOL_MSG_TRACK_STATUS    0x03
#define PROTOCOL_MSG_THERMAL_SUMMARY 0x04
#define PROTOCOL_MSG_THERMAL_ROW     0x05
#define PROTOCOL_MSG_COMMAND         0x10
#define PROTOCOL_MSG_PARAM_SET       0x11
#define PROTOCOL_MSG_FAULT_EVENT     0x12

typedef struct {
    uint32_t tick_ms;
    uint8_t state;
    uint8_t fault;
    uint8_t lap_count;
    uint8_t battery_percent;
    int16_t speed_l_cms;
    int16_t speed_r_cms;
    int16_t yaw_cdeg;
} CarStatusPayload_t;

typedef struct {
    uint32_t tick_ms;
    int16_t temp_centi;
    int16_t humi_centi;
    uint16_t battery_mv;
    uint8_t battery_percent;
    uint8_t power_state;
} EnvBatteryPayload_t;

typedef struct {
    uint32_t tick_ms;
    int16_t track_error;
    uint8_t track_ready;
    uint8_t use_analog;
    uint16_t analog[8];
} TrackStatusPayload_t;

typedef struct {
    uint32_t tick_ms;
    uint16_t frame_id;
    int16_t max_temp_centi;
    int16_t min_temp_centi;
    int16_t avg_temp_centi;
    uint8_t max_x;
    uint8_t max_y;
} ThermalSummaryPayload_t;

typedef struct {
    uint32_t tick_ms;
    uint16_t frame_id;
    uint8_t row;
    uint8_t width;
    uint16_t temp_raw[32];
} ThermalRowPayload_t;

typedef struct {
    uint8_t cmd;
    int16_t arg1;
    int16_t arg2;
} CommandPayload_t;

typedef struct {
    uint8_t param_id;
    int16_t value;
} ParamSetPayload_t;

uint8_t Protocol_SendFrame(uint8_t msgId, const uint8_t *payload, uint8_t len);
void Protocol_Poll(void);
void Protocol_OnFrame(uint8_t msgId, const uint8_t *payload, uint8_t len);

#endif
