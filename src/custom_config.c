/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2025 Georgy Dyuldin aka R2RFE
 */

#include "custom_config.h"

#include <aether_radio/x6100_control/control.h>
#include "../lvgl/lvgl.h"

#define x6100_custom_cmd 18

typedef struct __attribute__((__packed__)) {
    uint8_t cmd;
    uint32_t value:24;
} i2c_subcmd_t ;

typedef enum __attribute__((__packed__))
{
    x6100_subcmd_flow_params = 1,
    x6100_subcmd_dac_gain_offset,
} x6100_subcmd_enum_t;


void custom_cfg_set_flow_fmt(x6100_flow_fmt_t fmt) {
    i2c_subcmd_t val = {
        .cmd=x6100_subcmd_flow_params,
        .value=fmt & 0x0000ff,
    };
    x6100_control_cmd(x6100_custom_cmd, *(uint32_t*)&val);
    LV_LOG_USER("Switch flow to %s", fmt == x6100_flow_fp32 ? "FP32" : "BF16");
}

void custom_cfg_set_gain_offset(float offset) {
    uint32_t int_val = *(uint32_t*)&offset;
    i2c_subcmd_t val = {
        .cmd=x6100_subcmd_dac_gain_offset,
        .value=int_val >> 8,
    };
    x6100_control_cmd(x6100_custom_cmd, *(uint32_t*)&val);
    LV_LOG_USER("Set gain offset: %f", offset);
}
