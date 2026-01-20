/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2025 Georgy Dyuldin aka R2RFE
 */
#pragma once

#include <stdlib.h>
#include <stdint.h>

typedef enum __attribute__((__packed__))
{
    x6100_flow_fp32 = 1,
    x6100_flow_bf16,
} x6100_flow_fmt_t;


void custom_cfg_set_flow_fmt(x6100_flow_fmt_t fmt);
void custom_cfg_set_gain_offset(float offset);
